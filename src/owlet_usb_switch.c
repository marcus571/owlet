/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-12 终极安全版：
 *   硬件检测全瞎（全报3），电流与显示完全依赖 zmk_usb_is_hid_ready()。
 *   电脑口强制 300mA，充电器强制 500mA。表里如一，绝不欺骗硬件。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include <zmk/usb.h>
#include <zmk/battery.h>
#include <zmk/event_manager.h>
#include <zmk/events/usb_conn_state_changed.h>

#include "owlet_charger_event.h"

LOG_MODULE_REGISTER(owlet_usb_switch, CONFIG_ZMK_LOG_LEVEL);

ZMK_EVENT_IMPL(owlet_charger_state_changed);

/* BQ25601D 寄存器 */
#define BQ25601D_ADDR           CONFIG_OWLET_CHARGER_I2C_ADDR
#define BQ25601D_REG00          0x00 /* IINDPM */
#define BQ25601D_REG02          0x02 /* ICHG */
#define BQ25601D_REG07          0x07 /* IINDET_EN */
#define BQ25601D_REG08          0x08 /* CHRG_STAT/VBUS_STAT */
#define BQ25601D_REG0A          0x0A /* TS_ADC 高字节 */
#define BQ25601D_REG0B          0x0B /* TS_ADC 低字节 */

/* 掩码 */
#define REG00_IINDPM_MASK       0x1F
#define REG02_ICHG_MASK         0x3F
#define REG07_IINDET_EN_MASK    BIT(7)
#define REG08_VBUS_STAT_MASK    (BIT(7) | BIT(6) | BIT(5))
#define REG08_CHRG_STAT_MASK    (BIT(4) | BIT(3))

#define SEL_GPIO_PIN            CONFIG_OWLET_CHARGER_SEL_GPIO_PIN
static const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static struct k_work_delayable detect_work;
static struct k_work_delayable poll_work;
static struct k_work_delayable disconnect_work;
static int64_t charge_start_time;

/* 记录 BQ25601D 实际检测到的 VBUS 类型（仅供 UI 观察） */
static uint8_t bq_vbus_stat = 0;

/* 硬件状态缓存，只有目标值变化时才写 I2C，防止频繁写寄存器 */
static uint16_t current_hw_iindpm = 0;
static uint16_t current_hw_ichg = 0;

#define DETECT_DEAD_TIME_MS     3000 
static int64_t last_detect_time = 0;

/* NTC 参数 */
#define NTC_R25_OHM             10000.0f
#define NTC_B_VALUE             3435.0f
#define NTC_T0_K                298.15f
#define TS_R_TOP_OHM            5230.0f
#define TS_R_BOT_OHM            30100.0f

static int bq_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_reg_read_byte(i2c_dev, BQ25601D_ADDR, reg, val);
}
static int bq_write_reg(uint8_t reg, uint8_t val) {
    return i2c_reg_write_byte(i2c_dev, BQ25601D_ADDR, reg, val);
}

static int read_battery_temperature(int16_t *temp_c) {
    uint8_t hi, lo;
    if (bq_read_reg(BQ25601D_REG0A, &hi) != 0) return -EIO;
    if (bq_read_reg(BQ25601D_REG0B, &lo) != 0) return -EIO;

    uint16_t adc_code = ((uint16_t)(hi & 0x0F) << 8) | lo;
    if (adc_code == 0 || adc_code >= 4095) return -ERANGE;

    float ratio = (float)adc_code / 4095.0f;
    float r_parallel = (ratio * TS_R_TOP_OHM) / (1.0f - ratio);
    if (r_parallel >= TS_R_BOT_OHM) return -ERANGE;
    float r_ntc = (r_parallel * TS_R_BOT_OHM) / (TS_R_BOT_OHM - r_parallel);
    if (r_ntc <= 0.0f) return -ERANGE;

    float inv_t = (1.0f / NTC_T0_K) + (1.0f / NTC_B_VALUE) * logf(r_ntc / NTC_R25_OHM);
    *temp_c = (int16_t)((1.0f / inv_t) - 273.15f + 0.5f);
    return 0;
}

static void configure_charger(uint16_t iindpm, uint16_t ichg) {
    uint8_t val;
    if (bq_read_reg(BQ25601D_REG00, &val) == 0) {
        val = (val & ~REG00_IINDPM_MASK) | (iindpm / 100);
        bq_write_reg(BQ25601D_REG00, val);
    }
    if (bq_read_reg(BQ25601D_REG02, &val) == 0) {
        val = (val & ~REG02_ICHG_MASK) | (ichg / 60);
        bq_write_reg(BQ25601D_REG02, val);
    }
}

static void poll_handler(struct k_work *work) {
    uint8_t reg08 = 0;
    uint8_t chrg_stat = 0;
    int16_t temp = 0;
    uint8_t soc = 0;

    uint8_t raw_soc = zmk_battery_state_of_charge();
    if (raw_soc <= 100) soc = raw_soc;

    read_battery_temperature(&temp);
    
    if (bq_read_reg(BQ25601D_REG08, &reg08) == 0) {
        chrg_stat = (reg08 & REG08_CHRG_STAT_MASK) >> 3;
    }

    uint32_t duration = (uint32_t)((k_uptime_get() - charge_start_time) / 1000);

    /* 
     * 终极安全判断：以能否成功与电脑通信为准。
     * 电脑口绝对不能用 500mA，强制降到 300mA。
     */
    bool is_pc = zmk_usb_is_hid_ready();
    uint8_t ui_vbus_stat;
    uint16_t target_iindpm, target_ichg;

    if (is_pc) {
        ui_vbus_stat = 1; /* UI 显示 USB */
        target_iindpm = CONFIG_OWLET_CHARGER_SDP_IINDPM_MA;
        target_ichg = CONFIG_OWLET_CHARGER_SDP_ICHG_MA;
    } else {
        ui_vbus_stat = 3; /* UI 显示 CHG */
        target_iindpm = CONFIG_OWLET_CHARGER_DCP_IINDPM_MA;
        target_ichg = CONFIG_OWLET_CHARGER_DCP_ICHG_MA;
    }

    /* 只有当目标电流变化时，才去改写硬件，保持与 UI 100% 一致 */
    if (target_iindpm != current_hw_iindpm || target_ichg != current_hw_ichg) {
        configure_charger(target_iindpm, target_ichg);
        current_hw_iindpm = target_iindpm;
        current_hw_ichg = target_ichg;
        LOG_INF("Hardware updated to %dmA", target_ichg);
    }

    LOG_INF("UI Type: %s, raw_vbus: %d, ICHG: %dmA",
            ui_vbus_stat == 1 ? "USB" : "CHG", bq_vbus_stat, target_ichg);

    struct owlet_charger_state_changed ev = {
        .is_charging = true,
        .vbus_stat = ui_vbus_stat,       /* 根据软件状态告诉 UI */
        .raw_vbus = bq_vbus_stat,        /* 原始数据原样透传，供你观察 */
        .chrg_stat = chrg_stat,
        .ichg_ma = target_ichg,          /* 显示的值 = 真实写进硬件的值 */
        .battery_soc = soc,
        .battery_temp_c = temp,
        .charge_duration_s = duration,
    };
    raise_owlet_charger_state_changed(ev);

    k_work_reschedule(&poll_work, K_MSEC(CONFIG_OWLET_CHARGER_POLL_INTERVAL_MS));
}

static void disconnect_handler(struct k_work *work) {
    struct owlet_charger_state_changed ev = { .is_charging = false };
    raise_owlet_charger_state_changed(ev);
    LOG_INF("USB unplugged confirmed, sent is_charging = false");
}

static void detect_handler(struct k_work *work) {
    uint8_t val;

    last_detect_time = k_uptime_get();
    
    gpio_pin_set(gpio0_dev, SEL_GPIO_PIN, 1);
    k_msleep(CONFIG_OWLET_CHARGER_SEL_SETTLE_MS);

    if (bq_read_reg(BQ25601D_REG07, &val) == 0) {
        bq_write_reg(BQ25601D_REG07, val | REG07_IINDET_EN_MASK);
    }
    k_msleep(CONFIG_OWLET_CHARGER_DETECT_WAIT_MS);

    if (bq_read_reg(BQ25601D_REG08, &val) == 0) {
        bq_vbus_stat = (val & REG08_VBUS_STAT_MASK) >> 5;
    }

    /* 初始一律按最安全的 300mA 配置，防止在判断出结果前拉爆电脑 */
    configure_charger(CONFIG_OWLET_CHARGER_SDP_IINDPM_MA, CONFIG_OWLET_CHARGER_SDP_ICHG_MA);
    current_hw_iindpm = CONFIG_OWLET_CHARGER_SDP_IINDPM_MA;
    current_hw_ichg = CONFIG_OWLET_CHARGER_SDP_ICHG_MA;

    gpio_pin_set(gpio0_dev, SEL_GPIO_PIN, 0);
    
    charge_start_time = k_uptime_get();
    poll_handler(NULL); 
}

static int usb_switch_cb(const zmk_event_t *eh) {
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    int64_t now = k_uptime_get();
    if (now - last_detect_time < DETECT_DEAD_TIME_MS) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (ev->conn_state == ZMK_USB_CONN_POWERED) {
        k_work_cancel_delayable(&disconnect_work);
        k_work_reschedule(&detect_work, K_MSEC(1000));
    } else {
        k_work_cancel_delayable(&poll_work);
        k_work_reschedule(&disconnect_work, K_MSEC(500)); 
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(owlet_usb_switch, usb_switch_cb);
ZMK_SUBSCRIPTION(owlet_usb_switch, zmk_usb_conn_state_changed);

static int owlet_usb_switch_init(void) {
    if (!device_is_ready(gpio0_dev)) return -ENODEV;
    gpio_pin_configure(gpio0_dev, SEL_GPIO_PIN, GPIO_OUTPUT_INACTIVE);
    if (!device_is_ready(i2c_dev)) return -ENODEV;

    k_work_init_delayable(&detect_work, detect_handler);
    k_work_init_delayable(&poll_work, poll_handler);
    k_work_init_delayable(&disconnect_work, disconnect_handler);
    return 0;
}
SYS_INIT(owlet_usb_switch_init, APPLICATION, 80);
