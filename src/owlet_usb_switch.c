/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-12 修改记录：
 *   1. 移除对 iindpm_ma 的发送。
 *   2. 新增 raw_vbus 透传，直接将硬件检测值发给 UI。
 *   3. 根据 raw_vbus 硬件值直接配置三种不同的安全电流（SDP=300, CDP=500, DCP=500）。
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

/* 记录 BQ25601D 实际检测到的 VBUS 类型 */
static uint8_t bq_vbus_stat = 0;
static uint16_t detected_iindpm = 0;
static uint16_t detected_ichg = 0;

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

    LOG_INF("Charger: SOC=%d%%, Temp=%dC, raw_vbus=%d, chrg=%d, dur=%us",
            soc, temp, bq_vbus_stat, chrg_stat, duration);

    struct owlet_charger_state_changed ev = {
        .is_charging = true,
        .vbus_stat = bq_vbus_stat,       /* 直接透传 BQ 检测到的类型 */
        .raw_vbus = bq_vbus_stat,        /* 原始数据，与 vbus_stat 保持一致，供 UI 排查 */
        .chrg_stat = chrg_stat,
        .ichg_ma = detected_ichg,        /* 只保留电池充电电流 */
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

    /* 读取 BQ25601D 原始检测值 */
    if (bq_read_reg(BQ25601D_REG08, &val) == 0) {
        bq_vbus_stat = (val & REG08_VBUS_STAT_MASK) >> 5;
    }
    
    LOG_INF("Raw BQ25601D Detection: %d", bq_vbus_stat);

    /* 根据硬件检测的原始类型，配置三档电流 */
    if (bq_vbus_stat == 1) { 
        /* 1 = SDP (标准电脑口) */
        detected_iindpm = CONFIG_OWLET_CHARGER_SDP_IINDPM_MA;
        detected_ichg = CONFIG_OWLET_CHARGER_SDP_ICHG_MA;
    } else if (bq_vbus_stat == 2) {
        /* 2 = CDP (充电电脑口) */
        detected_iindpm = CONFIG_OWLET_CHARGER_CDP_IINDPM_MA;
        detected_ichg = CONFIG_OWLET_CHARGER_CDP_ICHG_MA;
    } else if (bq_vbus_stat == 3) { 
        /* 3 = DCP (专用充电器) */
        detected_iindpm = CONFIG_OWLET_CHARGER_DCP_IINDPM_MA;
        detected_ichg = CONFIG_OWLET_CHARGER_DCP_ICHG_MA;
    } else {
        /* 未知或非标，按最安全的 SDP 处理 */
        detected_iindpm = CONFIG_OWLET_CHARGER_SDP_IINDPM_MA;
        detected_ichg = CONFIG_OWLET_CHARGER_SDP_ICHG_MA;
    }

    configure_charger(detected_iindpm, detected_ichg);

    gpio_pin_set(gpio0_dev, SEL_GPIO_PIN, 0);
    
    charge_start_time = k_uptime_get();
    poll_handler(NULL); 
}

static int usb_switch_cb(const zmk_event_t *eh) {
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    int64_t now = k_uptime_get();
    /* 3 秒硬件死区，防止切换 D+/D- 引起枚举乱跳 */
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
