/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-11 修改记录：
 *   1. 新增 ZMK_EVENT_IMPL 以生成事件抛出函数实体。
 *   2. configure_charger 参数由 uint8_t 改为 uint16_t，消除 300/500 截断。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/usb_conn_state_changed.h>

#include "owlet_charger_event.h"

LOG_MODULE_REGISTER(owlet_usb_switch, CONFIG_ZMK_LOG_LEVEL);

/* 生成事件的实际定义与 raise/as 函数 */
ZMK_EVENT_IMPL(owlet_charger_state_changed);

/* BQ25601D I2C 地址与寄存器定义 */
#define BQ25601D_ADDR           CONFIG_OWLET_CHARGER_I2C_ADDR
#define BQ25601D_REG00          0x00 /* Input Current Limit */
#define BQ25601D_REG04          0x04 /* Battery Charge Current Limit */
#define BQ25601D_REG07          0x07 /* Input Detection */
#define BQ25601D_REG08          0x08 /* Charger Status */
#define BQ25601D_REG0A          0x0A /* TS ADC 高字节 */
#define BQ25601D_REG0B          0x0B /* TS ADC 低字节 */

/* 寄存器掩码与偏移 */
#define REG00_IINDPM_MASK       0x1F
#define REG04_ICHG_MASK         0x3F
#define REG07_IINDET_EN_MASK    BIT(7)
#define REG08_VBUS_STAT_MASK    (BIT(7) | BIT(6) | BIT(5))

/* FSUSB43 SEL 引脚定义 */
#define SEL_GPIO_PIN            CONFIG_OWLET_CHARGER_SEL_GPIO_PIN
static const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static struct k_work_delayable usb_detect_work;

/* NTC 与分压网络参数 */
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
    if (adc_code == 0 || adc_code >= 4095) {
        return -ERANGE;
    }

    float ratio = (float)adc_code / 4095.0f;
    float r_parallel = (ratio * TS_R_TOP_OHM) / (1.0f - ratio);
    if (r_parallel >= TS_R_BOT_OHM) {
        return -ERANGE;
    }
    float r_ntc = (r_parallel * TS_R_BOT_OHM) / (TS_R_BOT_OHM - r_parallel);
    if (r_ntc <= 0.0f) {
        return -ERANGE;
    }

    float inv_t = (1.0f / NTC_T0_K) + (1.0f / NTC_B_VALUE) * logf(r_ntc / NTC_R25_OHM);
    float temp_k = 1.0f / inv_t;
    *temp_c = (int16_t)(temp_k - 273.15f + 0.5f);
    return 0;
}

/* 修改：参数类型改为 uint16_t，避免 300/500 被截断 */
static void configure_charger(uint16_t iindpm_ma, uint16_t ichg_ma,
                              uint16_t *actual_iindpm, uint16_t *actual_ichg) {
    uint8_t val;

    if (bq_read_reg(BQ25601D_REG00, &val) == 0) {
        val = (val & ~REG00_IINDPM_MASK) | (iindpm_ma / 100);
        bq_write_reg(BQ25601D_REG00, val);
        *actual_iindpm = (uint16_t)(iindpm_ma / 100) * 100;
        LOG_INF("Set IINDPM to %umA", *actual_iindpm);
    }

    if (bq_read_reg(BQ25601D_REG04, &val) == 0) {
        val = (val & ~REG04_ICHG_MASK) | (ichg_ma / 60);
        bq_write_reg(BQ25601D_REG04, val);
        *actual_ichg = (uint16_t)(ichg_ma / 60) * 60;
        LOG_INF("Set ICHG to %umA", *actual_ichg);
    }
}

static void usb_detect_handler(struct k_work *work) {
    uint8_t val;
    uint8_t vbus_stat = 0;
    uint16_t actual_iindpm = 0, actual_ichg = 0;
    int16_t battery_temp = 0;

    LOG_INF("USB Inserted, switching FSUSB43 to BQ25601D");
    gpio_pin_set(gpio0_dev, SEL_GPIO_PIN, 1);
    k_msleep(CONFIG_OWLET_CHARGER_SEL_SETTLE_MS);

    if (bq_read_reg(BQ25601D_REG07, &val) == 0) {
        val |= REG07_IINDET_EN_MASK;
        bq_write_reg(BQ25601D_REG07, val);
    }
    k_msleep(CONFIG_OWLET_CHARGER_DETECT_WAIT_MS);

    if (bq_read_reg(BQ25601D_REG08, &val) == 0) {
        vbus_stat = (val & REG08_VBUS_STAT_MASK) >> 5;
        LOG_INF("VBUS_STAT detected: %d", vbus_stat);
    }

    if (vbus_stat == 1) {
        LOG_INF("Detected SDP (PC)");
        configure_charger(CONFIG_OWLET_CHARGER_SDP_IINDPM_MA,
                          CONFIG_OWLET_CHARGER_SDP_ICHG_MA,
                          &actual_iindpm, &actual_ichg);
    } else if (vbus_stat == 3) {
        LOG_INF("Detected DCP (Charger)");
        configure_charger(CONFIG_OWLET_CHARGER_DCP_IINDPM_MA,
                          CONFIG_OWLET_CHARGER_DCP_ICHG_MA,
                          &actual_iindpm, &actual_ichg);
    } else {
        LOG_WRN("Unknown USB charger type, using SDP limits");
        configure_charger(CONFIG_OWLET_CHARGER_SDP_IINDPM_MA,
                          CONFIG_OWLET_CHARGER_SDP_ICHG_MA,
                          &actual_iindpm, &actual_ichg);
    }

    if (read_battery_temperature(&battery_temp) != 0) {
        LOG_WRN("Failed to read battery temperature");
        battery_temp = 0;
    } else {
        LOG_INF("Battery temperature: %d C", battery_temp);
    }

    struct owlet_charger_state_changed ev = {
        .vbus_stat = vbus_stat,
        .iindpm_ma = actual_iindpm,
        .ichg_ma = actual_ichg,
        .battery_temp_c = battery_temp,
    };
    raise_owlet_charger_state_changed(ev);

    gpio_pin_set(gpio0_dev, SEL_GPIO_PIN, 0);
    LOG_INF("FSUSB43 handed back to nRF52840 for enumeration");
}

static int usb_switch_cb(const zmk_event_t *eh) {
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (ev && ev->conn_state == ZMK_USB_CONN_POWERED) {
        k_work_reschedule(&usb_detect_work, K_MSEC(20));
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(owlet_usb_switch, usb_switch_cb);
ZMK_SUBSCRIPTION(owlet_usb_switch, zmk_usb_conn_state_changed);

static int owlet_usb_switch_init(void) {
    if (!device_is_ready(gpio0_dev)) {
        LOG_ERR("GPIO0 not ready");
        return -ENODEV;
    }
    gpio_pin_configure(gpio0_dev, SEL_GPIO_PIN, GPIO_OUTPUT_INACTIVE);

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    k_work_init_delayable(&usb_detect_work, usb_detect_handler);
    LOG_INF("owlet usb switch ready");
    return 0;
}
SYS_INIT(owlet_usb_switch_init, APPLICATION, 80);
