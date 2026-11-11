/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-11 修改记录：
 *   新增 BQ25601D TS ADC 读取与 NTC 温度换算。
 *   新增抛出 owlet_charger_state_changed 事件，向 UI 广播充电状态。
 *   NTC 型号 103JT-025 (R25=10kΩ, B=3435K)，TS 分压电阻 5.23kΩ / 30.1kΩ。
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

/* BQ25601D I2C 地址与寄存器定义 */
#define BQ25601D_ADDR           CONFIG_OWLET_CHARGER_I2C_ADDR
#define BQ25601D_REG00          0x00 /* Input Current Limit */
#define BQ25601D_REG04          0x04 /* Battery Charge Current Limit */
#define BQ25601D_REG07          0x07 /* Input Detection */
#define BQ25601D_REG08          0x08 /* Charger Status */
#define BQ25601D_REG0A          0x0A /* TS ADC 高字节 (TS_HI) */
#define BQ25601D_REG0B          0x0B /* TS ADC 低字节 (TS_LO) */

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
#define ADC_FULL_SCALE          32767.0f  /* 15-bit ADC 满量程，以实际芯片为准 */

static int bq_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_reg_read_byte(i2c_dev, BQ25601D_ADDR, reg, val);
}

static int bq_write_reg(uint8_t reg, uint8_t val) {
    return i2c_reg_write_byte(i2c_dev, BQ25601D_ADDR, reg, val);
}

/*
 * 读取 TS ADC 并换算为摄氏度。
 * 返回 0 成功，负值表示读取失败。
 */
static int read_battery_temperature(int16_t *temp_c) {
    uint8_t hi, lo;
    if (bq_read_reg(BQ25601D_REG0A, &hi) != 0) return -EIO;
    if (bq_read_reg(BQ25601D_REG0B, &lo) != 0) return -EIO;

    /*
     * TS_HI 的 bit[3:0] 是 ADC 的高 4 位 (D11:D8)，
     * TS_LO 的全部 8 位是 ADC 的低 8 位 (D7:D0)。
     * 合计 12 位 ADC 码，范围 0 ~ 4095。
     */
    uint16_t adc_code = ((uint16_t)(hi & 0x0F) << 8) | lo;
    if (adc_code == 0 || adc_code >= 4095) {
        return -ERANGE; /* 开路或短路 */
    }

    /* V_TS / V_REGN = adc_code / 4095 */
    float ratio = (float)adc_code / 4095.0f;

    /*
     * ratio = R_parallel / (R_top + R_parallel)
     * => R_parallel = ratio * R_top / (1 - ratio)
     * 其中 R_parallel = (R_bot × R_ntc) / (R_bot + R_ntc)
     * => R_ntc = R_parallel × R_bot / (R_bot - R_parallel)
     */
    float r_parallel = (ratio * TS_R_TOP_OHM) / (1.0f - ratio);
    if (r_parallel >= TS_R_BOT_OHM) {
        return -ERANGE;
    }
    float r_ntc = (r_parallel * TS_R_BOT_OHM) / (TS_R_BOT_OHM - r_parallel);
    if (r_ntc <= 0.0f) {
        return -ERANGE;
    }

    /* B 值方程：1/T = 1/T0 + (1/B) × ln(R_ntc / R25) */
    float inv_t = (1.0f / NTC_T0_K) + (1.0f / NTC_B_VALUE) * logf(r_ntc / NTC_R25_OHM);
    float temp_k = 1.0f / inv_t;
    *temp_c = (int16_t)(temp_k - 273.15f + 0.5f); /* 四舍五入 */
    return 0;
}

static void configure_charger(uint8_t iindpm_ma, uint8_t ichg_ma,
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

    /* 读取电池温度（SEL 尚未拉低，BQ25601D 仍在总线上） */
    if (read_battery_temperature(&battery_temp) != 0) {
        LOG_WRN("Failed to read battery temperature");
        battery_temp = 0;
    } else {
        LOG_INF("Battery temperature: %d C", battery_temp);
    }

    /* 抛出事件，向 UI 广播本次检测结果 */
    raise_owlet_charger_state_changed((struct owlet_charger_state){
        .vbus_stat = vbus_stat,
        .iindpm_ma = actual_iindpm,
        .ichg_ma = actual_ichg,
        .battery_temp_c = battery_temp,
    });

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
