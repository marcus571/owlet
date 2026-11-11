/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-11 修改记录：
 *   所有 IINDPM 和 ICHG 参数均从 Kconfig 读取，方便通过 owlet.conf 修改。
 *   新增电池充电电流 (ICHG) 的配置。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/usb_conn_state_changed.h>

LOG_MODULE_REGISTER(owlet_usb_switch, CONFIG_ZMK_LOG_LEVEL);

/* BQ25601D I2C 地址与寄存器定义 */
#define BQ25601D_ADDR           CONFIG_OWLET_CHARGER_I2C_ADDR
#define BQ25601D_REG00          0x00 /* Input Current Limit */
#define BQ25601D_REG04          0x04 /* Battery Charge Current Limit */
#define BQ25601D_REG07          0x07 /* Input Detection */
#define BQ25601D_REG08          0x08 /* Charger Status */

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

/* 读取寄存器 */
static int bq_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_reg_read_byte(i2c_dev, BQ25601D_ADDR, reg, val);
}

/* 写入寄存器 */
static int bq_write_reg(uint8_t reg, uint8_t val) {
    return i2c_reg_write_byte(i2c_dev, BQ25601D_ADDR, reg, val);
}

/* 配置 BQ25601D 寄存器（IINDPM 步进 100mA，ICHG 步进 60mA） */
static void configure_charger(uint8_t iindpm_ma, uint8_t ichg_ma) {
    uint8_t val;

    /* 1. 配置输入限流 IINDPM (REG00) */
    if (bq_read_reg(BQ25601D_REG00, &val) == 0) {
        val = (val & ~REG00_IINDPM_MASK) | (iindpm_ma / 100);
        bq_write_reg(BQ25601D_REG00, val);
        LOG_INF("Set IINDPM to %dmA", iindpm_ma);
    }

    /* 2. 配置电池充电电流 ICHG (REG04) */
    if (bq_read_reg(BQ25601D_REG04, &val) == 0) {
        val = (val & ~REG04_ICHG_MASK) | (ichg_ma / 60);
        bq_write_reg(BQ25601D_REG04, val);
        LOG_INF("Set ICHG to %dmA", ichg_ma);
    }
}

/* Work Queue 回调：执行检测与电流配置 */
static void usb_detect_handler(struct k_work *work) {
    uint8_t val;
    
    LOG_INF("USB Inserted, switching FSUSB43 to BQ25601D");
    gpio_pin_set(gpio0_dev, SEL_GPIO_PIN, 1);
    k_msleep(CONFIG_OWLET_CHARGER_SEL_SETTLE_MS);

    if (bq_read_reg(BQ25601D_REG07, &val) == 0) {
        val |= REG07_IINDET_EN_MASK;
        bq_write_reg(BQ25601D_REG07, val);
    }

    k_msleep(CONFIG_OWLET_CHARGER_DETECT_WAIT_MS);

    uint8_t vbus_stat = 0;
    if (bq_read_reg(BQ25601D_REG08, &val) == 0) {
        vbus_stat = (val & REG08_VBUS_STAT_MASK) >> 5;
        LOG_INF("VBUS_STAT detected: %d", vbus_stat);
    }

    if (vbus_stat == 1) { 
        /* 001: USB SDP (电脑) */
        LOG_INF("Detected SDP (PC)");
        configure_charger(CONFIG_OWLET_CHARGER_SDP_IINDPM_MA, CONFIG_OWLET_CHARGER_SDP_ICHG_MA);
    } else if (vbus_stat == 3) { 
        /* 011: USB DCP (充电器) */
        LOG_INF("Detected DCP (Charger)");
        configure_charger(CONFIG_OWLET_CHARGER_DCP_IINDPM_MA, CONFIG_OWLET_CHARGER_DCP_ICHG_MA);
    } else {
        /* 未知电源 -> 默认 SDP 配置保底 */
        LOG_WRN("Unknown USB charger type, using SDP limits");
        configure_charger(CONFIG_OWLET_CHARGER_SDP_IINDPM_MA, CONFIG_OWLET_CHARGER_SDP_ICHG_MA);
    }

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
