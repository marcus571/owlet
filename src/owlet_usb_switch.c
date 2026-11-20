/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-21 修改记录：
 *   1. 事件结构体改为 static 变量，防止堆栈数据被覆盖清零。
 *   2. BQ25601 初始化时关闭看门狗，防止寄存器写入被复位。
 *   3. 保留硬编码 99/9900 用于验证事件传递。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <zmk/usb.h>
#include <zmk/battery.h>
#include <zmk/event_manager.h>
#include <zmk/events/usb_conn_state_changed.h>

#include "owlet_charger_event.h"

LOG_MODULE_REGISTER(owlet_usb_switch, CONFIG_ZMK_LOG_LEVEL);

ZMK_EVENT_IMPL(owlet_charger_state_changed);

#define BQ25601_ADDR           CONFIG_OWLET_CHARGER_I2C_ADDR
#define BQ25601_REG00          0x00 /* IINDPM */
#define BQ25601_REG01          0x01 /* 控制寄存器，含 WD_RST */
#define BQ25601_REG02          0x02 /* ICHG */
#define BQ25601_REG05          0x05 /* WATCHDOG 配置 */

#define REG00_IINDPM_MASK      0x1F
#define REG02_ICHG_MASK        0x3F

#define MAX17048_ADDR          0x36
#define MAX17048_REG_VCELL     0x02

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static struct k_work_delayable poll_work;
static int64_t charge_start_time;

static uint8_t detected_vbus_stat = 1;
static uint16_t detected_iindpm = 0;
static uint16_t detected_ichg = 0;

/* 静态事件结构体，防止堆栈覆盖 */
static struct owlet_charger_state_changed charger_ev;

static int bq_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_reg_read_byte(i2c_dev, BQ25601_ADDR, reg, val);
}

static int bq_write_reg(uint8_t reg, uint8_t val) {
    return i2c_reg_write_byte(i2c_dev, BQ25601_ADDR, reg, val);
}

/*
 * BQ25601 看门狗必须关闭，否则寄存器会被自动复位。
 * 参考 TI E2E 论坛：写入后立即读回可能仍是默认值。
 */
static void bq25601_disable_watchdog(void) {
    uint8_t val;
    /* REG05[5:4] = 00 禁用看门狗 */
    if (bq_read_reg(BQ25601_REG05, &val) == 0) {
        val &= ~(BIT(5) | BIT(4));
        bq_write_reg(BQ25601_REG05, val);
    }
    /* 写 WD_RST 位复位看门狗状态 */
    if (bq_read_reg(BQ25601_REG01, &val) == 0) {
        val |= BIT(6); /* WD_RST = 1 */
        bq_write_reg(BQ25601_REG01, val);
        /* 稍后清除 WD_RST */
        k_msleep(10);
        bq_read_reg(BQ25601_REG01, &val);
        val &= ~BIT(6);
        bq_write_reg(BQ25601_REG01, val);
    }
}

static void configure_charger(uint16_t iindpm, uint16_t ichg) {
    uint8_t val;
    if (bq_read_reg(BQ25601_REG00, &val) == 0) {
        val = (val & ~REG00_IINDPM_MASK) | (iindpm / 100);
        bq_write_reg(BQ25601_REG00, val);
    }
    if (bq_read_reg(BQ25601_REG02, &val) == 0) {
        val = (val & ~REG02_ICHG_MASK) | (ichg / 60);
        bq_write_reg(BQ25601_REG02, val);
    }
}

static uint16_t read_battery_voltage_mv(void) {
    uint8_t buf[2];
    if (i2c_burst_read(i2c_dev, MAX17048_ADDR, MAX17048_REG_VCELL, buf, 2) != 0) {
        return 0;
    }
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    return (uint16_t)((raw * 125) / 1600);
}

static void poll_handler(struct k_work *work) {
    /* 硬编码固定值，用于验证事件传递 */
    charger_ev.is_charging = true;
    charger_ev.vbus_stat = 1;
    charger_ev.chrg_stat = 2;
    charger_ev.ichg_ma = 99;
    charger_ev.battery_soc = 99;
    charger_ev.battery_voltage_mv = 9900;
    charger_ev.charge_duration_s = 99;

    raise_owlet_charger_state_changed(charger_ev);

    k_work_reschedule(&poll_work, K_MSEC(CONFIG_OWLET_CHARGER_POLL_INTERVAL_MS));
}

static int usb_switch_cb(const zmk_event_t *eh) {
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    if (ev->conn_state == ZMK_USB_CONN_POWERED) {
        charge_start_time = k_uptime_get();
        poll_handler(NULL);
    } else {
        k_work_cancel_delayable(&poll_work);
        charger_ev.is_charging = false;
        raise_owlet_charger_state_changed(charger_ev);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(owlet_usb_switch, usb_switch_cb);
ZMK_SUBSCRIPTION(owlet_usb_switch, zmk_usb_conn_state_changed);

static int owlet_usb_switch_init(void) {
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    /* BQ25601 初始化：关闭看门狗 */
    bq25601_disable_watchdog();

    k_work_init_delayable(&poll_work, poll_handler);
    return 0;
}
SYS_INIT(owlet_usb_switch_init, APPLICATION, 80);
