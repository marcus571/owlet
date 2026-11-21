/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-21 修改记录：
 *   1. 首次触发改为 k_work_reschedule，避免直接调用 handler 导致 work 状态不一致。
 *   2. 保留 static 事件结构体和硬编码 99/9900，继续验证。
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
#define BQ25601_REG00          0x00
#define BQ25601_REG01          0x01
#define BQ25601_REG02          0x02
#define BQ25601_REG05          0x05

#define REG00_IINDPM_MASK      0x1F
#define REG02_ICHG_MASK        0x3F

#define MAX17048_ADDR          0x36
#define MAX17048_REG_VCELL     0x02

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static struct k_work_delayable poll_work;
static int64_t charge_start_time;

static struct owlet_charger_state_changed charger_ev;

static int bq_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_reg_read_byte(i2c_dev, BQ25601_ADDR, reg, val);
}

static int bq_write_reg(uint8_t reg, uint8_t val) {
    return i2c_reg_write_byte(i2c_dev, BQ25601_ADDR, reg, val);
}

static void bq25601_disable_watchdog(void) {
    uint8_t val;
    if (bq_read_reg(BQ25601_REG05, &val) == 0) {
        val &= ~(BIT(5) | BIT(4));
        bq_write_reg(BQ25601_REG05, val);
    }
    if (bq_read_reg(BQ25601_REG01, &val) == 0) {
        val |= BIT(6);
        bq_write_reg(BQ25601_REG01, val);
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
        k_work_reschedule(&poll_work, K_NO_WAIT);
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

    bq25601_disable_watchdog();

    k_work_init_delayable(&poll_work, poll_handler);
    return 0;
}
SYS_INIT(owlet_usb_switch_init, APPLICATION, 80);
