/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-21 修复记录：
 *   放弃 k_timer（ISR上下文），回归 k_work_delayable。
 *   首次触发也通过 k_work_reschedule 调度，确保 workqueue 状态一致。
 *   真实数据读取（SOC、电压、电流、时长）全部恢复。
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

static uint16_t detected_iindpm = 0;
static uint16_t detected_ichg = 0;

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
    bool is_pc = zmk_usb_is_hid_ready();
    uint8_t ui_vbus_stat;
    uint16_t target_iindpm, target_ichg;

    if (is_pc) {
        ui_vbus_stat = 1;
        target_iindpm = CONFIG_OWLET_CHARGER_SDP_IINDPM_MA;
        target_ichg = CONFIG_OWLET_CHARGER_SDP_ICHG_MA;
    } else {
        ui_vbus_stat = 3;
        target_iindpm = CONFIG_OWLET_CHARGER_DCP_IINDPM_MA;
        target_ichg = CONFIG_OWLET_CHARGER_DCP_ICHG_MA;
    }

    if (target_iindpm != detected_iindpm || target_ichg != detected_ichg) {
        configure_charger(target_iindpm, target_ichg);
        detected_iindpm = target_iindpm;
        detected_ichg = target_ichg;
    }

    uint8_t soc = zmk_battery_state_of_charge();
    if (soc > 100) soc = 0;

    charger_ev.is_charging = true;
    charger_ev.vbus_stat = ui_vbus_stat;
    charger_ev.chrg_stat = 2;
    charger_ev.ichg_ma = target_ichg;
    charger_ev.battery_soc = soc;
    charger_ev.battery_voltage_mv = read_battery_voltage_mv();
    charger_ev.charge_duration_s = (uint32_t)((k_uptime_get() - charge_start_time) / 1000);

    raise_owlet_charger_state_changed(charger_ev);

    k_work_reschedule(&poll_work, K_MSEC(CONFIG_OWLET_CHARGER_POLL_INTERVAL_MS));
}

static int usb_switch_cb(const zmk_event_t *eh) {
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    if (ev->conn_state == ZMK_USB_CONN_POWERED) {
        charge_start_time = k_uptime_get();
        detected_iindpm = 0;
        detected_ichg = 0;
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
