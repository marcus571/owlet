/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-22：在 host 模式修复基础上，新增 CONFIG_OWLET_CHARGER_DEBUG
 *   诊断分支：每轮回读 REG00/05/08/09/0A 原始值上报，供屏幕显示定位。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <zmk/usb.h>
#include <zmk/battery.h>
#include <zmk/event_manager.h>
#include <zmk/events/usb_conn_state_changed.h>

#include "owlet_charger_event.h"

LOG_MODULE_REGISTER(owlet_usb_switch, CONFIG_ZMK_LOG_LEVEL);

ZMK_EVENT_IMPL(owlet_charger_state_changed);

#define BQ25601_NODE           DT_NODELABEL(bq25601)
#define BQ25601_ADDR           CONFIG_OWLET_CHARGER_I2C_ADDR
#define BQ25601_REG00          0x00
#define BQ25601_REG01          0x01
#define BQ25601_REG02          0x02
#define BQ25601_REG05          0x05
#define BQ25601_REG08          0x08
#define BQ25601_REG09          0x09
#define BQ25601_REG0A          0x0A

#define REG00_IINDPM_MASK      0x1F
#define REG02_ICHG_MASK        0x3F

#define MAX17048_ADDR          0x36
#define MAX17048_REG_VCELL     0x02

#define OWLET_FIRST_POLL_DELAY_MS  2000
#define OWLET_HID_REFRESH_DELAY_MS 200
#define OWLET_INT_REFRESH_DELAY_MS 100

static const struct device *i2c_dev = DEVICE_DT_GET(DT_BUS(BQ25601_NODE));

#if DT_NODE_HAS_PROP(BQ25601_NODE, int_gpios)
#define OWLET_HAS_CHARGER_INT 1
static const struct gpio_dt_spec charger_int = GPIO_DT_SPEC_GET(BQ25601_NODE, int_gpios);
static struct gpio_callback charger_int_cb;
#endif

static struct k_work_delayable poll_work;
static int64_t charge_start_time;
static bool powered;
static bool hid_refreshed;

static uint16_t detected_iindpm = 0;
static uint16_t detected_ichg = 0;

static struct owlet_charger_state_changed charger_ev;

static int bq_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_reg_read_byte(i2c_dev, BQ25601_ADDR, reg, val);
}

static int bq_write_reg(uint8_t reg, uint8_t val) {
    return i2c_reg_write_byte(i2c_dev, BQ25601_ADDR, reg, val);
}

static void bq25601_init_regs(void) {
    uint8_t val;

    if (bq_read_reg(BQ25601_REG01, &val) == 0) {
        bq_write_reg(BQ25601_REG01, val | BIT(6)); /* WD_RST=1 进 host 模式 */
        k_msleep(10);
    }
    if (bq_read_reg(BQ25601_REG05, &val) == 0) {
        val &= ~(BIT(5) | BIT(4)); /* WATCHDOG=00 保持 host 模式 */
        bq_write_reg(BQ25601_REG05, val);
    }
    if (bq_read_reg(BQ25601_REG0A, &val) == 0) {
        val |= BIT(1) | BIT(0); /* 屏蔽 DPM 中断 */
        bq_write_reg(BQ25601_REG0A, val);
    }
}

static uint16_t ichg_round_to_step(uint16_t ma) {
    uint16_t code = (ma + 30) / 60;
    if (code > 50) {
        code = 50;
    }
    return code * 60;
}

static void configure_charger(uint16_t iindpm, uint16_t ichg) {
    uint8_t val;
    uint16_t code;

    if (bq_read_reg(BQ25601_REG00, &val) == 0) {
        code = (iindpm > 100) ? ((iindpm - 100) / 100) : 0;
        if (code > 31) {
            code = 31;
        }
        bq_write_reg(BQ25601_REG00, (val & ~REG00_IINDPM_MASK) | code);
    }
    if (bq_read_reg(BQ25601_REG02, &val) == 0) {
        code = ichg / 60;
        if (code > 50) {
            code = 50;
        }
        bq_write_reg(BQ25601_REG02, (val & ~REG02_ICHG_MASK) | code);
    }
}

static uint16_t read_ichg_actual_ma(void) {
    uint8_t val;
    if (bq_read_reg(BQ25601_REG02, &val) == 0) {
        return (uint16_t)(val & REG02_ICHG_MASK) * 60;
    }
    return 0;
}

static uint16_t read_iindpm_actual_ma(void) {
    uint8_t val;
    if (bq_read_reg(BQ25601_REG00, &val) == 0) {
        return (uint16_t)((val & REG00_IINDPM_MASK) + 1) * 100;
    }
    return 0;
}

static uint8_t read_chrg_stat(void) {
    uint8_t val;
    if (bq_read_reg(BQ25601_REG08, &val) == 0) {
        return (val >> 3) & 0x03;
    }
    return 0;
}

static uint8_t read_therm_stat(void) {
    uint8_t val;
    if (bq_read_reg(BQ25601_REG08, &val) == 0) {
        return (val >> 1) & 0x01;
    }
    return 0;
}

static void bq25601_clear_faults(void) {
    uint8_t val;
    bq_read_reg(BQ25601_REG09, &val);
    bq_read_reg(BQ25601_REG09, &val);
}

static uint16_t read_battery_voltage_mv(void) {
    uint8_t buf[2];
    if (i2c_burst_read(i2c_dev, MAX17048_ADDR, MAX17048_REG_VCELL, buf, 2) != 0) {
        return 0;
    }
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    return (uint16_t)((raw * 125) / 1600);
}

static uint8_t read_battery_soc(void) {
    uint8_t soc = zmk_battery_state_of_charge();
    if (soc > 100) {
        soc = 0;
    }
    return soc;
}

static void poll_handler(struct k_work *work) {
    ARG_UNUSED(work);

    k_work_reschedule(&poll_work, K_MSEC(CONFIG_OWLET_CHARGER_POLL_INTERVAL_MS));

    bq25601_clear_faults();

    int64_t elapsed_ms = k_uptime_get() - charge_start_time;
    bool is_pc = zmk_usb_is_hid_ready();

    if (is_pc || elapsed_ms >= OWLET_FIRST_POLL_DELAY_MS) {
        uint8_t ui_vbus_stat;
        uint16_t target_iindpm, target_ichg;

        if (is_pc) {
            ui_vbus_stat = 1;
            target_iindpm = CONFIG_OWLET_CHARGER_SDP_IINDPM_MA;
            target_ichg = ichg_round_to_step(CONFIG_OWLET_CHARGER_SDP_ICHG_MA);
        } else {
            ui_vbus_stat = 3;
            target_iindpm = CONFIG_OWLET_CHARGER_DCP_IINDPM_MA;
            target_ichg = ichg_round_to_step(CONFIG_OWLET_CHARGER_DCP_ICHG_MA);
        }

        if (target_iindpm != detected_iindpm || target_ichg != detected_ichg) {
            configure_charger(target_iindpm, target_ichg);
            detected_iindpm = target_iindpm;
            detected_ichg = target_ichg;
        }

        uint16_t actual_ichg = read_ichg_actual_ma();
        charger_ev.vbus_stat = ui_vbus_stat;
        charger_ev.ichg_ma = (actual_ichg != 0) ? actual_ichg : target_ichg;
    } else {
        uint16_t actual_iindpm = read_iindpm_actual_ma();
        charger_ev.vbus_stat = 0;
        charger_ev.ichg_ma = (actual_iindpm != 0)
                                 ? actual_iindpm
                                 : CONFIG_OWLET_CHARGER_DEFAULT_ICHG_MA;
    }

    charger_ev.is_charging = true;
    charger_ev.chrg_stat = read_chrg_stat();
    charger_ev.therm_stat = read_therm_stat();
    charger_ev.battery_soc = read_battery_soc();
    charger_ev.battery_voltage_mv = read_battery_voltage_mv();
    charger_ev.charge_duration_s = (uint32_t)(elapsed_ms / 1000);

#if IS_ENABLED(CONFIG_OWLET_CHARGER_DEBUG)
    bq_read_reg(BQ25601_REG00, &charger_ev.dbg_reg00);
    bq_read_reg(BQ25601_REG05, &charger_ev.dbg_reg05);
    bq_read_reg(BQ25601_REG08, &charger_ev.dbg_reg08);
    bq_read_reg(BQ25601_REG09, &charger_ev.dbg_reg09);
    bq_read_reg(BQ25601_REG0A, &charger_ev.dbg_reg0a);
#endif

    raise_owlet_charger_state_changed(charger_ev);
}

static void publish_initial_state(void) {
    uint16_t actual_iindpm = read_iindpm_actual_ma();

    charger_ev.is_charging = true;
    charger_ev.vbus_stat = 0;
    charger_ev.chrg_stat = read_chrg_stat();
    charger_ev.therm_stat = read_therm_stat();
    charger_ev.ichg_ma = (actual_iindpm != 0)
                             ? actual_iindpm
                             : CONFIG_OWLET_CHARGER_DEFAULT_ICHG_MA;
    charger_ev.battery_soc = read_battery_soc();
    charger_ev.battery_voltage_mv = read_battery_voltage_mv();
    charger_ev.charge_duration_s = 0;

    raise_owlet_charger_state_changed(charger_ev);
}

static int usb_switch_cb(const zmk_event_t *eh) {
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    if (ev->conn_state != ZMK_USB_CONN_NONE) {
        if (!powered) {
            powered = true;
            hid_refreshed = false;
            charge_start_time = k_uptime_get();
            detected_iindpm = 0;
            detected_ichg = 0;

            publish_initial_state();
            k_work_reschedule(&poll_work, K_MSEC(OWLET_FIRST_POLL_DELAY_MS));
        }

        if (!hid_refreshed && zmk_usb_is_hid_ready()) {
            hid_refreshed = true;
            k_work_reschedule(&poll_work, K_MSEC(OWLET_HID_REFRESH_DELAY_MS));
        }
    } else {
        powered = false;
        hid_refreshed = false;
        k_work_cancel_delayable(&poll_work);
        detected_iindpm = 0;
        detected_ichg = 0;
        memset(&charger_ev, 0, sizeof(charger_ev));
        raise_owlet_charger_state_changed(charger_ev);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(owlet_usb_switch, usb_switch_cb);
ZMK_SUBSCRIPTION(owlet_usb_switch, zmk_usb_conn_state_changed);

#if OWLET_HAS_CHARGER_INT
static void charger_int_isr(const struct device *dev, struct gpio_callback *cb,
                            uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    if (powered) {
        k_work_reschedule(&poll_work, K_MSEC(OWLET_INT_REFRESH_DELAY_MS));
    }
}
#endif

static int owlet_usb_switch_init(void) {
    k_work_init_delayable(&poll_work, poll_handler);

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    bq25601_init_regs();

#if OWLET_HAS_CHARGER_INT
    if (device_is_ready(charger_int.port)) {
        gpio_pin_configure_dt(&charger_int, GPIO_INPUT | GPIO_PULL_UP);
        gpio_init_callback(&charger_int_cb, charger_int_isr, BIT(charger_int.pin));
        gpio_add_callback(charger_int.port, &charger_int_cb);
        gpio_pin_interrupt_configure_dt(&charger_int, GPIO_INT_EDGE_FALLING);
    }
#endif

    return 0;
}
SYS_INIT(owlet_usb_switch_init, APPLICATION, 80);
