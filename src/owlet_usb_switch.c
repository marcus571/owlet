/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-23（终版）：PSEL 硬件方案落地，删除全部排查补丁
 *   （WD_RST/host 模式、IINDET_EN、OVP 抬阈值、IINDPM 写入、诊断回读）。
 *   电流控制两条线：
 *   - ICHG(REG02) 设电池充电电流，写入有效，每轮校验、漂移即重写；
 *   - psel-gpios 选输入上限档：高=2.4A（充电器），低=500mA（电脑/默认）。
 *   顺序保证：先写 ICHG 再拉高 PSEL，避免"限流放开而 ICHG 仍为默认"的瞬态。
 *   保留：UNK 判定窗口、HID 快速刷新、INT 即时刷新、15s 轮询、
 *   REG09 双读清故障、REG0A 屏蔽 DPM、芯片看门狗关闭、热标志、拔线清零。
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

#if DT_NODE_HAS_PROP(BQ25601_NODE, psel_gpios)
#define OWLET_HAS_PSEL 1
static const struct gpio_dt_spec psel_ctl = GPIO_DT_SPEC_GET(BQ25601_NODE, psel_gpios);
#endif

static struct k_work_delayable poll_work;
static int64_t charge_start_time;
static bool powered;
static bool hid_refreshed;

static struct owlet_charger_state_changed charger_ev;

static int bq_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_reg_read_byte(i2c_dev, BQ25601_ADDR, reg, val);
}

static int bq_write_reg(uint8_t reg, uint8_t val) {
    return i2c_reg_write_byte(i2c_dev, BQ25601_ADDR, reg, val);
}

/* PSEL 档位：true=充电器档(2.4A)，false=电脑档/安全默认(500mA) */
static void psel_set_charger(bool charger) {
#if OWLET_HAS_PSEL
    gpio_pin_set_dt(&psel_ctl, charger ? 1 : 0);
#else
    ARG_UNUSED(charger);
#endif
}

static void bq25601_init_regs(void) {
    uint8_t val;
    /* 关芯片看门狗，防止寄存器被周期性打回默认 */
    if (bq_read_reg(BQ25601_REG05, &val) == 0) {
        val &= ~(BIT(5) | BIT(4));
        bq_write_reg(BQ25601_REG05, val);
    }
    /* 屏蔽 VINDPM/IINDPM 中断，避免 DPM 期间 INT 频繁触发 */
    if (bq_read_reg(BQ25601_REG0A, &val) == 0) {
        val |= BIT(1) | BIT(0);
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

static uint16_t read_ichg_actual_ma(void) {
    uint8_t val;
    if (bq_read_reg(BQ25601_REG02, &val) == 0) {
        return (uint16_t)(val & REG02_ICHG_MASK) * 60;
    }
    return 0;
}

static void write_ichg_ma(uint16_t ma) {
    uint8_t val;
    if (bq_read_reg(BQ25601_REG02, &val) == 0) {
        uint16_t code = ma / 60;
        if (code > 50) {
            code = 50;
        }
        bq_write_reg(BQ25601_REG02, (val & ~REG02_ICHG_MASK) | code);
    }
}

/* UNK 阶段显示用：回读芯片当前输入限流（PSEL 默认值） */
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
        uint16_t target_ichg = ichg_round_to_step(
            is_pc ? CONFIG_OWLET_CHARGER_SDP_ICHG_MA : CONFIG_OWLET_CHARGER_DCP_ICHG_MA);

        /* 每轮校验 ICHG，漂移即重写（ICHG 写入有效，不受 PSEL 影响） */
        uint16_t cur_ichg = read_ichg_actual_ma();
        if (cur_ichg != target_ichg) {
            write_ichg_ma(target_ichg);
            cur_ichg = read_ichg_actual_ma();
        }

        /* 先 ICHG 后 PSEL：充电器档放开输入上限前，电池电流目标已就位 */
        psel_set_charger(!is_pc);

        charger_ev.vbus_stat = is_pc ? 1 : 3;
        charger_ev.ichg_ma = (cur_ichg != 0) ? cur_ichg : target_ichg;
    } else {
        /* 判定窗口内：PSEL 保持安全档(500mA)，显示 UNK + 芯片实际输入限流 */
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

            psel_set_charger(false); /* 上电先回安全档 */
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
        psel_set_charger(false); /* 拔线回安全档 */
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

#if OWLET_HAS_PSEL
    if (device_is_ready(psel_ctl.port)) {
        /* 初始输出低 = MOSFET 关断 = PSEL 高 = 500mA 安全档 */
        gpio_pin_configure_dt(&psel_ctl, GPIO_OUTPUT_INACTIVE);
    }
#endif

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
