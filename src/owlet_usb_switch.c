/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-19 修改记录：
 *   1. 移除 FSUSB43 SEL 引脚控制。
 *   2. 移除 BQ25601D REG08 VBUS_STAT 检测逻辑。
 *   3. 保留通过 zmk_usb_is_hid_ready() 判断接口类型。
 *   4. 电流配置改为 USB 200mA / 充电器 400mA。
 *   5. 保留 MAX17048 电压读取（通过 I2C 直接读取 VCELL 寄存器）。
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

/* BQ25601 寄存器 */
#define BQ25601_ADDR           CONFIG_OWLET_CHARGER_I2C_ADDR
#define BQ25601_REG00          0x00 /* IINDPM */
#define BQ25601_REG02          0x02 /* ICHG */

/* MAX17048 寄存器与地址 */
#define MAX17048_ADDR          0x36
#define MAX17048_REG_VCELL     0x02

/* 掩码 */
#define REG00_IINDPM_MASK      0x1F
#define REG02_ICHG_MASK        0x3F

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static struct k_work_delayable poll_work;
static int64_t charge_start_time;

/* 记录初次检测的结果 */
static uint8_t detected_vbus_stat = 1; /* 默认按 SDP 处理 */
static uint16_t detected_iindpm = 0;
static uint16_t detected_ichg = 0;

static int bq_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_reg_read_byte(i2c_dev, BQ25601_ADDR, reg, val);
}

static int bq_write_reg(uint8_t reg, uint8_t val) {
    return i2c_reg_write_byte(i2c_dev, BQ25601_ADDR, reg, val);
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

/* 直接读取 MAX17048 的 VCELL 寄存器获取电压 */
static uint16_t read_battery_voltage_mv(void) {
    uint8_t buf[2];
    if (i2c_burst_read(i2c_dev, MAX17048_ADDR, MAX17048_REG_VCELL, buf, 2) != 0) {
        return 0;
    }
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    /* VCELL LSB = 78.125uV = 0.078125mV */
    return (uint16_t)((raw * 125) / 1600);
}

static void poll_handler(struct k_work *work) {
    int16_t temp = 0;
    uint8_t soc = 0;

    uint8_t raw_soc = zmk_battery_state_of_charge();
    if (raw_soc <= 100) soc = raw_soc;

    uint16_t vbat_mv = read_battery_voltage_mv();

    uint32_t duration = (uint32_t)((k_uptime_get() - charge_start_time) / 1000);

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

    struct owlet_charger_state_changed ev = {
        .is_charging = true,
        .vbus_stat = ui_vbus_stat,
        .chrg_stat = 2,
        .ichg_ma = target_ichg,
        .battery_soc = soc,
        .battery_voltage_mv = vbat_mv,
        .charge_duration_s = duration,
    };
    raise_owlet_charger_state_changed(ev);

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
        struct owlet_charger_state_changed ev = { .is_charging = false };
        raise_owlet_charger_state_changed(ev);
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
    k_work_init_delayable(&poll_work, poll_handler);
    return 0;
}
SYS_INIT(owlet_usb_switch_init, APPLICATION, 80);
