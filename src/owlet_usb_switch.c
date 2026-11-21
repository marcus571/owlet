/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-21 修复记录（第三轮，依据 BQ25601 手册 ZHCSGA3A 核对）：
 *   1. 快速刷新必须以 zmk_usb_is_hid_ready() 实际就绪为门槛。
 *      ZMK 把 SUSPEND/RESUME 也映射为 HID 状态，原逻辑在枚举完成前
 *      就触发轮询，导致插电脑先误显示 CHG/400mA。
 *   2. 插入后 2 秒判定窗口内未枚举成功则保持 UNK + 硬件默认电流，
 *      不提前写入 SDP/DCP 配置（INT 触发的早期刷新同样受此约束）。
 *   3. REG00 IINDPM 编码修正：偏移 100mA、步进 100mA，code=(mA-100)/100。
 *   4. REG02 ICHG 步进 60mA：设定值就近取整，并回读实际值用于显示。
 *   5. CHRG_STAT 改为实时读取 REG08[4:3]：
 *      00 未充电 / 01 预充 / 10 快充 / 11 充电完成。
 *   6. 看门狗：REG05[5:4]=00 关闭（默认 40s）；删除 REG01 WD_RST 脉冲，
 *      该操作会把全部寄存器复位回默认值（ICHG 回到 2040mA）。
 *   7. INT（P0.12，开漏低有效 256us 脉冲）下降沿中断触发立即刷新；
 *      每轮双读 REG09 清除锁存故障（否则芯片不再发新 INT）；
 *      REG0A[1:0]=11 屏蔽 VINDPM/IINDPM 中断，避免 DPM 期间频繁触发。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
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
#define BQ25601_REG02          0x02
#define BQ25601_REG05          0x05
#define BQ25601_REG08          0x08
#define BQ25601_REG09          0x09
#define BQ25601_REG0A          0x0A

#define REG00_IINDPM_MASK      0x1F
#define REG02_ICHG_MASK        0x3F

#define MAX17048_ADDR          0x36
#define MAX17048_REG_VCELL     0x02

/* 插入后的接口类型判定窗口：窗口内未枚举则保持 UNK 显示 */
#define OWLET_FIRST_POLL_DELAY_MS  2000
/* HID 枚举完成后的快速刷新延迟 */
#define OWLET_HID_REFRESH_DELAY_MS 200
/* INT 脉冲后的快速刷新延迟（兼作去抖） */
#define OWLET_INT_REFRESH_DELAY_MS 100

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct device *int_gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

static struct k_work_delayable poll_work;
static struct gpio_callback charger_int_cb;
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

/* 关看门狗 + 屏蔽 DPM 中断（REG0A 高 6 位为只读状态，写回无副作用） */
static void bq25601_init_regs(void) {
    uint8_t val;
    if (bq_read_reg(BQ25601_REG05, &val) == 0) {
        val &= ~(BIT(5) | BIT(4)); /* WATCHDOG[1:0] = 00 关闭 */
        bq_write_reg(BQ25601_REG05, val);
    }
    if (bq_read_reg(BQ25601_REG0A, &val) == 0) {
        val |= BIT(1) | BIT(0); /* 屏蔽 VINDPM / IINDPM INT 脉冲 */
        bq_write_reg(BQ25601_REG0A, val);
    }
}

/* ICHG 设定值就近取整到 60mA 步进（0-3000mA） */
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
        /* IINDPM：偏移 100mA，步进 100mA，范围 100mA-3.2A */
        code = (iindpm > 100) ? ((iindpm - 100) / 100) : 0;
        if (code > 31) {
            code = 31;
        }
        bq_write_reg(BQ25601_REG00, (val & ~REG00_IINDPM_MASK) | code);
    }
    if (bq_read_reg(BQ25601_REG02, &val) == 0) {
        /* ICHG：偏移 0，步进 60mA，范围 0-3000mA */
        code = ichg / 60;
        if (code > 50) {
            code = 50;
        }
        bq_write_reg(BQ25601_REG02, (val & ~REG02_ICHG_MASK) | code);
    }
}

/* 回读芯片内实际的 ICHG 设定值 */
static uint16_t read_ichg_actual_ma(void) {
    uint8_t val;
    if (bq_read_reg(BQ25601_REG02, &val) == 0) {
        return (uint16_t)(val & REG02_ICHG_MASK) * 60;
    }
    return 0;
}

/* REG08[4:3]：00 未充电 / 01 预充 / 10 快充 / 11 充电完成 */
static uint8_t read_chrg_stat(void) {
    uint8_t val;
    if (bq_read_reg(BQ25601_REG08, &val) == 0) {
        return (val >> 3) & 0x03;
    }
    return 0;
}

/* 双读 REG09：第一次清锁存故障，第二次取当前故障，恢复后续 INT 能力 */
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

    /* 先排下一轮，任何单次异常都不会断掉周期刷新 */
    k_work_reschedule(&poll_work, K_MSEC(CONFIG_OWLET_CHARGER_POLL_INTERVAL_MS));

    bq25601_clear_faults();

    int64_t elapsed_ms = k_uptime_get() - charge_start_time;
    bool is_pc = zmk_usb_is_hid_ready();

    /* 判定窗口内未枚举成功：保持 UNK + 硬件默认电流，不写芯片配置 */
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
            detected_ichg = read_ichg_actual_ma();
            if (detected_ichg == 0) {
                detected_ichg = target_ichg; /* 回读失败时退回目标值 */
            }
        }

        charger_ev.vbus_stat = ui_vbus_stat;
        charger_ev.ichg_ma = detected_ichg;
    } else {
        charger_ev.vbus_stat = 0; /* UNK */
        charger_ev.ichg_ma = CONFIG_OWLET_CHARGER_DEFAULT_ICHG_MA;
    }

    charger_ev.is_charging = true;
    charger_ev.chrg_stat = read_chrg_stat();
    charger_ev.battery_soc = read_battery_soc();
    charger_ev.battery_voltage_mv = read_battery_voltage_mv();
    charger_ev.charge_duration_s = (uint32_t)(elapsed_ms / 1000);

    raise_owlet_charger_state_changed(charger_ev);
}

/* 插入瞬间的初始状态：接口类型未知，电流按硬件默认值 */
static void publish_initial_state(void) {
    charger_ev.is_charging = true;
    charger_ev.vbus_stat = 0;
    charger_ev.chrg_stat = read_chrg_stat();
    charger_ev.ichg_ma = CONFIG_OWLET_CHARGER_DEFAULT_ICHG_MA;
    charger_ev.battery_soc = read_battery_soc();
    charger_ev.battery_voltage_mv = read_battery_voltage_mv();
    charger_ev.charge_duration_s = 0;

    raise_owlet_charger_state_changed(charger_ev);
}

static int usb_switch_cb(const zmk_event_t *eh) {
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    /* 只要不是 NONE 就代表 VBUS 在位（POWERED 或 HID 都在充电） */
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

        /* 只有真正枚举为 HID 设备才快速刷新；
         * SUSPEND/RESUME 虽也映射为 HID 状态，但 is_hid_ready 为 false */
        if (!hid_refreshed && zmk_usb_is_hid_ready()) {
            hid_refreshed = true;
            k_work_reschedule(&poll_work, K_MSEC(OWLET_HID_REFRESH_DELAY_MS));
        }
    } else {
        /* 真正拔线 */
        powered = false;
        hid_refreshed = false;
        k_work_cancel_delayable(&poll_work);
        detected_iindpm = 0;
        detected_ichg = 0;
        charger_ev.is_charging = false;
        raise_owlet_charger_state_changed(charger_ev);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(owlet_usb_switch, usb_switch_cb);
ZMK_SUBSCRIPTION(owlet_usb_switch, zmk_usb_conn_state_changed);

/* BQ25601 INT：开漏低有效 256us 脉冲。
 * 触发源：输入源识别 / 电源接入与移除 / 充电完成 / 故障（DPM 已屏蔽）。
 * ISR 中仅安排一次快速刷新，I2C 读取在工作队列中完成。 */
static void charger_int_isr(const struct device *dev, struct gpio_callback *cb,
                            uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    if (powered) {
        k_work_reschedule(&poll_work, K_MSEC(OWLET_INT_REFRESH_DELAY_MS));
    }
}

static int owlet_usb_switch_init(void) {
    k_work_init_delayable(&poll_work, poll_handler);

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    bq25601_init_regs();

    if (device_is_ready(int_gpio_dev)) {
        gpio_pin_configure(int_gpio_dev, CONFIG_OWLET_CHARGER_INT_GPIO_PIN,
                           GPIO_INPUT | GPIO_PULL_UP);
        gpio_init_callback(&charger_int_cb, charger_int_isr,
                           BIT(CONFIG_OWLET_CHARGER_INT_GPIO_PIN));
        gpio_add_callback(int_gpio_dev, &charger_int_cb);
        gpio_pin_interrupt_configure(int_gpio_dev, CONFIG_OWLET_CHARGER_INT_GPIO_PIN,
                                     GPIO_INT_EDGE_FALLING);
    }

    return 0;
}
SYS_INIT(owlet_usb_switch_init, APPLICATION, 80);
