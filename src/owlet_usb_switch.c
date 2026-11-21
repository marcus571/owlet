/*
* owlet_usb_switch.c
* 路径：src/owlet_usb_switch.c
*
* 2026-11-21 修复记录（第二次）：
* 根因修复：ZMK v0.3 的 zmk_usb_conn_state 有三个状态
* NONE / POWERED / HID。插电脑枚举成功后会再上报 HID 状态，
* 原代码把非 POWERED 一律当拔线，导致 HID 事件取消了首次轮询
* 并广播 is_charging=false（表现为 UNK/500mA 冻结、30 秒回落主屏）。
* 1. 充电判定改为 conn_state != ZMK_USB_CONN_NONE（与官方
* zmk_usb_is_powered() 口径一致），只有 NONE 才是拔线。
* 2. 首次收到 HID 状态视为枚举完成，立即（200ms 后）触发一次
* 轮询，尽快把 type 刷成 USB 并写入 SDP 电流。
* 3. 重复的 POWERED/HID（挂起、恢复等）上报一律忽略，
* 不重置计时、不打断轮询周期。
* 4. 保留上一轮修复：先排下一轮再做读数与广播；插入瞬间
* 广播 UNK + 硬件默认 500mA 初始状态。
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

#define BQ25601_ADDR CONFIG_OWLET_CHARGER_I2C_ADDR
#define BQ25601_REG00 0x00
#define BQ25601_REG01 0x01
#define BQ25601_REG02 0x02
#define BQ25601_REG05 0x05

#define REG00_IINDPM_MASK 0x1F
#define REG02_ICHG_MASK 0x3F

#define MAX17048_ADDR 0x36
#define MAX17048_REG_VCELL 0x02

/* 首次轮询延迟：等待 USB 枚举完成后再判定 SDP/DCP */
#define OWLET_FIRST_POLL_DELAY_MS 2000
/* 收到 HID（枚举完成）信号后的快速刷新延迟 */
#define OWLET_HID_REFRESH_DELAY_MS 200

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

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

static uint8_t read_battery_soc(void) {
uint8_t soc = zmk_battery_state_of_charge();
if (soc > 100) {
soc = 0;
}
return soc;
}

static void poll_handler(struct k_work *work) {
ARG_UNUSED(work);

/* 关键：先把下一轮排上，之后的任何异常都不会让轮询永久停止 */
k_work_reschedule(&poll_work, K_MSEC(CONFIG_OWLET_CHARGER_POLL_INTERVAL_MS));

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

charger_ev.is_charging = true;
charger_ev.vbus_stat = ui_vbus_stat;
charger_ev.chrg_stat = 2;
charger_ev.ichg_ma = target_ichg;
charger_ev.battery_soc = read_battery_soc();
charger_ev.battery_voltage_mv = read_battery_voltage_mv();
charger_ev.charge_duration_s = (uint32_t)((k_uptime_get() - charge_start_time) / 1000);

raise_owlet_charger_state_changed(charger_ev);
}

/* 插入瞬间的初始状态：接口类型未知，电流按硬件默认值 */
static void publish_initial_state(void) {
charger_ev.is_charging = true;
charger_ev.vbus_stat = 0;
charger_ev.chrg_stat = 0;
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
/* 首次上电：记录起点并广播 UNK + 硬件默认电流 */
powered = true;
hid_refreshed = false;
charge_start_time = k_uptime_get();
detected_iindpm = 0;
detected_ichg = 0;

publish_initial_state();

if (ev->conn_state == ZMK_USB_CONN_HID) {
/* 上电即已枚举（如复位后线未拔）：立刻刷新判定 */
hid_refreshed = true;
k_work_reschedule(&poll_work, K_MSEC(OWLET_HID_REFRESH_DELAY_MS));
} else {
k_work_reschedule(&poll_work, K_MSEC(OWLET_FIRST_POLL_DELAY_MS));
}
} else if (ev->conn_state == ZMK_USB_CONN_HID && !hid_refreshed) {
/* POWERED → HID：枚举完成，尽快判定为 USB 口 */
hid_refreshed = true;
k_work_reschedule(&poll_work, K_MSEC(OWLET_HID_REFRESH_DELAY_MS));
}
/* 其余重复上报（POWERED 重复、HID 挂起/恢复等）：忽略 */
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

static int owlet_usb_switch_init(void) {
k_work_init_delayable(&poll_work, poll_handler);

if (!device_is_ready(i2c_dev)) {
LOG_ERR("I2C device not ready");
return -ENODEV;
}

bq25601_disable_watchdog();
return 0;
}
SYS_INIT(owlet_usb_switch_init, APPLICATION, 80);
