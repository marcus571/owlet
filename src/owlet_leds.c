/*
 * owlet_leds.c
 * 路径：src/owlet_leds.c
 *
 * 2026-11-21：干净断连(0x13/0x15)闪两下后熄灭；超时/其他维持闪烁；
 *   重连或切 profile 恢复。通过自注册 Zephyr bt_conn_cb 获取断连原因码。
 * 2026-11-22（2B 模块化）：
 *   1. 蓝牙灯(led0)改为可选：无 led0 别名时不驱动蓝牙灯，不再 #error。
 *   2. caps/num 锁定灯本就按 DT 别名可选。
 *   3. HID 指示灯监听器加 CONFIG_ZMK_HID_INDICATORS 守卫：
 *      该事件实现文件是条件编译的，关闭时订阅会链接失败，必须守卫。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>

#include <zmk/activity.h>
#include <zmk/ble.h>
#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/ble_active_profile_changed.h>

LOG_MODULE_REGISTER(owlet_leds, CONFIG_ZMK_LOG_LEVEL);

#define OWLET_HID_INDICATOR_NUM_LOCK  (1 << 0)
#define OWLET_HID_INDICATOR_CAPS_LOCK (1 << 1)

/* 干净断连确认闪烁：每 120ms 翻转一次，4 次翻转 = 亮灭亮灭 = 两闪 */
#define OWLET_CONFIRM_TOGGLE_MS   120
#define OWLET_CONFIRM_TOGGLES     4

/* 蓝牙灯可选 */
#if DT_NODE_EXISTS(DT_ALIAS(led0))
#define OWLET_HAS_BLE_LED 1
static const struct gpio_dt_spec ble_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(caps_led))
#define CAPS_LED_NODE DT_ALIAS(caps_led)
static const struct gpio_dt_spec caps_led = GPIO_DT_SPEC_GET(CAPS_LED_NODE, gpios);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(num_led))
#define NUM_LED_NODE DT_ALIAS(num_led)
static const struct gpio_dt_spec num_led = GPIO_DT_SPEC_GET(NUM_LED_NODE, gpios);
#endif

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
static uint32_t cached_hid_indicators = 0;
#endif

static struct k_work_delayable ble_blink_work;
static bool ble_work_running;
static bool ble_led_is_on;

/* 对方正常离开标记与确认闪烁状态 */
static bool peer_left_clean;
static uint8_t confirm_toggles_left;
static uint8_t last_profile_index;

static void ble_led_set(bool on)
{
    if (on == ble_led_is_on) return;
#if OWLET_HAS_BLE_LED
    if (gpio_pin_set_dt(&ble_led, on ? 1 : 0) < 0) return;
#endif
    ble_led_is_on = on;
}

static bool owlet_is_connected(void)
{
#if IS_ENABLED(CONFIG_OWLET_BLE_LED_USB_COUNTS_AS_CONNECTED)
    if (zmk_usb_is_hid_ready()) return true;
#endif
    return zmk_ble_active_profile_is_connected();
}

static void ble_led_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (confirm_toggles_left > 0) {
        ble_led_set(!ble_led_is_on);
        confirm_toggles_left--;
        k_work_reschedule(&ble_blink_work, K_MSEC(OWLET_CONFIRM_TOGGLE_MS));
        return;
    }

    if (owlet_is_connected()) {
        ble_led_set(false);
        k_work_reschedule(&ble_blink_work, K_MSEC(CONFIG_OWLET_BLE_LED_POLL_MS));
    } else if (peer_left_clean) {
        ble_led_set(false);
        k_work_reschedule(&ble_blink_work, K_MSEC(CONFIG_OWLET_BLE_LED_POLL_MS));
    } else {
        ble_led_set(!ble_led_is_on);
        k_work_reschedule(&ble_blink_work, K_MSEC(ble_led_is_on ? CONFIG_OWLET_BLE_LED_BLINK_ON_MS : CONFIG_OWLET_BLE_LED_BLINK_OFF_MS));
    }
}

static void ble_led_start(void) {
#if OWLET_HAS_BLE_LED
    if (!ble_work_running) { ble_work_running = true; k_work_reschedule(&ble_blink_work, K_NO_WAIT); }
#endif
}
static void ble_led_stop(void) {
    if (ble_work_running) { ble_work_running = false; k_work_cancel_delayable(&ble_blink_work); }
    confirm_toggles_left = 0;
    ble_led_set(false);
}

/* ---------- Zephyr 连接回调：获取断连原因码 ---------- */
static void owlet_bt_connected(struct bt_conn *conn, uint8_t err)
{
    ARG_UNUSED(conn);
    if (err) {
        return;
    }
    peer_left_clean = false;
    confirm_toggles_left = 0;
}

static void owlet_bt_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    if (reason == BT_HCI_ERR_REMOTE_USER_TERM_CONN ||
        reason == BT_HCI_ERR_REMOTE_POWER_OFF) {
        peer_left_clean = true;
        ble_led_set(false);
        confirm_toggles_left = OWLET_CONFIRM_TOGGLES;
        if (ble_work_running) {
            k_work_reschedule(&ble_blink_work, K_NO_WAIT);
        }
    } else {
        peer_left_clean = false;
        confirm_toggles_left = 0;
    }
}

static struct bt_conn_cb owlet_conn_cb = {
    .connected = owlet_bt_connected,
    .disconnected = owlet_bt_disconnected,
};

/* ---------- 手动切换 profile：恢复闪烁指示 ---------- */
static int owlet_ble_profile_cb(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev = as_zmk_ble_active_profile_changed(eh);
    if (ev && ev->index != last_profile_index) {
        last_profile_index = ev->index;
        peer_left_clean = false;
        confirm_toggles_left = 0;
    }
    return ZMK_EV_EVENT_BUBBLE;
}
ZMK_LISTENER(owlet_ble_led_profile, owlet_ble_profile_cb);
ZMK_SUBSCRIPTION(owlet_ble_led_profile, zmk_ble_active_profile_changed);

/* ---------- HID 锁定指示灯（仅 CONFIG_ZMK_HID_INDICATORS 启用时） ---------- */
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
static int owlet_hid_indicators_cb(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;
    cached_hid_indicators = ev->indicators;
#if DT_NODE_EXISTS(DT_ALIAS(caps_led))
    gpio_pin_set_dt(&caps_led, (ev->indicators & OWLET_HID_INDICATOR_CAPS_LOCK) ? 1 : 0);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(num_led))
    gpio_pin_set_dt(&num_led, (ev->indicators & OWLET_HID_INDICATOR_NUM_LOCK) ? 1 : 0);
#endif
    return ZMK_EV_EVENT_BUBBLE;
}
ZMK_LISTENER(owlet_hid_led, owlet_hid_indicators_cb);
ZMK_SUBSCRIPTION(owlet_hid_led, zmk_hid_indicators_changed);
#endif /* CONFIG_ZMK_HID_INDICATORS */

/* ---------- 睡眠/唤醒 ---------- */
static int owlet_activity_cb(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev) {
        if (ev->state == ZMK_ACTIVITY_SLEEP) {
            ble_led_stop();
#if DT_NODE_EXISTS(DT_ALIAS(caps_led))
            gpio_pin_set_dt(&caps_led, 0);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(num_led))
            gpio_pin_set_dt(&num_led, 0);
#endif
        } else {
            ble_led_start();
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#if DT_NODE_EXISTS(DT_ALIAS(caps_led))
            gpio_pin_set_dt(&caps_led, (cached_hid_indicators & OWLET_HID_INDICATOR_CAPS_LOCK) ? 1 : 0);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(num_led))
            gpio_pin_set_dt(&num_led, (cached_hid_indicators & OWLET_HID_INDICATOR_NUM_LOCK) ? 1 : 0);
#endif
#endif /* CONFIG_ZMK_HID_INDICATORS */
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}
ZMK_LISTENER(owlet_ble_led_activity, owlet_activity_cb);
ZMK_SUBSCRIPTION(owlet_ble_led_activity, zmk_activity_state_changed);

static int owlet_leds_init(void) {
    k_work_init_delayable(&ble_blink_work, ble_led_work_handler);
    last_profile_index = zmk_ble_active_profile_index();
    /* 与 ZMK 核心的连接回调并存，Zephyr 支持多回调注册 */
    bt_conn_cb_register(&owlet_conn_cb);

#if OWLET_HAS_BLE_LED
    if (!device_is_ready(ble_led.port)) return -ENODEV;
    gpio_pin_configure_dt(&ble_led, GPIO_OUTPUT_INACTIVE);
    ble_led_is_on = false;
    ble_led_start();
#endif

#if DT_NODE_EXISTS(DT_ALIAS(caps_led))
    if (device_is_ready(caps_led.port)) gpio_pin_configure_dt(&caps_led, GPIO_OUTPUT_INACTIVE);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(num_led))
    if (device_is_ready(num_led.port)) gpio_pin_configure_dt(&num_led, GPIO_OUTPUT_INACTIVE);
#endif
    LOG_INF("owlet leds indicator ready");
    return 0;
}
SYS_INIT(owlet_leds_init, APPLICATION, 90);
