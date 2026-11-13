/*
 * owlet_leds.c
 * 路径：src/owlet_leds.c
 *
 * 2026-11-13 修改记录：
 *   蓝牙指示灯逻辑调整：已连接时熄灭，断开/寻找配对时闪烁。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zmk/activity.h>
#include <zmk/ble.h>
#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/hid_indicators_changed.h>

LOG_MODULE_REGISTER(owlet_leds, CONFIG_ZMK_LOG_LEVEL);

/* 标准 HID 协议 LED 状态位 */
#define OWLET_HID_INDICATOR_NUM_LOCK  (1 << 0)
#define OWLET_HID_INDICATOR_CAPS_LOCK (1 << 1)

/* ===================== LED 引用（设备树别名） ===================== */
#define BLE_LED_NODE DT_ALIAS(led0)

#if !DT_NODE_EXISTS(BLE_LED_NODE)
#error "owlet_leds: 缺少 aliases { led0 = <&blue_led>; }"
#endif
static const struct gpio_dt_spec ble_led = GPIO_DT_SPEC_GET(BLE_LED_NODE, gpios);

/* 锁定灯：按需编译 */
#if DT_NODE_EXISTS(DT_ALIAS(caps_led))
#define CAPS_LED_NODE DT_ALIAS(caps_led)
static const struct gpio_dt_spec caps_led = GPIO_DT_SPEC_GET(CAPS_LED_NODE, gpios);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(num_led))
#define NUM_LED_NODE DT_ALIAS(num_led)
static const struct gpio_dt_spec num_led = GPIO_DT_SPEC_GET(NUM_LED_NODE, gpios);
#endif

/* ============================= 全局状态缓存 ============================ */
static uint32_t cached_hid_indicators = 0; /* 缓存当前锁定状态，供唤醒时恢复 */

/* ============================= 蓝牙灯状态 ============================== */
static struct k_work_delayable ble_blink_work;
static bool ble_work_running;
static bool ble_led_is_on;

static void ble_led_set(bool on)
{
    if (on == ble_led_is_on) return;
    if (gpio_pin_set_dt(&ble_led, on ? 1 : 0) < 0) return;
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

    if (owlet_is_connected()) {
        /* 已连接：熄灭，只定期复查是否断开 */
        ble_led_set(false);
        k_work_reschedule(&ble_blink_work, K_MSEC(CONFIG_OWLET_BLE_LED_POLL_MS));
    } else {
        /* 未连接：闪烁 */
        ble_led_set(!ble_led_is_on);
        k_work_reschedule(&ble_blink_work, K_MSEC(ble_led_is_on ? CONFIG_OWLET_BLE_LED_BLINK_ON_MS : CONFIG_OWLET_BLE_LED_BLINK_OFF_MS));
    }
}

static void ble_led_start(void) {
    if (!ble_work_running) { ble_work_running = true; k_work_reschedule(&ble_blink_work, K_NO_WAIT); }
}
static void ble_led_stop(void) {
    if (ble_work_running) { ble_work_running = false; k_work_cancel_delayable(&ble_blink_work); }
    ble_led_set(false);
}

/* ============================= 锁定灯处理 ============================== */
static int owlet_hid_indicators_cb(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    /* 更新缓存，供唤醒时恢复 */
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

/* ======================== 睡眠与唤醒统管 ========================== */
static int owlet_activity_cb(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev) {
        if (ev->state == ZMK_ACTIVITY_SLEEP) {
            /* 进入睡眠：熄灭所有灯并停止蓝牙轮询 */
            ble_led_stop();
#if DT_NODE_EXISTS(DT_ALIAS(caps_led))
            gpio_pin_set_dt(&caps_led, 0);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(num_led))
            gpio_pin_set_dt(&num_led, 0);
#endif
        } else {
            /* 唤醒：恢复蓝牙轮询，并根据缓存状态恢复锁定灯 */
            ble_led_start();
            
#if DT_NODE_EXISTS(DT_ALIAS(caps_led))
            gpio_pin_set_dt(&caps_led, (cached_hid_indicators & OWLET_HID_INDICATOR_CAPS_LOCK) ? 1 : 0);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(num_led))
            gpio_pin_set_dt(&num_led, (cached_hid_indicators & OWLET_HID_INDICATOR_NUM_LOCK) ? 1 : 0);
#endif
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}
ZMK_LISTENER(owlet_ble_led_activity, owlet_activity_cb);
ZMK_SUBSCRIPTION(owlet_ble_led_activity, zmk_activity_state_changed);

/* ============================== 初始化 =============================== */
static int owlet_leds_init(void) {
    if (!device_is_ready(ble_led.port)) return -ENODEV;
    gpio_pin_configure_dt(&ble_led, GPIO_OUTPUT_INACTIVE);
    ble_led_is_on = false;
    k_work_init_delayable(&ble_blink_work, ble_led_work_handler);
    ble_led_start();

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
