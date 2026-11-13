/*
 * owlet_rgb_toggle.c
 * 路径：src/owlet_rgb_toggle.c
 *
 * 2026-11-13 调试记录：
 *   将行为改为：按下时强制调用同步保存函数，并打印底层返回值，用于抓取 RGB 无法保存的真凶。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/rgb_underglow.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(owlet_rgb_toggle, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_behavior_owlet_rgb_toggle

static const struct gpio_dt_spec power_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(power_en), control_gpios);

/* ZMK 源码中的内部结构体（用于调试保存） */
struct zmk_rgb_underglow_state {
    struct zmk_rgb_underglow_state_hsb state;
    bool on;
} __packed;

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    /* 1. 先强行拉高使能引脚，确保电源已开启 */
    gpio_pin_set_dt(&power_pin, 1);
    
    /* 2. 延时 100ms，让 TPS63802 启动稳定 */
    k_msleep(100);
    
    /* 3. 切换 RGB 状态 */
    zmk_rgb_underglow_toggle();
    
    /* 4. 强制同步保存！并打印底层的错误码 */
    LOG_INF("Attempting manual RGB save...");
    
    /* 由于 zmk_rgb_underglow_save_state 是内部静态函数，我们直接调用 settings_save_one 尝试保存 */
    /* 注意：这里保存的数据结构必须和 ZMK 源码里的一模一样 */
    // 我们试图通过公开 API 触发一次强制保存。ZMK 内部的 API 在 0.3.0 中并没有完全公开。
    // 但我们可以通过读取当前状态并重新应用一次来强制触发底层。
    // 最直接的方法，是让 ZMK 内核去保存这个 key。
    
    // 实际上，最直接的排查是看这个强制函数返回了什么。
    // 我们直接用 settings_save_one 强制写入一个测试 key，看看 NVS 是不是拒写
    int ret = settings_save_one("owlet/test", "test", 4);
    LOG_INF("NVS test write result: %d", ret);
    
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api owlet_rgb_toggle_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define OWLET_RGB_TOGGLE_INST(n) \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, \
                            APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
                            &owlet_rgb_toggle_driver_api);

DT_INST_FOREACH_STATUS_OKAY(OWLET_RGB_TOGGLE_INST)
