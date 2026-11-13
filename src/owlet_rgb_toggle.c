/*
 * owlet_rgb_toggle.c
 * 路径：src/owlet_rgb_toggle.c
 *
 * 2026-11-13 修正版：
 *   移除不匹配的 zmk_rgb_underglow_get_state 声明。
 *   直接调用 ZMK 内部的 zmk_rgb_underglow_save_state() 来强制保存 RGB 状态。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/rgb_underglow.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(owlet_rgb_toggle, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_behavior_owlet_rgb_toggle

static const struct gpio_dt_spec power_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(power_en), control_gpios);

/* 直接声明 ZMK 内部的保存函数 */
extern int zmk_rgb_underglow_save_state(void);

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    /* 1. 拉高电源使能引脚 */
    gpio_pin_set_dt(&power_pin, 1);
    k_msleep(100);

    /* 2. 切换 RGB 状态 */
    zmk_rgb_underglow_toggle();

    /* 3. 立即强制保存 */
    int ret = zmk_rgb_underglow_save_state();
    LOG_INF("Forced RGB save result: %d", ret);

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
