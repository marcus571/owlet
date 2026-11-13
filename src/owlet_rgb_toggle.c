/*
 * owlet_rgb_toggle.c
 * 路径：src/owlet_rgb_toggle.c
 *
 * 2026-11-13 最终修复版：
 *   在切换 RGB 状态后，直接获取当前 RGB 状态，并强制写入 ZMK 的标准 NVS key。
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

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    /* 1. 拉高电源使能引脚 */
    gpio_pin_set_dt(&power_pin, 1);
    k_msleep(100);

    /* 2. 切换 RGB 状态 */
    zmk_rgb_underglow_toggle();

    /* 3. 获取当前 RGB 状态，强制写入 ZMK 标准 NVS key */
    struct zmk_rgb_underglow_state state;
    if (zmk_rgb_underglow_get_state(&state) == 0) {
        int ret = settings_save_one("rgb_underglow/state", &state, sizeof(state));
        LOG_INF("Forced RGB save result: %d", ret);
    } else {
        LOG_ERR("Failed to get RGB state");
    }

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
