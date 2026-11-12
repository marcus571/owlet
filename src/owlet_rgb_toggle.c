/*
 * owlet_rgb_toggle.c
 * 路径：src/owlet_rgb_toggle.c
 *
 * 2026-11-12 修改记录：
 *   移除错误的 SYS_INIT 强制修改饱和度代码，回归纯粹的按键行为。
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/rgb_underglow.h>

#define DT_DRV_COMPAT zmk_behavior_owlet_rgb_toggle

static const struct gpio_dt_spec power_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(power_en), control_gpios);

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    /* 1. 先强行拉高使能引脚，确保电源已开启 */
    gpio_pin_set_dt(&power_pin, 1);
    
    /* 2. 延时 100ms，让 TPS63802 启动稳定，避开软启动过冲 */
    k_msleep(100);
    
    /* 3. 调用 ZMK 原生 API 切换 RGB */
    zmk_rgb_underglow_toggle();
    
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
