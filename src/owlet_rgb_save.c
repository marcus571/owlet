/*
 * owlet_rgb_save.c
 * 路径：src/owlet_rgb_save.c
 *
 * 2026-11-14 修改记录（方案A验证，修正 HSB 函数参数）：
 *   不读取任何当前 state，直接硬编码设定纯绿色 HSB(120, 100, 20)，
 *   然后立即保存。用于验证 Flash 存储链路本身是否正常。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/rgb_underglow.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(owlet_rgb_save, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_behavior_owlet_rgb_save

extern int zmk_rgb_underglow_save_state(void);

/* ZMK 内部使用的 HSB 结构体定义 */
struct zmk_led_hsb {
    uint16_t h;
    uint8_t s;
    uint8_t b;
};

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    /* 硬编码设定纯绿色 HSB(120, 100, 20) */
    struct zmk_led_hsb color = { .h = 120, .s = 100, .b = 20 };
    int ret_set = zmk_rgb_underglow_set_hsb(color);
    /* 立即保存 */
    int ret_save = zmk_rgb_underglow_save_state();
    LOG_INF("Hardcode set=%d, save=%d", ret_set, ret_save);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api owlet_rgb_save_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define OWLET_RGB_SAVE_INST(n) \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, \
                            APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
                            &owlet_rgb_save_driver_api);

DT_INST_FOREACH_STATUS_OKAY(OWLET_RGB_SAVE_INST)
