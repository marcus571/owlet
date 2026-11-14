/*
 * owlet_rgb_save.c
 * 路径：src/owlet_rgb_save.c
 *
 * 2026-11-14 修改记录（方案B验证）：
 *   在调用 zmk_rgb_underglow_save_state() 之前，连续调用两次
 *   zmk_rgb_underglow_toggle()，强制同步内存中的 RGB 状态结构体，
 *   验证“state 脱节导致保存幽灵色”的推断。
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

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    /* 1. 第一次 toggle：强制 ZMK 同步并刷新内部 state */
    zmk_rgb_underglow_toggle();
    /* 2. 第二次 toggle：切回原状态，再次刷新 state */
    zmk_rgb_underglow_toggle();
    /* 3. 保存 */
    int ret = zmk_rgb_underglow_save_state();
    LOG_INF("Force-sync save result: %d", ret);
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
