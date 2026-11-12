/*
 * owlet_rgb_toggle.c
 * 路径：src/owlet_rgb_toggle.c
 *
 * 2026-11-12 修改记录：
 *   新增 SYS_INIT，开机时强制覆盖 NVS 中保存的饱和度，
 *   防止历史设置的 SAT=0 导致无论怎么调色相都是白色。
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
    gpio_pin_set_dt(&power_pin, 1);
    k_msleep(100);
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

/*
 * 开机强制覆写饱和度。
 * 放在 APPLICATION 的 99 优先级，确保在 ZMK 读完了 NVS 之后执行。
 */
static int owlet_rgb_force_init(void) {
    /* 为了防止旧的 SAT=0 状态，强制按 15 次饱和度增加 */
    for (int i = 0; i < 15; i++) {
        zmk_rgb_underglow_sat_up();
    }
    return 0;
}
SYS_INIT(owlet_rgb_force_init, APPLICATION, 99);
