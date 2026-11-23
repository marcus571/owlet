/*
 * owlet_wdt.c
 * 路径：src/owlet_wdt.c
 *
 * MCU 看门狗：固件跑飞时复位 SoC。复位后 GPIO 回高阻默认态，
 * PSEL 的 100k 栅极下拉使 MOSFET 关断 → PSEL 高 → 500mA 安全档。
 * 深睡(System OFF)时 WDT 停止、唤醒即复位，不影响 ZMK 睡眠；
 * 进 bootloader 走 reboot，WDT 随之清零，不影响 UF2 烧录。
 */

#define DT_DRV_COMPAT nordic_nrf_wdt

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(owlet_wdt, CONFIG_ZMK_LOG_LEVEL);

static const struct device *const wdt_dev = DEVICE_DT_GET(DT_INST(0, nordic_nrf_wdt));
static struct k_work_delayable feed_work;
static int wdt_channel;

static void feed_handler(struct k_work *work) {
    ARG_UNUSED(work);
    wdt_feed(wdt_dev, wdt_channel);
    k_work_reschedule(&feed_work, K_MSEC(CONFIG_OWLET_WDT_FEED_MS));
}

static int owlet_wdt_init(void) {
    if (!device_is_ready(wdt_dev)) {
        LOG_ERR("WDT device not ready");
        return -ENODEV;
    }

    struct wdt_timeout_cfg cfg = {
        .window = { .min = 0, .max = CONFIG_OWLET_WDT_TIMEOUT_MS },
        .callback = NULL,
        .flags = WDT_FLAG_RESET_SOC,
    };

    wdt_channel = wdt_install_timeout(wdt_dev, &cfg);
    if (wdt_channel < 0) {
        LOG_ERR("WDT install timeout failed (%d)", wdt_channel);
        return wdt_channel;
    }

    int ret = wdt_setup(wdt_dev, 0);
    if (ret < 0) {
        LOG_ERR("WDT setup failed (%d)", ret);
        return ret;
    }

    k_work_init_delayable(&feed_work, feed_handler);
    k_work_reschedule(&feed_work, K_NO_WAIT);
    LOG_INF("owlet watchdog armed");
    return 0;
}
SYS_INIT(owlet_wdt_init, APPLICATION, 95);
