/*
 * owlet_usb_switch.c
 * 路径：src/owlet_usb_switch.c
 *
 * 2026-11-11 新增：
 *   实现 USB 插入后的 BQ25601D 充电类型检测与 IINDPM 动态配置。
 *   通过 FSUSB43 (P0.11) 切换 D+/D- 总线。
 *
 * 2026-11-11 修改记录：
 *   GPIO 控制改为直接获取 gpio0 设备并控制 pin 11，避免复杂的 DTS 节点定义。
 *   当前为保守值：SDP 300mA，DCP 500mA。
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/usb_conn_state_changed.h>

LOG_MODULE_REGISTER(owlet_usb_switch, CONFIG_ZMK_LOG_LEVEL);

/* BQ25601D I2C 地址与寄存器定义 */
#define BQ25601D_ADDR           0x6B
#define BQ25601D_REG00          0x00 /* Input Current Limit */
#define BQ25601D_REG07          0x07 /* Input Detection */
#define BQ25601D_REG08          0x08 /* Charger Status */

/* 寄存器掩码与偏移 */
#define REG00_IINDPM_MASK       0x1F
#define REG07_IINDET_EN_MASK    BIT(7)
#define REG08_VBUS_STAT_MASK    (BIT(7) | BIT(6) | BIT(5))

/* IINDPM 步进 100mA，数值 N = 电流(mA) / 100 */
#define IINDPM_300MA            3
#define IINDPM_500MA            5

/* FSUSB43 SEL 引脚定义 (P0.11) */
#define SEL_GPIO_PIN            11
static const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

/* 获取 I2C 设备 */
static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static struct k_work_delayable usb_detect_work;

/* 读取寄存器 */
static int bq_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_reg_read_byte(i2c_dev, BQ25601D_ADDR, reg, val);
}

/* 写入寄存器 */
static int bq_write_reg(uint8_t reg, uint8_t val) {
    return i2c_reg_write_byte(i2c_dev, BQ25601D_ADDR, reg, val);
}

/* Work Queue 回调：执行检测与电流配置 */
static void usb_detect_handler(struct k_work *work) {
    uint8_t val;
    
    LOG_INF("USB Inserted, switching FSUSB43 to BQ25601D");
    /* 1. 将 SEL 拉高，把 D+/D- 交给 BQ25601D */
    gpio_pin_set(gpio0_dev, SEL_GPIO_PIN, 1);
    k_msleep(10); // 等待开关切换稳定

    /* 2. 置位 IINDET_EN 强制重新检测 BC1.2 */
    if (bq_read_reg(BQ25601D_REG07, &val) == 0) {
        val |= REG07_IINDET_EN_MASK;
        bq_write_reg(BQ25601D_REG07, val);
    }

    /* 3. 等待检测完成 */
    k_msleep(100);

    /* 4. 读取 VBUS_STAT 判断充电器类型 */
    uint8_t vbus_stat = 0;
    if (bq_read_reg(BQ25601D_REG08, &val) == 0) {
        vbus_stat = (val & REG08_VBUS_STAT_MASK) >> 5;
        LOG_INF("VBUS_STAT detected: %d", vbus_stat);
    }

    /* 5. 配置 IINDPM (REG00) —— 当前使用安全保守值 */
    if (bq_read_reg(BQ25601D_REG00, &val) == 0) {
        if (vbus_stat == 1) { 
            /* 001: USB SDP (电脑) -> 300mA */
            val = (val & ~REG00_IINDPM_MASK) | IINDPM_300MA;
            LOG_INF("USB SDP detected, setting IINDPM to 300mA");
        } else if (vbus_stat == 3) { 
            /* 011: USB DCP (充电器) -> 500mA */
            val = (val & ~REG00_IINDPM_MASK) | IINDPM_500MA;
            LOG_INF("USB DCP detected, setting IINDPM to 500mA");
        } else {
            /* 未知电源 -> 默认 300mA 保底 */
            val = (val & ~REG00_IINDPM_MASK) | IINDPM_300MA;
            LOG_WRN("Unknown USB charger type, setting IINDPM to 300mA");
        }
        bq_write_reg(BQ25601D_REG00, val);
    }

    /* 6. 配置完成，SEL 拉低，将 D+/D- 交还给 nRF52840 以进行 USB 枚举 */
    gpio_pin_set(gpio0_dev, SEL_GPIO_PIN, 0);
    LOG_INF("FSUSB43 handed back to nRF52840 for enumeration");
}

/* USB 状态变化事件回调 */
static int usb_switch_cb(const zmk_event_t *eh) {
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (ev && ev->state == ZMK_USB_CONN_POWERED) {
        /* 在 USB 刚上电未枚举时触发检测 */
        k_work_reschedule(&usb_detect_work, K_MSEC(20));
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(owlet_usb_switch, usb_switch_cb);
ZMK_SUBSCRIPTION(owlet_usb_switch, zmk_usb_conn_state_changed);

/* 初始化函数 */
static int owlet_usb_switch_init(void) {
    if (!device_is_ready(gpio0_dev)) {
        LOG_ERR("GPIO0 not ready");
        return -ENODEV;
    }
    /* 初始化时默认输出低电平，连通 MCU */
    gpio_pin_configure(gpio0_dev, SEL_GPIO_PIN, GPIO_OUTPUT_INACTIVE);
    
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    k_work_init_delayable(&usb_detect_work, usb_detect_handler);
    LOG_INF("owlet usb switch ready");
    return 0;
}
SYS_INIT(owlet_usb_switch_init, APPLICATION, 80);
