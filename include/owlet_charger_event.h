/*
 * owlet_charger_event.h
 * 路径：include/owlet_charger_event.h
 *
 * 2026-11-22：诊断字段扩展（reg01/07/0b），用于定位 IINDPM 被钳根因。
 */
#pragma once

#include <zmk/event_manager.h>

struct owlet_charger_state_changed {
    bool     is_charging;
    uint8_t  vbus_stat;
    uint8_t  chrg_stat;
    uint8_t  therm_stat;
    uint16_t ichg_ma;
    uint8_t  battery_soc;
    uint16_t battery_voltage_mv;
    uint32_t charge_duration_s;
    /* 诊断原始寄存器（CONFIG_OWLET_CHARGER_DEBUG 时填充） */
    uint8_t  dbg_reg00;            /* IINDPM[4:0] */
    uint8_t  dbg_reg01;            /* CHG_CONFIG/WD_RST 等 */
    uint8_t  dbg_reg05;            /* WATCHDOG 等 */
    uint8_t  dbg_reg07;            /* bit7=IINDET_EN 强制源检测 */
    uint8_t  dbg_reg08;            /* 状态 */
    uint8_t  dbg_reg09;            /* bit7=WATCHDOG_FAULT */
    uint8_t  dbg_reg0a;            /* bit5=IINDPM_STAT bit6=VINDPM_STAT */
    uint8_t  dbg_reg0b;            /* PN[6:3] 芯片型号 */
};

ZMK_EVENT_DECLARE(owlet_charger_state_changed);
