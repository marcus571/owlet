/*
 * owlet_charger_event.h
 * 路径：include/owlet_charger_event.h
 *
 * 2026-11-22：诊断字段含 reg06（OVP/VINDPM 阈值）。
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
    uint8_t  dbg_reg00;
    uint8_t  dbg_reg01;
    uint8_t  dbg_reg05;
    uint8_t  dbg_reg06;
    uint8_t  dbg_reg07;
    uint8_t  dbg_reg08;
    uint8_t  dbg_reg09;
    uint8_t  dbg_reg0a;
    uint8_t  dbg_reg0b;
};

ZMK_EVENT_DECLARE(owlet_charger_state_changed);
