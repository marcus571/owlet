/*
 * owlet_charger_event.h
 * 路径：include/owlet_charger_event.h
 *
 * 2026-11-22：新增 dbg_rst_ichg（WD_RST 后立刻回读的 ICHG，2040=WD_RST生效）。
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
    uint16_t dbg_rst_ichg;         /* WD_RST 后回读 ICHG：2040=复位生效 */
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
