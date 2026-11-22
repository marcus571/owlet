/*
 * owlet_charger_event.h
 * 路径：include/owlet_charger_event.h
 *
 * 2026-11-22：新增 therm_stat；新增 dbg_reg* 诊断字段（仅 DEBUG 开关填充）。
 */
#pragma once

#include <zmk/event_manager.h>

struct owlet_charger_state_changed {
    bool     is_charging;
    uint8_t  vbus_stat;            /* 0=UNK 1=USB 3=CHG */
    uint8_t  chrg_stat;            /* REG08[4:3] */
    uint8_t  therm_stat;           /* REG08[1] */
    uint16_t ichg_ma;              /* REG02 回读电池充电电流 */
    uint8_t  battery_soc;
    uint16_t battery_voltage_mv;
    uint32_t charge_duration_s;
    /* 诊断原始寄存器（CONFIG_OWLET_CHARGER_DEBUG 时填充） */
    uint8_t  dbg_reg00;            /* IINDPM[4:0] */
    uint8_t  dbg_reg05;            /* WATCHDOG[5:4] 等 */
    uint8_t  dbg_reg08;            /* VBUS_STAT/CHRG_STAT/PG/THERM/VSYS */
    uint8_t  dbg_reg09;            /* bit7=WATCHDOG_FAULT(0=host,1=default) */
    uint8_t  dbg_reg0a;            /* bit5=IINDPM_STAT bit6=VINDPM_STAT */
};

ZMK_EVENT_DECLARE(owlet_charger_state_changed);
