/*
 * owlet_charger_event.h
 * 路径：include/owlet_charger_event.h
 *
 * 2026-11-23（终版）：移除诊断字段，保留正式功能字段。
 */
#pragma once

#include <zmk/event_manager.h>

struct owlet_charger_state_changed {
    bool     is_charging;          /* true=充电中, false=未充电 */
    uint8_t  vbus_stat;            /* 接口类型 (0=UNK 1=USB 3=CHG，固件判定) */
    uint8_t  chrg_stat;            /* 充电阶段 (REG08[4:3] 0..3) */
    uint8_t  therm_stat;           /* 热调节 (REG08[1] 1=芯片正在热降流) */
    uint16_t ichg_ma;              /* 芯片回读的真实设定充电电流 (mA) */
    uint8_t  battery_soc;          /* 电量百分比 0-100 */
    uint16_t battery_voltage_mv;   /* 当前电池电压 (mV) */
    uint32_t charge_duration_s;    /* 充电已持续秒数 */
};

ZMK_EVENT_DECLARE(owlet_charger_state_changed);
