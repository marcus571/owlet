/*
 * owlet_charger_event.h
 * 路径：include/owlet_charger_event.h
 *
 * 2026-11-11 修改记录：
 *   将结构体重命名为 owlet_charger_state_changed，与 ZMK_EVENT_DECLARE 参数一致。
 */
#pragma once

#include <zmk/event_manager.h>

struct owlet_charger_state_changed {
    uint8_t  vbus_stat;       /* 1=SDP(电脑), 3=DCP(充电器), 0=未知 */
    uint16_t iindpm_ma;       /* 当前设定输入限流 (mA) */
    uint16_t ichg_ma;         /* 当前设定电池充电电流 (mA) */
    int16_t  battery_temp_c;  /* 电池温度 (℃) */
};

ZMK_EVENT_DECLARE(owlet_charger_state_changed);
