/*
 * owlet_charger_event.h
 * 路径：include/owlet_charger_event.h
 *
 * 2026-11-11 新增：
 *   owlet 充电状态事件的公共声明，供主仓库与 UI 仓库共同包含。
 */
#pragma once

#include <zmk/event_manager.h>

struct owlet_charger_state {
    uint8_t  vbus_stat;       /* 1=SDP(电脑), 3=DCP(充电器), 0=未知 */
    uint16_t iindpm_ma;       /* 当前设定输入限流 (mA) */
    uint16_t ichg_ma;         /* 当前设定电池充电电流 (mA) */
    int16_t  battery_temp_c;  /* 电池温度 (℃，典型范围 -40 ~ 125) */
};

ZMK_EVENT_DECLARE(owlet_charger_state_changed);
