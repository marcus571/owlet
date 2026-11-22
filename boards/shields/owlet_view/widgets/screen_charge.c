// boards/shields/owlet_view/widgets/screen_charge.c
//
// 2026-11-22：诊断布局改为 IIN/PN/07/01/0A/08，定位 IINDPM 钳流根因。
#include "screen_charge.h"
#include "util.h"
#include "../assets/custom_fonts.h"
#include <stdio.h>

static void format_current(char *buf, size_t len, uint16_t current_ma) {
    if (current_ma < 1000) {
        snprintf(buf, len, "%umA", current_ma);
    } else {
        snprintf(buf, len, "%u.%02uA", current_ma / 1000, (current_ma % 1000) / 10);
    }
}

static void format_duration(char *buf, size_t len, uint32_t seconds) {
    uint32_t mins = seconds / 60;
    if (mins < 60) {
        snprintf(buf, len, "%um", mins);
    } else {
        snprintf(buf, len, "%uh%02um", mins / 60, mins % 60);
    }
}

void draw_charge_screen(lv_obj_t *canvas, const struct owlet_charger_state_changed *state) {
    lv_draw_label_dsc_t label_dsc;
    init_label_dsc(&label_dsc, LVGL_FOREGROUND, &pixel_operator_mono, LV_TEXT_ALIGN_LEFT);

    char text[32];

    lv_draw_rect_dsc_t line_dsc;
    init_rect_dsc(&line_dsc, LVGL_FOREGROUND);
    lv_canvas_draw_rect(canvas, 80, 4, 1, 61, &line_dsc);

#if IS_ENABLED(CONFIG_OWLET_CHARGER_DEBUG)
    /* 诊断布局：左 IIN/PN/07，右 01/0A/08 */
    uint16_t iin = (uint16_t)((state->dbg_reg00 & 0x1F) + 1) * 100;
    snprintf(text, sizeof(text), "IIN:%u", iin);
    lv_canvas_draw_text(canvas, 4, 8, 76, &label_dsc, text);

    snprintf(text, sizeof(text), "PN:%u", (state->dbg_reg0b >> 3) & 0x0F);
    lv_canvas_draw_text(canvas, 4, 28, 76, &label_dsc, text);

    snprintf(text, sizeof(text), "07:%02X", state->dbg_reg07);
    lv_canvas_draw_text(canvas, 4, 48, 76, &label_dsc, text);

    snprintf(text, sizeof(text), "01:%02X", state->dbg_reg01);
    lv_canvas_draw_text(canvas, 84, 8, 76, &label_dsc, text);

    snprintf(text, sizeof(text), "0A:%02X", state->dbg_reg0a);
    lv_canvas_draw_text(canvas, 84, 28, 76, &label_dsc, text);

    snprintf(text, sizeof(text), "08:%02X", state->dbg_reg08);
    lv_canvas_draw_text(canvas, 84, 48, 76, &label_dsc, text);
    return;
#endif

    /* --- 正常布局 --- */
    const char *type_str = "UNK";
    if (state->vbus_stat == 1 || state->vbus_stat == 2) {
        type_str = "USB";
    } else if (state->vbus_stat == 3) {
        type_str = "CHG";
    }
    snprintf(text, sizeof(text), "Type:%s", type_str);
    lv_canvas_draw_text(canvas, 4, 8, 76, &label_dsc, text);

    char cur_str[16];
    format_current(cur_str, sizeof(cur_str), state->ichg_ma);
    snprintf(text, sizeof(text), "Set:%s", cur_str);
    lv_canvas_draw_text(canvas, 4, 28, 76, &label_dsc, text);

    char time_str[16];
    format_duration(time_str, sizeof(time_str), state->charge_duration_s);
    snprintf(text, sizeof(text), "Time:%s", time_str);
    lv_canvas_draw_text(canvas, 4, 48, 76, &label_dsc, text);

    snprintf(text, sizeof(text), "SOC:%u%%", state->battery_soc);
    lv_canvas_draw_text(canvas, 84, 8, 76, &label_dsc, text);

    const char *chrg_str = "IDLE";
    if (state->chrg_stat == 1) {
        chrg_str = "PRE";
    } else if (state->chrg_stat == 2) {
        chrg_str = "FST";
    } else if (state->chrg_stat == 3) {
        chrg_str = "FULL";
    }
    if (state->therm_stat) {
        snprintf(text, sizeof(text), "Chg:%s T", chrg_str);
    } else {
        snprintf(text, sizeof(text), "Chg:%s", chrg_str);
    }
    lv_canvas_draw_text(canvas, 84, 28, 76, &label_dsc, text);

    uint16_t mv = state->battery_voltage_mv;
    snprintf(text, sizeof(text), "VBAT:%u.%02uV", mv / 1000, (mv % 1000) / 10);
    lv_canvas_draw_text(canvas, 84, 48, 76, &label_dsc, text);
}
