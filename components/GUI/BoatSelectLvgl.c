#include "GraphScreen.h"
#include "GuiLvgl.h"
#include "EndPoints.h"
#include "lcd.h"
#include "lvgl.h"
#include "app_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

#define RESCAN_MS    2000
#define BOTTOM_BAR_H 50
#define BORDER_W     10   /* must match lv_obj_set_style_border_width */
#define CARD_INSET   12   /* gap between border inner edge and card, each side */
#define CARD_W       (LCD_H_RES - 2 * BORDER_W - 2 * CARD_INSET)

static lv_obj_t   *s_cont      = NULL;
static lv_obj_t   *s_list_cont = NULL;
static TickType_t  s_last_scan = 0;

static uint8_t    s_n_boats = 0;
static uint16_t   s_loggerIDs[MAX_BOAT_RADIO_ENDPOINTS];
static eEPStatus  s_statuses[MAX_BOAT_RADIO_ENDPOINTS];
static lv_obj_t  *s_cards[MAX_BOAT_RADIO_ENDPOINTS];
static int8_t     s_highlight_idx = -1;
static bool       s_cr_was_ready  = false;

static const char *status_str(eEPStatus st)
{
    switch (st) {
        case eEPStatus_initial:                     return "INIT";
        case eEPStatus_notSeen:                     return "NO SIGNAL";
        case eEPStatus_joined:                      return "JOINED";
        case eEPStatus_sentPing:                    return "PING...";
        case eEPStatus_responsive:                  return "CONNECTED";
        case eEPStatus_gettingSessionHeader:
        case eEPStatus_getting_n_descriptors:
        case eEPStatus_waiting_n_descriptors_reply:
        case eEPStatus_getting_descriptor:
        case eEPStatus_waiting_descriptor:
        case eEPStatus_got_all_descriptors:         return "EXCHANGE";
        case eEPStatus_readyForData:                return "READY";
        case eEPStatus_updating:                    return "UPDATING";
        default:                                    return "...";
    }
}

static const char *boat_class_str(uint16_t loggerID)
{
    tBoatRadioEP *ep = GetpEPFromLoggerID(loggerID);
    if (!ep) return "??";

    uint8_t n = 0;
    eBoatType bt = eBoatType_Unknown;
    for (int i = 0; i < N_ROWERS_MAX; i++) {
        eAffinities a = ep->descriptors_05_12.Affinities.affinity[i];
        if (a == eAffinities_Sculling) {
            n++;
            bt = eBoatType_Sculling;
        } else if (a == eAffinities_Starboard_sweep || a == eAffinities_Port_sweep) {
            n++;
            bt = eBoatType_Sweep;
        }
    }

    if (bt == eBoatType_Sculling) {
        if (n == 1) return "SINGLE";
        if (n == 2) return "DOUBLE";
        if (n == 4) return "QUAD";
        if (n == 8) return "OCTUPLE";
    } else if (bt == eBoatType_Sweep) {
        if (n == 2) return "PAIR";
        if (n == 4) return "FOUR";
        if (n == 8) return "EIGHT";
    }
    return "??";
}

static void update_highlight(void)
{
    for (uint8_t i = 0; i < s_n_boats; i++) {
        if (!s_cards[i]) continue;
        bool ready = (s_statuses[i] == eEPStatus_readyForData);
        bool hl    = (s_highlight_idx >= 0 && i == (uint8_t)s_highlight_idx);
        lv_obj_set_style_border_color(s_cards[i],
            hl    ? lv_color_hex(0xFFFF00) :
            ready ? lv_color_hex(0x00FF00) : lv_color_hex(0xFF0000), 0);
        lv_obj_set_style_border_width(s_cards[i], hl ? 4 : 2, 0);
    }
}

static bool rescan_boats(void)
{
    uint8_t   n = GetNumValidLoggers();
    uint8_t   new_n = 0;
    uint16_t  new_ids[MAX_BOAT_RADIO_ENDPOINTS];
    eEPStatus new_st[MAX_BOAT_RADIO_ENDPOINTS];

    for (uint8_t i = 0; i < n && new_n < MAX_BOAT_RADIO_ENDPOINTS; i++) {
        new_ids[new_n] = GetLoggerIDFromIndex(i);
        new_st[new_n]  = GetEPStatus(new_ids[new_n]);
        new_n++;
    }

    bool changed = (new_n != s_n_boats)
                || memcmp(new_ids, s_loggerIDs, new_n * sizeof(uint16_t))  != 0
                || memcmp(new_st,  s_statuses,  new_n * sizeof(eEPStatus)) != 0;

    s_n_boats = new_n;
    memcpy(s_loggerIDs, new_ids, new_n * sizeof(uint16_t));
    memcpy(s_statuses,  new_st,  new_n * sizeof(eEPStatus));
    return changed;
}

/* Button press: user tapped a boat card */
static void boat_btn_cb(lv_event_t *e)
{
    uint8_t idx = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    if (idx >= s_n_boats) return;
    if (s_statuses[idx] != eEPStatus_readyForData) return;
    tBoatRadioEP *pEP = GetpEPFromLoggerID(s_loggerIDs[idx]);
    if (!pEP) return;
    GuiLvgl_SetSelectedEP(pEP);
    GuiLvgl_NextScreen();
}

static void rebuild_list(void)
{
    if (!s_list_cont) return;
    lv_obj_clean(s_list_cont);
    memset(s_cards, 0, sizeof(s_cards));

    /* Must match s_list_cont's height set in bs_enter(). */
    int32_t content_h = LCD_V_RES - BOTTOM_BAR_H - 58;
    int32_t card_h    = (s_n_boats > 0) ? (content_h - 60) / (int32_t)s_n_boats : 0;
    if (card_h < 40) card_h = 40;

    if (s_n_boats == 0) {
        lv_obj_t *lbl = lv_label_create(s_list_cont);
        lv_label_set_text(lbl, "No boats configured");
        lv_obj_center(lbl);
        return;
    }

    for (uint8_t i = 0; i < s_n_boats; i++) {
        bool ready = (s_statuses[i] == eEPStatus_readyForData);

        lv_obj_t *btn = lv_button_create(s_list_cont);
        s_cards[i] = btn;
        lv_obj_set_size(btn, CARD_W, card_h - 4);
        lv_obj_set_style_bg_color(btn,
            ready ? lv_color_hex(0x005500) : lv_color_hex(0x330000), 0);
        lv_obj_set_style_border_color(btn,
            ready ? lv_color_hex(0x00FF00) : lv_color_hex(0xFF0000), 0);
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_add_event_cb(btn, boat_btn_cb, LV_EVENT_CLICKED,
                            (void *)(uintptr_t)i);

        /* "Boat N  ID: XXXXX" */
        char buf[48];
        snprintf(buf, sizeof(buf), "Boat %u   ID: %u", (unsigned)(i + 1u),
                 (unsigned)s_loggerIDs[i]);
        lv_obj_t *lbl1 = lv_label_create(btn);
        lv_label_set_text(lbl1, buf);
        lv_obj_align(lbl1, LV_ALIGN_LEFT_MID, 8, -10);
        lv_obj_set_style_text_color(lbl1, lv_color_white(), 0);

        /* Boat class */
        if (ready) {
            lv_obj_t *lbl2 = lv_label_create(btn);
            lv_label_set_text(lbl2, boat_class_str(s_loggerIDs[i]));
            lv_obj_align(lbl2, LV_ALIGN_LEFT_MID, 8, 10);
            lv_obj_set_style_text_color(lbl2, lv_color_hex(0x88FF88), 0);
        }

        /* Status */
        lv_obj_t *lbl3 = lv_label_create(btn);
        lv_label_set_text(lbl3, status_str(s_statuses[i]));
        lv_obj_align(lbl3, LV_ALIGN_RIGHT_MID, -12, 0);
        lv_obj_set_style_text_color(lbl3,
            ready ? lv_color_hex(0x00FF00) : lv_color_hex(0xAAAAAA), 0);
    }

    update_highlight();
}

static void bs_enter(tBoatRadioEP *pEP, eBoatType boatType)
{
    (void)pEP; (void)boatType;

    s_cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_cont, LCD_H_RES, LCD_V_RES - BOTTOM_BAR_H);
    lv_obj_set_pos(s_cont, 0, 0);
    lv_obj_set_style_bg_color(s_cont, lv_color_hex(0x0a0a1a), 0);
    lv_obj_set_style_pad_all(s_cont, 0, 0);
    lv_obj_set_style_border_width(s_cont, BORDER_W, 0);
    s_cr_was_ready = GetCRReady();
    lv_obj_set_style_border_color(s_cont,
        s_cr_was_ready ? lv_color_hex(0x006400) : lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_radius(s_cont, 0, 0);
    lv_obj_clear_flag(s_cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(s_cont);
    lv_label_set_text(title, "SELECT BOAT");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_pos(title, 12, 12);

    s_list_cont = lv_obj_create(s_cont);
    lv_obj_set_size(s_list_cont, CARD_W, LCD_V_RES - BOTTOM_BAR_H - 58);
    lv_obj_align(s_list_cont, LV_ALIGN_TOP_MID, 0, 46);
    lv_obj_set_style_bg_color(s_list_cont, lv_color_hex(0x0a0a1a), 0);
    lv_obj_set_style_border_width(s_list_cont, 0, 0);
    lv_obj_set_style_pad_all(s_list_cont, 0, 0);
    lv_obj_set_style_pad_row(s_list_cont, 4, 0);
    lv_obj_set_flex_flow(s_list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(s_list_cont, LV_OBJ_FLAG_SCROLLABLE);

    s_last_scan = xTaskGetTickCount();
    rescan_boats();
    rebuild_list();
}

static void bs_exit(void)
{
    if (s_cont) { lv_obj_del(s_cont); s_cont = NULL; }
    s_list_cont     = NULL;
    s_highlight_idx = -1;
    s_cr_was_ready  = false;
    memset(s_cards, 0, sizeof(s_cards));
}

static void bs_on_periodic(tBoatRadioEP *pEP)
{
    (void)pEP;

    bool cr_ready = GetCRReady();
    if (cr_ready != s_cr_was_ready) {
        s_cr_was_ready = cr_ready;
        LcdLvgl_Lock();
        lv_obj_set_style_border_color(s_cont,
            cr_ready ? lv_color_hex(0x006400) : lv_color_hex(0xFF0000), 0);
        LcdLvgl_Unlock();
    }

    TickType_t now = xTaskGetTickCount();
    if ((now - s_last_scan) < pdMS_TO_TICKS(RESCAN_MS)) return;
    s_last_scan = now;

    LcdLvgl_Lock();
    if (rescan_boats()) rebuild_list();
    LcdLvgl_Unlock();
}

static void bs_on_stroke(tBoatRadioEP *pEP) { (void)pEP; }
static void bs_on_touch(int x, int y)        { (void)x; (void)y; }
static void bs_render(void)                  { LcdLvgl_Lock(); rebuild_list(); LcdLvgl_Unlock(); }

static bool bs_on_button(bool next)
{
    if (s_n_boats == 0) return false;
    if (s_highlight_idx < 0) {
        s_highlight_idx = next ? 0 : (int8_t)(s_n_boats - 1);
    } else {
        s_highlight_idx = next
            ? (int8_t)((s_highlight_idx + 1) % s_n_boats)
            : (int8_t)((s_highlight_idx - 1 + (int8_t)s_n_boats) % s_n_boats);
    }
    update_highlight();
    return true;
}

static void bs_on_confirm(void)
{
    if (s_highlight_idx < 0 || (uint8_t)s_highlight_idx >= s_n_boats) return;
    if (s_statuses[s_highlight_idx] != eEPStatus_readyForData) return;
    tBoatRadioEP *pEP = GetpEPFromLoggerID(s_loggerIDs[s_highlight_idx]);
    if (!pEP) return;
    GuiLvgl_SetSelectedEP(pEP);
    GuiLvgl_NextScreen();
}

const tGraphScreenVtable BoatSelectScreen = {
    .title      = "Boat Select",
    .enter      = bs_enter,
    .exit       = bs_exit,
    .on_periodic = bs_on_periodic,
    .on_stroke  = bs_on_stroke,
    .on_touch   = bs_on_touch,
    .render     = bs_render,
    .on_button  = bs_on_button,
    .on_confirm = bs_on_confirm,
};
