/*
 * FleetViewLvgl.c — Fleet comparison screen.
 *
 * Shows one panel per registered boat, arranged like the Boat Select screen.
 * Each panel displays t500 pace and stroke rating with horizontal bar graphs
 * so all boats can be compared at a glance.
 *
 * Data source: eSIDAPer_AvgBoatSpeed (→ t500) and eSIDAPer_Rating,
 * both boat-level (seat 0), from the aperiodic (per-stroke) extraction grid.
 *
 * Unlike other screens this one routes stroke callbacks for ALL endpoints,
 * not just the selected one (handled in GuiLvgl.c).
 */

#include "GraphScreen.h"
#include "GuiLvgl.h"
#include "EndPoints.h"
#include "lcd.h"
#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define BOTTOM_BAR_H  50
#define BORDER_W      10
#define CARD_INSET    12
#define CARD_W        (LCD_H_RES - 2 * BORDER_W - 2 * CARD_INSET)
#define TITLE_H       38
#define CONTENT_H     (LCD_V_RES - BOTTOM_BAR_H - 2 * BORDER_W - TITLE_H)

/* Bar ranges — full bar = these values */
#define SPEED_BAR_MAX_MPS   8.0f    /* 8 m/s ≈ 1:02 /500m */
#define RATE_BAR_MAX_SPM   50.0f

#define PANEL_MIN_H    50
#define PANEL_MAX_H   140

/* Label column: fixed width so pace & rate bars are left-aligned to the same x */
#define LBL_COL_W      72   /* px reserved for the text label */
#define BAR_X          LBL_COL_W
#define BAR_W          (CARD_W - LBL_COL_W - 8)
#define BAR_H_NORMAL   14
#define BAR_H_COMPACT  10

static lv_obj_t   *s_cont         = NULL;
static lv_obj_t   *s_list_cont    = NULL;
static lv_timer_t *s_rescan_timer = NULL;

static uint8_t   s_n_boats = 0;
static uint16_t  s_loggerIDs[MAX_BOAT_RADIO_ENDPOINTS];
static lv_obj_t *s_speed_lbl[MAX_BOAT_RADIO_ENDPOINTS];
static lv_obj_t *s_speed_bar[MAX_BOAT_RADIO_ENDPOINTS];
static lv_obj_t *s_rate_lbl [MAX_BOAT_RADIO_ENDPOINTS];
static lv_obj_t *s_rate_bar [MAX_BOAT_RADIO_ENDPOINTS];

/* ---- Helpers ---- */

static const char *fv_boat_class(uint16_t id)
{
    tBoatRadioEP *ep = GetpEPFromLoggerID(id);
    if (!ep) return "";
    uint8_t n = 0;
    eBoatType bt = eBoatType_Unknown;
    for (int i = 0; i < N_ROWERS_MAX; i++) {
        eAffinities a = ep->descriptors_05_12.Affinities.affinity[i];
        if (a == eAffinities_Sculling)
            { n++; bt = eBoatType_Sculling; }
        else if (a == eAffinities_Starboard_sweep || a == eAffinities_Port_sweep)
            { n++; bt = eBoatType_Sweep; }
    }
    if (bt == eBoatType_Sculling) {
        if (n == 1) return "1x";
        if (n == 2) return "2x";
        if (n == 4) return "4x";
        if (n == 8) return "8x";
    } else if (bt == eBoatType_Sweep) {
        if (n == 2) return "2-";
        if (n == 4) return "4-";
        if (n == 8) return "8+";
    }
    return "";
}

/* Mark/unmark Rating + AvgBoatSpeed plotting=true for every registered EP */
static void set_plotting_all(bool en)
{
    uint8_t n = GetNumValidLoggers();
    for (uint8_t i = 0; i < n; i++) {
        tBoatRadioEP *ep = GetpEPFromLoggerID(GetLoggerIDFromIndex(i));
        if (!ep) continue;
        tExtractionGrid *ga = &ep->ExtractionGridAPeriodic;
        for (uint8_t j = 0; j < ga->nGridElemDefined; j++) {
            if (ga->extractionGrid[j].seat != 0) continue;
            uint8_t sid = ga->extractionGrid[j].semanticId;
            if (sid == eSIDAPer_Rating || sid == eSIDAPer_AvgBoatSpeed)
                ga->extractionGrid[j].plotting = en;
        }
    }
}

/* Build (or rebuild) the panel list. Must be called with LVGL lock held. */
static void build_panels(void)
{
    if (!s_list_cont) return;
    lv_obj_clean(s_list_cont);
    memset(s_speed_lbl, 0, sizeof(s_speed_lbl));
    memset(s_speed_bar, 0, sizeof(s_speed_bar));
    memset(s_rate_lbl,  0, sizeof(s_rate_lbl));
    memset(s_rate_bar,  0, sizeof(s_rate_bar));

    uint8_t n = GetNumValidLoggers();
    if (n > MAX_BOAT_RADIO_ENDPOINTS) n = MAX_BOAT_RADIO_ENDPOINTS;
    for (uint8_t i = 0; i < n; i++)
        s_loggerIDs[i] = GetLoggerIDFromIndex(i);
    s_n_boats = n;

    if (n == 0) {
        lv_obj_t *lbl = lv_label_create(s_list_cont);
        lv_label_set_text(lbl, "No boats registered");
        lv_obj_center(lbl);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0x666666), 0);
        return;
    }

    int32_t panel_h = CONTENT_H / (int32_t)n;
    if (panel_h < PANEL_MIN_H) panel_h = PANEL_MIN_H;
    if (panel_h > PANEL_MAX_H) panel_h = PANEL_MAX_H;
    int32_t bar_h   = (panel_h >= 70) ? BAR_H_NORMAL : BAR_H_COMPACT;

    /* Vertical layout inside each panel:
     *   name_y: top of name label
     *   row1_y: centre of speed row
     *   row2_y: centre of rate row
     * We split the space below the name into two equal rows. */
    int32_t name_y   = 4;
    int32_t name_h   = 18;   /* estimated px height of montserrat_16 */
    int32_t rows_top = name_y + name_h + 2;
    int32_t rows_h   = (panel_h - 4) - rows_top;
    int32_t row_h    = rows_h / 2;
    int32_t speed_y  = rows_top + (row_h - bar_h) / 2;
    int32_t rate_y   = rows_top + row_h + (row_h - bar_h) / 2;
    int32_t lbl_y_off = (bar_h - 16) / 2;   /* centre 16px font in bar row */

    for (uint8_t i = 0; i < n; i++) {
        uint16_t lid  = s_loggerIDs[i];
        const char *cls = fv_boat_class(lid);

        /* Panel card */
        lv_obj_t *card = lv_obj_create(s_list_cont);
        lv_obj_set_size(card, CARD_W, panel_h - 4);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x08121e), 0);
        lv_obj_set_style_border_color(card, lv_color_hex(0x0a2040), 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_radius(card, 4, 0);
        lv_obj_set_style_pad_all(card, 0, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_CLICKABLE);

        /* Boat name / class */
        char nbuf[48];
        snprintf(nbuf, sizeof(nbuf), "Boat %u  ID:%u  %s",
                 (unsigned)(i + 1u), (unsigned)lid, cls);
        lv_obj_t *nlbl = lv_label_create(card);
        lv_label_set_text(nlbl, nbuf);
        lv_obj_set_pos(nlbl, 8, name_y);
        lv_obj_set_style_text_color(nlbl, lv_color_hex(0xBBBBBB), 0);

        /* Speed (t500) label */
        s_speed_lbl[i] = lv_label_create(card);
        lv_label_set_text(s_speed_lbl[i], "-:--");
        lv_obj_set_pos(s_speed_lbl[i], 8, speed_y + lbl_y_off);
        lv_obj_set_style_text_color(s_speed_lbl[i], lv_color_hex(0x00CCFF), 0);

        /* Speed bar */
        s_speed_bar[i] = lv_bar_create(card);
        lv_obj_set_size(s_speed_bar[i], BAR_W, bar_h);
        lv_obj_set_pos(s_speed_bar[i], BAR_X, speed_y);
        lv_bar_set_range(s_speed_bar[i], 0, 1000);
        lv_bar_set_value(s_speed_bar[i], 0, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(s_speed_bar[i], lv_color_hex(0x001a33),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_speed_bar[i], lv_color_hex(0x0088CC),
                                  LV_PART_INDICATOR);
        lv_obj_set_style_radius(s_speed_bar[i], 3, LV_PART_MAIN);
        lv_obj_set_style_radius(s_speed_bar[i], 3, LV_PART_INDICATOR);
        lv_obj_set_style_border_width(s_speed_bar[i], 0, LV_PART_MAIN);

        /* Rate label */
        s_rate_lbl[i] = lv_label_create(card);
        lv_label_set_text(s_rate_lbl[i], "--");
        lv_obj_set_pos(s_rate_lbl[i], 8, rate_y + lbl_y_off);
        lv_obj_set_style_text_color(s_rate_lbl[i], lv_color_hex(0x00FF88), 0);

        /* Rate bar */
        s_rate_bar[i] = lv_bar_create(card);
        lv_obj_set_size(s_rate_bar[i], BAR_W, bar_h);
        lv_obj_set_pos(s_rate_bar[i], BAR_X, rate_y);
        lv_bar_set_range(s_rate_bar[i], 0, 1000);
        lv_bar_set_value(s_rate_bar[i], 0, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(s_rate_bar[i], lv_color_hex(0x001a0d),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_color(s_rate_bar[i], lv_color_hex(0x00CC66),
                                  LV_PART_INDICATOR);
        lv_obj_set_style_radius(s_rate_bar[i], 3, LV_PART_MAIN);
        lv_obj_set_style_radius(s_rate_bar[i], 3, LV_PART_INDICATOR);
        lv_obj_set_style_border_width(s_rate_bar[i], 0, LV_PART_MAIN);
    }
}

/* Update one panel's labels and bars from EP's extraction grid.
 * Called from a background task — must acquire LVGL lock. */
static void update_panel(uint8_t idx, tBoatRadioEP *pEP)
{
    float speed = NAN, rating = NAN;
    tExtractionGrid *ga = &pEP->ExtractionGridAPeriodic;
    for (uint8_t j = 0; j < ga->nGridElemDefined; j++) {
        if (ga->extractionGrid[j].seat != 0) continue;
        uint8_t sid = ga->extractionGrid[j].semanticId;
        if (sid == eSIDAPer_AvgBoatSpeed) speed  = ga->extractionGrid[j].value;
        else if (sid == eSIDAPer_Rating)  rating = ga->extractionGrid[j].value;
    }

    LcdLvgl_Lock();

    if (s_speed_lbl[idx]) {
        char buf[16];
        if (!isnan(speed) && speed > 0.01f) {
            int t500 = (int)(500.0f / speed + 0.5f);
            snprintf(buf, sizeof(buf), "%d:%02d", t500 / 60, t500 % 60);
        } else {
            snprintf(buf, sizeof(buf), "-:--");
        }
        lv_label_set_text(s_speed_lbl[idx], buf);
    }
    if (s_speed_bar[idx]) {
        int32_t v = 0;
        if (!isnan(speed) && speed > 0.0f) {
            v = (int32_t)(speed / SPEED_BAR_MAX_MPS * 1000.0f);
            if (v > 1000) v = 1000;
        }
        lv_bar_set_value(s_speed_bar[idx], v, LV_ANIM_OFF);
    }

    if (s_rate_lbl[idx]) {
        char buf[16];
        if (!isnan(rating) && rating > 0.0f)
            snprintf(buf, sizeof(buf), "%.0f", (double)rating);
        else
            snprintf(buf, sizeof(buf), "--");
        lv_label_set_text(s_rate_lbl[idx], buf);
    }
    if (s_rate_bar[idx]) {
        int32_t v = 0;
        if (!isnan(rating) && rating > 0.0f) {
            v = (int32_t)(rating / RATE_BAR_MAX_SPM * 1000.0f);
            if (v > 1000) v = 1000;
        }
        lv_bar_set_value(s_rate_bar[idx], v, LV_ANIM_OFF);
    }

    LcdLvgl_Unlock();
}

/* Periodic rescan — runs as an lv_timer (LVGL context, lock already held) */
static void rescan_cb(lv_timer_t *t)
{
    (void)t;
    uint8_t n = GetNumValidLoggers();
    bool changed = (n != s_n_boats);
    for (uint8_t i = 0; i < n && !changed; i++)
        if (GetLoggerIDFromIndex(i) != s_loggerIDs[i]) changed = true;
    if (changed) {
        build_panels();
        set_plotting_all(true);
    }
}

/* ---- Vtable callbacks ---- */

static void fv_enter(tBoatRadioEP *pEP, eBoatType bt)
{
    (void)pEP; (void)bt;

    s_cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_cont, LCD_H_RES, LCD_V_RES - BOTTOM_BAR_H);
    lv_obj_set_pos(s_cont, 0, 0);
    lv_obj_set_style_bg_color(s_cont, lv_color_hex(0x050d18), 0);
    lv_obj_set_style_border_width(s_cont, BORDER_W, 0);
    lv_obj_set_style_border_color(s_cont, lv_color_hex(0x003366), 0);
    lv_obj_set_style_pad_all(s_cont, 0, 0);
    lv_obj_set_style_radius(s_cont, 0, 0);
    lv_obj_clear_flag(s_cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(s_cont);
    lv_label_set_text(title, "FLEET VIEW");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_pos(title, 12, 6);

    s_list_cont = lv_obj_create(s_cont);
    lv_obj_set_size(s_list_cont, CARD_W, CONTENT_H);
    lv_obj_align(s_list_cont, LV_ALIGN_TOP_MID, 0, TITLE_H);
    lv_obj_set_style_bg_color(s_list_cont, lv_color_hex(0x050d18), 0);
    lv_obj_set_style_border_width(s_list_cont, 0, 0);
    lv_obj_set_style_pad_all(s_list_cont, 0, 0);
    lv_obj_set_style_pad_row(s_list_cont, 4, 0);
    lv_obj_set_flex_flow(s_list_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_clear_flag(s_list_cont, LV_OBJ_FLAG_SCROLLABLE);

    build_panels();
    set_plotting_all(true);

    s_rescan_timer = lv_timer_create(rescan_cb, 2000, NULL);
}

static void fv_exit(void)
{
    if (s_rescan_timer) { lv_timer_delete(s_rescan_timer); s_rescan_timer = NULL; }
    set_plotting_all(false);
    if (s_cont) { lv_obj_del(s_cont); s_cont = NULL; }
    s_list_cont = NULL;
    memset(s_speed_lbl, 0, sizeof(s_speed_lbl));
    memset(s_speed_bar, 0, sizeof(s_speed_bar));
    memset(s_rate_lbl,  0, sizeof(s_rate_lbl));
    memset(s_rate_bar,  0, sizeof(s_rate_bar));
    s_n_boats = 0;
}

/* Called for ALL endpoints when on this screen (see GuiLvgl.c). */
static void fv_on_stroke(tBoatRadioEP *pEP)
{
    if (!pEP) return;
    for (uint8_t i = 0; i < s_n_boats; i++) {
        if (s_loggerIDs[i] == pEP->loggerID) {
            update_panel(i, pEP);
            return;
        }
    }
}

static void fv_on_periodic(tBoatRadioEP *pEP) { (void)pEP; }
static void fv_on_touch(int x, int y)          { (void)x; (void)y; }
static void fv_render(void)                    {}

const tGraphScreenVtable FleetViewScreen = {
    .title       = "Fleet View",
    .enter       = fv_enter,
    .exit        = fv_exit,
    .on_periodic = fv_on_periodic,
    .on_stroke   = fv_on_stroke,
    .on_touch    = fv_on_touch,
    .render      = fv_render,
};
