#include "GraphScreen.h"
#include "EndPoints.h"
#include "PeachDefs.h"
#include "lcd.h"
#include "lvgl.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAX_POWER_W   500.0f
#define BOTTOM_BAR_H  50
#define CONTENT_H     (LCD_V_RES - BOTTOM_BAR_H)
#define TITLE_H       24
#define LANES_H       (CONTENT_H - TITLE_H)

#define SEAT_LBL_X    4
#define SEAT_LBL_W    34
#define WATT_LBL_X    (SEAT_LBL_X + SEAT_LBL_W)
#define WATT_LBL_W    90
#define BAR_X         (WATT_LBL_X + WATT_LBL_W)
#define BAR_MARGIN_R  8
#define LANE_PAD_V    3   /* gap between a lane's bar area and the lane edge */
#define HALF_W        (LCD_H_RES / 2)

/* Pixel width of the bar track within a row of the given total width
 * (LCD_H_RES for sweep's single full-width row, HALF_W for each of
 * sculling's side-by-side port/starboard rows). */
#define BAR_W_FOR(row_w)  ((row_w) - BAR_X - BAR_MARGIN_R)

#define COL_BAR_SWEEP 0x3399FFu   /* blue */
#define COL_BAR_PORT  0xFF3333u   /* red */
#define COL_BAR_STAR  0x33CC33u   /* green */
#define COL_BAR_BG    0x222222u

/* side index: 0 = sweep (or scull port), 1 = scull star (unused for sweep) */
#define SIDE_SWEEP 0
#define SIDE_PORT  0
#define SIDE_STAR  1

/* ---- State ---- */

static eBoatType     s_bt;
static tBoatRadioEP  *s_pep;
static uint8_t        s_n;                 /* number of active rower lanes */
static uint8_t        s_active_seat[N_ROWERS_MAX];

static tExtractionGridElement *s_elemSweep[N_ROWERS_MAX];
static tExtractionGridElement *s_elemPort[N_ROWERS_MAX];
static tExtractionGridElement *s_elemStar[N_ROWERS_MAX];

static lv_obj_t *s_cont                       = NULL;
static lv_obj_t *s_lbl_seat[N_ROWERS_MAX]     = {NULL};
static lv_obj_t *s_lbl_watt[2][N_ROWERS_MAX]  = {{NULL}};
static lv_obj_t *s_bar_bg[2][N_ROWERS_MAX]    = {{NULL}};
static lv_obj_t *s_bar_fill[2][N_ROWERS_MAX]  = {{NULL}};
static int32_t   s_bar_w[2][N_ROWERS_MAX]     = {{0}};

static void collect_elems(tBoatRadioEP *pEP)
{
    s_n = 0;
    memset(s_elemSweep, 0, sizeof(s_elemSweep));
    memset(s_elemPort,  0, sizeof(s_elemPort));
    memset(s_elemStar,  0, sizeof(s_elemStar));

    if (!pEP) return;

    tExtractionGrid *g = &pEP->ExtractionGridAPeriodic;
    GraphScreen_ClearPlotting(g);

    bool scull = (s_bt == eBoatType_Sculling);

    for (uint8_t i = 0; i < g->nGridElemDefined; i++) {
        uint8_t sid  = g->extractionGrid[i].semanticId;
        uint8_t seat = g->extractionGrid[i].seat;
        if (seat < 1 || seat > N_ROWERS_MAX) continue;
        uint8_t r = seat - 1;

        if (!scull && sid == eSIDAPer_SweepPower) {
            s_elemSweep[r] = &g->extractionGrid[i];
            g->extractionGrid[i].plotting = true;
        } else if (scull && sid == eSIDAPer_ScullPowerPort) {
            s_elemPort[r] = &g->extractionGrid[i];
            g->extractionGrid[i].plotting = true;
        } else if (scull && sid == eSIDAPer_ScullPowerStar) {
            s_elemStar[r] = &g->extractionGrid[i];
            g->extractionGrid[i].plotting = true;
        }
    }

    /* Compact to an ordered (by seat) list of lanes that actually have data */
    for (uint8_t r = 0; r < N_ROWERS_MAX; r++) {
        if (s_elemSweep[r] || s_elemPort[r] || s_elemStar[r])
            s_active_seat[s_n++] = r + 1;
    }
}

static inline int32_t pw_to_px(float pw, int32_t bar_w)
{
    if (isnan(pw) || pw < 0.0f) pw = 0.0f;
    if (pw > MAX_POWER_W) pw = MAX_POWER_W;
    int32_t w = (int32_t)(pw / MAX_POWER_W * (float)bar_w);
    return w > 1 ? w : 1;
}

static void update_bar_side(uint8_t k, uint8_t side, float pw)
{
    if (!s_bar_bg[side][k]) return;
    if (isnan(pw)) pw = 0.0f;

    lv_obj_set_width(s_bar_fill[side][k], pw_to_px(pw, s_bar_w[side][k]));
    lv_obj_align(s_bar_fill[side][k], LV_ALIGN_LEFT_MID, 0, 0);

    char buf[16];
    snprintf(buf, sizeof(buf), "%d W", (int)pw);
    lv_label_set_text(s_lbl_watt[side][k], buf);
}

static void update_bars(void)
{
    bool scull = (s_bt == eBoatType_Sculling);
    for (uint8_t k = 0; k < s_n; k++) {
        uint8_t r = s_active_seat[k] - 1;
        if (!scull) {
            update_bar_side(k, SIDE_SWEEP, s_elemSweep[r] ? s_elemSweep[r]->value : NAN);
        } else {
            update_bar_side(k, SIDE_PORT, s_elemPort[r] ? s_elemPort[r]->value : NAN);
            update_bar_side(k, SIDE_STAR, s_elemStar[r] ? s_elemStar[r]->value : NAN);
        }
    }
}

static void create_bar_row(lv_obj_t *parent, int32_t x_base, int32_t row_w, int32_t h,
                            uint8_t k, uint8_t side, uint32_t color)
{
    int32_t bar_w = BAR_W_FOR(row_w);
    s_bar_w[side][k] = bar_w;

    lv_obj_t *rc = lv_obj_create(parent);
    lv_obj_set_size(rc, row_w, h);
    lv_obj_set_pos(rc, x_base, 0);
    lv_obj_set_style_bg_opa(rc, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(rc, 0, 0);
    lv_obj_set_style_border_width(rc, 0, 0);
    lv_obj_set_style_radius(rc, 0, 0);
    lv_obj_clear_flag(rc, LV_OBJ_FLAG_SCROLLABLE);

    s_lbl_watt[side][k] = lv_label_create(rc);
    lv_label_set_text(s_lbl_watt[side][k], "0 W");
    lv_obj_set_style_text_font(s_lbl_watt[side][k], &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_lbl_watt[side][k], lv_color_white(), 0);
    lv_obj_align(s_lbl_watt[side][k], LV_ALIGN_LEFT_MID, WATT_LBL_X, 0);

    int32_t bg_h = h - 2 * LANE_PAD_V;
    if (bg_h < 4) bg_h = 4;
    s_bar_bg[side][k] = lv_obj_create(rc);
    lv_obj_set_size(s_bar_bg[side][k], bar_w, bg_h);
    lv_obj_align(s_bar_bg[side][k], LV_ALIGN_LEFT_MID, BAR_X, 0);
    lv_obj_set_style_bg_color(s_bar_bg[side][k], lv_color_hex(COL_BAR_BG), 0);
    lv_obj_set_style_pad_all(s_bar_bg[side][k], 0, 0);
    lv_obj_set_style_border_width(s_bar_bg[side][k], 1, 0);
    lv_obj_set_style_border_color(s_bar_bg[side][k], lv_color_hex(0x444444), 0);
    lv_obj_set_style_radius(s_bar_bg[side][k], 0, 0);
    lv_obj_clear_flag(s_bar_bg[side][k], LV_OBJ_FLAG_SCROLLABLE);

    s_bar_fill[side][k] = lv_obj_create(s_bar_bg[side][k]);
    lv_obj_set_size(s_bar_fill[side][k], 1, bg_h - 2);
    lv_obj_set_style_bg_color(s_bar_fill[side][k], lv_color_hex(color), 0);
    lv_obj_set_style_pad_all(s_bar_fill[side][k], 0, 0);
    lv_obj_set_style_border_width(s_bar_fill[side][k], 0, 0);
    lv_obj_set_style_radius(s_bar_fill[side][k], 0, 0);
    lv_obj_clear_flag(s_bar_fill[side][k], LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(s_bar_fill[side][k], LV_ALIGN_LEFT_MID, 0, 0);
}

static void pw_enter(tBoatRadioEP *pEP, eBoatType bt)
{
    s_bt  = bt;
    s_pep = pEP;
    collect_elems(pEP);

    s_cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_cont, LCD_H_RES, CONTENT_H);
    lv_obj_set_pos(s_cont, 0, 0);
    lv_obj_set_style_bg_color(s_cont, lv_color_hex(0x0a0a1a), 0);
    lv_obj_set_style_pad_all(s_cont, 0, 0);
    lv_obj_set_style_border_width(s_cont, 0, 0);
    lv_obj_set_style_radius(s_cont, 0, 0);
    lv_obj_clear_flag(s_cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(s_cont);
    lv_label_set_text(title, "ROWER POWER");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_pos(title, 10, 2);

    bool scull     = (s_bt == eBoatType_Sculling);
    int32_t lane_h = LANES_H / N_ROWERS_MAX;   /* fixed slot height, independent of crew size */

    for (uint8_t k = 0; k < s_n; k++) {
        int32_t ly = TITLE_H + (int32_t)k * lane_h;

        lv_obj_t *lc = lv_obj_create(s_cont);
        lv_obj_set_size(lc, LCD_H_RES, lane_h);
        lv_obj_set_pos(lc, 0, ly);
        lv_obj_set_style_bg_opa(lc, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(lc, 0, 0);
        lv_obj_set_style_border_width(lc, 0, 0);
        lv_obj_set_style_radius(lc, 0, 0);
        lv_obj_clear_flag(lc, LV_OBJ_FLAG_SCROLLABLE);

        s_lbl_seat[k] = lv_label_create(lc);
        char sbuf[6];
        snprintf(sbuf, sizeof(sbuf), "%u", s_active_seat[k]);
        lv_label_set_text(s_lbl_seat[k], sbuf);
        lv_obj_set_style_text_font(s_lbl_seat[k], &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(s_lbl_seat[k], lv_color_hex(0xCCCCCC), 0);
        lv_obj_align(s_lbl_seat[k], LV_ALIGN_LEFT_MID, SEAT_LBL_X, 0);

        if (!scull) {
            create_bar_row(lc, 0, LCD_H_RES, lane_h, k, SIDE_SWEEP, COL_BAR_SWEEP);
        } else {
            /* Port and starboard share the same horizontal plane, each
             * occupying half the screen width, side by side. */
            create_bar_row(lc, 0,      HALF_W, lane_h, k, SIDE_PORT, COL_BAR_PORT);
            create_bar_row(lc, HALF_W, HALF_W, lane_h, k, SIDE_STAR, COL_BAR_STAR);
        }
    }

    update_bars();
}

static void pw_exit(void)
{
    if (s_pep) {
        GraphScreen_ClearPlotting(&s_pep->ExtractionGridAPeriodic);
        s_pep = NULL;
    }
    if (s_cont) { lv_obj_del(s_cont); s_cont = NULL; }
    memset(s_lbl_seat, 0, sizeof(s_lbl_seat));
    memset(s_lbl_watt, 0, sizeof(s_lbl_watt));
    memset(s_bar_bg,   0, sizeof(s_bar_bg));
    memset(s_bar_fill, 0, sizeof(s_bar_fill));
    memset(s_bar_w,    0, sizeof(s_bar_w));
}

static void pw_on_periodic(tBoatRadioEP *pEP) { (void)pEP; }

static void pw_on_stroke(tBoatRadioEP *pEP)
{
    if (!s_cont || !pEP) return;

    LcdLvgl_Lock();
    update_bars();
    LcdLvgl_Unlock();
}

static void pw_on_touch(int x, int y) { (void)x; (void)y; }
static void pw_render(void)           {}

const tGraphScreenVtable PowerScreen = {
    .title       = "Power",
    .enter       = pw_enter,
    .exit        = pw_exit,
    .on_periodic = pw_on_periodic,
    .on_stroke   = pw_on_stroke,
    .on_touch    = pw_on_touch,
    .render      = pw_render,
};
