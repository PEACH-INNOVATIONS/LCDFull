#include "GraphScreen.h"
#include "EndPoints.h"
#include "PeachDefs.h"
#include "lcd.h"
#include "lvgl.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAX_ARC_DEG   120.0f   /* full-scale span (max-min) the bar track represents */
#define BOTTOM_BAR_H  50
#define CONTENT_H     (LCD_V_RES - BOTTOM_BAR_H)
#define TITLE_H       24
#define LANES_H       (CONTENT_H - TITLE_H)

#define SEAT_LBL_X    4
#define SEAT_LBL_W    34
#define MIN_LBL_X     (SEAT_LBL_X + SEAT_LBL_W)
#define MIN_LBL_W     40
#define BAR_X         (MIN_LBL_X + MIN_LBL_W)
#define MAX_LBL_W     40
#define BAR_MARGIN_R  8
#define ROW_PAD_V     3
#define HALF_W        (LCD_H_RES / 2)

/* Pixel width of the bar track within a row of the given total width
 * (LCD_H_RES for sweep's single full-width row, HALF_W for each of
 * sculling's side-by-side port/starboard rows). */
#define BAR_W_FOR(row_w)  ((row_w) - BAR_X - MAX_LBL_W - BAR_MARGIN_R)

#define COL_SWEEP     0x3399FFu   /* blue */
#define COL_PORT      0xFF3333u   /* red */
#define COL_STAR      0x33CC33u   /* green */
#define COL_BAR_BG    0x222222u

/* side index: 0 = sweep (or scull port), 1 = scull star (unused for sweep) */
#define SIDE_SWEEP 0
#define SIDE_PORT  0
#define SIDE_STAR  1

typedef struct {
    tExtractionGridElement *min;
    tExtractionGridElement *max;
    tExtractionGridElement *catchSlip;
    tExtractionGridElement *finishSlip;
    bool present;
} tArcSet;

static eBoatType    s_bt;
static tBoatRadioEP *s_pep;
static uint8_t       s_n;                       /* number of active rower lanes */
static uint8_t       s_active_seat[N_ROWERS_MAX];

static tArcSet s_sweep[N_ROWERS_MAX];
static tArcSet s_port[N_ROWERS_MAX];
static tArcSet s_star[N_ROWERS_MAX];

static lv_obj_t *s_cont                       = NULL;
static lv_obj_t *s_lbl_seat[N_ROWERS_MAX]     = {NULL};
static lv_obj_t *s_lbl_min[2][N_ROWERS_MAX]   = {{NULL}};
static lv_obj_t *s_lbl_max[2][N_ROWERS_MAX]   = {{NULL}};
static lv_obj_t *s_bar_bg[2][N_ROWERS_MAX]    = {{NULL}};
static lv_obj_t *s_bar_main[2][N_ROWERS_MAX]  = {{NULL}};
static lv_obj_t *s_bar_catch[2][N_ROWERS_MAX] = {{NULL}};
static lv_obj_t *s_bar_finish[2][N_ROWERS_MAX]= {{NULL}};
static int32_t   s_bar_w[2][N_ROWERS_MAX]     = {{0}};

static inline uint32_t darken(uint32_t rgb, float factor)
{
    uint8_t r = (uint8_t)(((rgb >> 16) & 0xFF) * factor);
    uint8_t g = (uint8_t)(((rgb >>  8) & 0xFF) * factor);
    uint8_t b = (uint8_t)(((rgb      ) & 0xFF) * factor);
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

static void collect_elems(tBoatRadioEP *pEP)
{
    s_n = 0;
    memset(s_sweep, 0, sizeof(s_sweep));
    memset(s_port,  0, sizeof(s_port));
    memset(s_star,  0, sizeof(s_star));
    if (!pEP) return;

    tExtractionGrid *g = &pEP->ExtractionGridAPeriodic;
    GraphScreen_ClearPlotting(g);
    bool scull = (s_bt == eBoatType_Sculling);

    for (uint8_t i = 0; i < g->nGridElemDefined; i++) {
        uint8_t sid  = g->extractionGrid[i].semanticId;
        uint8_t seat = g->extractionGrid[i].seat;
        if (seat < 1 || seat > N_ROWERS_MAX) continue;
        uint8_t r = seat - 1;
        tExtractionGridElement *e = &g->extractionGrid[i];

        if (!scull) {
            switch (sid) {
                case eSIDAPer_MinSweepAngle:   s_sweep[r].min        = e; break;
                case eSIDAPer_MaxSweepAngle:   s_sweep[r].max        = e; break;
                case eSIDAPer_CatchSlipSweep:  s_sweep[r].catchSlip  = e; break;
                case eSIDAPer_FinishSlipSweep: s_sweep[r].finishSlip = e; break;
                default: continue;
            }
            e->plotting = true;
            s_sweep[r].present = true;
        } else {
            switch (sid) {
                case eSIDAPer_MinScullAnglePort:   s_port[r].min        = e; break;
                case eSIDAPer_MaxScullAnglePort:   s_port[r].max        = e; break;
                case eSIDAPer_CatchSlipScullPort:  s_port[r].catchSlip  = e; break;
                case eSIDAPer_FinishSlipScullPort: s_port[r].finishSlip = e; break;
                case eSIDAPer_MinScullAngleStar:   s_star[r].min        = e; break;
                case eSIDAPer_MaxScullAngleStar:   s_star[r].max        = e; break;
                case eSIDAPer_CatchSlipScullStar:  s_star[r].catchSlip  = e; break;
                case eSIDAPer_FinishSlipScullStar: s_star[r].finishSlip = e; break;
                default: continue;
            }
            e->plotting = true;
            switch (sid) {
                case eSIDAPer_MinScullAnglePort:
                case eSIDAPer_MaxScullAnglePort:
                case eSIDAPer_CatchSlipScullPort:
                case eSIDAPer_FinishSlipScullPort:
                    s_port[r].present = true;
                    break;
                default:
                    s_star[r].present = true;
                    break;
            }
        }
    }

    for (uint8_t r = 0; r < N_ROWERS_MAX; r++) {
        bool has = scull ? (s_port[r].present || s_star[r].present) : s_sweep[r].present;
        if (has) s_active_seat[s_n++] = r + 1;
    }
}

static inline int32_t deg_to_px(float deg, int32_t bar_w)
{
    if (isnan(deg) || deg < 0.0f) deg = 0.0f;
    if (deg > MAX_ARC_DEG) deg = MAX_ARC_DEG;
    return (int32_t)(deg / MAX_ARC_DEG * (float)bar_w);
}

static void update_bar_row(uint8_t k, uint8_t side, tArcSet *set)
{
    if (!s_bar_bg[side][k]) return;
    int32_t bar_w = s_bar_w[side][k];

    float mn = set->min ? set->min->value : NAN;
    float mx = set->max ? set->max->value : NAN;
    float cs = (set->catchSlip  && !isnan(set->catchSlip->value))  ? set->catchSlip->value  : 0.0f;
    float fs = (set->finishSlip && !isnan(set->finishSlip->value)) ? set->finishSlip->value : 0.0f;

    float span = (!isnan(mn) && !isnan(mx)) ? (mx - mn) : 0.0f;
    if (span < 0.0f) span = 0.0f;

    int32_t total_px  = deg_to_px(span, bar_w);
    int32_t catch_px  = deg_to_px(cs, bar_w);
    int32_t finish_px = deg_to_px(fs, bar_w);
    if (catch_px > total_px) catch_px = total_px;
    if (finish_px > total_px - catch_px) finish_px = total_px - catch_px;

    int32_t main_w  = total_px  > 1 ? total_px  : 1;
    int32_t catch_w = catch_px  > 0 ? catch_px  : 1;
    int32_t fin_w   = finish_px > 0 ? finish_px : 1;

    lv_obj_set_width(s_bar_main[side][k], main_w);
    lv_obj_align(s_bar_main[side][k], LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_set_width(s_bar_catch[side][k], catch_px > 0 ? catch_w : 0);
    lv_obj_align(s_bar_catch[side][k], LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_set_width(s_bar_finish[side][k], finish_px > 0 ? fin_w : 0);
    lv_obj_align(s_bar_finish[side][k], LV_ALIGN_LEFT_MID, total_px - finish_px, 0);

    char buf[8];
    if (!isnan(mn)) snprintf(buf, sizeof(buf), "%.0f", (double)mn); else snprintf(buf, sizeof(buf), "--");
    lv_label_set_text(s_lbl_min[side][k], buf);
    if (!isnan(mx)) snprintf(buf, sizeof(buf), "%.0f", (double)mx); else snprintf(buf, sizeof(buf), "--");
    lv_label_set_text(s_lbl_max[side][k], buf);
}

static void update_bars(void)
{
    bool scull = (s_bt == eBoatType_Sculling);
    for (uint8_t k = 0; k < s_n; k++) {
        uint8_t r = s_active_seat[k] - 1;
        if (!scull)
            update_bar_row(k, SIDE_SWEEP, &s_sweep[r]);
        else {
            update_bar_row(k, SIDE_PORT, &s_port[r]);
            update_bar_row(k, SIDE_STAR, &s_star[r]);
        }
    }
}

static void create_bar_row(lv_obj_t *parent, int32_t x_base, int32_t y, int32_t row_w, int32_t h,
                            uint8_t k, uint8_t side, uint32_t color)
{
    int32_t bar_w = BAR_W_FOR(row_w);
    s_bar_w[side][k] = bar_w;

    lv_obj_t *rc = lv_obj_create(parent);
    lv_obj_set_size(rc, row_w, h);
    lv_obj_set_pos(rc, x_base, y);
    lv_obj_set_style_bg_opa(rc, LV_OPA_TRANSP, 0);
    lv_obj_set_style_pad_all(rc, 0, 0);
    lv_obj_set_style_border_width(rc, 0, 0);
    lv_obj_set_style_radius(rc, 0, 0);
    lv_obj_clear_flag(rc, LV_OBJ_FLAG_SCROLLABLE);

    s_lbl_min[side][k] = lv_label_create(rc);
    lv_label_set_text(s_lbl_min[side][k], "--");
    lv_obj_set_style_text_color(s_lbl_min[side][k], lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(s_lbl_min[side][k], LV_ALIGN_LEFT_MID, MIN_LBL_X, 0);

    int32_t bg_h = h - 2 * ROW_PAD_V;
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

    int32_t fill_h = bg_h - 2;
    if (fill_h < 1) fill_h = 1;

    s_bar_main[side][k] = lv_obj_create(s_bar_bg[side][k]);
    lv_obj_set_size(s_bar_main[side][k], 1, fill_h);
    lv_obj_set_style_bg_color(s_bar_main[side][k], lv_color_hex(color), 0);
    lv_obj_set_style_pad_all(s_bar_main[side][k], 0, 0);
    lv_obj_set_style_border_width(s_bar_main[side][k], 0, 0);
    lv_obj_set_style_radius(s_bar_main[side][k], 0, 0);
    lv_obj_clear_flag(s_bar_main[side][k], LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(s_bar_main[side][k], LV_ALIGN_LEFT_MID, 0, 0);

    /* Catch slip (left/min end) and finish slip (right/max end) — darker
     * shades drawn on top of the main fill, at each end of the bar. */
    s_bar_catch[side][k] = lv_obj_create(s_bar_bg[side][k]);
    lv_obj_set_size(s_bar_catch[side][k], 1, fill_h);
    lv_obj_set_style_bg_color(s_bar_catch[side][k], lv_color_hex(darken(color, 0.4f)), 0);
    lv_obj_set_style_pad_all(s_bar_catch[side][k], 0, 0);
    lv_obj_set_style_border_width(s_bar_catch[side][k], 0, 0);
    lv_obj_set_style_radius(s_bar_catch[side][k], 0, 0);
    lv_obj_clear_flag(s_bar_catch[side][k], LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(s_bar_catch[side][k], LV_ALIGN_LEFT_MID, 0, 0);

    s_bar_finish[side][k] = lv_obj_create(s_bar_bg[side][k]);
    lv_obj_set_size(s_bar_finish[side][k], 1, fill_h);
    lv_obj_set_style_bg_color(s_bar_finish[side][k], lv_color_hex(darken(color, 0.4f)), 0);
    lv_obj_set_style_pad_all(s_bar_finish[side][k], 0, 0);
    lv_obj_set_style_border_width(s_bar_finish[side][k], 0, 0);
    lv_obj_set_style_radius(s_bar_finish[side][k], 0, 0);
    lv_obj_clear_flag(s_bar_finish[side][k], LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(s_bar_finish[side][k], LV_ALIGN_LEFT_MID, 0, 0);

    s_lbl_max[side][k] = lv_label_create(rc);
    lv_label_set_text(s_lbl_max[side][k], "--");
    lv_obj_set_style_text_color(s_lbl_max[side][k], lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(s_lbl_max[side][k], LV_ALIGN_LEFT_MID, BAR_X + bar_w + 4, 0);
}

static void al_enter(tBoatRadioEP *pEP, eBoatType bt)
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
    lv_label_set_text(title, "ARC LENGTH");
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
            create_bar_row(lc, 0, 0, LCD_H_RES, lane_h, k, SIDE_SWEEP, COL_SWEEP);
        } else {
            /* Port and starboard share the same horizontal plane, each
             * occupying half the screen width, side by side. */
            create_bar_row(lc, 0,      0, HALF_W, lane_h, k, SIDE_PORT, COL_PORT);
            create_bar_row(lc, HALF_W, 0, HALF_W, lane_h, k, SIDE_STAR, COL_STAR);
        }
    }

    update_bars();
}

static void al_exit(void)
{
    if (s_pep) {
        GraphScreen_ClearPlotting(&s_pep->ExtractionGridAPeriodic);
        s_pep = NULL;
    }
    if (s_cont) { lv_obj_del(s_cont); s_cont = NULL; }
    memset(s_lbl_seat,    0, sizeof(s_lbl_seat));
    memset(s_lbl_min,     0, sizeof(s_lbl_min));
    memset(s_lbl_max,     0, sizeof(s_lbl_max));
    memset(s_bar_bg,      0, sizeof(s_bar_bg));
    memset(s_bar_main,    0, sizeof(s_bar_main));
    memset(s_bar_catch,   0, sizeof(s_bar_catch));
    memset(s_bar_finish,  0, sizeof(s_bar_finish));
    memset(s_bar_w,       0, sizeof(s_bar_w));
}

static void al_on_periodic(tBoatRadioEP *pEP) { (void)pEP; }

static void al_on_stroke(tBoatRadioEP *pEP)
{
    if (!s_cont || !pEP) return;
    LcdLvgl_Lock();
    update_bars();
    LcdLvgl_Unlock();
}

static void al_on_touch(int x, int y) { (void)x; (void)y; }
static void al_render(void)           {}

const tGraphScreenVtable ArcLengthScreen = {
    .title       = "Arc Length",
    .enter       = al_enter,
    .exit        = al_exit,
    .on_periodic = al_on_periodic,
    .on_stroke   = al_on_stroke,
    .on_touch    = al_on_touch,
    .render      = al_render,
};
