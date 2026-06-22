#include "GraphScreen.h"
#include "GuiLvgl.h"
#include "EndPoints.h"
#include "PeachDefs.h"
#include "lcd.h"
#include "esp_attr.h"
#include "lvgl.h"
#include <math.h>
#include <stdio.h>

#define MAX_PTS       MAX_PERIODIC_PER_STROKE
#define BOTTOM_BAR_H  50
#define CANVAS_W      LCD_H_RES
#define CANVAS_H      (LCD_V_RES - BOTTOM_BAR_H)
#define TRACE_HW      4    /* half-width of square pen in pixels */
#define AXIS_HW       1

#define COL_TRACE_RGB 0x00FF00u   /* green */
#define COL_AXIS_RGB  0xCCCCCCu   /* light grey */
#define COL_CATCH_RGB 0xFFFF00u   /* yellow — marks the catch, fixed at screen center */

/* Boat-level (unseated) acceleration — a single trace, no rower selection */
#define ACCEL_MIN_MPS2  -12.0f
#define ACCEL_MAX_MPS2    8.0f
#define WINDOW_S          1.5f   /* total visible time window, centered on the catch */
#define HALF_WINDOW_S     (WINDOW_S / 2.0f)

/* ---- State ---- */

static tBoatRadioEP          *s_pep;
static eBoatType               s_bt;
static tExtractionGridElement *s_elem;

/* Crew-wide catch time (time since the stroke callback at which the drive
 * begins — see GraphScreen_CrewDriveStartTicks()), recomputed each stroke.
 * The x-axis is centered on this so the catch always sits mid-screen;
 * falls back to the middle of the window when not yet known. */
static bool  s_has_catch_t;
static float s_catch_t;

static lv_obj_t *s_title    = NULL;
static lv_obj_t *s_lbl_amax = NULL;
static lv_obj_t *s_lbl_amin = NULL;
static lv_obj_t *s_lbl_tmax = NULL;
static lv_obj_t *s_lbl_rate = NULL;
static lv_obj_t *s_lbl_pace = NULL;

/* Aperiodic grid indices — boat-level (seat 0): rating (SIDa0) and average
 * boat speed (SIDa1), the latter converted to a 500m pace for display. */
static uint8_t s_iRate;
static bool    s_has_rate;
static uint8_t s_iSpeed;
static bool    s_has_speed;

/* Trace buffer — PSRAM BSS. X = time since start of stroke (s), Y = accel (m/s^2) */
static float    s_tbx[MAX_PTS] EXT_RAM_BSS_ATTR;
static float    s_tby[MAX_PTS] EXT_RAM_BSS_ATTR;
static uint16_t s_tn;
static uint16_t s_sample_idx;   /* ticks since start of stroke, 1 tick = 1/SAMPLE_FREQUENCY_HZ */
static int      s_cur_lap;      /* current sweep across the canvas; bumped each time the trace wraps */

/* ---- Helpers ---- */

/* Unwrapped pixel x: keeps growing past CANVAS_W as t advances, so the
 * sweep ("lap") a point falls in can be recovered before wrapping it. */
static inline float map_cx_raw(float t) {
    float usable  = (float)(CANVAS_W - 2 * TRACE_HW);
    float catch_t = s_has_catch_t ? s_catch_t : HALF_WINDOW_S;
    float t_rel   = (t - catch_t) + HALF_WINDOW_S;   /* catch_t -> HALF_WINDOW_S -> screen center */
    return TRACE_HW + (t_rel / WINDOW_S) * usable;
}
static inline int wrap_lap(float xf) { return (int)floorf(xf / (float)CANVAS_W); }
static inline int wrap_x(float xf) {
    int w  = CANVAS_W;
    int xi = (int)floorf(xf) % w;
    if (xi < 0) xi += w;
    return xi;
}
static inline float map_cy(float a) {
    float usable = (float)(CANVAS_H - 2 * TRACE_HW);
    return TRACE_HW + (1.0f - (a - ACCEL_MIN_MPS2) / (ACCEL_MAX_MPS2 - ACCEL_MIN_MPS2)) * usable;
}

static void draw_axes(void)
{
    LcdFb_DrawLine(0, CANVAS_H - 1, CANVAS_W - 1, CANVAS_H - 1, COL_AXIS_RGB, AXIS_HW);
    float y_zero = map_cy(0.0f);
    if (y_zero >= 0.0f && y_zero < (float)CANVAS_H)
        LcdFb_DrawLine(0, (int)y_zero, CANVAS_W - 1, (int)y_zero, COL_AXIS_RGB, AXIS_HW);

    /* The catch always maps to the screen's horizontal center by construction. */
    int x_catch = CANVAS_W / 2;
    LcdFb_DrawLine(x_catch, 0, x_catch, CANVAS_H - 1, COL_CATCH_RGB, AXIS_HW);
}

/* Crew-wide catch time for this stroke. See GraphScreen_CrewDriveStartTicks(). */
static void compute_catch_t(tBoatRadioEP *pEP)
{
    s_has_catch_t = false;
    if (!pEP) return;

    float ticks = GraphScreen_CrewDriveStartTicks(&pEP->ExtractionGridAPeriodic, s_bt);
    if (isnan(ticks)) return;

    s_catch_t     = ticks / (float)SAMPLE_FREQUENCY_HZ;
    s_has_catch_t = true;
}

static void collect_elem(tBoatRadioEP *pEP)
{
    s_elem = NULL;
    if (!pEP) return;
    s_elem = GraphScreen_FindElem(&pEP->ExtractionGridPeriodic, eSIDPer_Acceleration);
    if (s_elem) s_elem->plotting = true;
}

static void collect_aper_indices(tBoatRadioEP *pEP)
{
    s_has_rate  = false;
    s_has_speed = false;
    if (!pEP) return;

    tExtractionGrid *ga = &pEP->ExtractionGridAPeriodic;
    for (uint8_t i = 0; i < ga->nGridElemDefined; i++) {
        uint8_t sid  = ga->extractionGrid[i].semanticId;
        uint8_t seat = ga->extractionGrid[i].seat;
        if (seat != 0) continue;

        if (sid == eSIDAPer_Rating) {
            s_iRate    = i;
            s_has_rate = true;
            ga->extractionGrid[i].plotting = true;
        } else if (sid == eSIDAPer_AvgBoatSpeed) {
            s_iSpeed    = i;
            s_has_speed = true;
            ga->extractionGrid[i].plotting = true;
        }
    }
}

static void update_stat_labels(void)
{
    if (!s_lbl_rate || !s_lbl_pace || !s_pep) return;

    tExtractionGrid *ga = &s_pep->ExtractionGridAPeriodic;
    char buf[24];

    if (s_has_rate && s_iRate < ga->nGridElemDefined) {
        float v = ga->extractionGrid[s_iRate].value;
        if (!isnan(v) && v > 0.0f)
            snprintf(buf, sizeof(buf), "R%.0f", (double)v);
        else
            snprintf(buf, sizeof(buf), "R--");
    } else {
        snprintf(buf, sizeof(buf), "R--");
    }
    lv_label_set_text(s_lbl_rate, buf);

    float speed = NAN;
    if (s_has_speed && s_iSpeed < ga->nGridElemDefined)
        speed = ga->extractionGrid[s_iSpeed].value;

    if (!isnan(speed) && speed > 0.0f) {
        float pace_s = 500.0f / speed;
        int   mins   = (int)(pace_s / 60.0f);
        float secs   = pace_s - (float)mins * 60.0f;
        snprintf(buf, sizeof(buf), "%d:%04.1f", mins, (double)secs);
    } else {
        snprintf(buf, sizeof(buf), "-:--.-");
    }
    lv_label_set_text(s_lbl_pace, buf);
}

/* Clear the graph area and redraw axes directly to the framebuffer, and
 * reset the stroke-relative sample clock. Must be called with LVGL lock held. */
static void clear_graph(void)
{
    LcdFb_FillRect(0, 0, CANVAS_W, CANVAS_H, 0x000000u);
    s_tn = 0;
    s_sample_idx = 0;
    s_cur_lap = 0;
    draw_axes();
    if (s_title)    lv_obj_invalidate(s_title);
    if (s_lbl_amax) lv_obj_invalidate(s_lbl_amax);
    if (s_lbl_amin) lv_obj_invalidate(s_lbl_amin);
    if (s_lbl_tmax) lv_obj_invalidate(s_lbl_tmax);
    if (s_lbl_rate) lv_obj_invalidate(s_lbl_rate);
    if (s_lbl_pace) lv_obj_invalidate(s_lbl_pace);
}

/* ---- Vtable callbacks ---- */

static void at_enter(tBoatRadioEP *pEP, eBoatType bt)
{
    s_bt  = bt;
    s_pep = pEP;
    s_has_catch_t = false;
    collect_elem(pEP);
    collect_aper_indices(pEP);

    LcdFb_FillRect(0, 0, CANVAS_W, CANVAS_H, 0x000000u);
    s_tn = 0;
    s_sample_idx = 0;
    s_cur_lap = 0;
    draw_axes();

    s_title = lv_label_create(lv_scr_act());
    lv_label_set_text(s_title, "ACCELERATION vs TIME");
    lv_obj_set_style_text_color(s_title, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_pos(s_title, 220, 4);

    char buf[20];
    s_lbl_amax = lv_label_create(lv_scr_act());
    snprintf(buf, sizeof(buf), "%.0f m/s^2", (double)ACCEL_MAX_MPS2);
    lv_label_set_text(s_lbl_amax, buf);
    lv_obj_set_style_text_color(s_lbl_amax, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_pos(s_lbl_amax, 4, 4);

    s_lbl_amin = lv_label_create(lv_scr_act());
    snprintf(buf, sizeof(buf), "%.0f m/s^2", (double)ACCEL_MIN_MPS2);
    lv_label_set_text(s_lbl_amin, buf);
    lv_obj_set_style_text_color(s_lbl_amin, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_pos(s_lbl_amin, 4, CANVAS_H - 24);

    s_lbl_tmax = lv_label_create(lv_scr_act());
    snprintf(buf, sizeof(buf), "+%.2fs", (double)HALF_WINDOW_S);
    lv_label_set_text(s_lbl_tmax, buf);
    lv_obj_set_style_text_color(s_lbl_tmax, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_pos(s_lbl_tmax, CANVAS_W - 56, CANVAS_H - 24);

    /* Rating / pace stat boxes — same size & format as the rate/power
     * boxes on the Force vs Angle screen, anchored to the bottom corners. */
    s_lbl_rate = lv_label_create(lv_scr_act());
    lv_label_set_text(s_lbl_rate, "R--");
    lv_obj_set_style_text_color(s_lbl_rate, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(s_lbl_rate, &lv_font_montserrat_48, 0);
    lv_obj_align(s_lbl_rate, LV_ALIGN_BOTTOM_LEFT, 4, -(4 + BOTTOM_BAR_H));

    s_lbl_pace = lv_label_create(lv_scr_act());
    lv_label_set_text(s_lbl_pace, "-:--.-");
    lv_obj_set_style_text_color(s_lbl_pace, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(s_lbl_pace, &lv_font_montserrat_48, 0);
    lv_obj_align(s_lbl_pace, LV_ALIGN_BOTTOM_RIGHT, -4, -(4 + BOTTOM_BAR_H));

    update_stat_labels();
}

static void at_exit(void)
{
    if (s_pep) {
        if (s_elem) s_elem->plotting = false;
        tExtractionGrid *ga = &s_pep->ExtractionGridAPeriodic;
        if (s_has_rate  && s_iRate  < ga->nGridElemDefined) ga->extractionGrid[s_iRate].plotting  = false;
        if (s_has_speed && s_iSpeed < ga->nGridElemDefined) ga->extractionGrid[s_iSpeed].plotting = false;
        s_pep  = NULL;
        s_elem = NULL;
        s_has_rate  = false;
        s_has_speed = false;
    }
    if (s_title)    { lv_obj_del(s_title);    s_title    = NULL; }
    if (s_lbl_amax) { lv_obj_del(s_lbl_amax); s_lbl_amax = NULL; }
    if (s_lbl_amin) { lv_obj_del(s_lbl_amin); s_lbl_amin = NULL; }
    if (s_lbl_tmax) { lv_obj_del(s_lbl_tmax); s_lbl_tmax = NULL; }
    if (s_lbl_rate) { lv_obj_del(s_lbl_rate); s_lbl_rate = NULL; }
    if (s_lbl_pace) { lv_obj_del(s_lbl_pace); s_lbl_pace = NULL; }
    s_tn = 0;
}

static void at_on_periodic(tBoatRadioEP *pEP)
{
    if (!s_title || !pEP || !s_elem) return;

    float a = s_elem->value;
    if (isnan(a)) return;

    float t = (float)s_sample_idx / (float)SAMPLE_FREQUENCY_HZ;
    s_sample_idx++;

    if (a < ACCEL_MIN_MPS2) a = ACCEL_MIN_MPS2;
    if (a > ACCEL_MAX_MPS2) a = ACCEL_MAX_MPS2;

    if (s_tn < MAX_PTS) {
        float xf  = map_cx_raw(t);
        int   lap = wrap_lap(xf);
        int   x   = wrap_x(xf);
        int   y   = (int)map_cy(a);

        if (lap != s_cur_lap) {
            /* Trace has swept off the right edge — retrace: blank the
             * canvas and redraw axes, then resume from the left edge
             * without a connecting line back across the whole screen. */
            LcdFb_FillRect(0, 0, CANVAS_W, CANVAS_H, 0x000000u);
            draw_axes();
            s_cur_lap = lap;
        } else if (s_tn > 0) {
            float xf_prev = map_cx_raw(s_tbx[s_tn - 1]);
            int   x_prev  = wrap_x(xf_prev);
            int   y_prev  = (int)map_cy(s_tby[s_tn - 1]);
            LcdFb_DrawLine(x_prev, y_prev, x, y, COL_TRACE_RGB, TRACE_HW);
        }

        s_tbx[s_tn] = t;
        s_tby[s_tn] = a;
        s_tn++;
    }
}

static void at_on_stroke(tBoatRadioEP *pEP)
{
    if (!s_title || !pEP) return;

    if (!s_elem) collect_elem(pEP);
    if (!s_has_rate || !s_has_speed) collect_aper_indices(pEP);
    compute_catch_t(pEP);

    LcdLvgl_Lock();
    clear_graph();
    update_stat_labels();
    LcdLvgl_Unlock();
}

static void at_on_touch(int x, int y) { (void)x; (void)y; }
static void at_render(void)           {}

const tGraphScreenVtable AccelTimeScreen = {
    .title       = "Accel vs Time",
    .enter       = at_enter,
    .exit        = at_exit,
    .on_periodic = at_on_periodic,
    .on_stroke   = at_on_stroke,
    .on_touch    = at_on_touch,
    .render      = at_render,
};
