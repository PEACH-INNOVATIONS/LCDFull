#include "GraphScreen.h"
#include "GuiLvgl.h"
#include "EndPoints.h"
#include "PeachDefs.h"
#include "lcd.h"
#include "esp_attr.h"
#include "lvgl.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* EXPERIMENTAL: sweep-only oarlock angle vs time, one rower at a time
 * (cycled with the < > buttons), with a vertical marker showing where in
 * the stroke the drive started (eSIDAPer_DriveStartSweep, SIDa 22). */

#define MAX_PTS       MAX_PERIODIC_PER_STROKE
#define BOTTOM_BAR_H  50
#define CANVAS_W      LCD_H_RES
#define CANVAS_H      (LCD_V_RES - BOTTOM_BAR_H)
#define TRACE_HW      4    /* half-width of square pen in pixels */
#define AXIS_HW       1

#define COL_TRACE_RGB      0x00FF00u   /* green */
#define COL_AXIS_RGB       0xCCCCCCu   /* light grey */
#define COL_DRIVE_LINE_RGB 0xFFFF00u   /* yellow — marks drive start */

#define ANGLE_MIN_DEG  -70.0f
#define ANGLE_MAX_DEG   40.0f
#define TIME_MAX_S      ((float)MAX_PERIODIC_PER_STROKE / (float)SAMPLE_FREQUENCY_HZ)

/* ---- State ---- */

static tBoatRadioEP *s_pep;
static eBoatType     s_bt;
static uint8_t       s_selected_rower;
static uint8_t       s_n_rowers;

typedef struct {
    uint8_t iAngle;
    bool    hasAngle;
} tRowerIdx;
static tRowerIdx s_rowers[N_ROWERS_MAX];

static tExtractionGridElement *s_elem;        /* selected rower's angle element */

static lv_obj_t *s_title    = NULL;
static lv_obj_t *s_lbl_amax = NULL;
static lv_obj_t *s_lbl_amin = NULL;
static lv_obj_t *s_lbl_tmax = NULL;
static lv_obj_t *s_lbl_na   = NULL;   /* "sweep boats only" message for scull */

static float    s_tbx[MAX_PTS] EXT_RAM_BSS_ATTR;
static float    s_tby[MAX_PTS] EXT_RAM_BSS_ATTR;
static uint16_t s_tn;
static uint16_t s_sample_idx;

static bool  s_has_drive_t;
static float s_drive_t;

/* ---- Helpers ---- */

static inline float map_cx(float t) {
    float usable = (float)(CANVAS_W - 2 * TRACE_HW);
    return TRACE_HW + (t / TIME_MAX_S) * usable;
}
static inline float map_cy(float a) {
    float usable = (float)(CANVAS_H - 2 * TRACE_HW);
    return TRACE_HW + (1.0f - (a - ANGLE_MIN_DEG) / (ANGLE_MAX_DEG - ANGLE_MIN_DEG)) * usable;
}

static void draw_axes(void)
{
    LcdFb_DrawLine(0, CANVAS_H - 1, CANVAS_W - 1, CANVAS_H - 1, COL_AXIS_RGB, AXIS_HW);
    float y_zero = map_cy(0.0f);
    if (y_zero >= 0.0f && y_zero < (float)CANVAS_H)
        LcdFb_DrawLine(0, (int)y_zero, CANVAS_W - 1, (int)y_zero, COL_AXIS_RGB, AXIS_HW);

    if (s_has_drive_t) {
        float x = map_cx(s_drive_t);
        if (x >= 0.0f && x < (float)CANVAS_W)
            LcdFb_DrawLine((int)x, 0, (int)x, CANVAS_H - 1, COL_DRIVE_LINE_RGB, AXIS_HW);
    }
}

static void collect_rowers(tBoatRadioEP *pEP)
{
    s_n_rowers = 0;
    memset(s_rowers, 0, sizeof(s_rowers));
    if (!pEP || s_bt != eBoatType_Sweep) return;

    tExtractionGrid *gp = &pEP->ExtractionGridPeriodic;
    for (uint8_t i = 0; i < gp->nGridElemDefined; i++) {
        uint8_t sid  = gp->extractionGrid[i].semanticId;
        uint8_t seat = gp->extractionGrid[i].seat;
        if (seat < 1 || seat > N_ROWERS_MAX) continue;
        if (sid != eSIDPer_SweepAngle) continue;

        uint8_t r = seat - 1;
        s_rowers[r].iAngle    = i;
        s_rowers[r].hasAngle  = true;
        if (r >= s_n_rowers) s_n_rowers = r + 1;
    }
}

/* Crew-wide drive-start ("catch") time, independent of which rower's angle
 * trace is currently selected. See GraphScreen_CrewDriveStartTicks(). */
static void compute_crew_drive_t(tBoatRadioEP *pEP)
{
    s_has_drive_t = false;
    if (!pEP) return;

    float ticks = GraphScreen_CrewDriveStartTicks(&pEP->ExtractionGridAPeriodic, s_bt);
    if (isnan(ticks)) return;

    float t = ticks / (float)SAMPLE_FREQUENCY_HZ;
    if (t < 0.0f) t = 0.0f;
    if (t > TIME_MAX_S) t = TIME_MAX_S;
    s_drive_t     = t;
    s_has_drive_t = true;
}

static void set_rower(uint8_t r)
{
    s_selected_rower = r;
    s_elem = NULL;

    GuiLvgl_SetSeatLabel(r + 1, s_n_rowers > 0 ? s_n_rowers : 1);

    if (!s_pep || r >= N_ROWERS_MAX) return;

    tExtractionGrid *gp = &s_pep->ExtractionGridPeriodic;
    GraphScreen_ClearPlotting(gp);
    if (s_rowers[r].hasAngle) {
        gp->extractionGrid[s_rowers[r].iAngle].plotting = true;
        s_elem = &gp->extractionGrid[s_rowers[r].iAngle];
    }
}

/* Clear the graph area and redraw axes (incl. drive-start marker) directly
 * to the framebuffer, and reset the stroke-relative sample clock.
 * Must be called with LVGL lock held. */
static void clear_graph(void)
{
    LcdFb_FillRect(0, 0, CANVAS_W, CANVAS_H, 0x000000u);
    s_tn = 0;
    s_sample_idx = 0;
    draw_axes();
    if (s_title)    lv_obj_invalidate(s_title);
    if (s_lbl_amax) lv_obj_invalidate(s_lbl_amax);
    if (s_lbl_amin) lv_obj_invalidate(s_lbl_amin);
    if (s_lbl_tmax) lv_obj_invalidate(s_lbl_tmax);
}

/* ---- Vtable callbacks ---- */

static void at_enter(tBoatRadioEP *pEP, eBoatType bt)
{
    s_bt  = bt;
    s_pep = pEP;
    s_has_drive_t = false;
    collect_rowers(pEP);

    LcdFb_FillRect(0, 0, CANVAS_W, CANVAS_H, 0x000000u);
    s_tn = 0;
    s_sample_idx = 0;
    draw_axes();

    s_title = lv_label_create(lv_scr_act());
    lv_label_set_text(s_title, "ANGLE vs TIME (sweep) [EXP]");
    lv_obj_set_style_text_color(s_title, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_pos(s_title, 180, 4);

    char buf[20];
    s_lbl_amax = lv_label_create(lv_scr_act());
    snprintf(buf, sizeof(buf), "%.0f deg", (double)ANGLE_MAX_DEG);
    lv_label_set_text(s_lbl_amax, buf);
    lv_obj_set_style_text_color(s_lbl_amax, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_pos(s_lbl_amax, 4, 4);

    s_lbl_amin = lv_label_create(lv_scr_act());
    snprintf(buf, sizeof(buf), "%.0f deg", (double)ANGLE_MIN_DEG);
    lv_label_set_text(s_lbl_amin, buf);
    lv_obj_set_style_text_color(s_lbl_amin, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_pos(s_lbl_amin, 4, CANVAS_H - 24);

    s_lbl_tmax = lv_label_create(lv_scr_act());
    snprintf(buf, sizeof(buf), "%.1fs", (double)TIME_MAX_S);
    lv_label_set_text(s_lbl_tmax, buf);
    lv_obj_set_style_text_color(s_lbl_tmax, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_pos(s_lbl_tmax, CANVAS_W - 40, CANVAS_H - 24);

    if (s_bt != eBoatType_Sweep) {
        s_lbl_na = lv_label_create(lv_scr_act());
        lv_label_set_text(s_lbl_na, "(sweep boats only)");
        lv_obj_set_style_text_color(s_lbl_na, lv_color_hex(0x888888), 0);
        lv_obj_align(s_lbl_na, LV_ALIGN_CENTER, 0, 0);
        GuiLvgl_SetSeatLabel(1, 1);
    } else {
        set_rower(0);
    }
}

static void at_exit(void)
{
    if (s_pep) {
        GraphScreen_ClearPlotting(&s_pep->ExtractionGridPeriodic);
        s_pep  = NULL;
        s_elem = NULL;
    }
    if (s_title)    { lv_obj_del(s_title);    s_title    = NULL; }
    if (s_lbl_amax) { lv_obj_del(s_lbl_amax); s_lbl_amax = NULL; }
    if (s_lbl_amin) { lv_obj_del(s_lbl_amin); s_lbl_amin = NULL; }
    if (s_lbl_tmax) { lv_obj_del(s_lbl_tmax); s_lbl_tmax = NULL; }
    if (s_lbl_na)   { lv_obj_del(s_lbl_na);   s_lbl_na   = NULL; }
    s_tn = 0;
}

static void at_on_periodic(tBoatRadioEP *pEP)
{
    if (!s_title || !pEP || !s_elem) return;

    float a = s_elem->value;
    if (isnan(a)) return;

    float t = (float)s_sample_idx / (float)SAMPLE_FREQUENCY_HZ;
    s_sample_idx++;

    if (t > TIME_MAX_S) return;
    if (a < ANGLE_MIN_DEG) a = ANGLE_MIN_DEG;
    if (a > ANGLE_MAX_DEG) a = ANGLE_MAX_DEG;

    if (s_tn < MAX_PTS) {
        s_tbx[s_tn] = t;
        s_tby[s_tn] = a;
        if (s_tn > 0) {
            LcdFb_DrawLine(
                (int)map_cx(s_tbx[s_tn - 1]), (int)map_cy(s_tby[s_tn - 1]),
                (int)map_cx(t),                (int)map_cy(a),
                COL_TRACE_RGB, TRACE_HW);
        }
        s_tn++;
    }
}

static void at_on_stroke(tBoatRadioEP *pEP)
{
    if (!s_title || !pEP) return;

    if (s_bt == eBoatType_Sweep && s_n_rowers == 0 &&
        pEP->ExtractionGridPeriodic.nGridElemDefined > 0) {
        s_pep = pEP;
        collect_rowers(pEP);
        LcdLvgl_Lock();
        set_rower(s_selected_rower);
        LcdLvgl_Unlock();
    }

    compute_crew_drive_t(pEP);

    LcdLvgl_Lock();
    clear_graph();
    LcdLvgl_Unlock();
}

static bool at_on_button(bool next)
{
    if (s_bt != eBoatType_Sweep) return false;
    uint8_t max_r = (s_n_rowers > 0) ? s_n_rowers - 1 : 0;
    if (max_r == 0) return false;

    uint8_t new_r = s_selected_rower;
    if (next) {
        if (new_r >= max_r) return false;
        new_r++;
    } else {
        if (new_r == 0) return false;
        new_r--;
    }

    set_rower(new_r);
    LcdLvgl_Lock();
    clear_graph();
    LcdLvgl_Unlock();
    return true;
}

static void at_on_touch(int x, int y) { (void)x; (void)y; }
static void at_render(void)           {}

const tGraphScreenVtable AngleTimeScreen = {
    .title       = "Angle vs Time",
    .enter       = at_enter,
    .exit        = at_exit,
    .on_periodic = at_on_periodic,
    .on_stroke   = at_on_stroke,
    .on_touch    = at_on_touch,
    .render      = at_render,
    .on_button   = at_on_button,
};
