#include "GraphScreen.h"
#include "GuiLvgl.h"
#include "GraphConfig.h"
#include "FileConfig.h"
#include "EndPoints.h"
#include "PeachDefs.h"
#include "lcd.h"
#include "esp_attr.h"
#include "lvgl.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAX_PTS       MAX_PERIODIC_PER_STROKE
#define BOTTOM_BAR_H  50
#define CANVAS_W      LCD_H_RES
#define CANVAS_H      (LCD_V_RES - BOTTOM_BAR_H)
#define TRACE_HW      3    /* half-width of square pen in pixels */
#define AXIS_HW       1
/* Port/starboard handle-speed curves for a sculling rower are nearly
 * identical (one rower drives both oars in sync), so without this the
 * second trace drawn (a hard pixel overwrite, no blending) completely
 * paints over the first wherever they coincide. Offset them vertically
 * by a few px so both stay visible. */
#define TRACE_SEP_PX  4

/* 24-bit RGB colours for direct framebuffer writes.
 * Sweep boats plot a single trace (blue). Sculling boats plot two traces
 * simultaneously per rower — port (red) and starboard (green). */
#define COL_TRACE_SWEEP_RGB  0x00AAFFu   /* blue */
#define COL_TRACE_PORT_RGB   0xFF0000u   /* red */
#define COL_TRACE_STAR_RGB   0x00FF00u   /* green */
#define COL_AXIS_RGB    0xCCCCCCu

/* ---- State ---- */

static eBoatType     s_bt;
static tBoatRadioEP *s_pep;
static uint8_t       s_selected_rower;
static uint8_t       s_n_rowers;

typedef struct { uint8_t iAngle1; uint8_t iAngle2; bool v1; bool v2; } tHsRower;
static tHsRower s_rowers[N_ROWERS_MAX];
static uint8_t s_iA1, s_iA2;
static bool    s_has_a2;

/* Trace buffers — PSRAM BSS */
static float    s_tb1x[MAX_PTS] EXT_RAM_BSS_ATTR;
static float    s_tb1y[MAX_PTS] EXT_RAM_BSS_ATTR;
static uint16_t s_t1n;
static float    s_prev_a1;
static bool     s_has_prev1;

static float    s_tb2x[MAX_PTS] EXT_RAM_BSS_ATTR;
static float    s_tb2y[MAX_PTS] EXT_RAM_BSS_ATTR;
static uint16_t s_t2n;
static float    s_prev_a2_val;
static bool     s_has_prev2;

static float s_amin, s_amax, s_smin, s_smax;

/* LVGL overlay labels (parented to lv_scr_act) */
static lv_obj_t *s_lbl_rate  = NULL;
static lv_obj_t *s_lbl_power = NULL;
static lv_obj_t *s_lbl_smax  = NULL;
static lv_obj_t *s_title     = NULL;

/* Aperiodic grid indices */
static uint8_t s_iRate;
static bool    s_has_rate;
static uint8_t s_iPow[N_ROWERS_MAX];
static uint8_t s_iPow2[N_ROWERS_MAX];
static bool    s_has_pow[N_ROWERS_MAX];
static bool    s_has_pow2[N_ROWERS_MAX];

/* ---- Helpers ---- */

static void get_ranges(void)
{
    s_amin = IsConfigParamSet(eConfigParam_HSVsAngle_GraphAngleMin_Deg) ?
             GetConfigValue(eConfigParam_HSVsAngle_GraphAngleMin_Deg).fVal : -60.0f;
    s_amax = IsConfigParamSet(eConfigParam_HSVsAngle_GraphAngleMax_Deg) ?
             GetConfigValue(eConfigParam_HSVsAngle_GraphAngleMax_Deg).fVal :  60.0f;
    s_smin = IsConfigParamSet(eConfigParam_HSVsAngle_GraphSMin_Deg_S) ?
             GetConfigValue(eConfigParam_HSVsAngle_GraphSMin_Deg_S).fVal : -300.0f;
    s_smax = IsConfigParamSet(eConfigParam_HSVsAngle_GraphSMax_Deg_S) ?
             GetConfigValue(eConfigParam_HSVsAngle_GraphSMax_Deg_S).fVal :  300.0f;
}

/* Map into a margin-shrunk range so a square pen of half-width TRACE_HW
 * centred on the mapped point never bleeds past the canvas edge (and, at
 * the bottom, into the button bar immediately below the canvas).
 * map_cy reserves extra margin (MARGIN_Y) so the +-TRACE_SEP_PX offset
 * applied to the port/starboard traces (see hs_on_periodic) can never
 * push a pen past the canvas edge either. */
#define MARGIN_Y (TRACE_HW + TRACE_SEP_PX)
static inline float map_cx(float a) {
    float usable = (float)(CANVAS_W - 2 * TRACE_HW);
    return TRACE_HW + (a - s_amin) / (s_amax - s_amin) * usable;
}
static inline float map_cy(float s) {
    float usable = (float)(CANVAS_H - 2 * MARGIN_Y);
    return MARGIN_Y + (1.0f - (s - s_smin) / (s_smax - s_smin)) * usable;
}

static void draw_axes(void)
{
    float y0 = map_cy(0.0f);
    if (y0 >= 0.0f && y0 < (float)CANVAS_H)
        LcdFb_DrawLine(0, (int)y0, CANVAS_W - 1, (int)y0, COL_AXIS_RGB, AXIS_HW);
    float x0 = map_cx(0.0f);
    if (x0 >= 0.0f && x0 < (float)CANVAS_W)
        LcdFb_DrawLine((int)x0, 0, (int)x0, CANVAS_H - 1, COL_AXIS_RGB, AXIS_HW);
}

static void collect_rowers(tBoatRadioEP *pEP)
{
    s_n_rowers = 0;
    memset(s_rowers, 0, sizeof(s_rowers));
    tExtractionGrid *g = &pEP->ExtractionGridPeriodic;
    uint8_t sem1 = (s_bt == eBoatType_Sculling) ?  9u : 2u;
    uint8_t sem2 = 10u;

    for (uint8_t i = 0; i < g->nGridElemDefined; i++) {
        uint8_t seat = g->extractionGrid[i].seat;
        if (seat < 1 || seat > N_ROWERS_MAX) continue;
        uint8_t r = seat - 1;
        if (g->extractionGrid[i].semanticId == sem1) {
            s_rowers[r].iAngle1 = i; s_rowers[r].v1 = true;
            if (r >= s_n_rowers) s_n_rowers = r + 1;
        }
        if (s_bt == eBoatType_Sculling && g->extractionGrid[i].semanticId == sem2) {
            s_rowers[r].iAngle2 = i; s_rowers[r].v2 = true;
        }
    }
}

static void collect_aper_indices(tBoatRadioEP *pEP)
{
    s_has_rate = false;
    memset(s_has_pow,  false, sizeof(s_has_pow));
    memset(s_has_pow2, false, sizeof(s_has_pow2));

    tExtractionGrid *ga = &pEP->ExtractionGridAPeriodic;
    uint8_t semPow  = (s_bt == eBoatType_Sculling) ? 4u : 6u;
    uint8_t semPow2 = 5u;

    for (uint8_t i = 0; i < ga->nGridElemDefined; i++) {
        uint8_t sid  = ga->extractionGrid[i].semanticId;
        uint8_t seat = ga->extractionGrid[i].seat;

        if (sid == 0 && seat == 0) {
            s_iRate = i; s_has_rate = true;
            ga->extractionGrid[i].plotting = true;
        } else if (sid == semPow && seat >= 1 && seat <= N_ROWERS_MAX) {
            uint8_t r = seat - 1;
            s_iPow[r] = i; s_has_pow[r] = true;
            ga->extractionGrid[i].plotting = true;
        } else if (s_bt == eBoatType_Sculling && sid == semPow2
                   && seat >= 1 && seat <= N_ROWERS_MAX) {
            uint8_t r = seat - 1;
            s_iPow2[r] = i; s_has_pow2[r] = true;
            ga->extractionGrid[i].plotting = true;
        }
    }
}

static void update_stat_labels(void)
{
    if (!s_lbl_rate || !s_lbl_power || !s_pep) return;

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

    uint8_t r = s_selected_rower;
    float pw = 0.0f;
    bool got = false;
    if (r < N_ROWERS_MAX && s_has_pow[r] && s_iPow[r] < ga->nGridElemDefined) {
        float v = ga->extractionGrid[s_iPow[r]].value;
        if (!isnan(v)) { pw += v; got = true; }
    }
    if (s_bt == eBoatType_Sculling
        && r < N_ROWERS_MAX && s_has_pow2[r] && s_iPow2[r] < ga->nGridElemDefined) {
        float v = ga->extractionGrid[s_iPow2[r]].value;
        if (!isnan(v)) { pw += v; got = true; }
    }
    snprintf(buf, sizeof(buf), got ? "%.0f W" : "-- W", (double)pw);
    lv_label_set_text(s_lbl_power, buf);
}

static void set_rower(uint8_t r)
{
    s_selected_rower = r;
    s_iA1    = s_rowers[r].iAngle1;
    s_iA2    = s_rowers[r].iAngle2;
    s_has_a2 = s_rowers[r].v2;
    s_t1n = s_t2n = 0;
    s_has_prev1 = s_has_prev2 = false;
    GuiLvgl_SetSeatLabel(r + 1, s_n_rowers > 0 ? s_n_rowers : 1);

    if (s_pep) {
        tExtractionGrid *g = &s_pep->ExtractionGridPeriodic;
        GraphScreen_ClearPlotting(g);
        if (s_rowers[r].v1) g->extractionGrid[s_iA1].plotting = true;
        if (s_rowers[r].v2) g->extractionGrid[s_iA2].plotting = true;
    }
}

static void clear_graph(void)
{
    LcdFb_FillRect(0, 0, CANVAS_W, CANVAS_H, 0x000000u);
    s_t1n = s_t2n = 0;
    s_has_prev1 = s_has_prev2 = false;
    draw_axes();
    if (s_title)    lv_obj_invalidate(s_title);
    if (s_lbl_smax) lv_obj_invalidate(s_lbl_smax);
}

/* ---- Vtable callbacks ---- */

static void hs_enter(tBoatRadioEP *pEP, eBoatType bt)
{
    s_bt  = bt;
    s_pep = pEP;
    get_ranges();
    if (pEP) { collect_rowers(pEP); collect_aper_indices(pEP); }

    /* Clear graph area and draw axes directly */
    LcdFb_FillRect(0, 0, CANVAS_W, CANVAS_H, 0x000000u);
    s_t1n = s_t2n = 0;
    s_has_prev1 = s_has_prev2 = false;
    draw_axes();

    /* Overlay labels parented to lv_scr_act */
    const char *title = (s_bt == eBoatType_Sculling) ?
                        "HANDLE SPEED vs ANGLE (scull)" : "HANDLE SPEED vs ANGLE (sweep)";
    s_title = lv_label_create(lv_scr_act());
    lv_label_set_text(s_title, title);
    lv_obj_set_style_text_color(s_title, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_pos(s_title, 160, 4);

    char buf[24];
    s_lbl_smax = lv_label_create(lv_scr_act());
    snprintf(buf, sizeof(buf), "%.0f deg/s", (double)s_smax);
    lv_label_set_text(s_lbl_smax, buf);
    lv_obj_set_style_text_color(s_lbl_smax, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_pos(s_lbl_smax, 4, 24);

    s_lbl_rate = lv_label_create(lv_scr_act());
    lv_label_set_text(s_lbl_rate, "R--");
    lv_obj_set_style_text_color(s_lbl_rate, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(s_lbl_rate, &lv_font_montserrat_48, 0);
    lv_obj_set_pos(s_lbl_rate, 4, 4);

    s_lbl_power = lv_label_create(lv_scr_act());
    lv_label_set_text(s_lbl_power, "-- W");
    lv_obj_set_style_text_color(s_lbl_power, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(s_lbl_power, &lv_font_montserrat_48, 0);
    lv_obj_align(s_lbl_power, LV_ALIGN_TOP_RIGHT, -4, 4);

    set_rower(0);
    update_stat_labels();
}

static void hs_exit(void)
{
    if (s_pep) {
        GraphScreen_ClearPlotting(&s_pep->ExtractionGridPeriodic);
        GraphScreen_ClearPlotting(&s_pep->ExtractionGridAPeriodic);
        s_pep = NULL;
    }
    if (s_title)    { lv_obj_del(s_title);    s_title    = NULL; }
    if (s_lbl_smax) { lv_obj_del(s_lbl_smax); s_lbl_smax = NULL; }
    if (s_lbl_rate) { lv_obj_del(s_lbl_rate); s_lbl_rate = NULL; }
    if (s_lbl_power){ lv_obj_del(s_lbl_power);s_lbl_power= NULL; }
    s_t1n = s_t2n = 0;
}

static void hs_on_periodic(tBoatRadioEP *pEP)
{
    if (!s_lbl_rate || !pEP) return;

    if (s_n_rowers == 0 && pEP->ExtractionGridPeriodic.nGridElemDefined > 0) {
        s_pep = pEP;
        collect_rowers(pEP);
        collect_aper_indices(pEP);
        LcdLvgl_Lock();
        set_rower(s_selected_rower);
        LcdLvgl_Unlock();
    }

    tExtractionGrid *g = &pEP->ExtractionGridPeriodic;

    /* Channel 1 */
    if (s_rowers[s_selected_rower].v1 && s_iA1 < g->nGridElemDefined) {
        float a = g->extractionGrid[s_iA1].value;
        if (!isnan(a)) {
            if (s_has_prev1 && s_t1n < MAX_PTS) {
                float sp = (a - s_prev_a1) * SAMPLE_FREQUENCY_HZ;
                if (a  < s_amin) a  = s_amin;
                if (a  > s_amax) a  = s_amax;
                if (sp < s_smin) sp = s_smin;
                if (sp > s_smax) sp = s_smax;
                s_tb1x[s_t1n] = a; s_tb1y[s_t1n] = sp;
                if (s_t1n > 0) {
                    int sep = (s_bt == eBoatType_Sculling) ? -TRACE_SEP_PX : 0;
                    LcdFb_DrawLine(
                        (int)map_cx(s_tb1x[s_t1n - 1]), (int)map_cy(s_tb1y[s_t1n - 1]) + sep,
                        (int)map_cx(a),                  (int)map_cy(sp) + sep,
                        (s_bt == eBoatType_Sculling) ? COL_TRACE_PORT_RGB : COL_TRACE_SWEEP_RGB,
                        TRACE_HW);
                }
                s_t1n++;
            }
            s_prev_a1 = a; s_has_prev1 = true;
        }
    }

    /* Channel 2 (sculling starboard) */
    if (s_has_a2 && s_iA2 < g->nGridElemDefined) {
        float a = g->extractionGrid[s_iA2].value;
        if (!isnan(a)) {
            if (s_has_prev2 && s_t2n < MAX_PTS) {
                float sp = (a - s_prev_a2_val) * SAMPLE_FREQUENCY_HZ;
                if (a  < s_amin) a  = s_amin;
                if (a  > s_amax) a  = s_amax;
                if (sp < s_smin) sp = s_smin;
                if (sp > s_smax) sp = s_smax;
                s_tb2x[s_t2n] = a; s_tb2y[s_t2n] = sp;
                if (s_t2n > 0)
                    LcdFb_DrawLine(
                        (int)map_cx(s_tb2x[s_t2n - 1]), (int)map_cy(s_tb2y[s_t2n - 1]) + TRACE_SEP_PX,
                        (int)map_cx(a),                  (int)map_cy(sp) + TRACE_SEP_PX,
                        COL_TRACE_STAR_RGB, TRACE_HW);
                s_t2n++;
            }
            s_prev_a2_val = a; s_has_prev2 = true;
        }
    }
}

static void hs_on_stroke(tBoatRadioEP *pEP)
{
    if (!s_lbl_rate || !pEP) return;

    if (s_n_rowers == 0 && pEP->ExtractionGridPeriodic.nGridElemDefined > 0) {
        s_pep = pEP;
        collect_rowers(pEP);
        collect_aper_indices(pEP);
        LcdLvgl_Lock();
        set_rower(s_selected_rower);
        LcdLvgl_Unlock();
    }

    LcdLvgl_Lock();
    clear_graph();
    update_stat_labels();
    LcdLvgl_Unlock();
}

static bool hs_on_button(bool next)
{
    uint8_t max_r = (s_n_rowers > 0) ? s_n_rowers - 1 : 0;
    if (max_r == 0) return false;

    uint8_t nr = s_selected_rower;
    if (next) { if (nr >= max_r) return false; nr++; }
    else      { if (nr == 0)    return false; nr--; }

    set_rower(nr);
    LcdLvgl_Lock();
    if (s_lbl_power) lv_label_set_text(s_lbl_power, "-- W");
    clear_graph();
    LcdLvgl_Unlock();
    return true;
}

static void hs_on_touch(int x, int y) { (void)x; (void)y; }
static void hs_render(void)           {}

const tGraphScreenVtable HandleSpeedScreen = {
    .title       = "Handle Speed vs Angle",
    .enter       = hs_enter,
    .exit        = hs_exit,
    .on_periodic = hs_on_periodic,
    .on_stroke   = hs_on_stroke,
    .on_touch    = hs_on_touch,
    .render      = hs_render,
    .on_button   = hs_on_button,
};
