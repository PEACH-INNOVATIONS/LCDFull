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
#define TRACE_HW      8    /* half-width of square pen in pixels */
#define AXIS_HW       1

/* 24-bit RGB colours for direct framebuffer writes.
 * Sweep boats plot a single trace (green). Sculling boats plot two traces
 * simultaneously per rower — port (red) and starboard (green). */
#define COL_TRACE_SWEEP_RGB  0x00FF00u   /* green */
#define COL_TRACE_PORT_RGB   0xFF0000u   /* red */
#define COL_TRACE_STAR_RGB   0x00FF00u   /* green */
#define COL_AXIS_RGB         0xCCCCCCu   /* light grey */

/* ---- State ---- */

static eBoatType     s_bt;
static tBoatRadioEP *s_pep;
static uint8_t       s_selected_rower;
static uint8_t       s_n_rowers;

/* Port = sweep boats' single oar, or sculling boats' port oar (SIDp 9/11).
 * Star = sculling boats' starboard oar only (SIDp 10/12). */
typedef struct {
    uint8_t iAnglePort, iForcePort;
    uint8_t iAngleStar, iForceStar;
    bool    valid;       /* port pair valid */
    bool    validStar;   /* star pair valid (sculling only) */
} tRowerIdx;
static tRowerIdx s_rowers[N_ROWERS_MAX];

static uint8_t s_iXPort, s_iYPort;
static uint8_t s_iXStar, s_iYStar;
static bool    s_has_star;

/* LVGL overlay labels (parented to lv_scr_act, not a container) */
static lv_obj_t *s_lbl_rate  = NULL;
static lv_obj_t *s_lbl_power = NULL;
static lv_obj_t *s_lbl_fmax  = NULL;
static lv_obj_t *s_lbl_amin  = NULL;
static lv_obj_t *s_lbl_amax  = NULL;
static lv_obj_t *s_title     = NULL;

/* Aperiodic grid indices */
static uint8_t s_iRate;
static bool    s_has_rate;
static uint8_t s_iPow[N_ROWERS_MAX];
static uint8_t s_iPow2[N_ROWERS_MAX];
static bool    s_has_pow[N_ROWERS_MAX];
static bool    s_has_pow2[N_ROWERS_MAX];

/* Trace buffers — PSRAM BSS. Port trace always used; star trace only
 * populated/drawn for sculling boats. */
static float    s_tbxP[MAX_PTS] EXT_RAM_BSS_ATTR;
static float    s_tbyP[MAX_PTS] EXT_RAM_BSS_ATTR;
static uint16_t s_tnP;

static float    s_tbxS[MAX_PTS] EXT_RAM_BSS_ATTR;
static float    s_tbyS[MAX_PTS] EXT_RAM_BSS_ATTR;
static uint16_t s_tnS;

/* Axis ranges */
static float s_amin, s_amax, s_fmin, s_fmax;

/* ---- Helpers ---- */

static void get_ranges(void)
{
    s_amin = IsConfigParamSet(eConfigParam_FvAngle_GraphAngleMin_Deg) ?
             GetConfigValue(eConfigParam_FvAngle_GraphAngleMin_Deg).fVal : -70.0f;
    s_amax = IsConfigParamSet(eConfigParam_FvAngle_GraphAngleMax_Deg) ?
             GetConfigValue(eConfigParam_FvAngle_GraphAngleMax_Deg).fVal :  40.0f;
    s_fmin = IsConfigParamSet(eConfigParam_FvAngle_GraphFMin_KgF) ?
             GetConfigValue(eConfigParam_FvAngle_GraphFMin_KgF).fVal : -10.0f;
    s_fmax = IsConfigParamSet(eConfigParam_FvAngle_GraphFMax_KgF) ?
             GetConfigValue(eConfigParam_FvAngle_GraphFMax_KgF).fVal : 140.0f;
}

/* Map into a margin-shrunk range so a square pen of half-width TRACE_HW
 * centred on the mapped point never bleeds past the canvas edge (and, at
 * the bottom, into the button bar immediately below the canvas). */
static inline float map_cx(float a) {
    float usable = (float)(CANVAS_W - 2 * TRACE_HW);
    return TRACE_HW + (a - s_amin) / (s_amax - s_amin) * usable;
}
static inline float map_cy(float f) {
    float usable = (float)(CANVAS_H - 2 * TRACE_HW);
    return TRACE_HW + (1.0f - (f - s_fmin) / (s_fmax - s_fmin)) * usable;
}

static void draw_axes(void)
{
    LcdFb_DrawLine(0, CANVAS_H - 1, CANVAS_W - 1, CANVAS_H - 1, COL_AXIS_RGB, AXIS_HW);
    float x_zero = map_cx(0.0f);
    if (x_zero >= 0.0f && x_zero < (float)CANVAS_W)
        LcdFb_DrawLine((int)x_zero, 0, (int)x_zero, CANVAS_H - 1, COL_AXIS_RGB, AXIS_HW);
}

static void collect_rowers(tBoatRadioEP *pEP)
{
    s_n_rowers = 0;
    memset(s_rowers, 0, sizeof(s_rowers));

    tExtractionGrid *g = &pEP->ExtractionGridPeriodic;
    bool    scull     = (s_bt == eBoatType_Sculling);
    uint8_t semAPort  = scull ? 9u  : 2u;
    uint8_t semFPort  = scull ? 11u : 4u;
    uint8_t semAStar  = 10u;
    uint8_t semFStar  = 12u;

    for (uint8_t i = 0; i < g->nGridElemDefined; i++) {
        uint8_t sid  = g->extractionGrid[i].semanticId;
        uint8_t seat = g->extractionGrid[i].seat;
        if (seat < 1 || seat > N_ROWERS_MAX) continue;
        uint8_t r = seat - 1;

        if (sid == semAPort) {
            s_rowers[r].iAnglePort = i;
            s_rowers[r].valid      = true;
            if (r >= s_n_rowers) s_n_rowers = r + 1;
        } else if (scull && sid == semAStar) {
            s_rowers[r].iAngleStar = i;
            s_rowers[r].validStar  = true;
            if (r >= s_n_rowers) s_n_rowers = r + 1;
        }
    }
    for (uint8_t i = 0; i < g->nGridElemDefined; i++) {
        uint8_t sid  = g->extractionGrid[i].semanticId;
        uint8_t seat = g->extractionGrid[i].seat;
        if (seat < 1 || seat > N_ROWERS_MAX) continue;
        uint8_t r = seat - 1;

        if (sid == semFPort) {
            if (s_rowers[r].valid) s_rowers[r].iForcePort = i;
        } else if (scull && sid == semFStar) {
            if (s_rowers[r].validStar) s_rowers[r].iForceStar = i;
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
        bool valid = ga->extractionGrid[i].valid;

        if(sid == 0)
        {
            printf("sid = 0 i = %d s = %d v = %d\r\n",i,seat,valid);
        }

        if (sid == 0 && seat == 0) {
            s_iRate = i;
            s_has_rate = true;
            ga->extractionGrid[i].plotting = true;
            printf("found rate i = %d\r\n",i);
        } else if (sid == semPow && seat >= 1 && seat <= N_ROWERS_MAX) {
            uint8_t r = seat - 1;
            s_iPow[r]    = i;
            s_has_pow[r] = true;
            ga->extractionGrid[i].plotting = true;
            printf("found power s = %d i = %d\r\n",seat,i);
        } else if (s_bt == eBoatType_Sculling && sid == semPow2
                   && seat >= 1 && seat <= N_ROWERS_MAX) {
            uint8_t r = seat - 1;
            s_iPow2[r]    = i;
            s_has_pow2[r] = true;
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
    if (got) {
        snprintf(buf, sizeof(buf), "%.0f W", (double)pw);
        printf("%.2f W\r\n", (double)pw);
        fflush(stdout);
    } else {
        snprintf(buf, sizeof(buf), "-- W");
    }
    lv_label_set_text(s_lbl_power, buf);
}

static void set_rower(uint8_t r)
{
    s_selected_rower = r;
    s_has_star = false;
    if (r < N_ROWERS_MAX && s_rowers[r].valid) {
        s_iXPort = s_rowers[r].iAnglePort;
        s_iYPort = s_rowers[r].iForcePort;
    }
    if (s_bt == eBoatType_Sculling && r < N_ROWERS_MAX && s_rowers[r].validStar) {
        s_iXStar  = s_rowers[r].iAngleStar;
        s_iYStar  = s_rowers[r].iForceStar;
        s_has_star = true;
    }
    GuiLvgl_SetSeatLabel(r + 1, s_n_rowers > 0 ? s_n_rowers : 1);

    if (s_pep) {
        tExtractionGrid *g = &s_pep->ExtractionGridPeriodic;
        GraphScreen_ClearPlotting(g);
        if (r < N_ROWERS_MAX && s_rowers[r].valid) {
            g->extractionGrid[s_iXPort].plotting = true;
            g->extractionGrid[s_iYPort].plotting = true;
        }
        if (s_has_star) {
            g->extractionGrid[s_iXStar].plotting = true;
            g->extractionGrid[s_iYStar].plotting = true;
        }
    }
    update_stat_labels();
}

/* Clear the graph area and redraw axes directly to the framebuffer.
 * Then invalidate overlay labels so LVGL repaints them on top.
 * Must be called with LVGL lock held. */
static void clear_graph(void)
{
    LcdFb_FillRect(0, 0, CANVAS_W, CANVAS_H, 0x000000u);
    s_tnP = 0;
    s_tnS = 0;
    draw_axes();
    if (s_title)    lv_obj_invalidate(s_title);
    if (s_lbl_fmax) lv_obj_invalidate(s_lbl_fmax);
    if (s_lbl_amin) lv_obj_invalidate(s_lbl_amin);
    if (s_lbl_amax) lv_obj_invalidate(s_lbl_amax);
}

/* ---- Vtable callbacks ---- */

static void fa_enter(tBoatRadioEP *pEP, eBoatType bt)
{
    s_bt  = bt;
    s_pep = pEP;
    get_ranges();
    if (pEP) {
        collect_rowers(pEP);
        collect_aper_indices(pEP);
    }

    /* Clear graph area and draw axes directly */
    LcdFb_FillRect(0, 0, CANVAS_W, CANVAS_H, 0x000000u);
    s_tnP = 0;
    s_tnS = 0;
    draw_axes();

    /* Overlay labels parented to lv_scr_act so LVGL only touches
     * their small bounding boxes, never the full graph area */
    char buf[20];
    const char *title_str = (s_bt == eBoatType_Sculling) ?
                            "FORCE vs ANGLE (scull)" : "FORCE vs ANGLE (sweep)";
    s_title = lv_label_create(lv_scr_act());
    lv_label_set_text(s_title, title_str);
    lv_obj_set_style_text_color(s_title, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_pos(s_title, 200, 4);

    s_lbl_fmax = lv_label_create(lv_scr_act());
    snprintf(buf, sizeof(buf), "%.0f kgF", (double)s_fmax);
    lv_label_set_text(s_lbl_fmax, buf);
    lv_obj_set_style_text_color(s_lbl_fmax, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_pos(s_lbl_fmax, 4, 24);

    s_lbl_amin = lv_label_create(lv_scr_act());
    snprintf(buf, sizeof(buf), "%.0f", (double)s_amin);
    lv_label_set_text(s_lbl_amin, buf);
    lv_obj_set_style_text_color(s_lbl_amin, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_pos(s_lbl_amin, 4, CANVAS_H - 24);

    s_lbl_amax = lv_label_create(lv_scr_act());
    snprintf(buf, sizeof(buf), "%.0f", (double)s_amax);
    lv_label_set_text(s_lbl_amax, buf);
    lv_obj_set_style_text_color(s_lbl_amax, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_pos(s_lbl_amax, CANVAS_W - 36, CANVAS_H - 24);

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
}

static void fa_exit(void)
{
    if (s_pep) {
        GraphScreen_ClearPlotting(&s_pep->ExtractionGridPeriodic);
        GraphScreen_ClearPlotting(&s_pep->ExtractionGridAPeriodic);
        s_pep = NULL;
    }
    if (s_title)    { lv_obj_del(s_title);    s_title    = NULL; }
    if (s_lbl_fmax) { lv_obj_del(s_lbl_fmax); s_lbl_fmax = NULL; }
    if (s_lbl_amin) { lv_obj_del(s_lbl_amin); s_lbl_amin = NULL; }
    if (s_lbl_amax) { lv_obj_del(s_lbl_amax); s_lbl_amax = NULL; }
    if (s_lbl_rate) { lv_obj_del(s_lbl_rate); s_lbl_rate = NULL; }
    if (s_lbl_power){ lv_obj_del(s_lbl_power);s_lbl_power= NULL; }
    s_tnP = 0;
    s_tnS = 0;
}

static void plot_point(tExtractionGrid *g, uint8_t iX, uint8_t iY,
                        float *tbx, float *tby, uint16_t *tn, uint32_t color)
{
    if (iX >= g->nGridElemDefined || iY >= g->nGridElemDefined) return;

    float a = g->extractionGrid[iX].value;
    float f = g->extractionGrid[iY].value;
    if (isnan(a) || isnan(f)) return;

    if (a < s_amin) a = s_amin;
    if (a > s_amax) a = s_amax;
    if (f < s_fmin) f = s_fmin;
    if (f > s_fmax) f = s_fmax;

    if (*tn < MAX_PTS) {
        tbx[*tn] = a;
        tby[*tn] = f;
        if (*tn > 0) {
            LcdFb_DrawLine(
                (int)map_cx(tbx[*tn - 1]), (int)map_cy(tby[*tn - 1]),
                (int)map_cx(a),             (int)map_cy(f),
                color, TRACE_HW);
        }
        (*tn)++;
    }
}

static void fa_on_periodic(tBoatRadioEP *pEP)
{
    if (!s_lbl_rate || !pEP) return;

    tExtractionGrid *g = &pEP->ExtractionGridPeriodic;
    uint32_t portColor = (s_bt == eBoatType_Sculling) ? COL_TRACE_PORT_RGB : COL_TRACE_SWEEP_RGB;
    plot_point(g, s_iXPort, s_iYPort, s_tbxP, s_tbyP, &s_tnP, portColor);
    if (s_has_star)
        plot_point(g, s_iXStar, s_iYStar, s_tbxS, s_tbyS, &s_tnS, COL_TRACE_STAR_RGB);
}

static void fa_on_stroke(tBoatRadioEP *pEP)
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

static bool fa_on_button(bool next)
{
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

static void fa_on_touch(int x, int y) { (void)x; (void)y; }
static void fa_render(void)           {}

const tGraphScreenVtable ForceAngleScreen = {
    .title       = "Force vs Angle",
    .enter       = fa_enter,
    .exit        = fa_exit,
    .on_periodic = fa_on_periodic,
    .on_stroke   = fa_on_stroke,
    .on_touch    = fa_on_touch,
    .render      = fa_render,
    .on_button   = fa_on_button,
};
