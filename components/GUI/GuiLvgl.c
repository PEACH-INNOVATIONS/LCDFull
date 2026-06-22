#include "GuiLvgl.h"
#include "GraphScreen.h"
#include "GraphConfig.h"
#include "FileConfig.h"
#include "EndPoints.h"
#include "lcd.h"
#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

/* Screen vtable externs */
extern const tGraphScreenVtable BoatSelectScreen;
extern const tGraphScreenVtable ForceAngleScreen;
extern const tGraphScreenVtable HandleSpeedScreen;
extern const tGraphScreenVtable AccelTimeScreen;
extern const tGraphScreenVtable PowerScreen;
extern const tGraphScreenVtable AngleTimeScreen;
extern const tGraphScreenVtable ArcLengthScreen;
extern const tGraphScreenVtable CalibScreen;

typedef enum {
    eScreen_BoatSelect = 0,
    eScreen_ForceAngle,
    eScreen_HandleSpeed,
    eScreen_AccelTime,
    eScreen_Power,
    eScreen_ArcLength,
    eScreen_AngleTime,
    eScreen_Calib,
    eScreen_Count
} eScreenIdx;

static const tGraphScreenVtable *s_screens[eScreen_Count] = {
    [eScreen_BoatSelect]  = &BoatSelectScreen,
    [eScreen_ForceAngle]  = &ForceAngleScreen,
    [eScreen_HandleSpeed] = &HandleSpeedScreen,
    [eScreen_AccelTime]   = &AccelTimeScreen,
    [eScreen_Power]       = &PowerScreen,
    [eScreen_ArcLength]   = &ArcLengthScreen,
    [eScreen_AngleTime]   = &AngleTimeScreen,
    [eScreen_Calib]       = &CalibScreen,
};

static eScreenIdx    s_cur     = eScreen_BoatSelect;
static tBoatRadioEP *s_sel_ep  = NULL;
static eBoatType     s_bt      = eBoatType_Unknown;

/* Bottom bar widgets (created once, live on lv_layer_top) */
static lv_obj_t *s_bottom_bar = NULL;
static lv_obj_t *s_seat_lbl   = NULL;

/* Tiny transparent object whose only purpose is to be invalidated so the
 * PC sim's SDL backend flushes the whole framebuffer to the window (see
 * GuiLvgl_ForceFlush). Unused on real hardware, where the RGB DMA engine
 * scans the PSRAM framebuffer continuously regardless of LVGL flushes. */
static lv_obj_t *s_heartbeat = NULL;

/* ---- EP callbacks (called from PacketParser task) ---- */

static void gui_periodic_cb(tBoatRadioEP *pEP)
{
    if (s_cur == eScreen_BoatSelect) {
        if (s_screens[eScreen_BoatSelect]->on_periodic)
            s_screens[eScreen_BoatSelect]->on_periodic(pEP);
        return;
    }
    if (pEP != s_sel_ep) return;

    /* Keep "Drive start sweep" (SIDa 22) always plotting, independent of
     * which screen is active, so EndpointCallbackStroke extracts its value
     * every stroke for the debug print in gui_stroke_cb below. Re-marked
     * here (well ahead of the next stroke) because screens routinely call
     * GraphScreen_ClearPlotting() on this same grid from their own enter(). */
    tExtractionGrid *ga = &pEP->ExtractionGridAPeriodic;
    for (uint8_t i = 0; i < ga->nGridElemDefined; i++)
        if (ga->extractionGrid[i].semanticId == eSIDAPer_DriveStartSweep)
            ga->extractionGrid[i].plotting = true;

    if (s_screens[s_cur]->on_periodic)
        s_screens[s_cur]->on_periodic(pEP);
}

static void gui_stroke_cb(tBoatRadioEP *pEP)
{
    if (pEP != s_sel_ep) return;

    /* Debug: print "Drive start sweep" (SIDa 22) for every rower on every
     * stroke, regardless of which screen is showing. */
    tExtractionGrid *ga = &pEP->ExtractionGridAPeriodic;
    for (uint8_t i = 0; i < ga->nGridElemDefined; i++) {
        if (ga->extractionGrid[i].semanticId == eSIDAPer_DriveStartSweep) {
            printf("DriveStartSweep: seat=%u value=%.1f ticks\n",
                   ga->extractionGrid[i].seat, (double)ga->extractionGrid[i].value);
        }
    }

    if (s_screens[s_cur]->on_stroke)
        s_screens[s_cur]->on_stroke(pEP);
}

/* ---- Screen switching ---- */

static void switch_to(eScreenIdx idx)
{
    if (s_screens[s_cur]->exit)
        s_screens[s_cur]->exit();
    s_cur = idx;
    lv_obj_scroll_to(lv_scr_act(), 0, 0, LV_ANIM_OFF);
    if (s_screens[s_cur]->enter)
        s_screens[s_cur]->enter(s_sel_ep, s_bt);
}

/* ---- Hardware button callback (button_task context, LVGL mutex held) ---- */

static void hw_button_cb(uint8_t mask)
{
    /* bits: 0=left(<), 1=R-, 2=R+, 3=right(>) */
    if (mask & 0x01) {
        int prev = (int)s_cur - 1;
        if (prev < 0) prev = eScreen_Count - 1;
        switch_to((eScreenIdx)prev);
    } else if (mask & 0x08) {
        if (s_screens[s_cur]->on_confirm) {
            s_screens[s_cur]->on_confirm();
        } else {
            if (s_cur == eScreen_BoatSelect && !s_sel_ep) return;
            int next = (int)s_cur + 1;
            if (next >= eScreen_Count) next = eScreen_BoatSelect;
            switch_to((eScreenIdx)next);
        }
    } else if (mask & 0x02) {
        if (s_screens[s_cur]->on_button)
            s_screens[s_cur]->on_button(false);
    } else if (mask & 0x04) {
        if (s_screens[s_cur]->on_button)
            s_screens[s_cur]->on_button(true);
    }
}

/* ---- Bottom bar button callbacks (LVGL task context — no mutex needed) ---- */

static void btn_prev_cb(lv_event_t *e)
{
    (void)e;
    GuiLvgl_PrevScreen();
}

static void btn_rower_prev_cb(lv_event_t *e)
{
    (void)e;
    if (s_cur != eScreen_BoatSelect && s_screens[s_cur]->on_button)
        s_screens[s_cur]->on_button(false);
}

static void btn_rower_next_cb(lv_event_t *e)
{
    (void)e;
    if (s_cur != eScreen_BoatSelect && s_screens[s_cur]->on_button)
        s_screens[s_cur]->on_button(true);
}

static void btn_next_cb(lv_event_t *e)
{
    (void)e;
    GuiLvgl_NextScreen();
}

/* ---- Create the persistent bottom bar on lv_layer_top ---- */

#define BOTTOM_BAR_H  50

static lv_obj_t *make_bar_btn(lv_obj_t *bar, int32_t x, int32_t w,
                               const char *lbl_text, lv_color_t bg,
                               lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_button_create(bar);
    lv_obj_set_size(btn, w, BOTTOM_BAR_H);
    lv_obj_set_pos(btn, x, 0);
    lv_obj_set_style_bg_color(btn, bg, 0);
    lv_obj_set_style_bg_color(btn, lv_color_darken(bg, 40), LV_STATE_PRESSED);
    lv_obj_set_style_radius(btn, 0, 0);
    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, lbl_text);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(lbl);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    return btn;
}

static void create_bottom_bar(void)
{
    lv_obj_t *bar = lv_obj_create(lv_layer_top());
    s_bottom_bar = bar;
    lv_obj_set_size(bar, LCD_H_RES, BOTTOM_BAR_H);
    lv_obj_set_pos(bar, 0, LCD_V_RES - BOTTOM_BAR_H);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x111122), 0);
    lv_obj_set_style_pad_all(bar, 0, 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    int32_t btn_w = LCD_H_RES / 4;   /* 200 px each on 800-wide panel */

    /* Screen navigation — blue tint */
    lv_color_t nav_col   = lv_color_hex(0x003366);
    /* Rower cycling — green tint */
    lv_color_t rower_col = lv_color_hex(0x004422);

    make_bar_btn(bar,         0, btn_w, "<",  nav_col,   btn_prev_cb);
    make_bar_btn(bar,     btn_w, btn_w, "R-", rower_col, btn_rower_prev_cb);
    make_bar_btn(bar, btn_w * 2, btn_w, "R+", rower_col, btn_rower_next_cb);
    make_bar_btn(bar, btn_w * 3, btn_w, ">",  nav_col,   btn_next_cb);

    /* Seat label — centred over R-/R+, non-clickable so touches pass through */
    s_seat_lbl = lv_label_create(bar);
    lv_label_set_text(s_seat_lbl, "");
    lv_obj_set_size(s_seat_lbl, btn_w * 2, BOTTOM_BAR_H);
    lv_obj_set_pos(s_seat_lbl, btn_w, 0);
    lv_obj_set_style_text_align(s_seat_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_seat_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_seat_lbl, &lv_font_montserrat_24, 0);
    lv_obj_clear_flag(s_seat_lbl, LV_OBJ_FLAG_CLICKABLE);
}

/* ---- Backlight dropdown (collapses to 30 px strip at top of screen) ---- */

#define TOPBAR_COLLAPSED_H  30
#define TOPBAR_EXPANDED_H   110
#define TOPBAR_ANIM_MS      200

static lv_obj_t   *s_topbar         = NULL;
static lv_obj_t   *s_bl_label       = NULL;
static lv_obj_t   *s_bl_slider      = NULL;
static lv_timer_t *s_bl_hide_timer  = NULL;
static bool        s_topbar_expanded = false;

static void set_height_anim(void *obj, int32_t h)
{
    lv_obj_set_height((lv_obj_t *)obj, (int32_t)h);
}

static void bl_hide_timer_cb(lv_timer_t *t)
{
    (void)t;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_topbar);
    lv_anim_set_exec_cb(&a, set_height_anim);
    lv_anim_set_values(&a, lv_obj_get_height(s_topbar), TOPBAR_COLLAPSED_H);
    lv_anim_set_time(&a, TOPBAR_ANIM_MS);
    lv_anim_start(&a);
    lv_obj_set_style_opa(s_topbar, LV_OPA_TRANSP, 0);
    s_topbar_expanded = false;
    s_bl_hide_timer   = NULL;   /* LVGL auto-deletes repeat_count=1 timers */
}

static void reset_hide_timer(void)
{
    if (s_bl_hide_timer) {
        lv_timer_reset(s_bl_hide_timer);
    } else {
        s_bl_hide_timer = lv_timer_create(bl_hide_timer_cb, 3000, NULL);
        lv_timer_set_repeat_count(s_bl_hide_timer, 1);
    }
}

static void topbar_header_click_cb(lv_event_t *e)
{
    (void)e;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_topbar);
    lv_anim_set_exec_cb(&a, set_height_anim);

    if (s_topbar_expanded) {
        lv_anim_set_values(&a, lv_obj_get_height(s_topbar), TOPBAR_COLLAPSED_H);
        lv_anim_set_time(&a, TOPBAR_ANIM_MS);
        lv_anim_start(&a);
        lv_obj_set_style_opa(s_topbar, LV_OPA_TRANSP, 0);
        if (s_bl_hide_timer) { lv_timer_delete(s_bl_hide_timer); s_bl_hide_timer = NULL; }
        s_topbar_expanded = false;
    } else {
        lv_anim_set_values(&a, lv_obj_get_height(s_topbar), TOPBAR_EXPANDED_H);
        lv_anim_set_time(&a, TOPBAR_ANIM_MS);
        lv_anim_start(&a);
        lv_obj_set_style_opa(s_topbar, LV_OPA_90, 0);
        s_topbar_expanded = true;
        reset_hide_timer();
    }
}

static void bl_slider_cb(lv_event_t *e)
{
    (void)e;
    uint8_t pct = (uint8_t)lv_slider_get_value(s_bl_slider);
    LcdBacklight_SetPercent(pct);
    char buf[10];
    snprintf(buf, sizeof(buf), "BL %u%%", pct);
    lv_label_set_text(s_bl_label, buf);
    reset_hide_timer();
}

static void create_top_dropdown(void)
{
    /* Prevent scroll-chain from escaping to lv_layer_top() — if allowed, any
     * slight finger movement on the header strip gets classified as a scroll
     * and the LV_EVENT_CLICKED on hdr is never fired. */
    lv_obj_clear_flag(lv_layer_top(), LV_OBJ_FLAG_SCROLLABLE);

    s_topbar = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_topbar, LCD_H_RES, TOPBAR_COLLAPSED_H);
    lv_obj_set_pos(s_topbar, 0, 0);
    lv_obj_set_style_bg_color(s_topbar, lv_color_hex(0x111122), 0);
    lv_obj_set_style_opa(s_topbar, LV_OPA_TRANSP, 0);  /* invisible until tapped */
    lv_obj_set_style_pad_all(s_topbar, 0, 0);
    lv_obj_set_style_border_width(s_topbar, 0, 0);
    lv_obj_set_style_radius(s_topbar, 0, 0);
    lv_obj_clear_flag(s_topbar, LV_OBJ_FLAG_SCROLLABLE);

    /* Header strip — the 30 px clickable toggle area, always visible */
    lv_obj_t *hdr = lv_obj_create(s_topbar);
    lv_obj_set_size(hdr, LCD_H_RES, TOPBAR_COLLAPSED_H);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_style_bg_opa(hdr, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(hdr, topbar_header_click_cb, LV_EVENT_CLICKED, NULL);

    s_bl_label = lv_label_create(hdr);
    lv_label_set_text(s_bl_label, "BL 50%");
    lv_obj_set_style_text_color(s_bl_label, lv_color_hex(0xAAAAAA), 0);
    lv_obj_center(s_bl_label);

    /* Slider — lives below the header strip; clipped by panel until expanded */
    s_bl_slider = lv_slider_create(s_topbar);
    lv_obj_set_size(s_bl_slider, LCD_H_RES - 40, 40);
    lv_obj_set_pos(s_bl_slider, 20, TOPBAR_COLLAPSED_H + 5);
    lv_slider_set_range(s_bl_slider, 0, 100);
    lv_slider_set_value(s_bl_slider, 50, LV_ANIM_OFF);
    lv_obj_add_event_cb(s_bl_slider, bl_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);
    LcdBacklight_SetPercent(50);
}

/* ---- Public API ---- */

void GuiLvgl_Start(const char *sd_mount_point)
{
    GraphConfig_Load(sd_mount_point);
    EP_SetGraphPeriodicCallback(gui_periodic_cb);
    EP_SetGraphStrokeCallback(gui_stroke_cb);

    if (IsConfigParamSet(eConfigParam_DisableTouch) &&
        GetConfigValue(eConfigParam_DisableTouch).bVal)
        LcdTouch_SetEnabled(false);

    LcdHwButton_SetCallback(hw_button_cb);

    LcdLvgl_Lock();
    /* Black background prevents the default white screen bg from flashing through
     * behind labels when LVGL redraws them (would look like colour inversion). */
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);
    /* Prevent the screen from scrolling — touch noise can produce phantom
     * gestures that shift the entire display by ~96 px (20%) at 1-2 Hz. */
    lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_scroll_to(lv_scr_act(), 0, 0, LV_ANIM_OFF);
    create_bottom_bar();
    create_top_dropdown();

    s_heartbeat = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_heartbeat, 1, 1);
    lv_obj_set_pos(s_heartbeat, 0, 0);
    lv_obj_set_style_opa(s_heartbeat, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_heartbeat, 0, 0);
    lv_obj_clear_flag(s_heartbeat, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(s_heartbeat, LV_OBJ_FLAG_SCROLLABLE);

    s_cur    = eScreen_BoatSelect;
    s_sel_ep = NULL;
    s_bt     = eBoatType_Unknown;
    if (s_screens[eScreen_BoatSelect]->enter)
        s_screens[eScreen_BoatSelect]->enter(NULL, eBoatType_Unknown);
    LcdLvgl_Unlock();
}

void GuiLvgl_ForceFlush(void)
{
    if (s_heartbeat) lv_obj_invalidate(s_heartbeat);
}

void GuiLvgl_Tick(void)
{
    if (s_cur != eScreen_BoatSelect) return;
    if (s_screens[eScreen_BoatSelect]->on_periodic)
        s_screens[eScreen_BoatSelect]->on_periodic(NULL);
}

void GuiLvgl_SetSelectedEP(tBoatRadioEP *pEP)
{
    s_sel_ep = pEP;
    s_bt     = pEP ? GetBoatType(pEP->loggerID) : eBoatType_Unknown;
}

void GuiLvgl_NextScreen(void)
{
    if (s_cur == eScreen_BoatSelect && s_sel_ep == NULL) return;
    int next = (int)s_cur + 1;
    if (next >= (int)eScreen_Count) next = (int)eScreen_BoatSelect;
    LcdLvgl_Lock();
    switch_to((eScreenIdx)next);
    LcdLvgl_Unlock();
}

void GuiLvgl_PrevScreen(void)
{
    int prev = (int)s_cur - 1;
    if (prev < 0) prev = (int)eScreen_Count - 1;
    LcdLvgl_Lock();
    switch_to((eScreenIdx)prev);
    LcdLvgl_Unlock();
}

void GuiLvgl_SetSeatLabel(uint8_t seat_1based, uint8_t n_seats)
{
    if (!s_seat_lbl) return;
    char buf[16];
    if (n_seats > 1)
        snprintf(buf, sizeof(buf), "%u / %u", seat_1based, n_seats);
    else
        snprintf(buf, sizeof(buf), "%u", seat_1based);
    lv_label_set_text(s_seat_lbl, buf);
}

void GuiLvgl_SetBottomBarVisible(bool visible)
{
    if (!s_bottom_bar) return;
    lv_obj_set_style_opa(s_bottom_bar,
                         visible ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
}
