#include "GraphScreen.h"
#include "GuiLvgl.h"
#include "lcd.h"
#include "lvgl.h"

/* Calibration screen — concentric rectangles rendered entirely via LVGL.
 *
 * Using LVGL objects instead of direct framebuffer writes avoids the race
 * where LVGL's dirty-region flush overwrites direct-fb content on screen entry.
 *
 * The outermost white rectangle is placed at LVGL coords (0,0)–(799,479).
 * When PANEL_OFS_X/Y are correct its 1-pixel border sits flush against all
 * four physical display edges.  Each inner ring is RING_STEP px smaller on
 * each side, so you can count rings to estimate the pixel error.
 */

#define RING_STEP  20

static const uint32_t k_ring_colors[] = {
    0xFFFFFF,  /* white   — outermost, primary alignment reference */
    0xFF4040,  /* red     */
    0xFFFF00,  /* yellow  */
    0x40FF40,  /* green   */
    0x00FFFF,  /* cyan    */
    0x4040FF,  /* blue    */
    0xFF40FF,  /* magenta */
    0xFF8000,  /* orange  */
};
#define N_COLORS  (sizeof(k_ring_colors) / sizeof(k_ring_colors[0]))

static lv_obj_t *s_root = NULL;

static void cb_enter(tBoatRadioEP *pEP, eBoatType bt)
{
    (void)pEP; (void)bt;
    GuiLvgl_SetBottomBarVisible(false);

    s_root = lv_obj_create(lv_scr_act());
    lv_obj_set_size(s_root, LCD_H_RES, LCD_V_RES);
    lv_obj_set_pos(s_root, 0, 0);
    lv_obj_set_style_pad_all(s_root, 0, 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_radius(s_root, 0, 0);
    lv_obj_set_style_bg_color(s_root, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(s_root, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    int n_rings = (LCD_V_RES / 2) / RING_STEP;
    for (int i = 0; i < n_rings; i++) {
        int32_t w = LCD_H_RES - 2 * i * RING_STEP;
        int32_t h = LCD_V_RES - 2 * i * RING_STEP;
        if (w <= 0 || h <= 0) break;

        lv_obj_t *r = lv_obj_create(s_root);
        lv_obj_set_size(r, w, h);
        lv_obj_align(r, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_radius(r, 0, 0);
        lv_obj_set_style_pad_all(r, 0, 0);
        lv_obj_set_style_border_color(r, lv_color_hex(k_ring_colors[i % N_COLORS]), 0);
        lv_obj_set_style_border_width(r, 1, 0);
        lv_obj_set_style_bg_opa(r, LV_OPA_TRANSP, 0);
        lv_obj_clear_flag(r, LV_OBJ_FLAG_SCROLLABLE);
    }

    /* Cyan crosshair — on top of rings. */
    lv_obj_t *hline = lv_obj_create(s_root);
    lv_obj_set_size(hline, LCD_H_RES, 1);
    lv_obj_align(hline, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(hline, 0, 0);
    lv_obj_set_style_border_width(hline, 0, 0);
    lv_obj_set_style_bg_color(hline, lv_color_hex(0x00FFFF), 0);
    lv_obj_set_style_bg_opa(hline, LV_OPA_COVER, 0);
    lv_obj_clear_flag(hline, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *vline = lv_obj_create(s_root);
    lv_obj_set_size(vline, 1, LCD_V_RES);
    lv_obj_align(vline, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(vline, 0, 0);
    lv_obj_set_style_border_width(vline, 0, 0);
    lv_obj_set_style_bg_color(vline, lv_color_hex(0x00FFFF), 0);
    lv_obj_set_style_bg_opa(vline, LV_OPA_COVER, 0);
    lv_obj_clear_flag(vline, LV_OBJ_FLAG_SCROLLABLE);
}

static void cb_exit(void)
{
    GuiLvgl_SetBottomBarVisible(true);
    if (s_root) { lv_obj_del(s_root); s_root = NULL; }
}

static void cb_on_periodic(tBoatRadioEP *pEP)  { (void)pEP; }
static void cb_on_stroke(tBoatRadioEP *pEP)    { (void)pEP; }
static void cb_on_touch(int x, int y)          { (void)x; (void)y; }
static void cb_render(void)                    {}

const tGraphScreenVtable CalibScreen = {
    .title       = "Calibration",
    .enter       = cb_enter,
    .exit        = cb_exit,
    .on_periodic = cb_on_periodic,
    .on_stroke   = cb_on_stroke,
    .on_touch    = cb_on_touch,
    .render      = cb_render,
};
