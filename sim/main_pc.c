/**
 * PC simulator entry point for the LCDFull GUI.
 *
 * Replaces the ESP32 hardware init with an SDL2 window and connects to a
 * PTY created by:
 *   socat PTY,link=/tmp/fakepic,rawer PTY,link=/tmp/simport,rawer
 *
 * Build:
 *   cd sim && mkdir -p build && cd build && cmake .. && make -j$(nproc)
 * Run (with FakePIC):
 *   socat PTY,link=/tmp/fakepic,rawer PTY,link=/tmp/simport,rawer &
 *   fakepic ...          # feed session_55_packets.bin to /tmp/fakepic
 *   ./build/lcdfull_sim [-s <scale%>]
 * Run (no hardware):
 *   ./build/lcdfull_sim  # PTY open will fail gracefully; static data only
 *
 * Options:
 *   -s <pct>   Window scale percentage (default 100).  e.g. -s 150 for 150%.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "lvgl.h"
#include "src/drivers/sdl/lv_sdl_window.h"
#include "src/drivers/sdl/lv_sdl_mouse.h"

#include "GuiLvgl.h"
#include "lcd.h"

#define SIM_W  800
#define SIM_H  480

/* Declared in sim_pipeline.c */
bool SimPipeline_Init(const char *pty_path);

/* ================================================================
 * Direct framebuffer helpers.
 *
 * On hardware these poke straight into the PSRAM framebuffer that the RGB
 * DMA continuously scans out. The sim's lv_sdl_window driver runs in
 * LV_DISPLAY_RENDER_MODE_DIRECT with LV_SDL_BUF_COUNT==1, so it keeps a
 * single persistent full-screen draw buffer that LVGL only partially
 * redraws each frame — the same model as the hardware fb. Writing directly
 * into that buffer (XRGB8888 @ LV_COLOR_DEPTH==32) reproduces the same
 * incremental-plot behavior in the sim.
 * ================================================================ */
static lv_display_t *s_disp = NULL;

static inline void put_px(int32_t x, int32_t y, uint32_t rgb888)
{
    if ((unsigned)x >= (unsigned)SIM_W || (unsigned)y >= (unsigned)SIM_H) return;
    lv_draw_buf_t *buf = lv_display_get_buf_active(s_disp);
    if (!buf) return;
    uint8_t *row = buf->data + (size_t)y * buf->header.stride;
    ((uint32_t *)row)[x] = rgb888 & 0xFFFFFFu;
}

void LcdFb_FillRect(int x, int y, int w, int h, uint32_t rgb888)
{
    for (int row = y; row < y + h; row++)
        for (int col = x; col < x + w; col++)
            put_px(col, row, rgb888);
}

void LcdFb_DrawLine(int x0, int y0, int x1, int y1, uint32_t rgb888, int half_w)
{
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        for (int ky = -half_w; ky <= half_w; ky++)
            for (int kx = -half_w; kx <= half_w; kx++)
                put_px(x0 + kx, y0 + ky, rgb888);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

int main(int argc, char *argv[])
{
    float scale = 1.0f;

    for (int i = 1; i < argc; i++) {
        if (argv[i][0] == '-' && argv[i][1] == 's' && argv[i][2] == '\0' && i + 1 < argc) {
            int pct = atoi(argv[++i]);
            if (pct > 0) scale = (float)pct / 100.0f;
        }
    }

    lv_init();

    lv_display_t *disp = lv_sdl_window_create(SIM_W, SIM_H);
    s_disp = disp;
    lv_sdl_window_set_zoom(disp, scale);
    lv_sdl_window_set_title(disp, "Peach Coaching Link LCDFull  [PC Sim]");

    lv_indev_t *mouse = lv_sdl_mouse_create();
    (void)mouse;

    /* Start GUI — loads graphcnf.txt from "." if present.
     * Lands on BoatSelect; click the boat once the pipeline shows it ready. */
    GuiLvgl_Start(".");

    /* Start the FakePIC serial pipeline.  Fails gracefully if PTY not present. */
    SimPipeline_Init("/tmp/simport");

    /* ---- Main loop -------------------------------------------------------- */
    while (1) {
        LcdLvgl_Lock();
        uint32_t delay_ms = lv_timer_handler();
        /* Force a window repaint every iteration: the SDL backend only
         * blits the framebuffer when LVGL itself flushes a dirty area, but
         * graph traces are written directly to the framebuffer and don't
         * mark anything dirty. Real hardware doesn't need this — the RGB
         * DMA engine scans the PSRAM framebuffer continuously. */
        GuiLvgl_ForceFlush();
        LcdLvgl_Unlock();
        if (delay_ms < 1)  delay_ms = 1;
        if (delay_ms > 10) delay_ms = 10;
        usleep(delay_ms * 1000u);
    }

    return 0;
}
