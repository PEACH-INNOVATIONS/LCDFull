#include "lcd.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "lvgl.h"

#define TAG "lcd"

#define LCD_PCLK_HZ  (16 * 1000 * 1000)

/* Panel orientation & dead-zone compensation (see CLAUDE.md) */
#define PANEL_INVERT  0
#define PANEL_OFS_X    0
#define PANEL_OFS_Y  480

/* LVGL draw buffer: 10 lines in PSRAM, flush cb converts RGB565→RGB332 */
#define LVGL_BUF_LINES  10

static esp_lcd_panel_handle_t s_panel      = NULL;
static uint8_t               *s_fb         = NULL;
static SemaphoreHandle_t      s_vsync_sem  = NULL;
static SemaphoreHandle_t      s_lvgl_mux   = NULL;
static lv_indev_t            *s_touch_indev = NULL;

/* ---- Backlight PWM (LEDC) ---- */

static void backlight_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .gpio_num   = LCD_BACKLIGHT_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,    /* 0% — caller sets brightness via LcdBacklight_SetPercent */
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

void LcdBacklight_SetPercent(uint8_t pct)
{
    if (pct > 100) pct = 100;
    uint32_t duty = (pct * 1023u + 50u) / 100u;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

/* ---- MCP23017 button layer ---- */
static SemaphoreHandle_t      s_btn_sem   = NULL;
static uint8_t                s_btn_prev  = 0;
static void                 (*s_btn_cb)(uint8_t mask) = NULL;

/* MCP23017 register addresses (port A) */
#define MCP_IODIRA    0x00
#define MCP_GPPUA     0x0C
#define MCP_GPINTENA  0x04
#define MCP_INTCONA   0x08
#define MCP_INTCAPA   0x10

static esp_err_t mcp_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(TOUCH_I2C_PORT, MCP23017_ADDR,
                                      buf, 2, pdMS_TO_TICKS(10));
}

static esp_err_t mcp_read(uint8_t reg, uint8_t *val)
{
    return i2c_master_write_read_device(TOUCH_I2C_PORT, MCP23017_ADDR,
                                        &reg, 1, val, 1, pdMS_TO_TICKS(10));
}

static void IRAM_ATTR mcp23017_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_btn_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

static void button_task(void *arg)
{
    for (;;) {
        xSemaphoreTake(s_btn_sem, portMAX_DELAY);
        uint8_t val = 0xFF;
        if (mcp_read(MCP_INTCAPA, &val) != ESP_OK) continue;

        uint8_t pressed      = (~val) & 0x0F;      /* active-low buttons */
        uint8_t newly_pressed = pressed & ~s_btn_prev;
        s_btn_prev = pressed;

        if (newly_pressed && s_btn_cb) {
            xSemaphoreTakeRecursive(s_lvgl_mux, portMAX_DELAY);
            s_btn_cb(newly_pressed);
            xSemaphoreGiveRecursive(s_lvgl_mux);
        }
    }
}

static void mcp23017_init(void)
{
    mcp_write(MCP_IODIRA,   0x0F);   /* bits 0-3 inputs */
    mcp_write(MCP_GPPUA,    0x0F);   /* internal pull-ups on bits 0-3 */
    mcp_write(MCP_GPINTENA, 0x0F);   /* interrupt-on-change bits 0-3 */
    mcp_write(MCP_INTCONA,  0x00);   /* compare to previous state */

    /* Clear any latched interrupt before enabling the GPIO ISR */
    uint8_t dummy;
    mcp_read(MCP_INTCAPA, &dummy);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << MCP23017_INT_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io);

    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
        ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(ret));

    gpio_isr_handler_add(MCP23017_INT_GPIO, mcp23017_isr, NULL);

    s_btn_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(button_task, "btn", 4096, NULL, 3, NULL, 1);
}

/* ---- Direct framebuffer helpers ---- */

static inline uint8_t fb_rgb888_to_332(uint32_t c)
{
    return (uint8_t)(((c >> 16) & 0xE0u) |
                     (((c >>  8) & 0xE0u) >> 3) |
                     ((c        & 0xC0u) >> 6));
}

static void fb_put(int x, int y, uint8_t c)
{
    if ((unsigned)x >= (unsigned)LCD_H_RES || (unsigned)y >= (unsigned)LCD_V_RES) return;
    s_fb[((y + PANEL_OFS_Y) % LCD_V_RES) * LCD_H_RES +
         ((x + PANEL_OFS_X) % LCD_H_RES)] = c;
}

void LcdFb_FillRect(int x, int y, int w, int h, uint32_t rgb888)
{
    if (!s_fb) return;
    uint8_t c = fb_rgb888_to_332(rgb888);
    for (int row = y; row < y + h; row++)
        for (int col = x; col < x + w; col++)
            fb_put(col, row, c);
}

void LcdFb_DrawLine(int x0, int y0, int x1, int y1, uint32_t rgb888, int half_w)
{
    if (!s_fb) return;
    uint8_t c = fb_rgb888_to_332(rgb888);
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        for (int ky = -half_w; ky <= half_w; ky++)
            for (int kx = -half_w; kx <= half_w; kx++)
                fb_put(x0 + kx, y0 + ky, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* ---- vsync ISR ---- */
static bool vsync_cb(esp_lcd_panel_handle_t panel,
                     const esp_lcd_rgb_panel_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)ctx, &woken);
    return woken == pdTRUE;
}

/* ---- LVGL flush callback: RGB565 → RGB332 with modulo coordinate wrap ---- */
static void lcd_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    const uint16_t *src = (const uint16_t *)(const void *)px_map;
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;

    for (int32_t row = 0; row < h; row++) {
        const uint16_t *line = src + row * w;
        for (int32_t col = 0; col < w; col++) {
            uint16_t c = line[col];
            uint8_t rgb332 = (uint8_t)(((c >> 13) & 0x07u) << 5 |
                                       ((c >>  8) & 0x07u) << 2 |
                                       ((c >>  3) & 0x03u));
#if PANEL_INVERT
            int32_t px = ((LCD_H_RES - 1 - (area->x1 + col)) + PANEL_OFS_X) % LCD_H_RES;
            int32_t py = ((LCD_V_RES - 1 - (area->y1 + row)) + PANEL_OFS_Y) % LCD_V_RES;
#else
            int32_t px = ((area->x1 + col) + PANEL_OFS_X) % LCD_H_RES;
            int32_t py = ((area->y1 + row) + PANEL_OFS_Y) % LCD_V_RES;
#endif
            s_fb[py * LCD_H_RES + px] = rgb332;
        }
    }
    lv_display_flush_ready(disp);
}

/* ---- Touch read callback ---- */
static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    uint8_t reg    = 0x00;
    uint8_t raw[7] = {0};

    esp_err_t err = i2c_master_write_read_device(
        TOUCH_I2C_PORT, HY4633_ADDR,
        &reg, 1, raw, sizeof(raw), pdMS_TO_TICKS(10));

    if (err != ESP_OK || (raw[2] & 0x0F) == 0) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }
    uint8_t event = (raw[3] >> 6) & 0x03;
    if (event == 1 || raw[3] == 0xFF) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    int32_t x = ((int32_t)(raw[3] & 0x0F) << 8) | raw[4];
    int32_t y = ((int32_t)(raw[5] & 0x0F) << 8) | raw[6];
#if PANEL_INVERT
    data->point.x = (lv_coord_t)(LCD_H_RES - 1 - x);
    data->point.y = (lv_coord_t)(LCD_V_RES - 1 - y);
#else
    data->point.x = (lv_coord_t)x;
    data->point.y = (lv_coord_t)y;
#endif
    data->state = LV_INDEV_STATE_PRESSED;
}

static uint32_t tick_get_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ---- LVGL handler task ---- */
static void lvgl_task(void *arg)
{
    for (;;) {
        xSemaphoreTakeRecursive(s_lvgl_mux, portMAX_DELAY);
        uint32_t delay_ms = lv_timer_handler();
        lv_obj_scroll_to(lv_scr_act(), 0, 0, LV_ANIM_OFF);
        xSemaphoreGiveRecursive(s_lvgl_mux);
        if (delay_ms < 1)  delay_ms = 1;
        if (delay_ms > 10) delay_ms = 10;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/* ---- Public API ---- */

void LcdLvgl_Init(void)
{
    s_vsync_sem = xSemaphoreCreateBinary();
    configASSERT(s_vsync_sem);

    esp_lcd_rgb_panel_config_t pc = {
        .clk_src   = LCD_CLK_SRC_PLL160M,
        .timings   = {
            .pclk_hz           = LCD_PCLK_HZ,
            .h_res             = LCD_H_RES,
            .v_res             = LCD_V_RES,
            .hsync_pulse_width = 24, .hsync_back_porch = 160, .hsync_front_porch = 40,
            .vsync_pulse_width =  2, .vsync_back_porch  = 40, .vsync_front_porch  = 20,
            .flags = { .pclk_active_neg = false },
        },
        .data_width            = 8,
        .bits_per_pixel        = 8,
        .num_fbs               = 1,
        .bounce_buffer_size_px = LCD_H_RES * 24,
        .hsync_gpio_num        = LCD_HSYNC,
        .vsync_gpio_num        = LCD_VSYNC,
        .de_gpio_num           = LCD_DE,
        .pclk_gpio_num         = LCD_PCLK,
        .disp_gpio_num         = LCD_DISP,
        .data_gpio_nums        = {
            LCD_DATA0, LCD_DATA1, LCD_DATA2, LCD_DATA3,
            LCD_DATA4, LCD_DATA5, LCD_DATA6, LCD_DATA7,
        },
        .flags = { .fb_in_psram = true, .refresh_on_demand = false },
    };

    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&pc, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));

    esp_lcd_rgb_panel_event_callbacks_t cbs = { .on_vsync = vsync_cb };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(s_panel, &cbs, s_vsync_sem));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(s_panel, 1, (void **)&s_fb));
    configASSERT(s_fb);
    memset(s_fb, 0x00, (size_t)LCD_H_RES * LCD_V_RES);

    xSemaphoreTake(s_vsync_sem, 0);
    xSemaphoreTake(s_vsync_sem, portMAX_DELAY);

    /* Touch I2C */
    i2c_config_t i2c_cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = TOUCH_SDA,
        .scl_io_num       = TOUCH_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(TOUCH_I2C_PORT, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(TOUCH_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    /* LVGL */
    lv_init();
    lv_tick_set_cb(tick_get_ms);

    size_t buf_bytes = (size_t)LCD_H_RES * LVGL_BUF_LINES * sizeof(uint16_t);
    void *draw_buf = heap_caps_malloc(buf_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    configASSERT(draw_buf);

    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_flush_cb(disp, lcd_flush_cb);
    lv_display_set_buffers(disp, draw_buf, NULL, buf_bytes,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    s_touch_indev = lv_indev_create();
    lv_indev_set_type(s_touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(s_touch_indev, touch_read_cb);
    lv_indev_set_scroll_limit(s_touch_indev, 30);

    s_lvgl_mux = xSemaphoreCreateRecursiveMutex();
    configASSERT(s_lvgl_mux);

    backlight_init();
    mcp23017_init();

    ESP_LOGI(TAG, "LCD + LVGL init OK");
}


void LcdLvgl_StartTask(void)
{
    /* Internal SRAM is exhausted by drivers and other tasks.
     * SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y permits PSRAM task stacks.
     * TCB stays in internal DRAM (DRAM_ATTR) as the kernel accesses it frequently. */
    static DRAM_ATTR StaticTask_t s_tcb;
    StackType_t *stack = heap_caps_malloc(8192 * sizeof(StackType_t),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!stack) { ESP_LOGE(TAG, "lvgl stack alloc failed"); return; }
    xTaskCreateStaticPinnedToCore(lvgl_task, "lvgl", 8192, NULL, 5, stack, &s_tcb, 0);
    ESP_LOGI(TAG, "lvgl task created (PSRAM stack, CPU0)");
}

void LcdLvgl_Lock(void)   { xSemaphoreTakeRecursive(s_lvgl_mux, portMAX_DELAY); }
void LcdLvgl_Unlock(void) { xSemaphoreGiveRecursive(s_lvgl_mux); }

void LcdHwButton_SetCallback(void (*cb)(uint8_t mask)) { s_btn_cb = cb; }

void LcdTouch_SetEnabled(bool en)
{
    if (s_touch_indev) lv_indev_enable(s_touch_indev, en);
}
