#ifndef LCD_H
#define LCD_H

#include "driver/gpio.h"
#include "driver/i2c.h"

#define LCD_H_RES  800
#define LCD_V_RES  480

/* ---- LCD parallel RGB 8-bit GPIO pins ---- */
#define LCD_DATA0   GPIO_NUM_6
#define LCD_DATA1   GPIO_NUM_7
#define LCD_DATA2   GPIO_NUM_18
#define LCD_DATA3   GPIO_NUM_8
#define LCD_DATA4   GPIO_NUM_10
#define LCD_DATA5   GPIO_NUM_11
#define LCD_DATA6   GPIO_NUM_12
#define LCD_DATA7   GPIO_NUM_13
#define LCD_PCLK    GPIO_NUM_9
#define LCD_HSYNC   GPIO_NUM_14
#define LCD_VSYNC   GPIO_NUM_17
#define LCD_DE      GPIO_NUM_21
#define LCD_DISP    GPIO_NUM_39

/* ---- Touch HY4633 (FT6206-compat) I2C ---- */
#define TOUCH_I2C_PORT  I2C_NUM_0
#define TOUCH_SDA       GPIO_NUM_2
#define TOUCH_SCL       GPIO_NUM_1
#define TOUCH_INT       GPIO_NUM_5
#define HY4633_ADDR     0x38

/* ---- Backlight PWM ---- */
#define LCD_BACKLIGHT_GPIO  GPIO_NUM_38

/* Set backlight brightness 0–100 %. Starts at 80 % after LcdLvgl_Init. */
void LcdBacklight_SetPercent(uint8_t pct);

/* Direct framebuffer draw helpers — bypasses LVGL, visible at next panel scan.
 * Coordinates are in LVGL space (0,0 = display top-left). Colors are 24-bit RGB. */
void LcdFb_FillRect(int x, int y, int w, int h, uint32_t rgb888);
void LcdFb_DrawLine(int x0, int y0, int x1, int y1, uint32_t rgb888, int half_w);

/* ---- MCP23017 I2C GPIO expander (hardware buttons, shared I2C bus) ---- */
#define MCP23017_ADDR     0x20
#define MCP23017_INT_GPIO GPIO_NUM_4

/* Initialise hardware and LVGL; creates the LVGL mutex. */
void LcdLvgl_Init(void);

/* Start the LVGL handler task (call after all UI is built). */
void LcdLvgl_StartTask(void);

/* Acquire / release the LVGL mutex.  Must be held for all lv_* calls made
 * from tasks other than the LVGL task itself. */
void LcdLvgl_Lock(void);
void LcdLvgl_Unlock(void);

/* Enable or disable the touch input device (e.g. to prevent phantom gestures
 * from rain/splash).  Default is enabled.  Reads DisableTouch from SD config. */
void LcdTouch_SetEnabled(bool en);

/* Register a callback for MCP23017 hardware button presses.
 * cb(mask): mask has bits {0=left, 1=R-, 2=R+, 3=right} set for newly-pressed keys.
 * Called from the button task with the LVGL mutex held. */
void LcdHwButton_SetCallback(void (*cb)(uint8_t mask));

#endif /* LCD_H */
