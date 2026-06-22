/**
 * LVGL configuration for the LCDFull PC simulator.
 * Only settings that differ from template defaults are listed here.
 */

#ifndef LV_CONF_H
#define LV_CONF_H

/* ---- Colour: full 32-bit on PC, RGB332 on hardware ---- */
#define LV_COLOR_DEPTH 32

/* ---- Memory ---- */
#define LV_USE_STDLIB_MALLOC    LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_STRING    LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_SPRINTF   LV_STDLIB_BUILTIN
#define LV_MEM_SIZE             (512 * 1024U)

/* ---- SDL2 display + input driver ---- */
#define LV_USE_SDL              1
#define LV_SDL_INCLUDE_PATH     <SDL2/SDL.h>
#define LV_SDL_RENDER_MODE      LV_DISPLAY_RENDER_MODE_DIRECT
#define LV_SDL_BUF_COUNT        1

/* ---- Fonts ---- */
#define LV_FONT_MONTSERRAT_24   1
#define LV_FONT_MONTSERRAT_48   1

/* ---- Dark theme ---- */
#define LV_USE_THEME_DEFAULT    1
#define LV_THEME_DEFAULT_DARK   1

/* ---- Logging ---- */
#define LV_USE_LOG              1
#define LV_LOG_LEVEL            LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF           1

#endif /* LV_CONF_H */
