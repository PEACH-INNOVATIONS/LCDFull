#ifndef GUI_LVGL_H
#define GUI_LVGL_H

#include "EndPoints.h"

/* Initialise the GUI: load SD config, register EP callbacks, show BoatSelect. */
void GuiLvgl_Start(const char *sd_mount_point);

/* Called from MainTask at 1Hz or faster to poll BoatSelect for new loggers. */
void GuiLvgl_Tick(void);

/* PC sim only: force the SDL backend to blit the framebuffer to the window
 * even when no LVGL widget changed. On real hardware the RGB DMA engine
 * scans the PSRAM framebuffer continuously, so direct-fb writes (graph
 * traces) are visible without any LVGL flush; the SDL window only repaints
 * when LVGL itself flushes a dirty area, so without this, direct-fb writes
 * can sit in the buffer for a long time (or never) before becoming visible. */
void GuiLvgl_ForceFlush(void);

/* Called by BoatSelectLvgl when the operator selects a boat. */
void GuiLvgl_SetSelectedEP(tBoatRadioEP *pEP);

/* Screen navigation — called by bottom-bar button events, or by BoatSelect. */
void GuiLvgl_NextScreen(void);
void GuiLvgl_PrevScreen(void);

/* Called by data screens to update the rower label between < > buttons. */
void GuiLvgl_SetSeatLabel(uint8_t seat_1based, uint8_t n_seats);

/* Hide or show the bottom touch bar visually (stays clickable either way). */
void GuiLvgl_SetBottomBarVisible(bool visible);

#endif /* GUI_LVGL_H */
