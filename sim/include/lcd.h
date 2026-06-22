#ifndef LCD_H_SIM
#define LCD_H_SIM
/* Sim override: provides resolution constants and no-op mutex calls */
#define LCD_H_RES  800
#define LCD_V_RES  480

void LcdLvgl_Lock(void);
void LcdLvgl_Unlock(void);
void LcdHwButton_SetCallback(void (*cb)(uint8_t mask));
void LcdTouch_SetEnabled(bool en);
void LcdBacklight_SetPercent(uint8_t pct);
void LcdFb_FillRect(int x, int y, int w, int h, uint32_t rgb888);
void LcdFb_DrawLine(int x0, int y0, int x1, int y1, uint32_t rgb888, int half_w);
#endif
