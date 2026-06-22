/**
 * PC simulator stubs for LCDFull.
 *
 * Provides no-op or minimal implementations of platform functions that the
 * GUI components call but that do not have a natural PC equivalent.
 *
 * EP / PacketProcessor / MessageProcessing API is provided by sim_pipeline.c.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <dirent.h>
#include <pthread.h>

#include "FileConfig.h"         /* tConfigValue, eConfigParameterName */

/* ================================================================
 * LCD mutex — recursive so LVGL button callbacks can re-acquire
 * from within lv_timer_handler (which the main loop holds locked).
 * Pipeline thread blocks on LcdLvgl_Lock() until rendering finishes.
 * ================================================================ */
static pthread_mutex_t     s_lvgl_mutex;
static pthread_mutexattr_t s_lvgl_mutex_attr;

__attribute__((constructor)) static void lvgl_mutex_init(void)
{
    pthread_mutexattr_init(&s_lvgl_mutex_attr);
    pthread_mutexattr_settype(&s_lvgl_mutex_attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&s_lvgl_mutex, &s_lvgl_mutex_attr);
}

void LcdLvgl_Lock(void)   { pthread_mutex_lock(&s_lvgl_mutex); }
void LcdLvgl_Unlock(void) { pthread_mutex_unlock(&s_lvgl_mutex); }

/* No hardware buttons in sim — touch panel bottom bar is used instead */
void LcdHwButton_SetCallback(void (*cb)(uint8_t mask)) { (void)cb; }
void LcdTouch_SetEnabled(bool en) { (void)en; }
void LcdBacklight_SetPercent(uint8_t pct) { (void)pct; }

/* Direct framebuffer helpers (LcdFb_FillRect/LcdFb_DrawLine) are implemented
 * in main_pc.c, where the SDL display's draw buffer is in scope. */

/* ================================================================
 * FreeRTOS tick stub
 * ================================================================ */
uint32_t xTaskGetTickCount(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + (uint32_t)(ts.tv_nsec / 1000000u));
}

/* ================================================================
 * FileConfig stubs — return safe default values
 * ================================================================ */
bool InitFileConfig(void)  { return true; }
bool ParseConfigFile(char *p) { (void)p; return false; }
bool IsConfigParamSet(eConfigParameterName p) { (void)p; return false; }

tConfigValue GetConfigValue(eConfigParameterName p)
{
    tConfigValue v = {0};
    switch (p) {
        case eConfigParam_FvAngle_GraphAngleMin_Deg:    v.fVal = -70.0f; break;
        case eConfigParam_FvAngle_GraphAngleMax_Deg:    v.fVal =  40.0f; break;
        case eConfigParam_FvAngle_GraphFMin_KgF:        v.fVal = -10.0f; break;
        case eConfigParam_FvAngle_GraphFMax_KgF:        v.fVal = 140.0f; break;
        case eConfigParam_HSVsAngle_GraphAngleMin_Deg:  v.fVal = -60.0f; break;
        case eConfigParam_HSVsAngle_GraphAngleMax_Deg:  v.fVal =  60.0f; break;
        case eConfigParam_HSVsAngle_GraphSMin_Deg_S:    v.fVal = -300.0f; break;
        case eConfigParam_HSVsAngle_GraphSMax_Deg_S:    v.fVal =  300.0f; break;
        default: v.fVal = 0.0f; break;
    }
    return v;
}

/* ================================================================
 * ShowDir stub — called by GraphConfig.c on PC to list config files
 * ================================================================ */
void ShowDir(const char *path)
{
    DIR *d = opendir(path);
    if (!d) { printf("ShowDir: cannot open %s\n", path); return; }
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        printf("  %s\n", ent->d_name);
    }
    closedir(d);
}
