#include "GraphConfig.h"
#include "FileSystemSD.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static tGraphConfig s_cfg = {
    .bgColour        = COLOUR_BLACK,
    .axisColour      = COLOUR_WHITE,
    .trace1Colour    = COLOUR_RED,
    .trace2Colour    = COLOUR_GREEN,
    .buttonColour    = COLOUR_BLUE,
    .lineThickness   = 2,
    .touchEnabled    = true,
    .buttonSide      = eSide_Bottom,
    .buttonPct       = 10.0f,
    .displayInverted = false,
};

static void apply_kv(const char *key, const char *val)
{
    if      (strcmp(key, "bgColour")      == 0) s_cfg.bgColour      = (uint8_t)atoi(val);
    else if (strcmp(key, "axisColour")    == 0) s_cfg.axisColour    = (uint8_t)atoi(val);
    else if (strcmp(key, "trace1Colour")  == 0) s_cfg.trace1Colour  = (uint8_t)atoi(val);
    else if (strcmp(key, "trace2Colour")  == 0) s_cfg.trace2Colour  = (uint8_t)atoi(val);
    else if (strcmp(key, "lineThickness") == 0) s_cfg.lineThickness = atoi(val);
    else if (strcmp(key, "touchEnabled")  == 0) s_cfg.touchEnabled  = (strcmp(val, "true") == 0);
    else if (strcmp(key, "buttonSide")         == 0) s_cfg.buttonSide      = (eButtonSide)atoi(val);
    else if (strcmp(key, "buttonPct")          == 0) s_cfg.buttonPct       = (float)atof(val);
    else if (strcmp(key, "buttonColour")       == 0) s_cfg.buttonColour    = (uint8_t)atoi(val);
    else if (strcmp(key, "displayOrientation") == 0) s_cfg.displayInverted = (strcmp(val, "inverted") == 0);
}

void GraphConfig_Load(const char *sd_mount_point)
{
    char path[64];
    snprintf(path, sizeof(path), "%s/graphcnf.txt", sd_mount_point);
    FILE *f = fopen(path, "r");
    if (!f) return;

    char line[80];
    while (fgets(line, sizeof(line), f))
    {
        char *eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        char *val = eq + 1;
        /* strip trailing newline */
        val[strcspn(val, "\r\n")] = '\0';
        apply_kv(line, val);
    }
    fclose(f);
}

tGraphConfig *GraphConfig_Get(void) { return &s_cfg; }

void GraphConfig_GetPlotArea(float *x0, float *y0, float *x1, float *y1)
{
    float pct = s_cfg.buttonPct;
    switch (s_cfg.buttonSide)
    {
        case eSide_Left:   *x0 = pct;  *y0 = 0.0f; *x1 = 100.0f; *y1 = 100.0f; break;
        case eSide_Right:  *x0 = 0.0f; *y0 = 0.0f; *x1 = 100.0f - pct; *y1 = 100.0f; break;
        case eSide_Top:    *x0 = 0.0f; *y0 = 0.0f; *x1 = 100.0f; *y1 = 100.0f - pct; break;
        case eSide_Bottom:
        default:           *x0 = 0.0f; *y0 = pct;  *x1 = 100.0f; *y1 = 100.0f; break;
    }
}

void GraphConfig_GetButtonArea(float *x0, float *y0, float *x1, float *y1)
{
    float pct = s_cfg.buttonPct;
    switch (s_cfg.buttonSide)
    {
        case eSide_Left:   *x0 = 0.0f; *y0 = 0.0f; *x1 = pct;  *y1 = 100.0f; break;
        case eSide_Right:  *x0 = 100.0f - pct; *y0 = 0.0f; *x1 = 100.0f; *y1 = 100.0f; break;
        case eSide_Top:    *x0 = 0.0f; *y0 = 100.0f - pct; *x1 = 100.0f; *y1 = 100.0f; break;
        case eSide_Bottom:
        default:           *x0 = 0.0f; *y0 = 0.0f; *x1 = 100.0f; *y1 = pct; break;
    }
}
