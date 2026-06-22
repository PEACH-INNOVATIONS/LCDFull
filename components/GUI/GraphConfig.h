#ifndef GRAPH_CONFIG_H
#define GRAPH_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "DisplayDriver.h"

/* graphcnf.txt keys (8.3 FAT filename):
 *   bgColour=0               (uint8_t RGB332)
 *   axisColour=255            (uint8_t RGB332)
 *   trace1Colour=224          (uint8_t RGB332, default red)
 *   trace2Colour=28           (uint8_t RGB332, default green)
 *   lineThickness=2           (int, 1-4)
 *   touchEnabled=true
 *   buttonSide=3              (0=left, 1=right, 2=top, 3=bottom)
 *   buttonPct=10.0            (float, % of screen height/width for button strip)
 *   buttonColour=3            (uint8_t RGB332, default blue)
 *   displayOrientation=normal (normal | inverted — 180° rotation)
 */

typedef enum { eSide_Left = 0, eSide_Right, eSide_Top, eSide_Bottom } eButtonSide;

typedef struct
{
    uint8_t     bgColour;
    uint8_t     axisColour;
    uint8_t     trace1Colour;
    uint8_t     trace2Colour;
    uint8_t     buttonColour;
    int         lineThickness;
    bool        touchEnabled;
    eButtonSide buttonSide;
    float       buttonPct;
    bool        displayInverted; /* true = 180° rotation */
} tGraphConfig;

void           GraphConfig_Load(const char *sd_mount_point);
tGraphConfig  *GraphConfig_Get(void);

/* Derived geometry helpers */
void GraphConfig_GetPlotArea(float *x0, float *y0, float *x1, float *y1);
void GraphConfig_GetButtonArea(float *x0, float *y0, float *x1, float *y1);

#endif /* GRAPH_CONFIG_H */
