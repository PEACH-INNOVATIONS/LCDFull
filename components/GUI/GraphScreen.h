#ifndef GRAPH_SCREEN_H
#define GRAPH_SCREEN_H

#include "EndPoints.h"
#include "lcd.h"
#include <math.h>

/* Internal vtable used by GraphApi to dispatch to screens */
typedef struct
{
    const char *title;
    void (*enter)(tBoatRadioEP *pEP, eBoatType boatType);
    void (*exit)(void);
    void (*on_periodic)(tBoatRadioEP *pEP);
    void (*on_stroke)(tBoatRadioEP *pEP);
    void (*on_touch)(int x, int y);   /* raw pixel touch within plot area */
    void (*render)(void);
    /* Called when < or > button is pressed. Return true to consume (rower
     * cycled); return false to let GraphApi navigate screens instead. */
    bool (*on_button)(bool next);
    /* Called on hardware confirm button (right key on BoatSelect, no-op elsewhere). */
    void (*on_confirm)(void);
} tGraphScreenVtable;

/* Helper: find first extraction grid element with given semantic ID */
static inline tExtractionGridElement *GraphScreen_FindElem(tExtractionGrid *grid, uint8_t semId)
{
    for (uint8_t i = 0; i < grid->nGridElemDefined; i++)
        if (grid->extractionGrid[i].semanticId == semId)
            return &grid->extractionGrid[i];
    return NULL;
}

/* Helper: clear plotting flag on all elements of a grid */
static inline void GraphScreen_ClearPlotting(tExtractionGrid *grid)
{
    for (uint8_t i = 0; i < grid->nGridElemDefined; i++)
        grid->extractionGrid[i].plotting = false;
}

/* Helper: set plotting=true on all elements with matching semId */
static inline void GraphScreen_EnableSemId(tExtractionGrid *grid, uint8_t semId)
{
    for (uint8_t i = 0; i < grid->nGridElemDefined; i++)
        if (grid->extractionGrid[i].semanticId == semId)
            grid->extractionGrid[i].plotting = true;
}

/* Crew-wide drive-start tick count (the "catch", i.e. ticks since the stroke
 * callback at which the drive begins): for sweep, the largest non-nan
 * DriveStartSweep value across all rowers; for sculling, the largest
 * per-rower average of port/star DriveStart values across all rowers
 * (a rower only counts if both port and star are non-nan). Returns NAN if
 * no valid data is present. */
static inline float GraphScreen_CrewDriveStartTicks(tExtractionGrid *gridAper, eBoatType bt)
{
    bool  found      = false;
    float best_ticks = 0.0f;

    if (bt == eBoatType_Sweep) {
        for (uint8_t i = 0; i < gridAper->nGridElemDefined; i++) {
            if (gridAper->extractionGrid[i].semanticId != eSIDAPer_DriveStartSweep) continue;
            float v = gridAper->extractionGrid[i].value;
            if (isnan(v)) continue;
            if (!found || v > best_ticks) { best_ticks = v; found = true; }
        }
    } else if (bt == eBoatType_Sculling) {
        float port[N_ROWERS_MAX], star[N_ROWERS_MAX];
        bool  hasPort[N_ROWERS_MAX] = {false};
        bool  hasStar[N_ROWERS_MAX] = {false};

        for (uint8_t i = 0; i < gridAper->nGridElemDefined; i++) {
            uint8_t sid  = gridAper->extractionGrid[i].semanticId;
            uint8_t seat = gridAper->extractionGrid[i].seat;
            if (seat < 1 || seat > N_ROWERS_MAX) continue;
            uint8_t r = seat - 1;
            float v = gridAper->extractionGrid[i].value;
            if (sid == eSIDAPer_DriveStartScullPort)      { port[r] = v; hasPort[r] = !isnan(v); }
            else if (sid == eSIDAPer_DriveStartScullStar) { star[r] = v; hasStar[r] = !isnan(v); }
        }

        for (uint8_t r = 0; r < N_ROWERS_MAX; r++) {
            if (!hasPort[r] || !hasStar[r]) continue;
            float avg = (port[r] + star[r]) * 0.5f;
            if (!found || avg > best_ticks) { best_ticks = avg; found = true; }
        }
    }

    return found ? best_ticks : NAN;
}

#endif /* GRAPH_SCREEN_H */
