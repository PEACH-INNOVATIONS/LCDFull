/*
 * sim_pipeline.c  –  PC-simulator serial pipeline for the LCDFull project.
 *
 * Connects to one or more PTYs. Each PTY runs through the full handshake
 * state machine (logger-ID → ping/pong → descriptors → data) and populates
 * its own tBoatRadioEP slot.  Multiple slots are opened when the SIM_PORTS
 * environment variable lists a colon-separated set of paths, e.g.:
 *   SIM_PORTS=/tmp/simport0:/tmp/simport1  ./build/lcdfull_sim
 *
 * Fallback: if SIM_PORTS is not set, the pty_path argument to
 * SimPipeline_Init is opened as a single slot (backward-compatible).
 *
 * Synthetic fleet EPs (IDs 201-204) are always added on top of the real EPs
 * so the Fleet View screen has demo data even with no PTY connected.
 */

#define _DEFAULT_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "EndPoints.h"       /* types: tBoatRadioEP, eEPStatus, eBoatType … */
#include "PacketProcessor.h" /* tExtractionGrid, tExtractionGridElement, tPacketCallbacks … */
#include "PhysicalParams.h"  /* AddPhysicalParameter, GetPhysicalParams* … */
#include "PeachMessages.h"   /* ePeachPacketID, eAperEventType … */
#include "DeSerUtils.h"
#include "SerUtils.h"

/* ═══════════════════════════════════════════════════════════════════════
 * Internal types (from EndPoints.c)
 * ═══════════════════════════════════════════════════════════════════════ */
typedef struct {
    uint16_t semanticID;
    uint8_t  seatPos;
} tBasicId_pl;

/* ═══════════════════════════════════════════════════════════════════════
 * Pipeline state machine
 * ═══════════════════════════════════════════════════════════════════════ */
typedef enum {
    ePL_WAIT_LOGGER_ID,
    ePL_WAIT_PONG,
    ePL_WAIT_DESC_COUNT,
    ePL_WAIT_DESCS,
    ePL_RUNNING
} ePipelineState;

/* ═══════════════════════════════════════════════════════════════════════
 * Packet buffer size
 * ═══════════════════════════════════════════════════════════════════════ */
#define MAX_PKT 260

/* ═══════════════════════════════════════════════════════════════════════
 * Real EP slot — one per open PTY
 * ═══════════════════════════════════════════════════════════════════════ */
#define MAX_REAL_EPS 4

typedef struct {
    tBoatRadioEP    ep;
    uint16_t        logger_id;   /* 0 until AdminMsgTypeLoggerID received */
    bool            active;      /* slot is open and running */
    bool            grids_built;
    ePipelineState  pl_state;
    uint16_t        n_descs;
    uint16_t        desc_idx;
    int             fd;
    uint8_t         pkt_buf[MAX_PKT];
    uint8_t         pkt_ll;      /* LL byte of last received packet */
} tRealEPSlot;

static tRealEPSlot s_slots[MAX_REAL_EPS];
static uint8_t     s_n_real = 0;  /* number of open slots */

static void (*s_graph_periodic_cb)(tBoatRadioEP *) = NULL;
static void (*s_graph_stroke_cb)(tBoatRadioEP *)   = NULL;

/* ---- Synthetic fleet endpoints (always present in the sim) ---- */
#define SIM_FLEET_N         4
#define SIM_FLEET_BASE_ID   201   /* logger IDs 201-204 */
#define FEP_IDX_RATE   0
#define FEP_IDX_SPEED  1

static tBoatRadioEP     s_fleet_eps[SIM_FLEET_N];
static volatile bool    s_fleet_active = false;

static const float s_fleet_speed[SIM_FLEET_N] = { 4.90f, 4.30f, 3.90f, 4.60f };
static const float s_fleet_rate [SIM_FLEET_N] = { 32.0f, 30.0f, 26.0f, 34.0f };
static const uint8_t s_fleet_crew[SIM_FLEET_N] = { 8, 4, 2, 4 };

static void init_fleet_eps(void)
{
    for (uint8_t b = 0; b < SIM_FLEET_N; b++) {
        tBoatRadioEP *ep = &s_fleet_eps[b];
        memset(ep, 0, sizeof(*ep));
        ep->loggerID = (uint16_t)(SIM_FLEET_BASE_ID + b);
        ep->ePStatus = eEPStatus_readyForData;

        uint8_t nr = s_fleet_crew[b];
        for (uint8_t r = 0; r < nr && r < N_ROWERS_MAX; r++)
            ep->descriptors_05_12.Affinities.affinity[r] =
                (r % 2 == 0) ? eAffinities_Starboard_sweep : eAffinities_Port_sweep;

        tExtractionGrid *ga = &ep->ExtractionGridAPeriodic;
        ga->nGridElemDefined = 2;
        ga->extractionGrid[FEP_IDX_RATE].semanticId  = eSIDAPer_Rating;
        ga->extractionGrid[FEP_IDX_RATE].seat        = 0;
        ga->extractionGrid[FEP_IDX_RATE].plotting    = false;
        ga->extractionGrid[FEP_IDX_RATE].value       = NAN;
        ga->extractionGrid[FEP_IDX_SPEED].semanticId = eSIDAPer_AvgBoatSpeed;
        ga->extractionGrid[FEP_IDX_SPEED].seat       = 0;
        ga->extractionGrid[FEP_IDX_SPEED].plotting   = false;
        ga->extractionGrid[FEP_IDX_SPEED].value      = NAN;
    }
}

static void *fleet_thread(void *arg)
{
    (void)arg;
    uint32_t tick = 0;
    /* Short initial pause so the GUI window has time to render before first stroke */
    usleep(800000);
    while (s_fleet_active) {
        for (uint8_t b = 0; b < SIM_FLEET_N; b++) {
            if (!s_fleet_active) break;
            tBoatRadioEP *ep = &s_fleet_eps[b];

            float spd = s_fleet_speed[b]
                        + sinf((float)tick * 0.35f + (float)b * 1.1f) * 0.18f;
            float rat = s_fleet_rate[b]
                        + sinf((float)tick * 0.25f + (float)b * 2.0f) * 1.5f;

            tExtractionGridElement *ge = ep->ExtractionGridAPeriodic.extractionGrid;
            if (ge[FEP_IDX_RATE].plotting)  ge[FEP_IDX_RATE].value  = rat;
            if (ge[FEP_IDX_SPEED].plotting) ge[FEP_IDX_SPEED].value = spd;

            if (s_graph_stroke_cb) s_graph_stroke_cb(ep);

            usleep(350000);
        }
        tick++;
        sleep(3);
    }
    return NULL;
}

static tPacketCallbacks s_pkt_cbs;   /* kept but unused in sim */

/* ═══════════════════════════════════════════════════════════════════════
 * EndPoints public API  (replaces EndPoints.c)
 * ═══════════════════════════════════════════════════════════════════════ */

void EP_SetGraphPeriodicCallback(void (*cb)(tBoatRadioEP *)) { s_graph_periodic_cb = cb; }
void EP_SetGraphStrokeCallback  (void (*cb)(tBoatRadioEP *)) { s_graph_stroke_cb   = cb; }

uint8_t GetNumValidLoggers(void)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < s_n_real; i++)
        if (s_slots[i].logger_id) n++;
    return n + SIM_FLEET_N;
}

uint16_t GetLoggerIDFromIndex(uint8_t idx)
{
    /* Only count slots that have received a logger ID */
    uint8_t ri = 0;
    for (uint8_t i = 0; i < s_n_real; i++) {
        if (!s_slots[i].logger_id) continue;
        if (ri == idx) return s_slots[i].logger_id;
        ri++;
    }
    uint8_t fi = idx - ri;
    if (fi < SIM_FLEET_N) return s_fleet_eps[fi].loggerID;
    return 0;
}

tBoatRadioEP *GetpEPFromLoggerID(uint16_t id)
{
    for (uint8_t i = 0; i < s_n_real; i++)
        if (s_slots[i].logger_id == id) return &s_slots[i].ep;
    for (uint8_t i = 0; i < SIM_FLEET_N; i++)
        if (s_fleet_eps[i].loggerID == id) return &s_fleet_eps[i];
    return NULL;
}

eEPStatus GetEPStatus(uint16_t id)
{
    tBoatRadioEP *ep = GetpEPFromLoggerID(id);
    return ep ? ep->ePStatus : eEPStatus_initial;
}

uint8_t GetEPIndexFromLoggerID(uint16_t id)
{
    uint8_t ri = 0;
    for (uint8_t i = 0; i < s_n_real; i++) {
        if (!s_slots[i].logger_id) continue;
        if (s_slots[i].logger_id == id) return ri;
        ri++;
    }
    for (uint8_t i = 0; i < SIM_FLEET_N; i++)
        if (s_fleet_eps[i].loggerID == id) return ri + i;
    return 0xFF;
}

bool InitEPs(void) { return true; }

void InitializeAllBoatRadioEP(void)
{
    for (uint8_t i = 0; i < s_n_real; i++)
        memset(&s_slots[i].ep, 0, sizeof(s_slots[i].ep));
    s_n_real = 0;
}

void SetEndpointUpdating(uint16_t id) { (void)id; }
void SetHoldAtResponsive(bool h)      { (void)h;  }

uint8_t GetResponsiveLoggerIDs(uint16_t *ids, uint8_t max)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < s_n_real && n < max; i++) {
        if (!s_slots[i].logger_id) continue;
        if (ids) ids[n] = s_slots[i].logger_id;
        n++;
    }
    for (uint8_t i = 0; i < SIM_FLEET_N && n < max; i++) {
        if (ids) ids[n] = s_fleet_eps[i].loggerID;
        n++;
    }
    return n;
}

eBoatType GetBoatType(uint16_t id)
{
    tBoatRadioEP *ep = GetpEPFromLoggerID(id);
    if (!ep) return eBoatType_Unknown;
    for (int i = 0; i < N_ROWERS_MAX; i++) {
        eAffinities a = ep->descriptors_05_12.Affinities.affinity[i];
        if (a == eAffinities_Sculling)                                       return eBoatType_Sculling;
        if (a == eAffinities_Starboard_sweep || a == eAffinities_Port_sweep) return eBoatType_Sweep;
    }
    return eBoatType_Unknown;
}

bool GetPongReceived(void) { return false; }

/* ═══════════════════════════════════════════════════════════════════════
 * PacketProcessor stubs  (pure-C functions from PacketProcessor.c)
 * ═══════════════════════════════════════════════════════════════════════ */

void RegisterPacketCallbacks(tPacketCallbacks cbs) { s_pkt_cbs = cbs; }

void DecodeDescriptorPacket(tParseableParameter *pp, uint8_t *buf, uint8_t seatPos)
{
    uint8_t *p = buf;
    p = DeSerialiseU8 (&pp->evId,                           p);
    p = DeSerialiseU16(&pp->seatedParams[seatPos].index,    p);
    p += 2;   /* skip semanticID */
    p += 1;   /* skip seatPos   */
    p = DeSerialiseS8 (&pp->gain,   p);
    p = DeSerialiseS16(&pp->offset, p);
    pp->seatedParams[seatPos].foundIn0304Packet = true;
    pp->nRowers++;
}

/* build_grid: constructs an extraction grid from all physical params that
 * belong to the given logger ID and have received their 0x03/0x04 descriptor. */
static void build_grid(tParseableParameter *tbl, uint8_t n, tExtractionGrid *g, uint16_t lid)
{
    for (uint8_t i = 0; i < n; i++) {
        if (tbl[i].loggerID != lid) continue;
        if (tbl[i].seated) {
            for (uint8_t s = 1; s <= N_ROWERS_MAX; s++) {
                if (g->nGridElemDefined >= MAX_GRID_ELEMENTS) break;
                if (!tbl[i].seatedParams[s].foundIn0304Packet) continue;
                uint8_t idx = g->nGridElemDefined++;
                g->extractionGrid[idx].index               = tbl[i].seatedParams[s].index;
                g->extractionGrid[idx].seat                = s;
                g->extractionGrid[idx].plotting            = false;
                g->extractionGrid[idx].semanticId          = tbl[i].semanticID;
                g->extractionGrid[idx].pParseableParameter = &tbl[i];
            }
        } else {
            if (g->nGridElemDefined >= MAX_GRID_ELEMENTS) continue;
            if (!tbl[i].seatedParams[0].foundIn0304Packet) continue;
            uint8_t idx = g->nGridElemDefined++;
            g->extractionGrid[idx].index               = tbl[i].seatedParams[0].index;
            g->extractionGrid[idx].seat                = 0;
            g->extractionGrid[idx].plotting            = false;
            g->extractionGrid[idx].semanticId          = tbl[i].semanticID;
            g->extractionGrid[idx].pParseableParameter = &tbl[i];
        }
    }
}

void ConstructExtractionGrids(tExtractionGrid *per, tExtractionGrid *aper)
{
    /* Kept for ABI compat; uses first ready slot when called externally */
    uint16_t lid = 0;
    for (uint8_t i = 0; i < s_n_real; i++) {
        if (s_slots[i].logger_id) { lid = s_slots[i].logger_id; break; }
    }
    if (!lid) return;
    build_grid(GetPhysicalParamsPeriodic(),  (uint8_t)Get_N_PhysicalParametersPeriodic(),  per,  lid);
    build_grid(GetPhysicalParamsAperiodic(), (uint8_t)Get_N_PhysicalParametersAperiodic(), aper, lid);
}

/* ConstuctExtractionGrids (note intentional typo matching EndPoints.c) */
void ConstuctExtractionGrids(void)
{
    /* Mark all slot grids dirty so they rebuild on next callback */
    for (uint8_t i = 0; i < s_n_real; i++)
        s_slots[i].grids_built = false;
}

QueueHandle_t GetSerialMessageQueue(void) { return NULL; }

bool  PktProcessorInit(void)                                       { return true; }
void  InsertPacketIntoSystem(uint8_t *b, uint8_t l, uint16_t id)   { (void)b; (void)l; (void)id; }

/* ═══════════════════════════════════════════════════════════════════════
 * Descriptor processing  (per-slot)
 * ═══════════════════════════════════════════════════════════════════════ */

static tBasicId_pl get_basic_id(uint8_t *buf)
{
    tBasicId_pl id = {0};
    DeSerialiseU16(&id.semanticID, &buf[SEMANTIC_ID_POS_IN_0304_PACKET]);
    DeSerialiseU8 (&id.seatPos,    &buf[SEAT_POSITION_POS_IN_0304_PACKET]);
    return id;
}

static void try_periodic_descriptor(tRealEPSlot *sl, uint8_t *buf)
{
    tParseableParameter *pp = GetPhysicalParamsPeriodic();
    size_t n = Get_N_PhysicalParametersPeriodic();
    tBasicId_pl bid = get_basic_id(buf);
    bool matched = false;
    for (size_t i = 0; i < n; i++) {
        if (pp[i].seatedParams[bid.seatPos].foundIn0304Packet) continue;
        if (pp[i].semanticID != bid.semanticID)                continue;
        if (pp[i].loggerID   != sl->logger_id)                 continue;
        pp[i].foundInDesc = true;
        DecodeDescriptorPacket(&pp[i], buf, bid.seatPos);
        sl->grids_built = false;
        matched = true;
    }
    printf("sim_pipeline[%u]: PER  descriptor semId=%u seat=%u -> %s\n",
           (unsigned)(sl - s_slots), bid.semanticID, bid.seatPos,
           matched ? "matched" : "NO MATCH (not registered)");
}

static void try_aperiodic_descriptor(tRealEPSlot *sl, uint8_t *buf)
{
    tParseableParameter *pp = GetPhysicalParamsAperiodic();
    size_t n = Get_N_PhysicalParametersAperiodic();
    tBasicId_pl bid = get_basic_id(buf);
    bool matched = false;
    for (size_t i = 0; i < n; i++) {
        if (pp[i].seatedParams[bid.seatPos].foundIn0304Packet) continue;
        if (pp[i].semanticID != bid.semanticID)                continue;
        if (pp[i].loggerID   != sl->logger_id)                 continue;
        pp[i].foundInDesc = true;
        DecodeDescriptorPacket(&pp[i], buf, bid.seatPos);
        sl->grids_built = false;
        matched = true;
    }
    printf("sim_pipeline[%u]: APER descriptor semId=%u seat=%u -> %s\n",
           (unsigned)(sl - s_slots), bid.semanticID, bid.seatPos,
           matched ? "matched" : "NO MATCH (not registered)");
}

static void process_descriptor_slot(tRealEPSlot *sl, ePeachPacketID msgID, uint8_t *pBuff)
{
    switch (msgID) {
        case eDataInPacketType_Periodic_Descriptor:  try_periodic_descriptor(sl, pBuff);   break;
        case eDataInPacketType_Aperiodic_Descriptor: try_aperiodic_descriptor(sl, pBuff);  break;
        case eDataInPacketType_ShortStatus:
            DecodeShortStatusPacket(pBuff, &sl->ep.descriptors_05_12);
            break;
        case eDataInPacketType_ParameterValue:
            DeserialiseLoggerParameters(&sl->ep.descriptors_05_12.LoggerParameters, pBuff);
            break;
        case eDataInPacketType_SessionHeader:
            DeserialiseSessionHeader(&sl->ep.descriptors_05_12.SessionHeader, pBuff);
            printf("sim_pipeline[%u]: session %lu logger %lu\n",
                   (unsigned)(sl - s_slots),
                   (unsigned long)sl->ep.descriptors_05_12.SessionHeader.sessionNumber,
                   (unsigned long)sl->ep.descriptors_05_12.SessionHeader.loggerNumber);
            break;
        case eDataInPacketType_SensorVersion:
            DeserialiseSensorVersion(&sl->ep.descriptors_05_12.SensorVersion, pBuff);
            break;
        default:
            break;
    }
}

/* Public wrapper: finds the owning slot and dispatches */
void ProcessDescriptorPacket(tBoatRadioEP *pEP, ePeachPacketID msgID, uint8_t *pBuff)
{
    for (uint8_t i = 0; i < s_n_real; i++) {
        if (&s_slots[i].ep == pEP) {
            process_descriptor_slot(&s_slots[i], msgID, pBuff);
            return;
        }
    }
}

bool ProcessMessageForEP(tAdminMsgUnion *msg) { (void)msg; return false; }

/* ═══════════════════════════════════════════════════════════════════════
 * Data-extraction callbacks  (adapted from EndPoints.c)
 * ═══════════════════════════════════════════════════════════════════════ */

static float scale_u16(uint16_t x, int8_t gain, int16_t offset)
{
    x &= 0x3FFF;
    if (x == 0) return NAN;
    int16_t sx = (int16_t)x + offset;
    float r = (float)sx;
    if      (gain >= 0) r *= (float)(1 << gain);
    else                r /= (float)(1 << (-gain));
    return r;
}

static tRealEPSlot *slot_from_ep(tBoatRadioEP *ep)
{
    for (uint8_t i = 0; i < s_n_real; i++)
        if (&s_slots[i].ep == ep) return &s_slots[i];
    return NULL;
}

static void ensure_grids_slot(tRealEPSlot *sl)
{
    if (sl->grids_built) return;
    memset(&sl->ep.ExtractionGridPeriodic,  0, sizeof(sl->ep.ExtractionGridPeriodic));
    memset(&sl->ep.ExtractionGridAPeriodic, 0, sizeof(sl->ep.ExtractionGridAPeriodic));
    build_grid(GetPhysicalParamsPeriodic(),  (uint8_t)Get_N_PhysicalParametersPeriodic(),
               &sl->ep.ExtractionGridPeriodic,  sl->logger_id);
    build_grid(GetPhysicalParamsAperiodic(), (uint8_t)Get_N_PhysicalParametersAperiodic(),
               &sl->ep.ExtractionGridAPeriodic, sl->logger_id);
    sl->grids_built = true;
    printf("sim_pipeline[%u]: extraction grids built (per=%u aper=%u)\n",
           (unsigned)(sl - s_slots),
           sl->ep.ExtractionGridPeriodic.nGridElemDefined,
           sl->ep.ExtractionGridAPeriodic.nGridElemDefined);
    tExtractionGrid *gp = &sl->ep.ExtractionGridPeriodic;
    for (uint8_t i = 0; i < gp->nGridElemDefined; i++)
        printf("  per[%u] semId=%u seat=%u idx=%u\n",
               i, gp->extractionGrid[i].semanticId,
               gp->extractionGrid[i].seat, gp->extractionGrid[i].index);
}

void EndpointCallbackPeriodic(uint8_t *pBuff, tBoatRadioEP *pEp)
{
    tRealEPSlot *sl = slot_from_ep(pEp);
    if (sl) ensure_grids_slot(sl);

    uint8_t n = pEp->ExtractionGridPeriodic.nGridElemDefined;
    tExtractionGridElement *g = pEp->ExtractionGridPeriodic.extractionGrid;
    static uint32_t s_call_count = 0;
    bool dbg = (s_call_count % 50) == 0;
    uint8_t nPlotting = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (!g[i].plotting) continue;
        nPlotting++;
        uint16_t off = (uint16_t)(4 + DATA_STARTS_AT_PERIODIC + g[i].index * 2u);
        uint16_t v = 0;
        DeSerialiseU16(&v, pBuff + off);
        g[i].value = scale_u16(v, g[i].pParseableParameter->gain,
                                  g[i].pParseableParameter->offset);
        if (dbg)
            printf("  per-extract[%u] semId=%u seat=%u idx=%u off=%u raw=%04X value=%f\n",
                   i, g[i].semanticId, g[i].seat, g[i].index, off, v, (double)g[i].value);
    }
    if (dbg)
        printf("sim_pipeline: EndpointCallbackPeriodic call#%u nGridElem=%u nPlotting=%u\n",
               s_call_count, n, nPlotting);
    s_call_count++;
    if (s_graph_periodic_cb) s_graph_periodic_cb(pEp);
}

void EndpointCallbackStroke(uint8_t *pBuff, tBoatRadioEP *pEp)
{
    tRealEPSlot *sl = slot_from_ep(pEp);
    if (sl) ensure_grids_slot(sl);

    uint8_t n = pEp->ExtractionGridAPeriodic.nGridElemDefined;
    tExtractionGridElement *g = pEp->ExtractionGridAPeriodic.extractionGrid;
    for (uint8_t i = 0; i < n; i++) {
        if (!g[i].plotting) continue;
        uint16_t off = (uint16_t)(4 + DATA_STARTS_AT_APERIODIC + g[i].index * 2u);
        uint16_t v = 0;
        DeSerialiseU16(&v, pBuff + off);
        g[i].value = scale_u16(v, g[i].pParseableParameter->gain,
                                  g[i].pParseableParameter->offset);
    }
    if (s_graph_stroke_cb) s_graph_stroke_cb(pEp);
}

void EndpointCallbackTick1s(uint8_t *b, tBoatRadioEP *ep) { (void)b; (void)ep; }

/* ═══════════════════════════════════════════════════════════════════════
 * MessageProcessing stubs
 * ═══════════════════════════════════════════════════════════════════════ */

bool InitMessageProcessing(void)   { return true; }
void SendRatioResetTo(eEntity to, uint8_t ep, eResetAction act) { (void)to; (void)ep; (void)act; }
eEntity GetRole(void)              { return eEntityCoachDisplay; }

/* ═══════════════════════════════════════════════════════════════════════
 * Wire helpers (per-slot)
 * ═══════════════════════════════════════════════════════════════════════ */

static const uint8_t s_sync[2] = {0xE3, 0x5A};
const uint8_t *GetSyncSeq(void) { return s_sync; }

static bool serial_write_slot(tRealEPSlot *sl, const uint8_t *data, size_t len)
{
    if (sl->fd < 0) return false;
    size_t sent = 0;
    while (sent < len) {
        ssize_t r = write(sl->fd, data + sent, len - sent);
        if (r <= 0) return false;
        sent += (size_t)r;
    }
    return true;
}

static bool send_packet_slot(tRealEPSlot *sl, uint8_t id, const uint8_t *body, uint8_t body_len)
{
    uint8_t ll = (uint8_t)(3 + body_len);
    uint8_t hdr[4] = {0xE3, 0x5A, ll, id};
    if (!serial_write_slot(sl, hdr, 4)) return false;
    if (body_len && !serial_write_slot(sl, body, body_len)) return false;
    return serial_write_slot(sl, &ll, 1);
}

static bool send_ping_to_pic(tRealEPSlot *sl)
{
    uint8_t body[] = {
        (uint8_t)eEntityPIC,
        (uint8_t)AdminMsgTypePing,
        (uint8_t)eEntityCoachDisplay,
        (uint8_t)eEntityPIC
    };
    return send_packet_slot(sl, 0xA0, body, sizeof(body));
}

static bool send_get_descriptor_number(tRealEPSlot *sl)
{
    uint8_t body[] = {
        (uint8_t)eEntityPIC,
        (uint8_t)AdminMsgTypeGetDescriptorNumber
    };
    return send_packet_slot(sl, 0xA0, body, sizeof(body));
}

static bool send_get_descriptor(tRealEPSlot *sl, uint16_t idx)
{
    uint8_t body[] = {
        (uint8_t)eEntityPIC,
        (uint8_t)AdminMsgTypeGetDescriptor,
        (uint8_t)(idx & 0xFF),
        (uint8_t)(idx >> 8)
    };
    return send_packet_slot(sl, 0xA0, body, sizeof(body));
}

static bool send_radio_status_sending_data(tRealEPSlot *sl)
{
    uint8_t body[] = {
        (uint8_t)eEntityPIC,
        (uint8_t)AdminMsgTypeRadioStatus,
        7   /* eBoatRadioStatus_SendingData */
    };
    return send_packet_slot(sl, 0xA0, body, sizeof(body));
}

/* TxSerialMessage: routes to first active slot */
bool TxSerialMessage(uint8_t *msg, uint16_t len, uint16_t destID)
{
    (void)destID;
    for (uint8_t i = 0; i < s_n_real; i++) {
        if (s_slots[i].fd >= 0) {
            serial_write_slot(&s_slots[i], s_sync, 2);
            return serial_write_slot(&s_slots[i], msg, len);
        }
    }
    return false;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Packet framer  (per-slot, blocking read)
 * ═══════════════════════════════════════════════════════════════════════ */

static bool read_byte_slot(tRealEPSlot *sl, uint8_t *b)
{
    ssize_t r = read(sl->fd, b, 1);
    return r == 1;
}

static int read_one_packet_slot(tRealEPSlot *sl)
{
    uint8_t b;
    for (;;) {
        if (!read_byte_slot(sl, &b)) return -1;
        if (b != 0xE3) continue;
        if (!read_byte_slot(sl, &b)) return -1;
        if (b == 0x5A) break;
    }
    uint8_t ll;
    if (!read_byte_slot(sl, &ll) || ll < 3) return -1;
    sl->pkt_ll = ll;
    uint8_t need = (uint8_t)(ll - 2);   /* id(1) + body_content(ll-3) */
    if (need > MAX_PKT - 1) return -1;
    size_t got = 0;
    while (got < need) {
        ssize_t r = read(sl->fd, sl->pkt_buf + got, need - got);
        if (r <= 0) return -1;
        got += (size_t)r;
    }
    uint8_t tail;
    if (!read_byte_slot(sl, &tail)) return -1;
    if (tail != ll)
        printf("sim_pipeline[%u]: tail LL mismatch (%02X vs %02X)\n",
               (unsigned)(sl - s_slots), tail, ll);
    return (int)need;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Coaching-param registration
 * ═══════════════════════════════════════════════════════════════════════ */

static void register_coaching_params(uint16_t id)
{
    /* Mirrors RegisterCoachingParams() in main/app_main.c — keep in sync. */

    static const uint8_t periodicSeatedIDs[] = { eSIDPer_SweepAngle, eSIDPer_SweepForceX,
                                                  eSIDPer_ScullPortForceX, eSIDPer_ScullStarForceX,
                                                  9, 10 };
    for (uint8_t si = 0; si < sizeof(periodicSeatedIDs); si++)
        AddPhysicalParameter(id, periodicSeatedIDs[si], true, ePerOrAper_Periodic, NULL);

    AddPhysicalParameter(id, eSIDPer_Acceleration, false, ePerOrAper_Periodic, NULL);

    AddPhysicalParameter(id, eSIDAPer_Rating,         false, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(id, eSIDAPer_AvgBoatSpeed,   false, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(id, eSIDAPer_SweepPower,     true,  ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(id, eSIDAPer_ScullPowerPort, true,  ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(id, eSIDAPer_ScullPowerStar, true,  ePerOrAper_Aperiodic, NULL);

    AddPhysicalParameter(id, eSIDAPer_DriveStartSweep,     true, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(id, eSIDAPer_DriveStartScullPort, true, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(id, eSIDAPer_DriveStartScullStar, true, ePerOrAper_Aperiodic, NULL);

    static const uint8_t arcLengthIDs[] = {
        eSIDAPer_MinScullAnglePort, eSIDAPer_MinScullAngleStar, eSIDAPer_MinSweepAngle,
        eSIDAPer_MaxScullAnglePort, eSIDAPer_MaxScullAngleStar, eSIDAPer_MaxSweepAngle,
        eSIDAPer_CatchSlipScullPort, eSIDAPer_CatchSlipScullStar, eSIDAPer_CatchSlipSweep,
        eSIDAPer_FinishSlipScullPort, eSIDAPer_FinishSlipScullStar, eSIDAPer_FinishSlipSweep,
    };
    for (uint8_t si = 0; si < sizeof(arcLengthIDs); si++)
        AddPhysicalParameter(id, arcLengthIDs[si], true, ePerOrAper_Aperiodic, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Admin-packet handling  (per-slot)
 * ═══════════════════════════════════════════════════════════════════════ */

static void handle_admin_slot(tRealEPSlot *sl, int body_total)
{
    if (body_total < 3) return;
    uint8_t  msg_type = sl->pkt_buf[2];
    uint8_t *payload  = sl->pkt_buf + 3;
    int      pay_len  = body_total - 3;
    uint8_t  sidx     = (uint8_t)(sl - s_slots);

    switch ((eAdminMsgType)msg_type) {

        case AdminMsgTypeLoggerID: {
            if (pay_len < 3) break;
            uint16_t lid = 0;
            DeSerialiseU16(&lid, payload);
            if (!sl->logger_id) {
                sl->logger_id = lid;
                memset(&sl->ep, 0, sizeof(sl->ep));
                sl->ep.loggerID = lid;
                sl->ep.ePStatus = eEPStatus_notSeen;
                register_coaching_params(lid);
                printf("sim_pipeline[%u]: logger ID %u – sending Ping\n", sidx, lid);
                send_ping_to_pic(sl);
                sl->ep.ePStatus = eEPStatus_sentPing;
                sl->pl_state    = ePL_WAIT_PONG;
            }
            break;
        }

        case AdminMsgTypePong: {
            if (sl->pl_state == ePL_WAIT_PONG) {
                printf("sim_pipeline[%u]: Pong received – requesting descriptors\n", sidx);
                sl->ep.ePStatus = eEPStatus_responsive;
                send_get_descriptor_number(sl);
                sl->pl_state = ePL_WAIT_DESC_COUNT;
            }
            break;
        }

        case AdminMsgTypeGetDescriptorNumberResponse: {
            if (sl->pl_state == ePL_WAIT_DESC_COUNT && pay_len >= 2) {
                DeSerialiseU16(&sl->n_descs, payload);
                printf("sim_pipeline[%u]: %u descriptor(s) available\n", sidx, sl->n_descs);
                if (sl->n_descs == 0) {
                    send_radio_status_sending_data(sl);
                    sl->pl_state = ePL_RUNNING;
                } else {
                    sl->desc_idx = 0;
                    send_get_descriptor(sl, 0);
                    sl->pl_state = ePL_WAIT_DESCS;
                }
            }
            break;
        }

        case AdminMsgTypeGetDescriptorResponse: {
            if (sl->pl_state == ePL_WAIT_DESCS && pay_len >= 3) {
                uint8_t desc_len   = payload[2];
                if (pay_len >= 3 + (int)desc_len && desc_len >= 3) {
                    uint8_t desc_id       = payload[4];
                    uint8_t *body_content = payload + 5;
                    process_descriptor_slot(sl, (ePeachPacketID)desc_id, body_content);
                }
                sl->desc_idx++;
                if (sl->desc_idx < sl->n_descs) {
                    send_get_descriptor(sl, sl->desc_idx);
                } else {
                    printf("sim_pipeline[%u]: all %u descriptor(s) received – sending RadioStatus\n",
                           sidx, sl->n_descs);
                    send_radio_status_sending_data(sl);
                    sl->ep.ePStatus = eEPStatus_got_all_descriptors;
                    sl->pl_state    = ePL_RUNNING;
                    PrintAllPhysicalParams();
                }
            }
            break;
        }

        default:
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Per-slot reader thread
 * ═══════════════════════════════════════════════════════════════════════ */

static void *pipeline_thread(void *arg)
{
    tRealEPSlot *sl   = (tRealEPSlot *)arg;
    uint8_t      sidx = (uint8_t)(sl - s_slots);
    printf("sim_pipeline[%u]: reader thread started (fd=%d)\n", sidx, sl->fd);

    for (;;) {
        int n = read_one_packet_slot(sl);
        if (n < 0) {
            printf("sim_pipeline[%u]: read error – exiting thread\n", sidx);
            break;
        }
        if (n < 1) continue;

        uint8_t  id      = sl->pkt_buf[0];
        uint8_t *content = sl->pkt_buf + 1;
        int      clen    = n - 1;

        switch ((ePeachPacketID)id) {

            /* ── Admin (0xA0) ─────────────────────────────────────── */
            case eDataInPacketType_RadioSubsystemMessage:
                handle_admin_slot(sl, n);
                break;

            /* ── Periodic data (0x01) ─────────────────────────────── */
            case eDataInPacketType_Periodic: {
                if (sl->pl_state != ePL_RUNNING) break;
                uint8_t compat[4 + MAX_PKT];
                compat[0] = 0xE3; compat[1] = 0x5A;
                compat[2] = sl->pkt_ll; compat[3] = id;
                if (clen > 0) memcpy(compat + 4, content, (size_t)clen);
                if (sl->ep.ePStatus < eEPStatus_readyForData)
                    sl->ep.ePStatus = eEPStatus_readyForData;
                EndpointCallbackPeriodic(compat, &sl->ep);
                break;
            }

            /* ── Aperiodic / stroke (0x02) ────────────────────────── */
            case eDataInPacketType_Aperiodic: {
                if (sl->pl_state != ePL_RUNNING || clen < 1) break;
                uint8_t evid = content[0];
                if (evid != (uint8_t)eAperEventType_STROKE) break;
                uint8_t compat[4 + MAX_PKT];
                compat[0] = 0xE3; compat[1] = 0x5A;
                compat[2] = sl->pkt_ll; compat[3] = id;
                if (clen > 0) memcpy(compat + 4, content, (size_t)clen);
                EndpointCallbackStroke(compat, &sl->ep);
                break;
            }

            /* ── Descriptor packets ───────────────────────────────── */
            case eDataInPacketType_Periodic_Descriptor:
            case eDataInPacketType_Aperiodic_Descriptor:
            case eDataInPacketType_ShortStatus:
            case eDataInPacketType_ParameterValue:
            case eDataInPacketType_SessionHeader:
            case eDataInPacketType_SensorVersion:
                process_descriptor_slot(sl, (ePeachPacketID)id, content);
                break;

            default:
                break;
        }
    }
    return NULL;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Open a single PTY as a new slot
 * ═══════════════════════════════════════════════════════════════════════ */

static bool open_slot(const char *pty_path)
{
    if (s_n_real >= MAX_REAL_EPS) {
        fprintf(stderr, "sim_pipeline: slot limit (%d) reached\n", MAX_REAL_EPS);
        return false;
    }
    tRealEPSlot *sl = &s_slots[s_n_real];
    memset(sl, 0, sizeof(*sl));
    sl->fd = open(pty_path, O_RDWR | O_NOCTTY);
    if (sl->fd < 0) {
        perror(pty_path);
        printf("sim_pipeline: WARNING – could not open %s\n", pty_path);
        return false;
    }

    struct termios t;
    tcgetattr(sl->fd, &t);
    cfmakeraw(&t);
    cfsetispeed(&t, B460800);
    cfsetospeed(&t, B460800);
    tcsetattr(sl->fd, TCSANOW, &t);

    sl->pl_state = ePL_WAIT_LOGGER_ID;
    sl->active   = true;
    s_n_real++;

    pthread_t tid;
    pthread_create(&tid, NULL, pipeline_thread, sl);
    pthread_detach(tid);
    printf("sim_pipeline: opened %s (fd=%d, slot %u)\n", pty_path, sl->fd, s_n_real - 1u);
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Public init function
 * ═══════════════════════════════════════════════════════════════════════ */

bool SimPipeline_Init(const char *pty_path)
{
    /* Always start the synthetic fleet EPs */
    init_fleet_eps();
    s_fleet_active = true;
    pthread_t fleet_tid;
    pthread_create(&fleet_tid, NULL, fleet_thread, NULL);
    pthread_detach(fleet_tid);

    /* Multi-EP: SIM_PORTS=path0:path1:... overrides the single pty_path arg */
    const char *ports_env = getenv("SIM_PORTS");
    if (ports_env && ports_env[0]) {
        char buf[1024];
        strncpy(buf, ports_env, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        char *p = buf;
        while (p && *p && s_n_real < MAX_REAL_EPS) {
            char *end = strchr(p, ':');
            if (end) *end = '\0';
            if (*p) open_slot(p);
            p = end ? end + 1 : NULL;
        }
    } else if (pty_path) {
        open_slot(pty_path);
    }

    if (s_n_real == 0)
        printf("sim_pipeline: WARNING – no serial ports opened; synthetic fleet data only\n");

    return s_n_real > 0;
}
