/*
 * sim_pipeline.c  –  PC-simulator serial pipeline for the LCDFull project.
 *
 * Connects to a PTY created by:
 *   socat PTY,link=/tmp/fakepic,rawer PTY,link=/tmp/simport,rawer
 *
 * then tells FakePIC we are ready, receives the packet stream, processes
 * descriptors and data, and drives the LVGL GUI through the registered
 * EP callbacks.
 *
 * Wire format (FakePIC ↔ sim, no loggerID prefix):
 *   E3 5A | LL | id | body[LL-3] | LL
 *
 * This file also provides ALL EndPoints / PacketProcessor / MessageProcessing
 * functions that the GUI and the rest of the sim reference, replacing the
 * stubs that were previously in sim_stubs.c.
 *
 * Usage (main_pc.c):
 *   SimPipeline_Init("/tmp/simport");   // call once; starts reader thread
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
 * Static endpoint state  (one EP for the sim)
 * ═══════════════════════════════════════════════════════════════════════ */
static tBoatRadioEP s_ep;
static uint8_t      s_n_loggers  = 0;
static uint16_t     s_logger_id  = 0;
static bool         s_grids_built = false;

static void (*s_graph_periodic_cb)(tBoatRadioEP *) = NULL;
static void (*s_graph_stroke_cb)(tBoatRadioEP *)   = NULL;

static tPacketCallbacks s_pkt_cbs;   /* kept but unused in sim */

/* ═══════════════════════════════════════════════════════════════════════
 * Serial state
 * ═══════════════════════════════════════════════════════════════════════ */
static int s_fd = -1;

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
static ePipelineState s_pl_state  = ePL_WAIT_LOGGER_ID;
static uint16_t       s_n_descs   = 0;
static uint16_t       s_desc_idx  = 0;

/* ═══════════════════════════════════════════════════════════════════════
 * EndPoints public API  (replaces EndPoints.c)
 * ═══════════════════════════════════════════════════════════════════════ */

void EP_SetGraphPeriodicCallback(void (*cb)(tBoatRadioEP *)) { s_graph_periodic_cb = cb; }
void EP_SetGraphStrokeCallback  (void (*cb)(tBoatRadioEP *)) { s_graph_stroke_cb   = cb; }

uint8_t      GetNumValidLoggers(void)                              { return s_n_loggers; }
uint16_t     GetLoggerIDFromIndex(uint8_t idx)                     { (void)idx; return s_logger_id; }
tBoatRadioEP *GetpEPFromLoggerID(uint16_t id)                      { return (id == s_logger_id && s_n_loggers) ? &s_ep : NULL; }
eEPStatus    GetEPStatus(uint16_t id)                              { return GetpEPFromLoggerID(id) ? s_ep.ePStatus : eEPStatus_initial; }
uint8_t      GetEPIndexFromLoggerID(uint16_t id)                   { return (id == s_logger_id && s_n_loggers) ? 0 : 0xFF; }
bool         InitEPs(void)                                         { return true; }
void         InitializeAllBoatRadioEP(void)                        { memset(&s_ep, 0, sizeof(s_ep)); s_n_loggers = 0; }
void         SetEndpointUpdating(uint16_t id)                      { (void)id; }
void         SetHoldAtResponsive(bool h)                           { (void)h; }

uint8_t GetResponsiveLoggerIDs(uint16_t *ids, uint8_t max)
{
    if (s_n_loggers && ids && max) { ids[0] = s_logger_id; return 1; }
    return 0;
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

static void build_grid(tParseableParameter *tbl, uint8_t n, tExtractionGrid *g)
{
    for (uint8_t i = 0; i < n; i++) {
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
    build_grid(GetPhysicalParamsPeriodic(),  (uint8_t)Get_N_PhysicalParametersPeriodic(),  per);
    build_grid(GetPhysicalParamsAperiodic(), (uint8_t)Get_N_PhysicalParametersAperiodic(), aper);
}

/* ConstuctExtractionGrids (note intentional typo matching EndPoints.c) */
void ConstuctExtractionGrids(void)
{
    ConstructExtractionGrids(&s_ep.ExtractionGridPeriodic,
                              &s_ep.ExtractionGridAPeriodic);
}

QueueHandle_t GetSerialMessageQueue(void) { return NULL; }

bool  PktProcessorInit(void)                                       { return true; }
void  InsertPacketIntoSystem(uint8_t *b, uint8_t l, uint16_t id)   { (void)b; (void)l; (void)id; }

/* ═══════════════════════════════════════════════════════════════════════
 * Descriptor processing  (adapted from EndPoints.c)
 * ═══════════════════════════════════════════════════════════════════════ */

static tBasicId_pl get_basic_id(uint8_t *buf)
{
    tBasicId_pl id = {0};
    DeSerialiseU16(&id.semanticID, &buf[SEMANTIC_ID_POS_IN_0304_PACKET]);
    DeSerialiseU8 (&id.seatPos,    &buf[SEAT_POSITION_POS_IN_0304_PACKET]);
    return id;
}

static void try_periodic_descriptor(uint8_t *buf)
{
    tParseableParameter *pp = GetPhysicalParamsPeriodic();
    size_t n = Get_N_PhysicalParametersPeriodic();
    tBasicId_pl bid = get_basic_id(buf);
    bool matched = false;
    for (size_t i = 0; i < n; i++) {
        if (pp[i].seatedParams[bid.seatPos].foundIn0304Packet) continue;
        if (pp[i].semanticID != bid.semanticID)                continue;
        if (pp[i].loggerID   != s_logger_id)                   continue;
        pp[i].foundInDesc = true;
        DecodeDescriptorPacket(&pp[i], buf, bid.seatPos);
        s_grids_built = false;   /* invalidate; rebuild before next use */
        matched = true;
    }
    printf("sim_pipeline: PER  descriptor semId=%u seat=%u -> %s\n",
           bid.semanticID, bid.seatPos, matched ? "matched" : "NO MATCH (not registered)");
}

static void try_aperiodic_descriptor(uint8_t *buf)
{
    tParseableParameter *pp = GetPhysicalParamsAperiodic();
    size_t n = Get_N_PhysicalParametersAperiodic();
    tBasicId_pl bid = get_basic_id(buf);
    bool matched = false;
    for (size_t i = 0; i < n; i++) {
        if (pp[i].seatedParams[bid.seatPos].foundIn0304Packet) continue;
        if (pp[i].semanticID != bid.semanticID)                continue;
        if (pp[i].loggerID   != s_logger_id)                   continue;
        pp[i].foundInDesc = true;
        DecodeDescriptorPacket(&pp[i], buf, bid.seatPos);
        s_grids_built = false;
        matched = true;
    }
    printf("sim_pipeline: APER descriptor semId=%u seat=%u -> %s\n",
           bid.semanticID, bid.seatPos, matched ? "matched" : "NO MATCH (not registered)");
}

/* pBuff points to BODY content (everything after the id byte) */
void ProcessDescriptorPacket(tBoatRadioEP *pEP, ePeachPacketID msgID, uint8_t *pBuff)
{
    (void)pEP;
    switch (msgID) {
        case eDataInPacketType_Periodic_Descriptor:  try_periodic_descriptor(pBuff);  break;
        case eDataInPacketType_Aperiodic_Descriptor: try_aperiodic_descriptor(pBuff); break;
        case eDataInPacketType_ShortStatus:
            DecodeShortStatusPacket(pBuff, &s_ep.descriptors_05_12);
            break;
        case eDataInPacketType_ParameterValue:
            DeserialiseLoggerParameters(&s_ep.descriptors_05_12.LoggerParameters, pBuff);
            break;
        case eDataInPacketType_SessionHeader:
            DeserialiseSessionHeader(&s_ep.descriptors_05_12.SessionHeader, pBuff);
            printf("sim_pipeline: session %lu logger %lu\n",
                   (unsigned long)s_ep.descriptors_05_12.SessionHeader.sessionNumber,
                   (unsigned long)s_ep.descriptors_05_12.SessionHeader.loggerNumber);
            break;
        case eDataInPacketType_SensorVersion:
            DeserialiseSensorVersion(&s_ep.descriptors_05_12.SensorVersion, pBuff);
            break;
        default:
            break;
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

static void ensure_grids(void)
{
    if (s_grids_built) return;
    memset(&s_ep.ExtractionGridPeriodic,  0, sizeof(s_ep.ExtractionGridPeriodic));
    memset(&s_ep.ExtractionGridAPeriodic, 0, sizeof(s_ep.ExtractionGridAPeriodic));
    ConstructExtractionGrids(&s_ep.ExtractionGridPeriodic,
                              &s_ep.ExtractionGridAPeriodic);
    s_grids_built = true;
    printf("sim_pipeline: extraction grids built (per=%u aper=%u)\n",
           s_ep.ExtractionGridPeriodic.nGridElemDefined,
           s_ep.ExtractionGridAPeriodic.nGridElemDefined);

    /* Dump grid so we can verify seat numbering */
    tExtractionGrid *gp = &s_ep.ExtractionGridPeriodic;
    for (uint8_t i = 0; i < gp->nGridElemDefined; i++)
        printf("  per[%u] semId=%u seat=%u idx=%u\n",
               i, gp->extractionGrid[i].semanticId,
               gp->extractionGrid[i].seat, gp->extractionGrid[i].index);
}

/* pBuff = full compat buffer: [E3, 5A, LL, id, body...] */
void EndpointCallbackPeriodic(uint8_t *pBuff, tBoatRadioEP *pEp)
{
    ensure_grids();
    uint8_t n = pEp->ExtractionGridPeriodic.nGridElemDefined;
    tExtractionGridElement *g = pEp->ExtractionGridPeriodic.extractionGrid;
    static uint32_t s_call_count = 0;
    bool dbg = (s_call_count % 50) == 0;   /* ~1/sec at 50 Hz periodic rate */
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

/* pBuff = full compat buffer: [E3, 5A, LL, id, body...] */
void EndpointCallbackStroke(uint8_t *pBuff, tBoatRadioEP *pEp)
{
    ensure_grids();
    uint8_t n = pEp->ExtractionGridAPeriodic.nGridElemDefined;
    tExtractionGridElement *g = pEp->ExtractionGridAPeriodic.extractionGrid;
    for (uint8_t i = 0; i < n; i++) {
        if (!g[i].plotting) continue;
        uint16_t off = (uint16_t)(4 + DATA_STARTS_AT_APERIODIC
                                  + g[i].index * 2u);
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
 * Wire helpers
 * ═══════════════════════════════════════════════════════════════════════ */

static const uint8_t s_sync[2] = {0xE3, 0x5A};
const uint8_t *GetSyncSeq(void) { return s_sync; }

static bool serial_write(const uint8_t *data, size_t len)
{
    if (s_fd < 0) return false;
    size_t sent = 0;
    while (sent < len) {
        ssize_t r = write(s_fd, data + sent, len - sent);
        if (r <= 0) return false;
        sent += (size_t)r;
    }
    return true;
}

/* Send a complete framed packet: E3 5A | LL | id | body[body_len] | LL */
static bool send_packet(uint8_t id, const uint8_t *body, uint8_t body_len)
{
    uint8_t ll = (uint8_t)(3 + body_len);
    uint8_t hdr[4] = {0xE3, 0x5A, ll, id};
    if (!serial_write(hdr, 4)) return false;
    if (body_len && !serial_write(body, body_len)) return false;
    return serial_write(&ll, 1);
}

/* ── Admin packet builders ────────────────────────────────────────────── */

static bool send_ping_to_pic(void)
{
    uint8_t body[] = {
        (uint8_t)eEntityPIC,
        (uint8_t)AdminMsgTypePing,
        (uint8_t)eEntityCoachDisplay,
        (uint8_t)eEntityPIC
    };
    return send_packet(0xA0, body, sizeof(body));
}

static bool send_get_descriptor_number(void)
{
    uint8_t body[] = {
        (uint8_t)eEntityPIC,
        (uint8_t)AdminMsgTypeGetDescriptorNumber
    };
    return send_packet(0xA0, body, sizeof(body));
}

static bool send_get_descriptor(uint16_t idx)
{
    uint8_t body[] = {
        (uint8_t)eEntityPIC,
        (uint8_t)AdminMsgTypeGetDescriptor,
        (uint8_t)(idx & 0xFF),
        (uint8_t)(idx >> 8)
    };
    return send_packet(0xA0, body, sizeof(body));
}

static bool send_radio_status_sending_data(void)
{
    uint8_t body[] = {
        (uint8_t)eEntityPIC,
        (uint8_t)AdminMsgTypeRadioStatus,
        7   /* eBoatRadioStatus_SendingData */
    };
    return send_packet(0xA0, body, sizeof(body));
}

/* TxSerialMessage: called by nothing in the sim (SendAdminMessageTo not used),
 * but declared in PacketProcessor.h – provide a no-op. */
bool TxSerialMessage(uint8_t *msg, uint16_t len, uint16_t destID)
{
    (void)destID;
    if (s_fd < 0 || !msg || !len) return false;
    /* prepend E3 5A and write */
    serial_write(s_sync, 2);
    return serial_write(msg, len);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Packet framer  (blocking read from PTY)
 * ═══════════════════════════════════════════════════════════════════════ */

#define MAX_PKT 260

/* Filled by read_one_packet; [0]=id, [1..n-1]=body_content */
static uint8_t  s_pkt_buf[MAX_PKT];
static uint8_t  s_pkt_ll;       /* saved LL for compat-buffer building */

static bool read_byte(uint8_t *b)
{
    ssize_t r = read(s_fd, b, 1);
    return r == 1;
}

/* Returns body_len (= LL-2, which is 1 id + (LL-3) content), or -1 on error. */
static int read_one_packet(void)
{
    uint8_t b;
    for (;;) {
        if (!read_byte(&b)) return -1;
        if (b != 0xE3) continue;
        if (!read_byte(&b)) return -1;
        if (b == 0x5A) break;
    }
    uint8_t ll;
    if (!read_byte(&ll) || ll < 3) return -1;
    s_pkt_ll = ll;
    uint8_t need = (uint8_t)(ll - 2);   /* id(1) + body_content(ll-3) */
    if (need > MAX_PKT - 1) return -1;
    size_t got = 0;
    while (got < need) {
        ssize_t r = read(s_fd, s_pkt_buf + got, need - got);
        if (r <= 0) return -1;
        got += (size_t)r;
    }
    uint8_t tail;
    if (!read_byte(&tail)) return -1;
    if (tail != ll)
        printf("sim_pipeline: tail LL mismatch (%02X vs %02X)\n", tail, ll);
    return (int)need;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Coaching-param registration
 * ═══════════════════════════════════════════════════════════════════════ */

static void register_coaching_params(uint16_t id)
{
    /* Mirrors RegisterCoachingParams() in main/app_main.c — keep in sync. */

    /* Periodic seated: sweep angle (2), sweep force (4), scull forces (11,12),
     * scull port/star angle (9,10 — seated=true disambiguates these from the
     * non-seated GPS speed/acceleration meanings of the same IDs) */
    static const uint8_t periodicSeatedIDs[] = { eSIDPer_SweepAngle, eSIDPer_SweepForceX,
                                                  eSIDPer_ScullPortForceX, eSIDPer_ScullStarForceX,
                                                  9, 10 };
    for (uint8_t si = 0; si < sizeof(periodicSeatedIDs); si++)
        AddPhysicalParameter(id, periodicSeatedIDs[si], true, ePerOrAper_Periodic, NULL);

    /* Periodic non-seated: boat acceleration (3), for the Accel vs Time screen */
    AddPhysicalParameter(id, eSIDPer_Acceleration, false, ePerOrAper_Periodic, NULL);

    /* Aperiodic non-seated: rating (0), avg speed (1) */
    AddPhysicalParameter(id, eSIDAPer_Rating, false, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(id, eSIDAPer_AvgBoatSpeed, false, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(id, eSIDAPer_SweepPower, true, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(id, eSIDAPer_ScullPowerPort, true, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(id, eSIDAPer_ScullPowerStar, true, ePerOrAper_Aperiodic, NULL);

    /* Aperiodic seated: drive start sweep (22) and scull port/star (20/21),
     * used for the crew-wide drive-start marker on the Angle vs Time screen */
    AddPhysicalParameter(id, eSIDAPer_DriveStartSweep,     true, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(id, eSIDAPer_DriveStartScullPort, true, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(id, eSIDAPer_DriveStartScullStar, true, ePerOrAper_Aperiodic, NULL);

    /* Aperiodic seated: min/max angle and catch/finish slip, for the Arc Length screen */
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
 * Admin-packet handling
 * ═══════════════════════════════════════════════════════════════════════ */

/* s_pkt_buf = [id=0xA0, entity, msgType, payload...]
 * body_total = number of bytes stored in s_pkt_buf (id + entity + msgType + payload)
 */
static void handle_admin(int body_total)
{
    if (body_total < 3) return;
    /* s_pkt_buf: [0]=0xA0(id), [1]=entity, [2]=msgType, [3..]=payload */
    uint8_t  msg_type = s_pkt_buf[2];
    uint8_t *payload  = s_pkt_buf + 3;
    int      pay_len  = body_total - 3;

    switch ((eAdminMsgType)msg_type) {

        case AdminMsgTypeLoggerID: {
            if (pay_len < 3) break;
            uint16_t lid = 0;
            DeSerialiseU16(&lid, payload);
            if (s_n_loggers == 0) {
                s_logger_id = lid;
                memset(&s_ep, 0, sizeof(s_ep));
                s_ep.loggerID = lid;
                s_ep.ePStatus = eEPStatus_notSeen;
                s_n_loggers   = 1;
                register_coaching_params(lid);
                printf("sim_pipeline: logger ID %u – sending Ping\n", lid);
                send_ping_to_pic();
                s_ep.ePStatus = eEPStatus_sentPing;
                s_pl_state    = ePL_WAIT_PONG;
            }
            break;
        }

        case AdminMsgTypePong: {
            if (s_pl_state == ePL_WAIT_PONG) {
                printf("sim_pipeline: Pong received – requesting descriptors\n");
                s_ep.ePStatus = eEPStatus_responsive;
                send_get_descriptor_number();
                s_pl_state = ePL_WAIT_DESC_COUNT;
            }
            break;
        }

        case AdminMsgTypeGetDescriptorNumberResponse: {
            if (s_pl_state == ePL_WAIT_DESC_COUNT && pay_len >= 2) {
                DeSerialiseU16(&s_n_descs, payload);
                printf("sim_pipeline: %u descriptor(s) available\n", s_n_descs);
                if (s_n_descs == 0) {
                    send_radio_status_sending_data();
                    s_pl_state = ePL_RUNNING;
                } else {
                    s_desc_idx = 0;
                    send_get_descriptor(0);
                    s_pl_state = ePL_WAIT_DESCS;
                }
            }
            break;
        }

        case AdminMsgTypeGetDescriptorResponse: {
            if (s_pl_state == ePL_WAIT_DESCS && pay_len >= 3) {
                /* payload: [msgIdx(2LE), descLen(1), desc_data(descLen)]
                 * desc_data = LL | id | body_content | LL  (sync stripped) */
                uint8_t desc_len = payload[2];
                if (pay_len >= 3 + (int)desc_len && desc_len >= 3) {
                    uint8_t desc_id      = payload[4];          /* desc_data[1] */
                    uint8_t *body_content = payload + 5;        /* desc_data[2..] */
                    ProcessDescriptorPacket(&s_ep, (ePeachPacketID)desc_id,
                                            body_content);
                }
                s_desc_idx++;
                if (s_desc_idx < s_n_descs) {
                    send_get_descriptor(s_desc_idx);
                } else {
                    printf("sim_pipeline: all %u descriptor(s) received – sending RadioStatus\n",
                           s_n_descs);
                    send_radio_status_sending_data();
                    s_ep.ePStatus = eEPStatus_got_all_descriptors;
                    s_pl_state    = ePL_RUNNING;
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
 * Dispatch loop  (runs in a pthread)
 * ═══════════════════════════════════════════════════════════════════════ */

static void *pipeline_thread(void *arg)
{
    (void)arg;
    printf("sim_pipeline: reader thread started (fd=%d)\n", s_fd);

    for (;;) {
        int n = read_one_packet();
        if (n < 0) {
            printf("sim_pipeline: read error – exiting thread\n");
            break;
        }
        if (n < 1) continue;

        uint8_t id       = s_pkt_buf[0];
        uint8_t *content = s_pkt_buf + 1;   /* body content after id */
        int      clen    = n - 1;            /* content bytes */

        switch ((ePeachPacketID)id) {

            /* ── Admin (0xA0) ─────────────────────────────────────── */
            case eDataInPacketType_RadioSubsystemMessage:
                handle_admin(n);   /* n = id + entity + msgType + payload */
                break;

            /* ── Periodic data (0x01) ─────────────────────────────── */
            case eDataInPacketType_Periodic: {
                if (s_pl_state != ePL_RUNNING) break;
                /* Build compat buffer expected by EndpointCallbackPeriodic:
                 *   [E3, 5A, LL, id, body_content...]
                 * Data index formula: 4 + DATA_STARTS_AT_PERIODIC + i*2 = 9+i*2
                 * In this layout: [2]=LL, [3]=id, [4..]=content → data at [9] ✓ */
                uint8_t compat[4 + MAX_PKT];
                compat[0] = 0xE3;
                compat[1] = 0x5A;
                compat[2] = s_pkt_ll;
                compat[3] = id;
                if (clen > 0) memcpy(compat + 4, content, (size_t)clen);

                if (s_ep.ePStatus < eEPStatus_readyForData)
                    s_ep.ePStatus = eEPStatus_readyForData;

                EndpointCallbackPeriodic(compat, &s_ep);
                break;
            }

            /* ── Aperiodic / stroke (0x02) ────────────────────────── */
            case eDataInPacketType_Aperiodic: {
                if (s_pl_state != ePL_RUNNING || clen < 1) break;
                uint8_t evid = content[0];
                if (evid != (uint8_t)eAperEventType_STROKE) break;

                uint8_t compat[4 + MAX_PKT];
                compat[0] = 0xE3;
                compat[1] = 0x5A;
                compat[2] = s_pkt_ll;
                compat[3] = id;
                if (clen > 0) memcpy(compat + 4, content, (size_t)clen);

                EndpointCallbackStroke(compat, &s_ep);
                break;
            }

            /* ── Descriptor packets ───────────────────────────────── */
            case eDataInPacketType_Periodic_Descriptor:
            case eDataInPacketType_Aperiodic_Descriptor:
            case eDataInPacketType_ShortStatus:
            case eDataInPacketType_ParameterValue:
            case eDataInPacketType_SessionHeader:
            case eDataInPacketType_SensorVersion: {
                /* content = body content after id; descriptor functions
                 * expect a pointer to the body content (not the full packet) */
                ProcessDescriptorPacket(&s_ep, (ePeachPacketID)id,
                                        content);
                break;
            }

            default:
                break;
        }
    }
    return NULL;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Public init function
 * ═══════════════════════════════════════════════════════════════════════ */

bool SimPipeline_Init(const char *pty_path)
{
    s_fd = open(pty_path, O_RDWR | O_NOCTTY);
    if (s_fd < 0) {
        perror(pty_path);
        printf("sim_pipeline: WARNING – no serial port; synthetic data only\n");
        return false;
    }

    /* Raw mode – baud doesn't matter for PTY but set it for real ports */
    struct termios t;
    tcgetattr(s_fd, &t);
    cfmakeraw(&t);
    cfsetispeed(&t, B460800);
    cfsetospeed(&t, B460800);
    tcsetattr(s_fd, TCSANOW, &t);

    pthread_t tid;
    pthread_create(&tid, NULL, pipeline_thread, NULL);
    pthread_detach(tid);

    printf("sim_pipeline: opened %s  (fd=%d)\n", pty_path, s_fd);
    return true;
}
