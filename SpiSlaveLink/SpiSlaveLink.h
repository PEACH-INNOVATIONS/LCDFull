/*
 * SpiSlaveLink.h — ESP32-S3 (Coach Display) SPI slave interface to Coach Radio.
 *
 * The CD shares the SD-card SPI bus (SPI2_HOST) but operates as slave after
 * the SD card is released at start-up.  The CR (Newracom NRC7394) is master.
 *
 * GPIO assignments (ESP32-S3 side):
 *   CS   in  : GPIO48  ← asserted by CR (Newracom GPIO28)
 *   INT  out : GPIO46  → monitored by CR (Newracom GPIO14)
 *   MOSI in  : GPIO40  (shared with SD card)
 *   MISO out : GPIO42  (shared with SD card)
 *   SCLK in  : GPIO41  (shared with SD card)
 *
 * Frame format (260 bytes, full-duplex — same layout in both directions):
 *   [0]     Magic  0xA5
 *   [1]     Flags  0x00 (reserved)
 *   [2-3]   Length big-endian (bytes of valid payload; 0 = no message)
 *   [4-259] Payload: raw Peach message bytes (LoggerID + packet body)
 *             zero-padded to fill the frame
 *
 * No sync bytes, no STX/ETX — SPI CS assertion provides frame boundaries.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

/* ── GPIO ───────────────────────────────────────────────────────────────── */
#define SPI_LINK_CD_CS_GPIO   GPIO_NUM_48   /* slave-select input from CR    */
#define SPI_LINK_CD_INT_GPIO  GPIO_NUM_46   /* interrupt output to CR        */

/* ── Frame geometry ─────────────────────────────────────────────────────── */
#define SPI_LINK_FRAME_SIZE   260           /* total bytes per transaction    */
#define SPI_LINK_PAYLOAD_MAX  256           /* usable payload bytes (4..259)  */
#define SPI_LINK_FRAME_MAGIC  0xA5u

/* ── RX queue item ──────────────────────────────────────────────────────── */
typedef struct {
    uint16_t len;
    uint8_t  data[SPI_LINK_PAYLOAD_MAX];
} SpiLinkMsg_t;

/* ── Public API ─────────────────────────────────────────────────────────── */

/*
 * SpiSlaveLink_Init — configure GPIO46 as INT output, initialise SPI2 as
 * slave, and start the background task.
 *
 * Call AFTER the SD card has been unmounted and spi_bus_free(SPI2_HOST)
 * has been called — the bus must be free when this is called.
 *
 * Returns true on success.
 */
bool SpiSlaveLink_Init(void);

/*
 * SpiSlaveLink_Transmit — enqueue a payload to send to the CR on the next
 * SPI transaction.  len must be <= SPI_LINK_PAYLOAD_MAX.
 *
 * Raises the INT line immediately so the CR knows data is waiting.
 * Thread-safe; may be called from any task.
 *
 * Returns true if the message was accepted into the TX queue.
 */
bool SpiSlaveLink_Transmit(const uint8_t *payload, uint16_t len);

/*
 * SpiSlaveLink_GetRxQueue — returns the FreeRTOS queue on which
 * SpiLinkMsg_t items are posted for every valid packet received from the CR.
 */
QueueHandle_t SpiSlaveLink_GetRxQueue(void);

/*
 * Bus-sharing helpers — needed only if the SD card must be accessed again
 * after SpiSlaveLink_Init().  Calling code must bracket SD operations:
 *
 *   SpiSlaveLink_SuspendForSD();
 *   // ... spi_bus_initialize / SD mount / read / unmount / spi_bus_free ...
 *   SpiSlaveLink_ResumeAfterSD();
 */
void SpiSlaveLink_SuspendForSD(void);
void SpiSlaveLink_ResumeAfterSD(void);
