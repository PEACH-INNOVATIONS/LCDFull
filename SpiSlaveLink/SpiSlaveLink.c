#include "SpiSlaveLink.h"
#include "DebugConsole.h"

#include <string.h>
#include <stdio.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "app_main.h"     /* SD_CARD_PIN_MOSI/MISO/SCLK */
#include "SerUtils.h"

/* ── Internal constants ─────────────────────────────────────────────────── */

#define SPI_HOST            SPI2_HOST
#define SPI_TX_QUEUE_DEPTH  8
#define SPI_RX_QUEUE_DEPTH  8
#define SLAVE_TASK_STACK    4096
#define SLAVE_TASK_PRIORITY 10

/* ── Frame layout offsets ───────────────────────────────────────────────── */
#define FRAME_OFF_MAGIC   0
#define FRAME_OFF_FLAGS   1
#define FRAME_OFF_LEN_H   2
#define FRAME_OFF_LEN_L   3
#define FRAME_OFF_PAYLOAD 4

/* ── Static state ───────────────────────────────────────────────────────── */

/*
 * DMA-capable, 4-byte-aligned buffers in internal SRAM.
 * Two fixed frames are alternated so the task always has one ready to
 * queue while the other is in flight.
 */
static uint8_t s_tx_frame[SPI_LINK_FRAME_SIZE] __attribute__((aligned(4)));
static uint8_t s_rx_frame[SPI_LINK_FRAME_SIZE] __attribute__((aligned(4)));
/* Completed RX is copied here so the DMA can be re-armed before processing. */
static uint8_t s_rx_copy[SPI_LINK_FRAME_SIZE]  __attribute__((aligned(4)));

static QueueHandle_t s_rx_queue = NULL;
static QueueHandle_t s_tx_queue = NULL;

/*
 * Tracks whether the frame currently queued for transmission contains real
 * data.  Used to keep INT asserted until the CR has clocked out that frame,
 * even if the TX software queue has already been drained.
 */
static bool s_queued_frame_has_data = false;

/*
 * True while a transaction is queued in the slave DMA (between
 * spi_slave_queue_trans() and spi_slave_get_trans_result() returning).
 * SpiSlaveLink_Transmit() checks this flag after queuing a message: if the
 * DMA is already armed it calls update_int() immediately so the CR sees INT
 * go high without waiting for the next loop iteration.  Must be set BEFORE
 * update_int() in slave_task so that any concurrent Transmit() call that
 * misses update_int() but sees the flag will still raise INT.
 */
static volatile bool s_dma_armed = false;

/* ── INT line helpers ───────────────────────────────────────────────────── */

static void int_deassert(void) { gpio_set_level(SPI_LINK_CD_INT_GPIO, 0); }

/*
 * Refresh INT: high when the queued frame contains data OR the TX software
 * queue has more messages waiting.
 */
static void update_int(void)
{
    bool pending = s_queued_frame_has_data ||
                   (uxQueueMessagesWaiting(s_tx_queue) > 0);
    gpio_set_level(SPI_LINK_CD_INT_GPIO, pending ? 1 : 0);
}

/* ── Frame builders ─────────────────────────────────────────────────────── */

/*
 * Pull one message from the TX queue and encode it into s_tx_frame.
 * If the queue is empty, encode a NOOP frame (len = 0).
 * Updates s_queued_frame_has_data.
 */
static void build_tx_frame(void)
{
    SpiLinkMsg_t msg;

    s_tx_frame[FRAME_OFF_MAGIC] = SPI_LINK_FRAME_MAGIC;
    s_tx_frame[FRAME_OFF_FLAGS] = 0x00;

    if (xQueueReceive(s_tx_queue, &msg, 0) == pdTRUE) {
        uint16_t len = msg.len;
        s_tx_frame[FRAME_OFF_LEN_H] = (uint8_t)(len >> 8);
        s_tx_frame[FRAME_OFF_LEN_L] = (uint8_t)(len & 0xFF);
        memcpy(&s_tx_frame[FRAME_OFF_PAYLOAD], msg.data, len);
        /* zero-pad the rest of the payload area */
        if (len < SPI_LINK_PAYLOAD_MAX) {
            memset(&s_tx_frame[FRAME_OFF_PAYLOAD + len], 0,
                   SPI_LINK_PAYLOAD_MAX - len);
        }
        s_queued_frame_has_data = true;
    } else {
        /* NOOP frame */
        s_tx_frame[FRAME_OFF_LEN_H] = 0;
        s_tx_frame[FRAME_OFF_LEN_L] = 0;
        memset(&s_tx_frame[FRAME_OFF_PAYLOAD], 0, SPI_LINK_PAYLOAD_MAX);
        s_queued_frame_has_data = false;
    }
}

/*
 * Validate and post the received RX frame to the RX queue.
 */
static void process_rx_frame(const uint8_t *frame)
{
    if (frame[FRAME_OFF_MAGIC] != SPI_LINK_FRAME_MAGIC) {
        printf("SpiSlaveLink: bad magic 0x%02X (expected 0xA5)\n",
               frame[FRAME_OFF_MAGIC]);
        return;
    }

    uint16_t len = ((uint16_t)frame[FRAME_OFF_LEN_H] << 8) |
                    (uint16_t)frame[FRAME_OFF_LEN_L];

    if (len == 0 || len > SPI_LINK_PAYLOAD_MAX) return;

    SpiLinkMsg_t msg;
    msg.len = len;
    memcpy(msg.data, &frame[FRAME_OFF_PAYLOAD], len);

#if 0
    DBG("process_rx_frame msg rxed\r\n");
    if (g_debug_verbose) {
        for(uint8_t i = 0;i < len;i++) printf("%02X ",msg.data[i]);
        printf("\r\n");
    }
#endif

    /* Drop silently if RX queue is full — caller responsible for draining */
    xQueueSend(s_rx_queue, &msg, 0);
}

/* ── Slave task ─────────────────────────────────────────────────────────── */

static void slave_task(void *arg)
{
    (void)arg;

    spi_slave_transaction_t  trans;
    spi_slave_transaction_t *completed;
    bool has_prev_rx = false;

    for (;;) {
        /* 1. Build the next TX frame. */
        build_tx_frame();

        /* 2. Re-arm the DMA immediately — keeping the unarmed window as short
         *    as possible (just steps 1-2, no printf calls).  The previous RX
         *    is processed in step 4, after the DMA is already armed again. */
        memset(&trans, 0, sizeof(trans));
        trans.length    = SPI_LINK_FRAME_SIZE * 8;
        trans.tx_buffer = s_tx_frame;
        trans.rx_buffer = s_rx_frame;

        esp_err_t err = spi_slave_queue_trans(SPI_HOST, &trans, portMAX_DELAY);
        if (err != ESP_OK) {
            printf("SpiSlaveLink: queue_trans error %s\n", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(5));
            has_prev_rx = false;
            continue;
        }

        /* 3. DMA armed; raise INT and then process previous RX.
         *    Any CR transfer that arrives during process_rx_frame hits an
         *    armed DMA, not an unarmed window. */
        s_dma_armed = true;
        update_int();

        if (has_prev_rx) {
            process_rx_frame(s_rx_copy);
        }

        /* 4. Block until the current transfer completes. */
        err = spi_slave_get_trans_result(SPI_HOST, &completed, portMAX_DELAY);

        s_dma_armed = false;
        s_queued_frame_has_data = false;
        int_deassert();

        if (err != ESP_OK) {
            printf("SpiSlaveLink: get_trans_result error %s\n", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(5));
            has_prev_rx = false;
            continue;
        }

        /* 5. Copy RX before the next queue_trans overwrites s_rx_frame. */
        memcpy(s_rx_copy, s_rx_frame, SPI_LINK_FRAME_SIZE);
        has_prev_rx = true;
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

bool SpiSlaveLink_Init(void)
{
    /* Configure INT output, default LOW (no pending data). */
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << SPI_LINK_CD_INT_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io_conf) != ESP_OK) {
        printf("SpiSlaveLink: gpio_config failed\n");
        return false;
    }
    int_deassert();

    /* SPI2 bus — same pins as the SD card (bus must be free at this point). */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = SD_CARD_PIN_MOSI,
        .miso_io_num   = SD_CARD_PIN_MISO,
        .sclk_io_num   = SD_CARD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_LINK_FRAME_SIZE,
    };

    spi_slave_interface_config_t slave_cfg = {
        .spics_io_num = SPI_LINK_CD_CS_GPIO,
        .flags        = 0,
        .queue_size   = 2,
        .mode         = 0,      /* CPOL=0, CPHA=0 — must match CR master */
    };

    esp_err_t err = spi_slave_initialize(SPI_HOST, &bus_cfg, &slave_cfg,
                                         SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        printf("SpiSlaveLink: spi_slave_initialize failed: %s\n",
               esp_err_to_name(err));
        return false;
    }

    /* Create queues. */
    s_rx_queue = xQueueCreate(SPI_RX_QUEUE_DEPTH, sizeof(SpiLinkMsg_t));
    s_tx_queue = xQueueCreate(SPI_TX_QUEUE_DEPTH, sizeof(SpiLinkMsg_t));
    if (!s_rx_queue || !s_tx_queue) {
        printf("SpiSlaveLink: queue creation failed\n");
        return false;
    }

    /* Start the slave task on core 0 (same as the UART parser was). */
    BaseType_t ret = xTaskCreatePinnedToCore(slave_task, "SpiSlaveTask",
                                             SLAVE_TASK_STACK, NULL,
                                             SLAVE_TASK_PRIORITY, NULL, 0);
    if (ret != pdPASS) {
        printf("SpiSlaveLink: task create failed\n");
        return false;
    }

    printf("SpiSlaveLink: init OK (CS=GPIO%d, INT=GPIO%d)\n",
           SPI_LINK_CD_CS_GPIO, SPI_LINK_CD_INT_GPIO);
    return true;
}

bool SpiSlaveLink_Transmit(const uint8_t *payload, uint16_t len)
{
    if (!payload || len == 0 || len > SPI_LINK_PAYLOAD_MAX) return false;

    SpiLinkMsg_t msg;
    msg.len = len;
    memcpy(msg.data, payload, len);

    if (xQueueSend(s_tx_queue, &msg, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;   /* TX queue full */
    }

    /* If the DMA is armed (slave_task is in spi_slave_get_trans_result),
     * raise INT now — the CR needs to know there is data to fetch.
     * If not armed, slave_task will raise INT after the next queue_trans().
     * Queue the message BEFORE checking s_dma_armed so we cannot both miss
     * update_int() inside slave_task AND skip the raise here. */
    if (s_dma_armed) {
        update_int();
    }
    return true;
}

bool SpiSlaveLink_Transmit_with_LoggerID(const uint8_t *payload, uint16_t len,uint16_t destLoggerID)
{
    if (!payload || len == 0 || len > SPI_LINK_PAYLOAD_MAX) return false;

    SpiLinkMsg_t msg;
    uint8_t* pBuff = msg.data;
    pBuff = SerialiseU16(destLoggerID,pBuff);
    memcpy(pBuff, payload, len);
    msg.len = len + sizeof(uint16_t);

    DBG("SpiSlaveLink_Transmit_with_LoggerID bytes\r\n");
    if (g_debug_verbose) {
        for(uint8_t i = 0;i < msg.len;i++) printf("%02X ", (msg.data)[i]);
        printf("\r\n");
    }

    if (xQueueSend(s_tx_queue, &msg, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;   /* TX queue full */
    }

    if (s_dma_armed) {
        update_int();
    }
    return true;
}

QueueHandle_t SpiSlaveLink_GetRxQueue(void)
{
    return s_rx_queue;
}

void SpiSlaveLink_SuspendForSD(void)
{
    /*
     * The slave task will block in spi_slave_transmit() while this happens.
     * Free the slave driver so the caller can reinitialise SPI2 as master.
     * The task will get an error on the next transmit call and retry, so it
     * is not explicitly suspended — it will spin harmlessly for a few cycles.
     */
    int_deassert();
    spi_slave_free(SPI_HOST);
}

void SpiSlaveLink_ResumeAfterSD(void)
{
    /* Caller has already freed SPI2 master; re-run slave initialisation. */
    SpiSlaveLink_Init();
}
