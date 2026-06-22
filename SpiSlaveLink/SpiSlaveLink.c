#include "SpiSlaveLink.h"

#include <string.h>
#include <stdio.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "app_main.h"     /* SD_CARD_PIN_MOSI/MISO/SCLK */

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

static QueueHandle_t s_rx_queue = NULL;
static QueueHandle_t s_tx_queue = NULL;

/*
 * Tracks whether the frame currently queued for transmission contains real
 * data.  Used to keep INT asserted until the CR has clocked out that frame,
 * even if the TX software queue has already been drained.
 */
static bool s_queued_frame_has_data = false;

/* ── INT line helpers ───────────────────────────────────────────────────── */

static void int_assert(void)   { gpio_set_level(SPI_LINK_CD_INT_GPIO, 1); }
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
static void process_rx_frame(void)
{
    if (s_rx_frame[FRAME_OFF_MAGIC] != SPI_LINK_FRAME_MAGIC) return;

    uint16_t len = ((uint16_t)s_rx_frame[FRAME_OFF_LEN_H] << 8) |
                    (uint16_t)s_rx_frame[FRAME_OFF_LEN_L];

    if (len == 0 || len > SPI_LINK_PAYLOAD_MAX) return;

    SpiLinkMsg_t msg;
    msg.len = len;
    memcpy(msg.data, &s_rx_frame[FRAME_OFF_PAYLOAD], len);

    /* Drop silently if RX queue is full — caller responsible for draining */
    xQueueSend(s_rx_queue, &msg, 0);
}

/* ── Slave task ─────────────────────────────────────────────────────────── */

static void slave_task(void *arg)
{
    (void)arg;

    spi_slave_transaction_t trans;

    for (;;) {
        /*
         * 1. Build the next TX frame and refresh the INT line.
         *    INT stays high while either s_queued_frame_has_data OR the
         *    TX queue still has messages.
         */
        build_tx_frame();
        update_int();

        /*
         * 2. Arm the transaction and block until the CR asserts CS and
         *    clocks out the full frame.
         */
        memset(&trans, 0, sizeof(trans));
        trans.length    = SPI_LINK_FRAME_SIZE * 8;   /* bits */
        trans.tx_buffer = s_tx_frame;
        trans.rx_buffer = s_rx_frame;

        esp_err_t err = spi_slave_transmit(SPI_HOST, &trans, portMAX_DELAY);

        /*
         * 3. The frame has been clocked out; our TX data was consumed.
         *    Lower INT immediately then re-evaluate based on pending queue.
         */
        s_queued_frame_has_data = false;
        update_int();

        if (err != ESP_OK) {
            printf("SpiSlaveLink: transmit error %s\n", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* 4. Process whatever the CR sent to us. */
        process_rx_frame();
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

    /* Signal the CR immediately — the slave task will load the frame on the
     * next transfer cycle even if it is currently blocked waiting for CS. */
    int_assert();
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
