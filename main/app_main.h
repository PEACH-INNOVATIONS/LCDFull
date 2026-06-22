#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "PeachDefs.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#define TARGET_PLATFORM_ESP32_CYD  0
#define TARGET_PLATFORM_ESP32_S3   1
#define TARGET_PLATFORM  TARGET_PLATFORM_ESP32_S3

/* ---- SD card ---- */
#define SD_CARD_PIN_MISO  GPIO_NUM_42
#define SD_CARD_PIN_MOSI  GPIO_NUM_40
#define SD_CARD_PIN_SCLK  GPIO_NUM_41
#define SD_CARD_PIN_CS    GPIO_NUM_47

/* ---- UART to coach radio (legacy — superseded by SPI link) ---- */
#define RADIO_TO_DISPLAY_TXD       GPIO_NUM_15
#define RADIO_TO_DISPLAY_RXD       GPIO_NUM_16
#define RADIO_TO_DISPLAY_TEST_TXD  GPIO_NUM_15
#define RADIO_TO_DISPLAY_TEST_RXD  GPIO_NUM_16

/* ---- SPI link to coach radio ---- */
/* CS  (slave-select input from CR) : shared with onboard RGB LED pad — LED unused */
/* INT (interrupt output to CR)     : GPIO46 — safest strapping pin (ROM-log),
 *                                    naturally LOW when idle (no pending data),
 *                                    so boot-time strapping is unaffected.   */
/* MOSI / MISO / SCLK are reused from the SD-card SPI bus (SPI2_HOST).       */
/* Change SPI_LINK_CD_INT_GPIO in SpiSlaveLink.h if a different pin is used. */

#define SD_CARD_MOUNT_POINT  "/sdcard"

#define USE_ENUM_DESERIALISER

#include <stdbool.h>
/* Returns true once the CR state machine reaches eState_operating. */
bool GetCRReady(void);

#endif /* APP_MAIN_H */
