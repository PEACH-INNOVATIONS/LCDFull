#ifndef FREERTOS_H_STUB
#define FREERTOS_H_STUB
/* Minimal FreeRTOS stub for PC simulator */
#include <stdint.h>
#include <stdbool.h>
typedef uint32_t TickType_t;
typedef uint32_t UBaseType_t;
#define portTICK_PERIOD_MS  1u
#define portMAX_DELAY       0xFFFFFFFFu
#endif
