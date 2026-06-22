#ifndef FREERTOS_QUEUE_H_STUB
#define FREERTOS_QUEUE_H_STUB
#include "FreeRTOS.h"
typedef void *QueueHandle_t;
static inline QueueHandle_t xQueueCreate(int len, int sz)    { (void)len; (void)sz; return NULL; }
static inline int xQueueSend(QueueHandle_t q, const void *i, TickType_t t) { (void)q; (void)i; (void)t; return 1; }
static inline int xQueueReceive(QueueHandle_t q, void *i, TickType_t t)    { (void)q; (void)i; (void)t; return 0; }
#endif
