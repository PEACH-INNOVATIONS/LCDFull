#ifndef TASK_H_STUB
#define TASK_H_STUB
#include "FreeRTOS.h"
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
TickType_t xTaskGetTickCount(void);

typedef int BaseType_t;
#define pdPASS  1
#define pdFAIL  0
#define portMAX_DELAY  ((TickType_t)0xFFFFFFFFu)

#define vTaskDelay(ticks)  ((void)(ticks))
#define xTaskCreate(fn, nm, stk, arg, pri, h) \
    ((void)(fn),(void)(nm),(void)(stk),(void)(arg),(void)(pri),(void)(h),pdPASS)
#define xTaskCreatePinnedToCore(fn, nm, stk, arg, pri, h, core) \
    ((void)(fn),(void)(nm),(void)(stk),(void)(arg),(void)(pri),(void)(h),(void)(core),pdPASS)
#endif
