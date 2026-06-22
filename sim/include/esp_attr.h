#ifndef ESP_ATTR_H_STUB
#define ESP_ATTR_H_STUB
#include <stdlib.h>

/* On PC, PSRAM placement attributes are no-ops */
#define EXT_RAM_BSS_ATTR
#define DRAM_ATTR
#define IRAM_ATTR
#define RTC_DATA_ATTR

/* On PC, heap_caps_malloc falls back to plain malloc */
#define heap_caps_malloc(size, caps)  malloc(size)
#define MALLOC_CAP_SPIRAM  0u
#define MALLOC_CAP_8BIT    0u
#define MALLOC_CAP_DMA     0u
#endif
