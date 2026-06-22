# Peach LCDFull Project

## System topology

- **CD** = Coach Display — this ESP32-S3 board running LCDFull
- **CR** = Coach Radio — Newracom NRC7394 board running `peach_coaching_link` (WiFiHaLow SDK at `/home/joe/Peach/WiFiHaLow/nrc7394_sdk/...`)
- **BR** = Boat Radio — separate Newracom devices, connected to CR over WiFi HaLow
- CD ↔ CR: UART serial link for admin messages
- CR ↔ BR: WiFi HaLow for data packets

---

## Display hardware

**Panel:** JD9165A, 800×480, 8-bit parallel RGB interface, DE-mode  
**Touch:** HY4633 (FT6206-compatible), I2C address 0x38  
**MCU:** ESP32-S3

### Pure DE-mode panel
The JD9165A uses the DE (Data Enable) signal exclusively to determine active pixels.
HSYNC/VSYNC porch timing (HBP, HFP, VBP, VFP) has **no effect** on where content appears on screen.
Do not attempt to fix display positioning by adjusting porch values.

### Framebuffer scan offset
The panel does not start scanning from fb[0,0]. Its physical top-left pixel reads from a
non-zero point in the ESP32-S3 PSRAM framebuffer:

| Axis | Offset | Meaning |
|------|--------|---------|
| X    | 16 px  | Panel reads fb_x=16 as physical x=0 |
| Y    | 456 px | Panel reads fb_y=456 as physical y=0 |

Content wraps around both axes. The flush callback must use **modulo arithmetic**, not
bounds-clipping:

```c
#define PANEL_OFS_X  16
#define PANEL_OFS_Y  456

int32_t px = ((lvgl_x) + PANEL_OFS_X) % LCD_H_RES;
int32_t py = ((lvgl_y) + PANEL_OFS_Y) % LCD_V_RES;
s_fb[py * LCD_H_RES + px] = rgb332_pixel;
```

Without modulo, rows 30–479 map to fb_y 486–935 and are silently discarded — only the
top ~30 rows of the UI would be visible.

### Panel orientation
Panel is mounted right-way-up. `PANEL_INVERT = 0` (no 180° rotation). Touch coordinates
are naturally aligned and also require no inversion.

### RGB332 framebuffer
The panel is driven with an 8-bit data bus in RGB332 mode. LVGL renders to an RGB565
scratch buffer; the flush callback converts each pixel:

```c
uint8_t rgb332 = ((c >> 13) & 0x07) << 5 |
                 ((c >>  8) & 0x07) << 2 |
                 ((c >>  3) & 0x03);
```

### PSRAM draw buffer
The LVGL draw buffer must be allocated in PSRAM (`MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`).
Internal SRAM is exhausted by the RTOS and driver allocations at 800×480.
CPU-copy in the flush callback makes PSRAM valid here (no DMA constraint).

---

## Build setup

### idf.py is not on PATH by default
Always source the IDF environment first:
```bash
bash -c "source /home/joe/esp/v5.5.1/esp-idf/export.sh && idf.py build"
bash -c "source /home/joe/esp/v5.5.1/esp-idf/export.sh && idf.py -p /dev/ttyACM0 flash"
```

### IDF_TARGET must be in CMakeLists.txt
Setting `CONFIG_IDF_TARGET` in sdkconfig.defaults is **not sufficient** — CMake reads the
target before processing sdkconfig.defaults. The top-level `CMakeLists.txt` must have:

```cmake
cmake_minimum_required(VERSION 3.16)
set(IDF_TARGET "esp32s3")          # ← must come before include
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(LCD)
```

### Required sdkconfig.defaults settings
```
CONFIG_IDF_TARGET="esp32s3"

# PSRAM (Octal SPI, 80 MHz)
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_SPIRAM_USE_MALLOC=y
CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384

# FreeRTOS: 1 kHz tick so LVGL ms timers stay accurate
CONFIG_FREERTOS_HZ=1000

# Larger main stack for LVGL init path
CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192

# CRITICAL: DMA restarts from fb[0] at every vsync — prevents row-drift
CONFIG_LCD_RGB_RESTART_IN_VSYNC=y

# Flash: 80 MHz QIO
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_ESPTOOLPY_FLASHMODE_QIO=y
```

### RGB panel driver config essentials
```c
.num_fbs               = 1,
.bounce_buffer_size_px = LCD_H_RES * 24,   /* ~37 KB; needed for DMA→PSRAM transfers */
.flags = { .fb_in_psram = true, .refresh_on_demand = false },
```

### Working timing config (PCLK 16 MHz)
```c
.hsync_pulse_width = 24, .hsync_back_porch = 160, .hsync_front_porch = 40,
.vsync_pulse_width =  2, .vsync_back_porch  = 40, .vsync_front_porch  = 20,
```

---

## LVGL notes

- LVGL v9 API is used (`lv_display_create`, `lv_display_set_flush_cb`, etc.)
- Tab content containers carry a default 12 px padding. Call
  `lv_obj_set_style_pad_all(tab, 0, 0)` on every tab or widgets placed near the
  right edge will overflow the 800 px screen width.
- `lv_chart_set_axis_range` (not `lv_chart_set_range`) is the correct API for setting
  chart axis ranges in this LVGL build. `lv_chart_set_axis_tick` does not exist;
  axis labels must be added manually as `lv_label` objects.
- `lv_chart_set_series_value_by_id2` (not `lv_chart_set_value_by_id2`) is the correct
  API for updating individual scatter chart points.

---

## Serial protocol (CD ↔ CR)

### Packet format
```
[SYNC 0xE3 0x5A][LoggerID(2 bytes)][MsgLen(1)][MsgID(1)][EvID(1)][Data(MsgLen-4)][EndMsgLen(1)]
```

MsgLen includes MsgID + EvID + Data bytes (not SYNC or LoggerID).
EndMsgLen must equal MsgLen for the packet to be marked valid.

### Valid Peach MsgIDs
`0x01–0x07, 0x09, 0x0A, 0x11, 0x12, 0xA0, 0xB0–0xB3, 0xC0–0xC3`
Any other MsgID causes the parser to treat the sync as a false sync and resync.

### ASCII hex serial (infrastructure present, currently disabled on CD)
`PacketParser/PacketParser.h` has `//#define USE_ASCII_HEX_SERIAL` (commented out).
The CR side (`UartCmdParser/UartCmdParser.h`) still has `#define USE_ASCII_HEX_SERIAL` enabled.
Both sides must match. The encoding: binary byte 0xAB → ASCII chars `'A','B'`;
STX(0x02)/ETX(0x03) are frame delimiters (cannot appear in hex payload).

**If re-enabling**: uncomment `#define USE_ASCII_HEX_SERIAL` in `PacketParser/PacketParser.h`
to match the CR side.

---

## Admin message flow (CD → CR)

```
SendAdminMessageTo(eEntityCoachRadio, &msg, epIndex)
  → ProcessMessage()           [MessageProcessing.c]
    → adminNotForMe == true
      → TxSerialMessage()      [UartHW.c / Hardware/Esp32/UartHW.c]
        → UART TX to CR
```

On the CR side:
```
ProcessByteIn() → ProcessSerialMessage() → ProcessMessage() → AdminMessageQueue
AdminMessageProcessingTask → ProcessMsgCommon()
  → AdminMsgTypeRadioReset → nrc_sw_reset()   (hard reset)
  → AdminMsgTypePing       → sends Pong back to CD
```

### CR startup gate
The CR runs `StartupMessageStateMachine()` on boot. It sends a ping to the CD and
**blocks** in `AdminMessageProcessingTask` waiting for a pong back before entering the
main loop. Until the CD sends a pong in response to the CR's ping, **the CR silently
ignores all other admin messages** — including `AdminMsgTypeRadioReset`.

This creates a chicken-and-egg situation: both sides ping each other on boot and wait for
a pong before proceeding. The UART serial link must be physically connected at boot time.

---

## CoachDisplayStateMachine (main/app_main.c)

State sequence: `initial → pinging → waiting_pong → resetting → pinging2 → waiting_pong2 → operating`

- Runs every 1 ms from `MainTask`.
- Ping timeout: 10 seconds before retrying.
- On first pong received: sends `AdminMsgTypeRadioReset` (hard reset) to CR, then pings again.
- On second pong (post-reset): sets `s_cr_ready = true`, calls `SendAllLoggerIDsToCoachRadio()`, enters `operating`.
- Debug prints prefixed `CDSM:` are present on all state transitions (added to aid diagnosis).

`GetPongReceived()` clears the flag on read (not idempotent) — safe to call in both
`waiting_pong` and `waiting_pong2`.

### Known issue under investigation
When tested with UDP data (serial UART may or may not be physically connected), the CR
does not appear to reset. Likely causes in order of probability:
1. Serial UART not physically connected in the UDP test setup.
2. CR is stuck in `StartupMessageStateMachine` (waiting for pong from CD) and therefore
   ignoring the reset message.
3. Message serialisation / routing bug in the CD→CR admin path.

Watch the `CDSM:` prints on the CD console to determine whether the pong is arriving at
the CD before investigating the CR side.

---

## Newracom UART — DO NOT MODIFY

The Newracom UART driver/hardware has known driver/hardware issues.
**Do not modify** any of the following files:
- `NewracomHW/NewracomHW.c`
- `BufferedUartHW_Newracom.c` (or any `BufferedUart*` Newracom variant)
- Any other file under the Newracom HW layer

`nrc_uart_write` is a closed-source function compiled into the Newracom modem libraries
(`libModem.a`, `libModemAP.a`, etc.) — no C source is available.

---

## Known latent bugs

### GetLoggerIDFromIndex(0xFF) out-of-bounds
`SendRatioResetTo(eEntityCoachRadio, 0xFF, eResetAction_Hard)` passes `epIndex=0xFF`
which flows through to `GetLoggerIDFromIndex(0xFF)`. The `BoatRadioEP` array has 10
elements — index 255 is out of bounds. Currently returns 0 safely due to PSRAM
zero-initialisation, but is a real bug. `ENDPOINT_LOCAL_SYSTEM` has since been used in
some call sites as a cleaner alternative.
