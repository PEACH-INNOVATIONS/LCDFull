#!/usr/bin/env python3
"""Generate pinout PDF for the Peach LCD ESP32-S3 project."""

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

FONT_PATH  = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
FONT_BOLD  = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
OUT_FILE   = "/home/joe/Peach/LCDFull/pinout.pdf"

pdfmetrics.registerFont(TTFont("Mono",     FONT_PATH))
pdfmetrics.registerFont(TTFont("MonoBold", FONT_BOLD))

PAGE_W, PAGE_H = A4          # 595 x 842 pt, portrait
MARGIN_X = 18 * mm
MARGIN_Y = 18 * mm

FONT_SZ      = 7.5           # pt — body monospace
TITLE_SZ     = 11
SECTION_SZ   = 8.5
LINE_H       = FONT_SZ * 1.35

def make_pdf():
    c = canvas.Canvas(OUT_FILE, pagesize=A4)
    c.setTitle("Peach LCD — ESP32-S3 Project Pinout")

    # ── Page 1: board diagrams ───────────────────────────────────────────────
    y = PAGE_H - MARGIN_Y

    def title(text, sz=TITLE_SZ):
        nonlocal y
        c.setFont("MonoBold", sz)
        c.setFillColorRGB(0.1, 0.1, 0.4)
        c.drawString(MARGIN_X, y, text)
        y -= sz * 1.8

    def section(text):
        nonlocal y
        c.setFont("MonoBold", SECTION_SZ)
        c.setFillColorRGB(0.2, 0.2, 0.6)
        c.drawString(MARGIN_X, y, text)
        y -= SECTION_SZ * 1.6

    def mono(text, indent=0, bold=False):
        nonlocal y
        font = "MonoBold" if bold else "Mono"
        c.setFont(font, FONT_SZ)
        c.setFillColorRGB(0, 0, 0)
        c.drawString(MARGIN_X + indent, y, text)
        y -= LINE_H

    def gap(n=1):
        nonlocal y
        y -= LINE_H * n

    def rule():
        nonlocal y
        c.setStrokeColorRGB(0.7, 0.7, 0.7)
        c.setLineWidth(0.4)
        c.line(MARGIN_X, y + LINE_H * 0.3, PAGE_W - MARGIN_X, y + LINE_H * 0.3)
        y -= LINE_H * 0.6

    def new_page():
        nonlocal y
        c.showPage()
        y = PAGE_H - MARGIN_Y

    # ════════════════════════════════════════
    title("Peach LCD  —  ESP32-S3-DevKitC-1  Project Pinout")
    rule()

    section("TOP VIEW  (USB-C connector at bottom)")

    lines_top = [
        " Function             GPIO  ┌──────────────────────┐  GPIO  Function",
        "─────────────────────────── │    ┌────────────┐    │ ───────────────────────────",
        " GND                   GND ─┤1   │  ESP32-S3  │  1 ├─ GND",
        " 3.3 V                 3V3 ─┤2   │  WROOM-1   │  2 ├─ 5 V (VBUS)",
        " Reset                [RST]─┤3   │            │  3 ├─ 43  USB-Serial TXD0 (debug)",
        " MCP23017 button IRQ    4  ─┤4   │            │  4 ├─ 44  USB-Serial RXD0 (debug)",
        " Touch INT (nc)         5  ─┤5   │            │  5 ├─  1  I²C SCL  (touch+buttons)",
        " LCD D0                 6  ─┤6   │            │  6 ├─  2  I²C SDA  (touch+buttons)",
        " LCD D1                 7  ─┤7   │            │  7 ├─ 42  SD MISO",
        " Radio UART2 TX        15  ─┤8   │            │  8 ├─ 41  SD SCLK",
        " Radio UART2 RX        16  ─┤9   │            │  9 ├─ 40  SD MOSI",
        " LCD VSYNC             17  ─┤10  │            │ 10 ├─ 39  LCD DISP (display on/off)",
        " LCD D2                18  ─┤11  │            │ 11 ├─ 38  LCD Backlight PWM",
        " LCD D3                 8  ─┤12  │            │ 12 ├─ 37  free",
        " free                   3  ─┤13  │            │ 13 ├─ 36  free  (input-only)",
        " free                  46  ─┤14  │            │ 14 ├─ 35  free  (input-only)",
        " LCD PCLK               9  ─┤15  │            │ 15 ├─  0  BOOT button (strapping)",
        " LCD D4                10  ─┤16  │            │ 16 ├─ 45  free  (strapping)",
        " LCD D5                11  ─┤17  │            │ 17 ├─ 48  onboard RGB LED (nc)",
        " LCD D6                12  ─┤18  │            │ 18 ├─ 47  SD CS",
        " LCD D7                13  ─┤19  │            │ 19 ├─ 21  LCD DE (data enable)",
        " LCD HSYNC             14  ─┤20  │            │ 20 ├─ 20  USB D+  (native USB, nc)",
        " LCD DE / free         21  ─┤21  │            │ 21 ├─ 19  USB D-  (native USB, nc)",
        "─────────────────────────── │    └────────────┘    │ ───────────────────────────",
        "     J3 (left header)       │  [RST btn] [BOOT btn] │       J2 (right header)",
        "                            │       [USB-C]         │",
        "                            └──────────────────────┘",
        "  nc = not connected in this project",
    ]

    for ln in lines_top:
        mono(ln)

    gap(0.5)
    rule()
    gap(0.5)

    section("BOTTOM VIEW  (board flipped left-right, USB-C at bottom)")

    lines_bot = [
        "  GPIO  Function                                             GPIO  Function",
        " ─────────────────────────────────────────────────────────────────────────────",
        "  GND ─┐  ┌─ GND",
        "   5V ─┤  ├─ 3V3",
        "   43 ─┤  ├─ [RST]    Reset",
        "   44 ─┤  ├─ 4        MCP23017 button IRQ",
        "    1 ─┤  ├─ 5        Touch INT (nc)",
        "    2 ─┤  ├─ 6        LCD D0",
        "   42 ─┤  ├─ 7        LCD D1",
        "   41 ─┤  ├─ 15       Radio UART2 TX",
        "   40 ─┤  ├─ 16       Radio UART2 RX",
        "   39 ─┤  ├─ 17       LCD VSYNC",
        "   38 ─┤  ├─ 18       LCD D2",
        "   37 ─┤  ├─ 8        LCD D3",
        "   36 ─┤  ├─ 3        free",
        "   35 ─┤  ├─ 46       free",
        "    0 ─┤  ├─ 9        LCD PCLK",
        "   45 ─┤  ├─ 10       LCD D4",
        "   48 ─┤  ├─ 11       LCD D5",
        "   47 ─┤  ├─ 12       LCD D6",
        "   21 ─┤  ├─ 13       LCD D7",
        "   20 ─┤  ├─ 14       LCD HSYNC",
        "   19 ─┘  └─ 21       LCD DE",
        "",
        "  (J2 now left)    (J3 now right)    — pin order top-to-bottom unchanged",
        "  USB-Serial TXD0=43  RXD0=44  |  SD MISO=42  SCLK=41  MOSI=40  CS=47",
        "  I²C SCL=1  SDA=2  |  LCD BL=38  DISP=39  |  BOOT=0  USB-D+=20  USB-D-=19",
    ]

    for ln in lines_bot:
        mono(ln)

    # ── Page 2: functional summary ───────────────────────────────────────────
    new_page()

    title("Peach LCD  —  ESP32-S3  Functional Pin Summary")
    rule()

    blocks = [
        ("LCD Panel  JD9165A  (8-bit parallel RGB, DE-mode)", [
            "  Data bus   D0 = IO6     D1 = IO7     D2 = IO18    D3 = IO8",
            "             D4 = IO10    D5 = IO11    D6 = IO12    D7 = IO13",
            "  Control    PCLK  = IO9     HSYNC = IO14    VSYNC = IO17",
            "             DE    = IO21    DISP  = IO39    BL    = IO38  (backlight PWM)",
        ]),
        ("Touch  HY4633 / FT6206-compat  (I²C addr 0x38)  — shared bus", [
            "  SDA = IO2     SCL = IO1     INT = IO5  (pin defined, not wired in lcd.c)",
        ]),
        ("Buttons  MCP23017 GPIO expander  (I²C addr 0x20)  — same bus", [
            "  SDA = IO2     SCL = IO1     INT = IO4",
        ]),
        ("SD Card  (SPI)", [
            "  MISO = IO42    MOSI = IO40    SCLK = IO41    CS = IO47",
        ]),
        ("Radio Link  (UART2)", [
            "  TX = IO15     RX = IO16",
        ]),
        ("Fixed / Board", [
            "  USB-Serial bridge    TXD0 = IO43    RXD0 = IO44   (programming / debug)",
            "  Native USB           D-   = IO19    D+   = IO20   (not used by firmware)",
            "  BOOT button          IO0  (strapping pin — hold low at reset for flash mode)",
            "  Onboard RGB LED      IO48 (not used by project)",
        ]),
        ("Free GPIOs", [
            "  IO3, IO22 – IO37  (excluding IO38, IO39),  IO45, IO46",
            "  Note: IO35, IO36 are input-only on the DevKitC-1  (no pull-up/pull-down)",
        ]),
    ]

    for heading, body_lines in blocks:
        section(heading)
        for ln in body_lines:
            mono(ln)
        gap(0.4)

    rule()
    gap(0.3)
    mono("Pin assignments read from source files:")
    mono("  components/LCD/include/lcd.h  —  LCD data/control, touch I²C, MCP23017 INT", indent=8)
    mono("  main/app_main.h               —  SD card SPI, radio UART2", indent=8)
    mono("  Hardware/UartHW.h             —  UART2 port number", indent=8)

    c.save()
    print(f"Written: {OUT_FILE}")

make_pdf()
