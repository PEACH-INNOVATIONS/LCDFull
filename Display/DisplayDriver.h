#ifndef DISPLAY_DRIVER_H
#define DISPLAY_DRIVER_H

/* Constants shared with GraphConfig — no driver implementation in LCDFull */
#define DISPLAY_H_RES  800
#define DISPLAY_V_RES  480

/* RGB332 colour constants (kept for GraphConfig defaults) */
#define COLOUR_RED     0xE0u
#define COLOUR_GREEN   0x1Cu
#define COLOUR_BLUE    0x03u
#define COLOUR_CYAN    0x1Fu
#define COLOUR_MAGENTA 0xE3u
#define COLOUR_YELLOW  0xFCu
#define COLOUR_WHITE   0xFFu
#define COLOUR_BLACK   0x00u

#endif /* DISPLAY_DRIVER_H */
