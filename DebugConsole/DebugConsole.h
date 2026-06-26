#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include <stdbool.h>
#include <stdio.h>

extern volatile bool g_debug_verbose;

#define DBG(fmt, ...) \
    do { if (g_debug_verbose) printf(fmt, ##__VA_ARGS__); } while (0)

void InitDebugConsole(void);

#endif
