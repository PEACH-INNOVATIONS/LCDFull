#include "DebugConsole.h"
#include "esp_console.h"
#include <string.h>
#include <stdio.h>

volatile bool g_debug_verbose = false;

static int cmd_debug(int argc, char **argv)
{
    if (argc < 2) {
        printf("debug: %s\n", g_debug_verbose ? "on" : "off");
        return 0;
    }
    if (strcmp(argv[1], "on") == 0) {
        g_debug_verbose = true;
        printf("debug on\n");
        return 0;
    }
    if (strcmp(argv[1], "off") == 0) {
        g_debug_verbose = false;
        printf("debug off\n");
        return 0;
    }
    printf("usage: debug on|off\n");
    return 1;
}

void InitDebugConsole(void)
{
    esp_console_repl_t *repl = NULL;

    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "cd> ";
    repl_config.max_cmdline_length = 64;

    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    const esp_console_cmd_t debug_cmd = {
        .command = "debug",
        .help    = "Enable or disable verbose debug output",
        .hint    = "on|off",
        .func    = cmd_debug,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&debug_cmd));

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
