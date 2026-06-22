idf_component_register(
    SRCS "lcd.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_lcd
        driver
        esp_timer
        lvgl__lvgl
)
