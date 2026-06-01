idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        bt
        nvs_flash
        driver
        freertos
        esp_hw_support
)