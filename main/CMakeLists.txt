idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        sdmmc
        fatfs
        esp-tflite-micro
        esp_timer
        esp_jpeg
    PRIV_REQUIRES
        nvs_flash
)
    