idf_component_register(
    SRCS
        "main.cpp"
        "modem.cpp"
        "init.cpp"
        "pmu.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp32-camera
        sdmmc
        fatfs
        esp_jpeg
        esp-tflite-micro
        esp_timer
    PRIV_REQUIRES
        nvs_flash
)

    