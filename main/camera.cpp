// File: main/camera.cpp
//
// Camera initialization for LILYGO T-SIM7080G-S3
// OV2640, ESP-IDF v5.x
//
// Fixes applied:
//  - fb_count = 2 (buffering to absorb SD latency)

#include "camera.h"

#include "esp_camera.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CAMERA";

// -----------------------------------------------------------------------------
// Camera pin map – LILYGO T-SIM7080G-S3
// -----------------------------------------------------------------------------

#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  18

#define CAM_PIN_XCLK    8
#define CAM_PIN_SIOD    2
#define CAM_PIN_SIOC    1

#define CAM_PIN_D7      9
#define CAM_PIN_D6     10
#define CAM_PIN_D5     11
#define CAM_PIN_D4     13
#define CAM_PIN_D3     21
#define CAM_PIN_D2     48
#define CAM_PIN_D1     47
#define CAM_PIN_D0     14

#define CAM_PIN_VSYNC  16
#define CAM_PIN_HREF   17
#define CAM_PIN_PCLK   12

// -----------------------------------------------------------------------------
// OV2640 software reset (known-good workaround)
// -----------------------------------------------------------------------------

static esp_err_t ov2640_software_reset(void)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return ESP_FAIL;

    // DSP bank
    s->set_reg(s, 0x30, 0xFF, 0x01);
    s->set_reg(s, 0x30, 0x10, 0x40); // standby
    vTaskDelay(pdMS_TO_TICKS(50));
    s->set_reg(s, 0x30, 0x10, 0x00); // wake
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Camera init
// -----------------------------------------------------------------------------

esp_err_t camera_init(void)
{
    ESP_LOGI(TAG, "Starting camera init");

    // Hardware reset
    gpio_config_t rst = {};
    rst.pin_bit_mask = 1ULL << CAM_PIN_RESET;
    rst.mode = GPIO_MODE_OUTPUT;
    gpio_config(&rst);

    gpio_set_level((gpio_num_t)CAM_PIN_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)CAM_PIN_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    camera_config_t config = {
        .pin_pwdn       = CAM_PIN_PWDN,
        .pin_reset      = CAM_PIN_RESET,

        .pin_xclk       = CAM_PIN_XCLK,
        .pin_sccb_sda   = CAM_PIN_SIOD,
        .pin_sccb_scl   = CAM_PIN_SIOC,

        .pin_d7         = CAM_PIN_D7,
        .pin_d6         = CAM_PIN_D6,
        .pin_d5         = CAM_PIN_D5,
        .pin_d4         = CAM_PIN_D4,
        .pin_d3         = CAM_PIN_D3,
        .pin_d2         = CAM_PIN_D2,
        .pin_d1         = CAM_PIN_D1,
        .pin_d0         = CAM_PIN_D0,

        .pin_vsync      = CAM_PIN_VSYNC,
        .pin_href       = CAM_PIN_HREF,
        .pin_pclk       = CAM_PIN_PCLK,

        .xclk_freq_hz   = 20000000,
        .ledc_timer     = LEDC_TIMER_0,
        .ledc_channel   = LEDC_CHANNEL_0,

        .pixel_format   = PIXFORMAT_JPEG,
        .frame_size     = FRAMESIZE_QVGA,
        .jpeg_quality   = 10,

        // FIX 1: double buffering
        .fb_count       = 2,

        .fb_location    = CAMERA_FB_IN_PSRAM,
        .grab_mode      = CAMERA_GRAB_WHEN_EMPTY,

        .sccb_i2c_port  = 1
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        return err;
    }

    ov2640_software_reset();

    sensor_t *s = esp_camera_sensor_get();
    if (!s) return ESP_FAIL;

    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_quality(s, 10);

    ESP_LOGI(TAG, "Camera init complete");
    return ESP_OK;
}
