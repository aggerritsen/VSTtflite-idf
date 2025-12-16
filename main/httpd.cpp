#include "httpd.h"

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_http_server.h"

static const char *TAG = "HTTPD";

static httpd_handle_t s_server = NULL;

// -----------------------------------------------------------------------------
// Last-frame storage (RAM only)
// -----------------------------------------------------------------------------

static uint8_t *s_last_jpeg = NULL;
static size_t   s_last_jpeg_len = 0;
static SemaphoreHandle_t s_jpeg_mutex = NULL;

// -----------------------------------------------------------------------------
// Public API: update last captured frame
// -----------------------------------------------------------------------------

void httpd_update_last_frame(const uint8_t *data, size_t len)
{
    if (!data || len == 0 || !s_jpeg_mutex) {
        return;
    }

    if (xSemaphoreTake(s_jpeg_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {

        free(s_last_jpeg);
        s_last_jpeg = (uint8_t *)malloc(len);

        if (s_last_jpeg) {
            memcpy(s_last_jpeg, data, len);
            s_last_jpeg_len = len;
        } else {
            s_last_jpeg_len = 0;
            ESP_LOGE(TAG, "JPEG buffer alloc failed (%u bytes)", (unsigned)len);
        }

        xSemaphoreGive(s_jpeg_mutex);
    }
}

// -----------------------------------------------------------------------------
// HTTP handler: /latest.jpg
// -----------------------------------------------------------------------------

static esp_err_t latest_handler(httpd_req_t *req)
{
    if (!s_jpeg_mutex) {
        httpd_resp_send_err(req,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Not initialized");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(s_jpeg_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        httpd_resp_send_err(req,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Busy");
        return ESP_FAIL;
    }

    if (!s_last_jpeg || s_last_jpeg_len == 0) {
        xSemaphoreGive(s_jpeg_mutex);
        httpd_resp_send_err(req,
            HTTPD_404_NOT_FOUND,
            "No frame yet");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_send(req,
        (const char *)s_last_jpeg,
        s_last_jpeg_len);

    xSemaphoreGive(s_jpeg_mutex);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// HTTP handler: index page
// -----------------------------------------------------------------------------

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    httpd_resp_sendstr(req,
        "<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'>"
        "<title>Camera Monitor</title>"
        "<style>"
        "body{background:#111;color:#eee;font-family:sans-serif;text-align:center}"
        "img{max-width:90vw;border:1px solid #444}"
        "</style>"
        "</head><body>"
        "<h1>Live Capture</h1>"
        "<img id='cam' src='/latest.jpg'>"
        "<script>"
        "setInterval(()=>{"
        "document.getElementById('cam').src="
        "'/latest.jpg?nocache='+Date.now();"
        "},1000);"
        "</script>"
        "</body></html>"
    );

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Public entry point
// -----------------------------------------------------------------------------

void http_server_start(void)
{
    if (s_server) {
        ESP_LOGW(TAG, "HTTP server already running");
        return;
    }

    if (!s_jpeg_mutex) {
        s_jpeg_mutex = xSemaphoreCreateMutex();
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);

    if (httpd_start(&s_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    httpd_uri_t index_uri = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = index_handler,
        .user_ctx = NULL
    };

    httpd_uri_t latest_uri = {
        .uri      = "/latest.jpg",
        .method   = HTTP_GET,
        .handler  = latest_handler,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(s_server, &index_uri);
    httpd_register_uri_handler(s_server, &latest_uri);

    ESP_LOGI(TAG, "HTTP server started");
}
