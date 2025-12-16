#include "wifi.h"

#include <string.h>

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "WIFI";

void wifi_ap_start(void)
{
    ESP_LOGI(TAG, "Initializing Wi-Fi Access Point");

    // -------------------------------------------------------------------------
    // 1. NVS (REQUIRED FOR WIFI)
    // -------------------------------------------------------------------------
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_LOGW(TAG, "NVS init failed (%s), erasing...",
                 esp_err_to_name(err));

        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    // -------------------------------------------------------------------------
    // 2. Network stack
    // -------------------------------------------------------------------------
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    // -------------------------------------------------------------------------
    // 3. Wi-Fi driver
    // -------------------------------------------------------------------------
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t ap_config = {};
    strcpy((char *)ap_config.ap.ssid, "CAM-S3");
    strcpy((char *)ap_config.ap.password, "");

    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    if (strlen((char *)ap_config.ap.password) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP started");
    ESP_LOGI(TAG, "SSID     : CAM-S3");
    ESP_LOGI(TAG, "Password : cam12345");
    ESP_LOGI(TAG, "IP       : 192.168.4.1");
}
