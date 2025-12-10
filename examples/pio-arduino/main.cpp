/**
 * @file      main.cpp
 * @brief     Minimal Camera Example with Diagnostics + Web Server
 * @date      2025
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include "esp_camera.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#include "utilities.h"

// Diagnostics module (external)
#include "diag_dump.h"

// HTTP camera server
void startCameraServer();

// replace this:
// XPowersPMU PMU;

// with this:
XPowersAXP2101 PMU;
WiFiMulti   wifiMulti;

String      hostName = "LilyGo-Cam-";
String      ipAddress = "";
bool        use_ap_mode = true;


void setup()
{
    Serial.begin(115200);
    while (!Serial);
    delay(3000);
    Serial.println();


    /*********************************
     *  step 1 : Initialize power chip
    **********************************/
    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        Serial.println("Failed to initialize power.....");
        while (1) delay(5000);
    }

    // Camera power rails (keep original values)
    PMU.setALDO1Voltage(1800);  // CAM DVDD
    PMU.enableALDO1();
    PMU.setALDO2Voltage(2800);  // CAM DVDD
    PMU.enableALDO2();
    PMU.setALDO4Voltage(3000);  // CAM AVDD
    PMU.enableALDO4();

    PMU.disableTSPinMeasure();  // important


    // 💥 NEW → diagnostics PMU setup
    initDiagnostics();



    /*********************************
     *  step 2 : start network
    **********************************/
    if (use_ap_mode) {

        WiFi.mode(WIFI_AP);
        hostName += WiFi.macAddress().substring(0, 5);
        WiFi.softAP(hostName.c_str());
        ipAddress = WiFi.softAPIP().toString();

        Serial.print("Started AP mode host name :");
        Serial.print(hostName);
        Serial.print("IP address is :");
        Serial.println(ipAddress);

    } else {

        wifiMulti.addAP("ssid_from_AP_1", "your_password_for_AP_1");
        wifiMulti.addAP("ssid_from_AP_2", "your_password_for_AP_2");
        wifiMulti.addAP("ssid_from_AP_3", "your_password_for_AP_3");

        Serial.println("Connecting Wifi...");
        if (wifiMulti.run() == WL_CONNECTED) {
            Serial.println("\nWiFi connected");
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());
        }
    }



    /*********************************
     *  step 3 : Initialize camera
    **********************************/
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk  = XCLK_GPIO_NUM;
    config.pin_pclk  = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href  = HREF_GPIO_NUM;

    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.frame_size   = FRAMESIZE_UXGA;
    config.pixel_format = PIXFORMAT_JPEG;    // streaming
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count     = 1;

    if (config.pixel_format == PIXFORMAT_JPEG) {
        if (psramFound()) {
            config.jpeg_quality = 10;
            config.fb_count = 2;
            config.grab_mode = CAMERA_GRAB_LATEST;
        } else {
            config.frame_size = FRAMESIZE_SVGA;
            config.fb_location = CAMERA_FB_IN_DRAM;
        }
    } else {
        config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
        config.fb_count = 2;
#endif
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        while (1) delay(5000);
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, -2);
    }
    if (config.pixel_format == PIXFORMAT_JPEG) {
        s->set_framesize(s, FRAMESIZE_QVGA);
    }



    // 💥 NEW → run complete diagnostics (PMU, GPIO, LEDC, camera registers)
    runDiagnostics();



    /*********************************
     *  step 4 : start camera web server
    **********************************/
    startCameraServer();

    Serial.println("Camera Ready!");
    Serial.print("Open http://");
    Serial.println(ipAddress);
}


void loop()
{
    delay(10000);
}
