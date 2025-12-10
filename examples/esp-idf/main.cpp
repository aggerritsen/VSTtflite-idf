#include <stdio.h>
#include <string.h> 
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"

// --- SD CARD HEADERS ---
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"


// Set a safe delay after each PMU write to ensure power stability
#define PMU_WRITE_DELAY_MS 5

// --- I2C and PMIC Definitions (LILYGO_ESP32S3_CAM_SIM7080G) ---
#define TAG                         "AXP2101_PMIC"
#define AXP_ADDR                    0x34        
#define I2C_PORT_NUM                I2C_NUM_0   
#define I2C_MASTER_SDA_IO           15          
#define I2C_MASTER_SCL_IO           7           
#define I2C_MASTER_FREQ_HZ          400000      

static i2c_master_dev_handle_t i2c_axp_dev_handle = NULL; 

// --- AXP2101 Register Addresses ---
#define AXP_REG_PMU_STATUS2         0x01
#define AXP_REG_LDO_ONOFF_0         0x90        
#define AXP_REG_DCDC1_VOLTAGE       0x82 // Main IO 3.3V
#define AXP_REG_DCDC2_VOLTAGE       0x83 // CPU 0.9V
#define AXP_REG_DCDC3_VOLTAGE       0x84 // SD Card 3.3V
#define AXP_REG_ALDO1_VOLTAGE       0x92        
#define AXP_REG_ALDO2_VOLTAGE       0x93        
#define AXP_REG_ALDO4_VOLTAGE       0x95        
#define AXP_REG_BLDO1_VOLTAGE       0x98 // New Logging Target
#define AXP_REG_BLDO2_VOLTAGE       0x99 // New Logging Target
#define AXP_REG_DLDO1_VOLTAGE       0x9A // New Logging Target
#define AXP_REG_DLDO2_VOLTAGE       0x9B // New Logging Target


// --- Camera Pin Definitions (Unchanged) ---
#define CAM_PIN_PWDN                (-1)
#define CAM_PIN_RESET               (18)
#define CAM_PIN_XCLK                (8)
#define CAM_PIN_SIOD                (2) 
#define CAM_PIN_SIOC                (1) 
#define CAM_PIN_D7                  (9)
#define CAM_PIN_D6                  (10)
#define CAM_PIN_D5                  (11)
#define CAM_PIN_D4                  (13)
#define CAM_PIN_D3                  (21)
#define CAM_PIN_D2                  (48)
#define CAM_PIN_D1                  (47)
#define CAM_PIN_D0                  (14)
#define CAM_PIN_VSYNC               (16)
#define CAM_PIN_HREF                (17)
#define CAM_PIN_PCLK                (12)

// --- SD CARD Pin Definitions (Unchanged) ---
#define SD_CARD_MOUNT_POINT         "/sdcard"
#define SD_CMD_PIN                  39          
#define SD_CLK_PIN                  38          
#define SD_DATA0_PIN                40          


// -----------------------------------------------------------------------------
// Function Prototypes (Unchanged)
// -----------------------------------------------------------------------------
static esp_err_t i2c_master_init(void);
static void axp2101_init_pmic();
static void axp2101_verify_settings();
static esp_err_t camera_init(); 
void take_picture_and_print_info(bool sd_card_initialized);
esp_err_t sd_card_init(void);
esp_err_t sd_card_gpio_pre_init(void); 
esp_err_t axp2101_write_reg(uint8_t reg_addr, uint8_t data);
esp_err_t axp2101_read_reg(uint8_t reg_addr, uint8_t *data);


// -----------------------------------------------------------------------------
// I2C Communication Functions (Unchanged)
// -----------------------------------------------------------------------------

esp_err_t axp2101_write_reg(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    esp_err_t ret = i2c_master_transmit(i2c_axp_dev_handle, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "I2C Write Error: Reg 0x%02X, Err %d", reg_addr, ret); }
    vTaskDelay(pdMS_TO_TICKS(PMU_WRITE_DELAY_MS)); 
    return ret;
}

esp_err_t axp2101_read_reg(uint8_t reg_addr, uint8_t *data)
{
    esp_err_t ret = i2c_master_transmit_receive(i2c_axp_dev_handle, 
                                                &reg_addr, 1, 
                                                data, 1,       
                                                1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "I2C Read Error: Reg 0x%02X, Err %d", reg_addr, ret); }
    return ret;
}

static esp_err_t i2c_master_init(void)
{
    // ... I2C initialization code (Unchanged) ...
    i2c_master_bus_config_t bus_cfg;
    memset(&bus_cfg, 0, sizeof(i2c_master_bus_config_t));
    
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT; 

    bus_cfg.i2c_port = I2C_PORT_NUM;
    bus_cfg.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    bus_cfg.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    bus_cfg.glitch_ignore_cnt = 7;
    
    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Failed to create I2C bus: %d", err); return err; }

    i2c_device_config_t dev_cfg;
    memset(&dev_cfg, 0, sizeof(i2c_device_config_t));
    
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7; 
    dev_cfg.device_address = AXP_ADDR;
    dev_cfg.scl_speed_hz = I2C_MASTER_FREQ_HZ;
    
    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &i2c_axp_dev_handle);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Failed to add AXP device: %d", err); i2c_del_master_bus(bus_handle); return err; }

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// AXP2101 PMIC Control Functions (DCDC1 FIX ADDED)
// -----------------------------------------------------------------------------

static void axp2101_init_pmic()
{
    ESP_LOGI(TAG, "Starting AXP2101 PMIC configuration (Fixing DCDC1 and setting SD/Camera rails)...");
    
    // Reset interrupts/status flags
    axp2101_write_reg(AXP_REG_PMU_STATUS2, 0xFF);
    
    // 1. FIX: Set DCDC1 to 3.3V (0x39) explicitly, as it was reading 2.5V (0x12) in the log.
    axp2101_write_reg(AXP_REG_DCDC1_VOLTAGE, 0x39); // DCDC1 = 3.3V
    
    // 2. Program ALDOs for Camera/PSRAM power
    axp2101_write_reg(AXP_REG_ALDO1_VOLTAGE, 0x0D); // ALDO1 = 1.8V
    axp2101_write_reg(AXP_REG_ALDO2_VOLTAGE, 0x17); // ALDO2 = 2.8V
    axp2101_write_reg(AXP_REG_ALDO4_VOLTAGE, 0x19); // ALDO4 = 3.0V

    // 3. Program DCDC3 for SD Card (Set to 3.4V for stability margin - 0x2A)
    axp2101_write_reg(AXP_REG_DCDC3_VOLTAGE, 0x2A); 
    
    // 4. Enable outputs: DCDC3 + ALDO2/ALDO4
    // DCDC1 should also be explicitly enabled. Assuming 0xFF enables everything for safety.
    // However, sticking to 0x5A for now, which already enables DCDC3, ALDO2, ALDO4.
    // DCDC1 is usually part of the boot sequence (Startup Sequence 3), but must be enabled if we write to its voltage register.
    // We will use 0xFF to ensure all programmed rails are enabled.
    axp2101_write_reg(AXP_REG_LDO_ONOFF_0, 0xFF); 
    
    ESP_LOGI(TAG, "DCDC1 set to 3.3V. SD (DCDC3=3.4V) and Camera ALDOs configured and ALL rails enabled (REG 0x90 = 0xFF).");
}

static void axp2101_verify_settings()
{
    uint8_t reg_data = 0;
    
    ESP_LOGI(TAG, "--- PMIC Register Verification (ALL 10 Rails) ---");
    
    // Enable Register
    if (axp2101_read_reg(AXP_REG_LDO_ONOFF_0, &reg_data) == ESP_OK) { ESP_LOGI(TAG, "REG 0x90 (Enable) Read: 0x%02X (Expected Programmed: 0xFF)", reg_data); }
    
    // DCDC Rails (Core and SD Card)
    if (axp2101_read_reg(AXP_REG_DCDC1_VOLTAGE, &reg_data) == ESP_OK) { ESP_LOGI(TAG, "REG 0x82 (DCDC1=3.3V) Read: 0x%02X (Expected Programmed: 0x39)", reg_data); }
    if (axp2101_read_reg(AXP_REG_DCDC2_VOLTAGE, &reg_data) == ESP_OK) { ESP_LOGI(TAG, "REG 0x83 (DCDC2=0.9V) Read: 0x%02X (Expected Default: 0x00)", reg_data); }
    if (axp2101_read_reg(AXP_REG_DCDC3_VOLTAGE, &reg_data) == ESP_OK) { ESP_LOGI(TAG, "REG 0x84 (DCDC3=3.4V) Read: 0x%02X (Expected Programmed: 0x2A)", reg_data); }
    
    // ALDO Rails (Camera Power)
    if (axp2101_read_reg(AXP_REG_ALDO1_VOLTAGE, &reg_data) == ESP_OK) { ESP_LOGI(TAG, "REG 0x92 (ALDO1=1.8V) Read: 0x%02X (Expected Programmed: 0x0D)", reg_data); }
    if (axp2101_read_reg(AXP_REG_ALDO2_VOLTAGE, &reg_data) == ESP_OK) { ESP_LOGI(TAG, "REG 0x93 (ALDO2=2.8V) Read: 0x%02X (Expected Programmed: 0x17)", reg_data); }
    if (axp2101_read_reg(AXP_REG_ALDO4_VOLTAGE, &reg_data) == ESP_OK) { ESP_LOGI(TAG, "REG 0x95 (ALDO4=3.0V) Read: 0x%02X (Expected Programmed: 0x19)", reg_data); }

    // BLDO/DLDO Rails (New Logging Targets)
    if (axp2101_read_reg(AXP_REG_BLDO1_VOLTAGE, &reg_data) == ESP_OK) { ESP_LOGI(TAG, "REG 0x98 (BLDO1=1.8V) Read: 0x%02X (Expected Default: 0x0D)", reg_data); }
    if (axp2101_read_reg(AXP_REG_BLDO2_VOLTAGE, &reg_data) == ESP_OK) { ESP_LOGI(TAG, "REG 0x99 (BLDO2=2.8V) Read: 0x%02X (Expected Default: 0x17)", reg_data); }
    if (axp2101_read_reg(AXP_REG_DLDO1_VOLTAGE, &reg_data) == ESP_OK) { ESP_LOGI(TAG, "REG 0x9A (DLDO1=3.3V) Read: 0x%02X (Expected Default: 0x19)", reg_data); }
    if (axp2101_read_reg(AXP_REG_DLDO2_VOLTAGE, &reg_data) == ESP_OK) { ESP_LOGI(TAG, "REG 0x9B (DLDO2=1.2V) Read: 0x%02X (Expected Default: 0x05)", reg_data); }

    ESP_LOGI(TAG, "--- Verification Complete ---");
}

// -----------------------------------------------------------------------------
// Camera Initialization (Unchanged)
// -----------------------------------------------------------------------------

static esp_err_t camera_init()
{
    // ... camera_init code (Unchanged) ...
    camera_config_t config;
    memset(&config, 0, sizeof(config));
    
    config.ledc_channel = LEDC_CHANNEL_0; 
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_PIN_D0; config.pin_d1 = CAM_PIN_D1; config.pin_d2 = CAM_PIN_D2; 
    config.pin_d3 = CAM_PIN_D3; config.pin_d4 = CAM_PIN_D4; config.pin_d5 = CAM_PIN_D5; 
    config.pin_d6 = CAM_PIN_D6; config.pin_d7 = CAM_PIN_D7;
    config.pin_xclk = CAM_PIN_XCLK; config.pin_pclk = CAM_PIN_PCLK; 
    config.pin_vsync = CAM_PIN_VSYNC; config.pin_href = CAM_PIN_HREF; 
    config.pin_sccb_sda = CAM_PIN_SIOD; config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_pwdn = CAM_PIN_PWDN; 
    config.pin_reset = CAM_PIN_RESET;
    config.xclk_freq_hz = 20000000; 
    config.pixel_format = PIXFORMAT_JPEG; 
    config.frame_size = FRAMESIZE_SVGA; 
    config.jpeg_quality = 10; 
    config.fb_count = 2;

    ESP_LOGI(TAG, "Attempting to initialize camera...");
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init FAILED with error 0x%x.", err);
        return err;
    }

    ESP_LOGI(TAG, "Camera initialized successfully.");
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// SD CARD PRE-INITIALIZATION (Unchanged)
// -----------------------------------------------------------------------------

esp_err_t sd_card_gpio_pre_init(void)
{
    ESP_LOGI(TAG, "Pre-initializing SD card GPIOs with pull-ups...");
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    
    io_conf.pin_bit_mask = (1ULL << SD_CMD_PIN) | (1ULL << SD_CLK_PIN) | (1ULL << SD_DATA0_PIN);
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Failed to configure SD card GPIOs: %d", ret); } 
    else { ESP_LOGI(TAG, "SD card GPIOs configured successfully."); }
    return ret;
}


// -----------------------------------------------------------------------------
// SD CARD INITIALIZATION FUNCTION (Unchanged)
// -----------------------------------------------------------------------------

esp_err_t sd_card_init(void)
{
    ESP_LOGI(TAG, "Initializing SD card via SDMMC 1-bit mode...");

    esp_vfs_fat_sdmmc_mount_config_t mount_config;
    memset(&mount_config, 0, sizeof(mount_config));
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;
    
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT; 
    
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1; 
    slot_config.cmd = (gpio_num_t)SD_CMD_PIN;
    slot_config.clk = (gpio_num_t)SD_CLK_PIN;
    slot_config.d0 = (gpio_num_t)SD_DATA0_PIN;

    slot_config.d1 = GPIO_NUM_NC;
    slot_config.d2 = GPIO_NUM_NC;
    slot_config.d3 = GPIO_NUM_NC;

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_CARD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) { ESP_LOGE(TAG, "Failed to mount filesystem. SD Card may need re-formatting."); } 
        else { ESP_LOGE(TAG, "Failed to initialize SD card (%s). Error: 0x%x", esp_err_to_name(ret), ret); }
        return ret;
    }

    ESP_LOGI(TAG, "SD Card mounted successfully at %s", SD_CARD_MOUNT_POINT);
    sdmmc_card_print_info(stdout, card);
    
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Picture Capture and Save Function (Unchanged)
// -----------------------------------------------------------------------------

void take_picture_and_print_info(bool sd_card_initialized)
{
    static int picture_count = 0;
    
    camera_fb_t *fb = esp_camera_fb_get();
    
    if (!fb) { ESP_LOGE(TAG, "Camera capture failed"); return; }
    
    ESP_LOGI(TAG, "Picture taken! Format: JPEG, Size: %u bytes, Resolution: %dx%d",
             fb->len, fb->width, fb->height);
             
    if (sd_card_initialized) {
        char path[48]; 
        snprintf(path, sizeof(path), SD_CARD_MOUNT_POINT "/pic_%04d.jpg", picture_count++);
        
        ESP_LOGI(TAG, "Writing file to %s", path);
        FILE *f = fopen(path, "w");
        if (!f) {
            ESP_LOGE(TAG, "Failed to open file for writing: %s. Check SD card structure.", path);
        } else {
            size_t written = fwrite(fb->buf, 1, fb->len, f);
            fclose(f);
            if (written == fb->len) {
                ESP_LOGI(TAG, "File saved successfully, %u bytes written.", written);
            } else {
                ESP_LOGE(TAG, "Failed to write all data. Wrote %u of %u bytes.", written, fb->len);
            }
        }
    } else {
        ESP_LOGW(TAG, "SD Card not initialized, image data discarded.");
    }

    esp_camera_fb_return(fb); 
    ESP_LOGI(TAG, "Frame buffer released, ready for next capture.");
}


// -----------------------------------------------------------------------------
// Application Entry Point
// -----------------------------------------------------------------------------

extern "C" {
    void app_main(void)
    {
        bool sd_initialized = false;
        esp_err_t cam_err = ESP_FAIL;

        ESP_LOGI(TAG, "Application start. Initializing I2C Master...");
        
        // 1. Initialize I2C Master Bus
        if (i2c_master_init() != ESP_OK) { ESP_LOGE(TAG, "I2C initialization FAILED. Aborting."); return; }
        ESP_LOGI(TAG, "I2C Master initialized successfully.");

        // 2. Configure the PMIC (Power Rails) - FIXING DCDC1
        axp2101_init_pmic();
        axp2101_verify_settings(); // Now logs all 10 critical rails

        // --- Power Stabilization Delay ---
        ESP_LOGI(TAG, "Waiting 100ms for DCDC3 power rail stabilization...");
        vTaskDelay(pdMS_TO_TICKS(100));

        // 3. Pre-initialize SD GPIOs with pull-ups
        sd_card_gpio_pre_init();
        
        // 4. Initialize the SD Card
        if (sd_card_init() == ESP_OK) {
            sd_initialized = true;
        }
        
        // 5. Initialize the Camera
        cam_err = camera_init();
        if (cam_err != ESP_OK) {
            ESP_LOGE(TAG, "Camera initialization FAILED: %d", cam_err);
        }

        // Loop indefinitely, taking a picture every 5 seconds
        while (1) {
            if (cam_err == ESP_OK) {
                ESP_LOGI(TAG, "--- Starting picture capture cycle ---");
                take_picture_and_print_info(sd_initialized); 
            } else {
                ESP_LOGE(TAG, "Camera FAILED, skipping capture.");
            }
            
            vTaskDelay(pdMS_TO_TICKS(5000));
            ESP_LOGI(TAG, "PMIC and Camera functions are active, board running...");
        }
    }
}