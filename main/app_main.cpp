#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>

#include <dirent.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "driver/gpio.h"

// REAL JPEG decoder (TJpgDec via espressif/esp_jpeg)
#include "jpeg_decoder.h"

// TFLite Micro / esp-tflite-micro
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

// ESP32 Camera (supports OV7675, OV7670, OV2640)
#include "esp_camera.h"

static const char *TAG = "VESPA_YOLO_S3";

// SDMMC GPIOs for T-SIM7080G-S3 (1-bit SDMMC)
#define SDMMC_CMD_GPIO  39
#define SDMMC_CLK_GPIO  38
#define SDMMC_D0_GPIO   40

// Camera GPIOs for T-SIM7080G-S3
// Compatible with OV7675 (working), OV7670, and OV2640 (DVP interface)
#define CAM_PIN_PWDN   -1  // Not used (GPIO32 causes watchdog issues)
#define CAM_PIN_RESET  18  // Hardware reset pin
#define CAM_PIN_XCLK   8   // External clock
#define CAM_PIN_SIOD   2   // I2C data (SCCB)
#define CAM_PIN_SIOC   1   // I2C clock (SCCB)
#define CAM_PIN_D7      9   // Data bit 7 (MSB)
#define CAM_PIN_D6     10  // Data bit 6
#define CAM_PIN_D5     11  // Data bit 5 (GPIO39 conflicts with SDMMC - must unmount SD first)
#define CAM_PIN_D4     13  // Data bit 4
#define CAM_PIN_D3     21  // Data bit 3
#define CAM_PIN_D2     48  // Data bit 2
#define CAM_PIN_D1     47  // Data bit 1
#define CAM_PIN_D0     14  // Data bit 0 (LSB)
#define CAM_PIN_VSYNC   16  // Vertical sync
#define CAM_PIN_HREF    17  // Horizontal reference
#define CAM_PIN_PCLK    12  // Pixel clock

// Default frame size (QQVGA works reliably with OV7675)
// Can be changed to FRAMESIZE_QVGA, FRAMESIZE_VGA, etc. if needed
#define CAMERA_DEFAULT_FRAME_SIZE FRAMESIZE_QQVGA

// Camera configuration structure
static camera_config_t get_camera_config(framesize_t frame_size = FRAMESIZE_QQVGA)
{
    camera_config_t config = {};  // Zero-initialize to ensure all fields are set
    
    // Pin assignments - using correct T-SIM7080G-S3 pin configuration
    config.pin_pwdn     = CAM_PIN_PWDN;
    config.pin_reset    = CAM_PIN_RESET;
    config.pin_xclk     = CAM_PIN_XCLK;
    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_d7       = CAM_PIN_D7;
    config.pin_d6       = CAM_PIN_D6;
    config.pin_d5       = CAM_PIN_D5;
    config.pin_d4       = CAM_PIN_D4;
    config.pin_d3       = CAM_PIN_D3;
    config.pin_d2       = CAM_PIN_D2;
    config.pin_d1       = CAM_PIN_D1;
    config.pin_d0       = CAM_PIN_D0;
    config.pin_vsync    = CAM_PIN_VSYNC;
    config.pin_href     = CAM_PIN_HREF;
    config.pin_pclk     = CAM_PIN_PCLK;
    
    config.xclk_freq_hz = 24000000;  // 24 MHz - works well with OV7675/OV7670
    config.frame_size = frame_size;
    config.pixel_format = PIXFORMAT_RGB565;  // RGB565 format for direct processing
    config.jpeg_quality = 12;  // Not used for RGB565
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    
    return config;
}

// Tensor arena in PSRAM (adjust if needed)
static constexpr size_t TENSOR_ARENA_SIZE = 2 * 1024 * 1024; // 2MB

// Max JPEG decode buffer (must be small enough for PSRAM fragmentation)
static constexpr size_t JPEG_DECODE_MAX_BYTES = 512 * 1024; // 512 KB

// Class labels (index 0..3)
static const char *kClassNames[4] = {
    "Apis mellifera (Honeybee)",        // Class 0
    "Vespa crabro (European hornet)",   // Class 1
    "Vespula sp. (Yellowjacket)",       // Class 2
    "Vespa velutina (Asian hornet)"     // Class 3 - target
};

// Globals for TFLM
static const tflite::Model       *g_model        = nullptr;
static uint8_t                   *g_model_data   = nullptr;
static uint8_t                   *g_tensor_arena = nullptr;
static tflite::MicroInterpreter  *g_interpreter  = nullptr;

static void log_heap(const char *stage)
{
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t free_psram    = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "[HEAP] %s: free internal=%u bytes, free PSRAM=%u bytes",
             stage,
             static_cast<unsigned>(free_internal),
             static_cast<unsigned>(free_psram));
}

static bool init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "NVS initialized");
    log_heap("after NVS init");
    return true;
}

// Global SD card pointer for unmounting
static sdmmc_card_t *g_sd_card = nullptr;

static bool mount_sdcard()
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT; // 1-bit mode for reliability

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk   = (gpio_num_t)SDMMC_CLK_GPIO;
    slot_config.cmd   = (gpio_num_t)SDMMC_CMD_GPIO;
    slot_config.d0    = (gpio_num_t)SDMMC_D0_GPIO;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed   = false;
    mount_config.max_files                = 10;
    mount_config.allocation_unit_size     = 16 * 1024;
    mount_config.disk_status_check_enable = true;
    mount_config.use_one_fat              = false;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(
        "/sdcard",
        &host,
        &slot_config,
        &mount_config,
        &g_sd_card
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "SD mounted OK");
    sdmmc_card_print_info(stdout, g_sd_card);

    log_heap("after SD mount");
    return true;
}

static bool unmount_sdcard()
{
    if (g_sd_card == nullptr) {
        ESP_LOGW(TAG, "SD card not mounted, nothing to unmount");
        return true;
    }

    ESP_LOGI(TAG, "Unmounting SD card to free GPIO39 for camera...");
    
    esp_err_t ret = esp_vfs_fat_sdcard_unmount("/sdcard", g_sd_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
        return false;
    }

    g_sd_card = nullptr;
    ESP_LOGI(TAG, "SD card unmounted successfully");
    
    // Give system time to release GPIO39
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Explicitly reset GPIO39 to input mode to release it from SDMMC
    gpio_reset_pin((gpio_num_t)SDMMC_CMD_GPIO);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    ESP_LOGI(TAG, "GPIO39 released and reset - now available for camera");
    log_heap("after SD unmount");
    return true;
}

// Verify camera pin configuration against documentation
static void verify_camera_pins(const camera_config_t &config)
{
    ESP_LOGI(TAG, "=== Camera Pin Verification ===");
    
    // Expected pins from T-SIM7080G-S3 documentation
    struct {
        const char *name;
        int expected;
        int actual;
    } pin_check[] = {
        {"XCLK",  8,  config.pin_xclk},
        {"SIOD",  2,  config.pin_sccb_sda},
        {"SIOC",  1,  config.pin_sccb_scl},
        {"VSYNC", 16, config.pin_vsync},
        {"HREF",  17, config.pin_href},
        {"PCLK",  12, config.pin_pclk},
        {"D0/Y2", 14, config.pin_d0},
        {"D1/Y3", 47, config.pin_d1},
        {"D2/Y4", 48, config.pin_d2},
        {"D3/Y5", 21, config.pin_d3},
        {"D4/Y6", 13, config.pin_d4},
        {"D5/Y7", 11, config.pin_d5},
        {"D6/Y8", 10, config.pin_d6},
        {"D7/Y9", 9,  config.pin_d7},
    };
    
    bool all_match = true;
    for (size_t i = 0; i < sizeof(pin_check)/sizeof(pin_check[0]); i++) {
        if (pin_check[i].expected == pin_check[i].actual) {
            ESP_LOGI(TAG, "  [OK] %-8s: GPIO%2d (matches docs)", 
                     pin_check[i].name, pin_check[i].actual);
        } else {
            ESP_LOGW(TAG, "  [MISMATCH] %-8s: GPIO%2d (expected GPIO%2d from docs)", 
                     pin_check[i].name, pin_check[i].actual, pin_check[i].expected);
            all_match = false;
        }
    }
    
    // Check for known conflicts
    ESP_LOGI(TAG, "=== Pin Conflict Check ===");
    if (config.pin_pclk == 12) {
        ESP_LOGW(TAG, "  GPIO12 (PCLK) is shared with FSPICLK (SPI) - ensure SPI is not active");
    }
    if (config.pin_d5 == 11) {
        ESP_LOGW(TAG, "  GPIO11 (D5/Y7) is shared with FSPID (SPI) - ensure SPI is not active");
    }
    if (config.pin_d4 == 13) {
        ESP_LOGW(TAG, "  GPIO13 (D4/Y6) is shared with FSPIQ (SPI) - ensure SPI is not active");
    }
    if (config.pin_d6 == 10) {
        ESP_LOGW(TAG, "  GPIO10 (D6/Y8) is shared with FSPICS0 (SPI) - ensure SPI is not active");
    }
    
    // Check SDMMC conflicts (should be resolved by unmounting)
    if (config.pin_d5 == 39) {
        ESP_LOGE(TAG, "  [CONFLICT] GPIO39 (D5) conflicts with SDMMC CMD - SD must be unmounted!");
    }
    
    if (all_match) {
        ESP_LOGI(TAG, "=== Pin verification: ALL PINS MATCH DOCUMENTATION ===");
    } else {
        ESP_LOGW(TAG, "=== Pin verification: SOME PINS DIFFER FROM DOCUMENTATION ===");
        ESP_LOGW(TAG, "If camera doesn't work, verify actual hardware pinout");
    }
}

static bool init_camera(framesize_t frame_size = CAMERA_DEFAULT_FRAME_SIZE)
{
    ESP_LOGI(TAG, "Initializing camera...");
    
    // Hardware reset sequence (required for reliable initialization)
    if (CAM_PIN_RESET >= 0) {
        gpio_config_t reset_gpio = {};
        reset_gpio.pin_bit_mask = (1ULL << CAM_PIN_RESET);
        reset_gpio.mode = GPIO_MODE_OUTPUT;
        reset_gpio.pull_up_en = GPIO_PULLUP_DISABLE;
        reset_gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
        reset_gpio.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&reset_gpio);
        
        // Reset sequence: LOW -> wait -> HIGH -> wait
        gpio_set_level((gpio_num_t)CAM_PIN_RESET, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level((gpio_num_t)CAM_PIN_RESET, 1);
        vTaskDelay(pdMS_TO_TICKS(500));  // Wait for sensor to stabilize
    }
    
    vTaskDelay(pdMS_TO_TICKS(300));  // Additional stabilization delay
    camera_config_t config = get_camera_config(frame_size);
    
    // Verify pin configuration (optional, useful for debugging)
    verify_camera_pins(config);
    
    // Validate that all pins have valid GPIO numbers before calling camera init
    // This helps catch pin configuration errors early
    bool pins_valid = true;
    if (config.pin_pclk < 0 || config.pin_pclk > 48) pins_valid = false;
    if (config.pin_vsync < 0 || config.pin_vsync > 48) pins_valid = false;
    if (config.pin_href < 0 || config.pin_href > 48) pins_valid = false;
    if (config.pin_xclk < 0 || config.pin_xclk > 48) pins_valid = false;
    for (int i = 0; i < 8; i++) {
        int pin = (i == 0) ? config.pin_d0 : (i == 1) ? config.pin_d1 : 
                  (i == 2) ? config.pin_d2 : (i == 3) ? config.pin_d3 :
                  (i == 4) ? config.pin_d4 : (i == 5) ? config.pin_d5 :
                  (i == 6) ? config.pin_d6 : config.pin_d7;
        if (pin < 0 || pin > 48) pins_valid = false;
    }
    
    if (!pins_valid) {
        ESP_LOGE(TAG, "Invalid pin configuration - GPIO numbers out of range");
        return false;
    }
    
    ESP_LOGI(TAG, "Calling esp_camera_init()...");
    esp_err_t err = esp_camera_init(&config);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Error code: 0x%x", err);
        return false;
    }
    
    ESP_LOGI(TAG, "Camera initialized successfully");
    
    // Configure sensor to explicitly start frame generation
    sensor_t *s = esp_camera_sensor_get();
    if (s != nullptr) {
        // Log detected sensor type (PID: OV2640=0x26, OV7670=0x76, OV7675=0x75)
        ESP_LOGI(TAG, "Detected sensor: PID=0x%02X, VER=0x%02X", s->id.PID, s->id.VER);
        if (s->id.PID == 0x26) {
            ESP_LOGI(TAG, "Sensor: OV2640");
        } else if (s->id.PID == 0x76) {
            ESP_LOGI(TAG, "Sensor: OV7670");
        } else if (s->id.PID == 0x75) {
            ESP_LOGI(TAG, "Sensor: OV7675");
        }
        
        // Configure sensor (null checks prevent crashes with sensors that don't support all functions)
        if (s->set_framesize != nullptr) {
            s->set_framesize(s, frame_size);
            vTaskDelay(pdMS_TO_TICKS(200));  // Give sensor time to configure
        } else {
            ESP_LOGW(TAG, "set_framesize function pointer is null - skipping");
        }
        
        // Enable auto exposure and gain - required for frame generation
        if (s->set_exposure_ctrl != nullptr) {
            s->set_exposure_ctrl(s, 1);  // Auto exposure ON
        } else {
            ESP_LOGW(TAG, "set_exposure_ctrl function pointer is null - skipping");
        }
        
        if (s->set_gain_ctrl != nullptr) {
            s->set_gain_ctrl(s, 1);      // Auto gain ON
        } else {
            ESP_LOGW(TAG, "set_gain_ctrl function pointer is null - skipping");
        }
        
        if (s->set_whitebal != nullptr) {
            s->set_whitebal(s, 1);       // Auto white balance ON
        } else {
            ESP_LOGW(TAG, "set_whitebal function pointer is null - skipping");
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));  // Give sensor time to stabilize
        
        // Set gain ceiling to allow sensor to work in various lighting
        if (s->set_gainceiling != nullptr) {
            s->set_gainceiling(s, GAINCEILING_4X);
        } else {
            ESP_LOGW(TAG, "set_gainceiling function pointer is null - skipping");
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));  // Final stabilization delay
        
        // Set brightness and contrast to default values
        if (s->set_brightness != nullptr) {
            s->set_brightness(s, 0);  // 0 = default brightness
        } else {
            ESP_LOGW(TAG, "set_brightness function pointer is null - skipping");
        }
        
        if (s->set_contrast != nullptr) {
            s->set_contrast(s, 0);   // 0 = default contrast
        } else {
            ESP_LOGW(TAG, "set_contrast function pointer is null - skipping");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Set saturation to default
        if (s->set_saturation != nullptr) {
            s->set_saturation(s, 0);  // 0 = default saturation
        } else {
            ESP_LOGW(TAG, "set_saturation function pointer is null - skipping");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
        
        ESP_LOGI(TAG, "Sensor configured for frame generation");
    } else {
        ESP_LOGE(TAG, "Failed to get sensor handle - cannot configure");
    }
    
    log_heap("after camera init");
    return true;
}

static camera_fb_t* capture_frame()
{
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return nullptr;
    }
    
    if (fb->len == 0) {
        ESP_LOGE(TAG, "Camera frame is empty");
        esp_camera_fb_return(fb);
        return nullptr;
    }
    
    ESP_LOGI(TAG, "Frame captured: %dx%d, %u bytes, format=%d",
             fb->width, fb->height, (unsigned)fb->len, fb->format);
    
    return fb;
}

static void dump_operators(const tflite::Model *model)
{
    if (!model || !model->operator_codes()) {
        ESP_LOGW(TAG, "Model has no operator codes");
        return;
    }

    auto *op_codes = model->operator_codes();
    int count = op_codes->size();
    ESP_LOGI(TAG, "Model has %d operators", count);

    for (int i = 0; i < count; ++i) {
        const tflite::OperatorCode *oc = op_codes->Get(i);
        if (!oc) {
            ESP_LOGW(TAG, "OP[%d]: null OperatorCode", i);
            continue;
        }

        auto builtin = oc->builtin_code();
        auto custom  = oc->custom_code();

        if (builtin == tflite::BuiltinOperator_CUSTOM) {
            std::string cname = custom ? std::string(custom->c_str()) : std::string("<null>");
            ESP_LOGW(TAG, "OP[%d]: CUSTOM builtin_code=%d name='%s'",
                     i,
                     static_cast<int>(builtin),
                     cname.c_str());
        } else {
            ESP_LOGI(TAG, "OP[%d]: builtin builtin_code=%d",
                     i,
                     static_cast<int>(builtin));
        }
    }
}

static bool load_model_from_sd(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open model file: %s", path);
        return false;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0) {
        ESP_LOGE(TAG, "Model file size invalid: %ld", size);
        fclose(f);
        return false;
    }

    ESP_LOGI(TAG, "Model size: %ld bytes", size);
    log_heap("before model alloc");

    g_model_data = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_model_data) {
        ESP_LOGE(TAG, "Failed to allocate %ld bytes for model in PSRAM", size);
        fclose(f);
        return false;
    }

    ESP_LOGI(TAG, "Reading model from SD card (this may take a few seconds)...");
    // Read in chunks to allow watchdog feeding and progress logging
    const size_t chunk_size = 64 * 1024;  // 64KB chunks
    size_t total_read = 0;
    size_t remaining = size;
    
    while (remaining > 0) {
        size_t to_read = (remaining > chunk_size) ? chunk_size : remaining;
        size_t chunk_read = fread(g_model_data + total_read, 1, to_read, f);
        
        if (chunk_read == 0) {
            ESP_LOGE(TAG, "Read error at offset %u", (unsigned)total_read);
            fclose(f);
            return false;
        }
        
        total_read += chunk_read;
        remaining -= chunk_read;
        
        // Log progress every 512KB
        if (total_read % (512 * 1024) < chunk_size || total_read == size) {
            ESP_LOGI(TAG, "Model read progress: %u / %ld bytes (%.1f%%)",
                     (unsigned)total_read, size, (100.0f * total_read) / size);
        }
        
        // Feed watchdog during long read operation
        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay to allow watchdog feeding
    }
    
    fclose(f);
    ESP_LOGI(TAG, "Model file read complete: %u bytes", (unsigned)total_read);

    if (total_read != (size_t)size) {
        ESP_LOGE(TAG, "Model read mismatch: expected %ld, got %u",
                 size,
                 static_cast<unsigned>(total_read));
        return false;
    }

    log_heap("after model alloc");
    ESP_LOGI(TAG, "Model loaded, size=%ld bytes", size);

    return true;
}

static bool init_tflm(uint8_t *tensor_arena, size_t arena_size)
{
    if (!g_model_data) {
        ESP_LOGE(TAG, "Model data is null, cannot init TFLM");
        return false;
    }

    g_model = tflite::GetModel(g_model_data);
    if (g_model == nullptr) {
        ESP_LOGE(TAG, "GetModel() returned null");
        return false;
    }

    dump_operators(g_model);

    ESP_LOGW(TAG, "Skipping TFLite schema version check (no TFLITE_SCHEMA_VERSION)");

    ESP_LOGI(TAG, "[HEAP] before arena alloc: free internal=%u bytes, free PSRAM=%u bytes",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    static tflite::MicroMutableOpResolver<64> resolver;

    // Convolutions & pooling
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddTransposeConv();
    resolver.AddAveragePool2D();
    resolver.AddMaxPool2D();

    // Elementwise arithmetic
    resolver.AddAdd();
    resolver.AddMul();
    resolver.AddSub();
    resolver.AddMaximum();
    resolver.AddMinimum();

    // Quantization
    resolver.AddQuantize();
    resolver.AddDequantize();

    // Activation functions
    resolver.AddRelu();
    resolver.AddRelu6();
    resolver.AddLogistic();
    resolver.AddSoftmax();

    // Reshaping / slicing / concat
    resolver.AddReshape();
    resolver.AddStridedSlice();
    resolver.AddPad();
    resolver.AddPadV2();
    resolver.AddConcatenation();
    resolver.AddTranspose();

    // FC & resize
    resolver.AddFullyConnected();
    resolver.AddResizeNearestNeighbor();
    resolver.AddResizeBilinear();

    static tflite::MicroInterpreter static_interpreter(
        g_model,
        resolver,
        tensor_arena,
        arena_size
    );

    g_interpreter = &static_interpreter;

    ESP_LOGI(TAG, "[HEAP] after arena alloc: free internal=%u bytes, free PSRAM=%u bytes",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    TfLiteStatus alloc_status = g_interpreter->AllocateTensors();
    if (alloc_status != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed");
        return false;
    }

    ESP_LOGI(TAG, "TFLM interpreter initialized and tensors allocated");
    log_heap("after AllocateTensors");

    TfLiteTensor *input = g_interpreter->input(0);
    if (input) {
        ESP_LOGI(TAG, "Input tensor: type=%d, dims_count=%d",
                 input->type,
                 input->dims ? input->dims->size : -1);
        if (input->dims) {
            std::string shape = "[";
            for (int i = 0; i < input->dims->size; ++i) {
                shape += std::to_string(input->dims->data[i]);
                if (i + 1 < input->dims->size) shape += ", ";
            }
            shape += "]";
            ESP_LOGI(TAG, "Input dims: %s", shape.c_str());
        }
    }

    return true;
}

// --- Save preprocessed RGB as PPM next to original ---

static bool save_rgb_to_ppm_next_to_original(const char *orig_path,
                                             const std::vector<uint8_t> &rgb,
                                             int w,
                                             int h)
{
    if (rgb.size() < (size_t)w * (size_t)h * 3) {
        ESP_LOGE(TAG, "RGB buffer too small for %dx%d image", w, h);
        return false;
    }

    std::string p(orig_path);
    size_t dot_pos = p.find_last_of('.');
    if (dot_pos == std::string::npos) {
        dot_pos = p.size();
    }
    std::string base = p.substr(0, dot_pos);
    std::string out_path = base + "_192x192.ppm";

    FILE *f = fopen(out_path.c_str(), "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open PPM file for write: %s", out_path.c_str());
        return false;
    }

    fprintf(f, "P6\n%d %d\n255\n", w, h);

    size_t written = fwrite(rgb.data(), 1, (size_t)w * (size_t)h * 3, f);
    fclose(f);

    if (written != (size_t)w * (size_t)h * 3) {
        ESP_LOGW(TAG, "PPM write size mismatch: expected %u, wrote %u",
                 (unsigned)((size_t)w * (size_t)h * 3),
                 (unsigned)written);
    }

    ESP_LOGI(TAG, "Saved preprocessed PPM: %s", out_path.c_str());
    return true;
}

// --- REAL JPEG → RGB888 using the correct Espressif decoder ---

static bool decode_jpeg_to_rgb888(const char *path,
                                  std::vector<uint8_t> &rgb_data,
                                  int &width,
                                  int &height)
{
    ESP_LOGI(TAG, "decode_jpeg_to_rgb888(): decoding %s via esp_jpeg (TJpgDec)", path);

    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open image file: %s", path);
        return false;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0) {
        ESP_LOGE(TAG, "Invalid JPEG size: %ld", size);
        fclose(f);
        return false;
    }

    // Read JPEG into RAM
    std::vector<uint8_t> jpeg_bytes(size);
    fread(jpeg_bytes.data(), 1, size, f);
    fclose(f);

    // Allocate output buffer (RGB888)
    uint8_t *out_buf = (uint8_t *)heap_caps_malloc(
        JPEG_DECODE_MAX_BYTES,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );

    if (!out_buf) {
        ESP_LOGE(TAG, "Failed to allocate JPEG decode buffer (%u bytes)",
                 (unsigned)JPEG_DECODE_MAX_BYTES);
        return false;
    }

    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata          = jpeg_bytes.data(),
        .indata_size     = (uint32_t)jpeg_bytes.size(),
        .outbuf          = out_buf,
        .outbuf_size     = (uint32_t)JPEG_DECODE_MAX_BYTES,
        .out_format      = JPEG_IMAGE_FORMAT_RGB888,
        .out_scale       = JPEG_IMAGE_SCALE_1_4,

        .flags = {
            .swap_color_bytes = 0   // no RGB/BGR swap
        },

        .advanced = {
            .working_buffer       = NULL,   // let decoder allocate internal buffer
            .working_buffer_size  = 0       // required when working_buffer is NULL
        },

        .priv = {
            .read = 0   // MUST initialize!
        }
    };

    esp_jpeg_image_output_t outimg = {};
    esp_err_t ret = esp_jpeg_decode(&jpeg_cfg, &outimg);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_jpeg_decode failed: %s", esp_err_to_name(ret));
        free(out_buf);
        return false;
    }

    width  = outimg.width;
    height = outimg.height;

    size_t out_size = width * height * 3;
    if (out_size > JPEG_DECODE_MAX_BYTES) {
        ESP_LOGE(TAG, "Decoded image too large (%u bytes)", (unsigned)out_size);
        free(out_buf);
        return false;
    }

    rgb_data.assign(out_buf, out_buf + out_size);
    free(out_buf);

    ESP_LOGI(TAG, "JPEG decoded OK: %dx%d RGB888 (%u bytes)",
             width, height, (unsigned)out_size);

    return true;
}

static void resize_rgb888_nearest(const std::vector<uint8_t> &src,
                                  int src_w, int src_h,
                                  std::vector<uint8_t> &dst,
                                  int dst_w, int dst_h)
{
    dst.resize((size_t)dst_w * (size_t)dst_h * 3);

    for (int y = 0; y < dst_h; ++y) {
        int src_y = (y * src_h) / dst_h;
        if (src_y >= src_h) src_y = src_h - 1;

        for (int x = 0; x < dst_w; ++x) {
            int src_x = (x * src_w) / dst_w;
            if (src_x >= src_w) src_x = src_w - 1;

            size_t dst_idx = ((size_t)y * dst_w + x) * 3;
            size_t src_idx = ((size_t)src_y * src_w + src_x) * 3;

            dst[dst_idx + 0] = src[src_idx + 0];
            dst[dst_idx + 1] = src[src_idx + 1];
            dst[dst_idx + 2] = src[src_idx + 2];
        }
    }
}

// Convert RGB565 camera frame to RGB888
// RGB565 format: RRRRR GGGGGG BBBBB (16 bits per pixel)
// RGB888 format: RRRRRRRR GGGGGGGG BBBBBBBB (24 bits per pixel)
static void convert_rgb565_to_rgb888(const uint8_t *rgb565_data,
                                     int width,
                                     int height,
                                     std::vector<uint8_t> &rgb888_data)
{
    if (!rgb565_data || width <= 0 || height <= 0) {
        ESP_LOGE(TAG, "Invalid parameters for RGB565 conversion: width=%d, height=%d", width, height);
        return;
    }

    size_t pixel_count = (size_t)width * (size_t)height;
    size_t expected_rgb565_size = pixel_count * 2;  // 2 bytes per pixel
    
    rgb888_data.resize(pixel_count * 3);

    for (size_t i = 0; i < pixel_count; ++i) {
        size_t rgb565_idx = i * 2;
        if (rgb565_idx + 1 >= expected_rgb565_size) {
            ESP_LOGE(TAG, "RGB565 buffer overflow at pixel %u", (unsigned)i);
            break;
        }
        
        // Read RGB565 pixel (little-endian: low byte first)
        uint16_t pixel = rgb565_data[rgb565_idx] | (rgb565_data[rgb565_idx + 1] << 8);
        
        // Extract color components (5 bits R, 6 bits G, 5 bits B)
        uint8_t r5 = (pixel >> 11) & 0x1F;  // 5 bits
        uint8_t g6 = (pixel >> 5) & 0x3F;   // 6 bits
        uint8_t b5 = pixel & 0x1F;          // 5 bits
        
        // Convert to 8-bit by scaling (5-bit: multiply by 8.23, 6-bit: multiply by 4.03)
        // Simplified: 5-bit -> 8-bit: (value * 255) / 31, 6-bit -> 8-bit: (value * 255) / 63
        rgb888_data[i * 3 + 0] = (r5 * 255) / 31;  // Red
        rgb888_data[i * 3 + 1] = (g6 * 255) / 63;  // Green
        rgb888_data[i * 3 + 2] = (b5 * 255) / 31;  // Blue
    }
}

// Preprocess camera frame (RGB565) to model input tensor (INT8)
static bool preprocess_camera_frame_to_input(camera_fb_t *fb)
{
    if (!fb || !fb->buf || fb->len == 0) {
        ESP_LOGE(TAG, "Invalid camera frame");
        return false;
    }

    if (fb->format != PIXFORMAT_RGB565) {
        ESP_LOGE(TAG, "Expected RGB565 format, got %d", fb->format);
        return false;
    }

    TfLiteTensor *input = g_interpreter->input(0);
    if (!input) {
        ESP_LOGE(TAG, "Input tensor null");
        return false;
    }

    if (!input->dims || input->dims->size != 4) {
        ESP_LOGE(TAG, "Expected input dims size 4");
        return false;
    }

    int n  = input->dims->data[0];
    int h  = input->dims->data[1];
    int w  = input->dims->data[2];
    int c  = input->dims->data[3];

    if (n != 1 || h != 192 || w != 192 || c != 3) {
        ESP_LOGW(TAG, "Model expects [1,192,192,3], got [%d,%d,%d,%d]",
                 n, h, w, c);
    }

    if (input->type != kTfLiteInt8) {
        ESP_LOGE(TAG, "Expected INT8 input tensor");
        return false;
    }

    // Convert RGB565 to RGB888
    std::vector<uint8_t> rgb888_src;
    convert_rgb565_to_rgb888(fb->buf, fb->width, fb->height, rgb888_src);
    
    // Verify conversion succeeded
    size_t expected_rgb888_size = (size_t)fb->width * (size_t)fb->height * 3;
    if (rgb888_src.size() != expected_rgb888_size) {
        ESP_LOGE(TAG, "RGB565 conversion failed: expected %u bytes, got %u",
                 (unsigned)expected_rgb888_size, (unsigned)rgb888_src.size());
        return false;
    }

    // Resize RGB888 to model input size (192x192)
    std::vector<uint8_t> rgb_resized;
    resize_rgb888_nearest(rgb888_src, fb->width, fb->height, rgb_resized, w, h);

    // Quantize to INT8 using model's scale and zero point
    const float scale   = input->params.scale;
    const int32_t zp    = input->params.zero_point;

    int8_t *dst = input->data.int8;
    int total   = w * h * c;

    for (int i = 0; i < total; ++i) {
        float f  = (float)rgb_resized[i] / 255.0f;
        float q  = f / scale + zp;
        int32_t qi = (int32_t)roundf(q);
        if (qi < -128) qi = -128;
        if (qi > 127)  qi = 127;
        dst[i] = (int8_t)qi;
    }

    ESP_LOGI(TAG, "Camera frame preprocessed: %dx%d→%dx%d",
             fb->width, fb->height, w, h);

    return true;
}

static bool preprocess_jpeg_to_input(const char *image_path)
{
    TfLiteTensor *input = g_interpreter->input(0);
    if (!input) {
        ESP_LOGE(TAG, "Input tensor null");
        return false;
    }

    if (!input->dims || input->dims->size != 4) {
        ESP_LOGE(TAG, "Expected input dims size 4");
        return false;
    }

    int n  = input->dims->data[0];
    int h  = input->dims->data[1];
    int w  = input->dims->data[2];
    int c  = input->dims->data[3];

    if (n != 1 || h != 192 || w != 192 || c != 3) {
        ESP_LOGW(TAG, "Model expects [1,192,192,3], got [%d,%d,%d,%d]",
                 n, h, w, c);
    }

    if (input->type != kTfLiteInt8) {
        ESP_LOGE(TAG, "Expected INT8 input tensor");
        return false;
    }

    std::vector<uint8_t> rgb_src;
    int src_w = 0, src_h = 0;

    if (!decode_jpeg_to_rgb888(image_path, rgb_src, src_w, src_h)) {
        ESP_LOGE(TAG, "decode_jpeg_to_rgb888() failed");
        return false;
    }

    std::vector<uint8_t> rgb_resized;
    resize_rgb888_nearest(rgb_src, src_w, src_h, rgb_resized, w, h);

    // Save preprocessed RGB as PPM next to the original for EVERY image
    save_rgb_to_ppm_next_to_original(image_path, rgb_resized, w, h);

    const float scale   = input->params.scale;
    const int32_t zp    = input->params.zero_point;

    int8_t *dst = input->data.int8;
    int total   = w * h * c;

    for (int i = 0; i < total; ++i) {
        float f  = (float)rgb_resized[i] / 255.0f;
        float q  = f / scale + zp;
        int32_t qi = (int32_t)roundf(q);
        if (qi < -128) qi = -128;
        if (qi > 127)  qi = 127;
        dst[i] = (int8_t)qi;
    }

    ESP_LOGI(TAG, "Preprocessing OK for %s (%dx%d→%dx%d)",
             image_path, src_w, src_h, w, h);

    return true;
}

static bool has_image_extension(const char *name)
{
    const char *dot = strrchr(name, '.');
    if (!dot) return false;
    return (strcasecmp(dot, ".jpg") == 0 ||
            strcasecmp(dot, ".jpeg") == 0);
}

static void collect_images_recursive(const std::string &root,
                                     std::vector<std::string> &out)
{
    DIR *dir = opendir(root.c_str());
    if (!dir) return;

    struct dirent *e;
    while ((e = readdir(dir)) != nullptr) {
        if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, ".."))
            continue;
        std::string full = root + "/" + e->d_name;

        struct stat st{};
        if (stat(full.c_str(), &st) != 0) continue;

        if (S_ISDIR(st.st_mode))
            collect_images_recursive(full, out);
        else if (S_ISREG(st.st_mode) && has_image_extension(e->d_name))
            out.push_back(full);
    }
    closedir(dir);
}

static bool run_inference_on_image(const char *image_path)
{
    ESP_LOGI(TAG, "=== Running inference: %s ===", image_path);

    if (!preprocess_jpeg_to_input(image_path)) {
        ESP_LOGE(TAG, "preprocessing FAILED");
        return false;
    }

    int64_t t0 = esp_timer_get_time();
    TfLiteStatus st = g_interpreter->Invoke();
    int64_t t1 = esp_timer_get_time();
    int64_t ms = (t1 - t0) / 1000;

    if (st != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke() FAILED (%lld ms)", (long long)ms);
        return false;
    }

    ESP_LOGI(TAG, "Invoke OK (%lld ms)", (long long)ms);

    TfLiteTensor *out0 = g_interpreter->output(0);
    if (!out0 || !out0->dims || out0->dims->size != 3) {
        ESP_LOGW(TAG, "Unexpected output shape");
        return true;
    }

    if (out0->type != kTfLiteInt8) {
        ESP_LOGW(TAG, "Output type not INT8");
        return true;
    }

    int anchors = out0->dims->data[2];
    int ch      = out0->dims->data[1];

    if (ch < 8) {
        ESP_LOGW(TAG, "Expected at least 8 channels");
        return true;
    }

    // ---- Classification & logging for EVERY image ----
    const float scale = out0->params.scale;
    const int   zp    = out0->params.zero_point;

    auto deq = [&](int8_t v) {
        return (float(v) - zp) * scale;
    };

    int8_t *d = out0->data.int8;

    float best_score = -1e30f;
    int   best_anchor = -1;
    int   best_class  = -1;

    for (int i = 0; i < anchors; ++i) {
        int base = i * ch;
        for (int cls = 0; cls < 4; ++cls) {
            float score = deq(d[base + 4 + cls]);
            if (score > best_score) {
                best_score  = score;
                best_anchor = i;
                best_class  = cls;
            }
        }
    }

    ESP_LOGI(TAG, "Result for %s:", image_path);
    ESP_LOGI(TAG, "  Best class: %s (cls=%d, anchor=%d, score=%.4f)",
             kClassNames[best_class], best_class, best_anchor, best_score);

    ESP_LOGI(TAG, "  Scores at best anchor:");
    int base = best_anchor * ch;
    for (int cls = 0; cls < 4; ++cls) {
        float s = deq(d[base + 4 + cls]);
        ESP_LOGI(TAG, "    cls %d (%s): %.4f",
                 cls, kClassNames[cls], s);
    }

    return true;
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting VESPA YOLO S3");

    if (!init_nvs()) return;
    if (!mount_sdcard()) return;

    // Load model from SD card before unmounting
    const char *model_path =
        "/sdcard/models/yolov8n_2025-07-15_192_full_integer_quant.tflite";

    if (!load_model_from_sd(model_path)) {
        ESP_LOGE(TAG, "Failed to load model from SD card");
        return;
    }

    // Allocate tensor arena in PSRAM
    g_tensor_arena = (uint8_t*)heap_caps_malloc(
        TENSOR_ARENA_SIZE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!g_tensor_arena) {
        ESP_LOGE(TAG, "Arena alloc FAILED");
        return;
    }

    if (!init_tflm(g_tensor_arena, TENSOR_ARENA_SIZE)) {
        ESP_LOGE(TAG, "TFLM initialization failed");
        return;
    }

    // Unmount SD card to free GPIO39 for camera (GPIO39 is shared with SDMMC CMD)
    if (!unmount_sdcard()) {
        ESP_LOGE(TAG, "Failed to unmount SD card - camera may fail due to GPIO39 conflict");
        return;
    }
    
    // Initialize camera with default frame size
    if (!init_camera(CAMERA_DEFAULT_FRAME_SIZE)) {
        ESP_LOGE(TAG, "Camera initialization failed");
        return;
    }
    
    // Wait for camera to stabilize and discard initial frames
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (int i = 0; i < 3; i++) {
        camera_fb_t* discard = esp_camera_fb_get();
        if (discard) {
            esp_camera_fb_return(discard);
        } else {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Verify camera is working with a test capture
    camera_fb_t* test_frame = capture_frame();
    if (!test_frame) {
        ESP_LOGE(TAG, "Camera test capture failed - camera may not be generating frames");
        return;
    }
    esp_camera_fb_return(test_frame);
    
    ESP_LOGI(TAG, "Camera initialized and verified - ready for inference");
    log_heap("after camera init");
    
    // Main loop: capture frame and run inference continuously
    ESP_LOGI(TAG, "Starting camera-based inference loop (continuous)");
    
    int loop_count = 0;
    while (true) {
        loop_count++;
        ESP_LOGI(TAG, "=== Loop iteration %d ===", loop_count);
        
        // Capture frame from camera
        camera_fb_t* fb = capture_frame();
        if (!fb) {
            ESP_LOGW(TAG, "Failed to capture frame, retrying...");
            vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to prevent tight loop
            continue;
        }

        ESP_LOGI(TAG, "=== Processing camera frame: %dx%d ===", fb->width, fb->height);

        // Preprocess camera frame to model input
        if (!preprocess_camera_frame_to_input(fb)) {
            ESP_LOGE(TAG, "Frame preprocessing failed");
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to prevent tight loop
            continue;
        }

        // Run inference
        int64_t t0 = esp_timer_get_time();
        TfLiteStatus st = g_interpreter->Invoke();
        int64_t t1 = esp_timer_get_time();
        int64_t ms = (t1 - t0) / 1000;

        if (st != kTfLiteOk) {
            ESP_LOGE(TAG, "Invoke() FAILED (%lld ms)", (long long)ms);
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to prevent tight loop
            continue;
        }

        ESP_LOGI(TAG, "Inference completed in %lld ms", (long long)ms);

        // Process and log results
        TfLiteTensor *out0 = g_interpreter->output(0);
        if (!out0 || !out0->dims || out0->dims->size != 3) {
            ESP_LOGW(TAG, "Unexpected output shape");
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to prevent tight loop
            continue;
        }

        if (out0->type != kTfLiteInt8) {
            ESP_LOGW(TAG, "Output type not INT8");
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to prevent tight loop
            continue;
        }

        int anchors = out0->dims->data[2];
        int ch      = out0->dims->data[1];

        if (ch < 8) {
            ESP_LOGW(TAG, "Expected at least 8 channels");
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to prevent tight loop
            continue;
        }

        // Classification and logging
        const float scale = out0->params.scale;
        const int   zp    = out0->params.zero_point;

        auto deq = [&](int8_t v) {
            return (float(v) - zp) * scale;
        };

        int8_t *d = out0->data.int8;

        float best_score = -1e30f;
        int   best_anchor = -1;
        int   best_class  = -1;

        for (int i = 0; i < anchors; ++i) {
            int base = i * ch;
            for (int cls = 0; cls < 4; ++cls) {
                float score = deq(d[base + 4 + cls]);
                if (score > best_score) {
                    best_score  = score;
                    best_anchor = i;
                    best_class  = cls;
                }
            }
        }

        // Safety check: ensure we found valid results
        if (best_anchor < 0 || best_class < 0 || best_class >= 4) {
            ESP_LOGW(TAG, "Invalid inference results (anchor=%d, class=%d), skipping logging",
                     best_anchor, best_class);
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to prevent tight loop
            continue;
        }

        ESP_LOGI(TAG, "Camera inference result:");
        ESP_LOGI(TAG, "  Best class: %s (cls=%d, anchor=%d, score=%.4f)",
                 kClassNames[best_class], best_class, best_anchor, best_score);

        ESP_LOGI(TAG, "  Scores at best anchor:");
        int base = best_anchor * ch;
        for (int cls = 0; cls < 4; ++cls) {
            float s = deq(d[base + 4 + cls]);
            ESP_LOGI(TAG, "    cls %d (%s): %.4f",
                     cls, kClassNames[cls], s);
        }

        // Return frame buffer to camera driver
        esp_camera_fb_return(fb);

        // Loop continues immediately to capture next frame
    }
}