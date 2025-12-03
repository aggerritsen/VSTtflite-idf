#include "sdkconfig.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <dirent.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

// TensorFlow Lite Micro
#include "tensorflow/lite/core/api/error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

static constexpr gpio_num_t SDMMC_CMD_GPIO = GPIO_NUM_39;
static constexpr gpio_num_t SDMMC_CLK_GPIO = GPIO_NUM_38;
static constexpr gpio_num_t SDMMC_D0_GPIO  = GPIO_NUM_40;

static const char *TAG = "VESPA_YOLO_S3";

// ===================== TFLM GLOBALS =====================

static constexpr size_t TENSOR_ARENA_SIZE = 2 * 1024 * 1024; // 2 MB in PSRAM

static uint8_t *tensor_arena = nullptr;
static const tflite::Model *g_model = nullptr;
static tflite::MicroInterpreter *g_interpreter = nullptr;

// ===================== HEAP LOGGING =====================

static void log_heap_usage(const char *where)
{
    size_t free_int   = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "[HEAP] %s: free internal=%u bytes, free PSRAM=%u bytes",
             where, (unsigned)free_int, (unsigned)free_psram);
}

// ===================== MODEL LOAD =====================

static uint8_t *load_model_from_sd(const char *path, size_t &out_size)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open model file: %s", path);
        return nullptr;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0) {
        ESP_LOGE(TAG, "Model file size invalid: %ld", size);
        fclose(f);
        return nullptr;
    }

    ESP_LOGI(TAG, "Model size: %ld bytes", size);
    log_heap_usage("before model alloc");

    uint8_t *buffer = static_cast<uint8_t *>(
        heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate %ld bytes for model in PSRAM", size);
        fclose(f);
        return nullptr;
    }

    size_t read = fread(buffer, 1, size, f);
    fclose(f);

    if (read != static_cast<size_t>(size)) {
        ESP_LOGE(TAG, "Model read mismatch: read=%u expected=%ld",
                 static_cast<unsigned>(read), size);
        heap_caps_free(buffer);
        return nullptr;
    }

    out_size = static_cast<size_t>(size);
    log_heap_usage("after model alloc");
    return buffer;
}

// ===================== JPG HELPERS =====================

static bool has_jpg_extension(const std::string &name)
{
    auto dot_pos = name.find_last_of('.');
    if (dot_pos == std::string::npos) return false;
    std::string ext = name.substr(dot_pos + 1);
    for (auto &c : ext) c = static_cast<char>(tolower(c));
    return (ext == "jpg" || ext == "jpeg");
}

// ===================== MODEL OPERATOR LOGGING =====================

static void log_model_operators(const tflite::Model *model)
{
    if (!model) {
        ESP_LOGW(TAG, "log_model_operators: model is null");
        return;
    }

    auto subgraphs = model->subgraphs();
    if (!subgraphs || subgraphs->size() == 0) {
        ESP_LOGW(TAG, "Model has no subgraphs");
        return;
    }

    auto subgraph = subgraphs->Get(0);
    auto ops = subgraph->operators();
    auto op_codes = model->operator_codes();

    if (!ops || !op_codes) {
        ESP_LOGW(TAG, "Model has no operators or operator_codes");
        return;
    }

    ESP_LOGI(TAG, "Model has %d operators", static_cast<int>(ops->size()));

    for (int i = 0; i < ops->size(); ++i) {
        auto op = ops->Get(i);
        int opcode_index = op->opcode_index();
        auto op_code = op_codes->Get(opcode_index);

        auto builtin = op_code->builtin_code();
        auto custom  = op_code->custom_code();

        if (builtin == tflite::BuiltinOperator_CUSTOM) {
            const char *name = custom ? custom->c_str() : "<null>";
            ESP_LOGW(TAG, "OP[%d]: CUSTOM op_code_index=%d name='%s'",
                     i, opcode_index, name);
        } else {
            ESP_LOGI(TAG, "OP[%d]: builtin op_code_index=%d builtin_code=%d",
                     i, opcode_index, static_cast<int>(builtin));
        }
    }
}

// ===================== TFLM INIT =====================

static bool init_tflm(uint8_t *model_data, size_t model_size)
{
    (void)model_size;

    g_model = tflite::GetModel(model_data);
    if (!g_model) {
        ESP_LOGE(TAG, "GetModel() returned null");
        return false;
    }

    // Nieuw: log alle operators, inclusief CUSTOM namen
    log_model_operators(g_model);

    ESP_LOGW(TAG, "Skipping TFLite schema version check (no TFLITE_SCHEMA_VERSION)");

    static tflite::MicroMutableOpResolver<16> resolver;
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddReshape();
    resolver.AddLogistic();
    resolver.AddSoftmax();
    resolver.AddPad();
    resolver.AddAdd();
    resolver.AddMul();
    resolver.AddMaxPool2D();
    resolver.AddFullyConnected();
    resolver.AddRelu();
    resolver.AddRelu6();
    resolver.AddLeakyRelu();
    // let op: CUSTOM ops zijn hier (nog) NIET geregistreerd

    log_heap_usage("before arena alloc");

    tensor_arena = static_cast<uint8_t *>(
        heap_caps_malloc(TENSOR_ARENA_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!tensor_arena) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena (%u bytes) in PSRAM",
                 (unsigned)TENSOR_ARENA_SIZE);
        return false;
    }

    log_heap_usage("after arena alloc");

    static tflite::MicroInterpreter static_interpreter(
        g_model, resolver, tensor_arena, TENSOR_ARENA_SIZE);

    g_interpreter = &static_interpreter;

    TfLiteStatus alloc_status = g_interpreter->AllocateTensors();
    if (alloc_status != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed");
        return false;
    }

    TfLiteTensor *input = g_interpreter->input(0);
    ESP_LOGI(TAG, "Input tensor: type=%d, dims=%d",
             input->type, input->dims->size);
    for (int i = 0; i < input->dims->size; ++i) {
        ESP_LOGI(TAG, "  in dim[%d]=%d", i, input->dims->data[i]);
    }

    TfLiteTensor *output = g_interpreter->output(0);
    ESP_LOGI(TAG, "Output tensor: type=%d, dims=%d",
             output->type, output->dims->size);
    for (int i = 0; i < output->dims->size; ++i) {
        ESP_LOGI(TAG, "  out dim[%d]=%d", i, output->dims->data[i]);
    }

    log_heap_usage("after AllocateTensors");
    return true;
}

// ===================== INFERENCE (dummy input) =====================

static bool run_inference_for_image(const std::string &path)
{
    if (!g_interpreter) {
        ESP_LOGE(TAG, "Interpreter not initialized");
        return false;
    }

    TfLiteTensor *input = g_interpreter->input(0);
    size_t input_bytes = input->bytes;

    ESP_LOGI(TAG, "Preparing dummy input for %s (%u bytes)",
             path.c_str(), (unsigned)input_bytes);

    memset(input->data.raw, 0, input_bytes);

    TfLiteStatus status = g_interpreter->Invoke();
    if (status != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke failed for %s", path.c_str());
        return false;
    }

    TfLiteTensor *output = g_interpreter->output(0);
    ESP_LOGI(TAG, "Inference OK for %s", path.c_str());
    ESP_LOGI(TAG, "Output dims=%d", output->dims->size);
    for (int i = 0; i < output->dims->size; ++i) {
        ESP_LOGI(TAG, "  out dim[%d]=%d", i, output->dims->data[i]);
    }

    return true;
}

// ===================== DIRECTORY WALK =====================

static void classify_jpgs_recursive(const std::string &root)
{
    DIR *dir = opendir(root.c_str());
    if (!dir) {
        ESP_LOGE(TAG, "Can't open %s", root.c_str());
        return;
    }

    ESP_LOGI(TAG, "Scanning directory: %s", root.c_str());

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        if (name == "." || name == "..") continue;

        std::string full = root + "/" + name;
        struct stat st{};
        if (stat(full.c_str(), &st) != 0) {
            ESP_LOGW(TAG, "stat failed for %s", full.c_str());
            continue;
        }

        if (S_ISDIR(st.st_mode)) {
            ESP_LOGI(TAG, "DIR : %s", full.c_str());
            classify_jpgs_recursive(full);
        } else if (S_ISREG(st.st_mode)) {
            if (has_jpg_extension(name)) {
                ESP_LOGI(TAG, "IMG : %s", full.c_str());
                run_inference_for_image(full);
            } else {
                ESP_LOGI(TAG, "FILE: %s (ignored, not jpg)", full.c_str());
            }
        } else {
            ESP_LOGI(TAG, "OTHER: %s", full.c_str());
        }
    }

    closedir(dir);
}

// ===================== app_main =====================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting VESPA YOLO S3 (IDF + TFLM)");

    // 1. NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated or new version found, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    log_heap_usage("after NVS init");

    // 2. SDMMC host + slot
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = SDMMC_CLK_GPIO;
    slot_config.cmd = SDMMC_CMD_GPIO;
    slot_config.d0  = SDMMC_D0_GPIO;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    // 3. Mount SD
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 10;
    mount_config.allocation_unit_size = 16 * 1024;

    sdmmc_card_t *card = nullptr;
    ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "SD mounted OK");
    sdmmc_card_print_info(stdout, card);

    log_heap_usage("after SD mount");

    // 4. Model laden vanaf SD
    size_t model_size = 0;
    const char *model_path = "/sdcard/models/vespcv_swiftyolo_int8_vela.tflite";

    uint8_t *model_data = load_model_from_sd(model_path, model_size);
    if (!model_data) {
        ESP_LOGE(TAG, "Failed to load model from %s", model_path);
        return;
    }

    ESP_LOGI(TAG, "Model loaded, size=%u bytes", (unsigned)model_size);

    if (!init_tflm(model_data, model_size)) {
        ESP_LOGE(TAG, "Failed to init TFLM");
        return;
    }

    // 5. Alle JPG’s onder /sdcard/images classificeren (nu nog dummy input)
    classify_jpgs_recursive("/sdcard/images");

    ESP_LOGI(TAG, "Image classification loop complete (dummy input)");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
