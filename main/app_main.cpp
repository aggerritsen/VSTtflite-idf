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

// REAL JPEG decoder (TJpgDec via espressif/esp_jpeg)
#include "jpeg_decoder.h"

// TFLite Micro / esp-tflite-micro
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

static const char *TAG = "VESPA_YOLO_S3";

// SDMMC GPIOs for T-SIM7080G-S3 (1-bit SDMMC)
#define SDMMC_CMD_GPIO  39
#define SDMMC_CLK_GPIO  38
#define SDMMC_D0_GPIO   40

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

    sdmmc_card_t *card = nullptr;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(
        "/sdcard",
        &host,
        &slot_config,
        &mount_config,
        &card
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "SD mounted OK");
    sdmmc_card_print_info(stdout, card);

    log_heap("after SD mount");
    return true;
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

    size_t read = fread(g_model_data, 1, size, f);
    fclose(f);

    if (read != (size_t)size) {
        ESP_LOGE(TAG, "Model read mismatch: expected %ld, got %u",
                 size,
                 static_cast<unsigned>(read));
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

    const char *model_path =
        "/sdcard/models/yolov8n_2025-07-15_192_full_integer_quant.tflite";

    if (!load_model_from_sd(model_path)) return;

    g_tensor_arena = (uint8_t*)heap_caps_malloc(
        TENSOR_ARENA_SIZE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!g_tensor_arena) {
        ESP_LOGE(TAG, "Arena alloc FAILED");
        return;
    }

    if (!init_tflm(g_tensor_arena, TENSOR_ARENA_SIZE))
        return;

    std::vector<std::string> paths;
    collect_images_recursive("/sdcard/images", paths);

    if (paths.empty()) {
        ESP_LOGW(TAG, "No images found");
    } else {
        ESP_LOGI(TAG, "Found %u images", (unsigned)paths.size());
        for (size_t i = 0; i < paths.size(); ++i) {
            ESP_LOGI(TAG, "Processing %u/%u", (unsigned)(i + 1),
                     (unsigned)paths.size());
            run_inference_on_image(paths[i].c_str());
        }
    }

    while (true) {
        ESP_LOGI(TAG, "Idle");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
