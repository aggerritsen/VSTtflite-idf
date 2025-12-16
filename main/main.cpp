// -----------------------------------------------------------------------------
// main/main.cpp
// RFC – Single deterministic camera → SD → preview → inference pipeline
// Deep inference logging + decoding debug (additive only)
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <math.h>
#include <algorithm>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_camera.h"

#include "pmu.h"
#include "modem.h"
#include "camera.h"
#include "sdcard.h"
#include "wifi.h"
#include "httpd.h"
#include "ppm.h"

#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

// -----------------------------------------------------------------------------
// CONFIG
// -----------------------------------------------------------------------------
#define INPUT_W 192
#define INPUT_H 192
#define INPUT_CH 3

#define CONFIG_PATH "/sdcard/config/config.txt"
#define MODEL_DIR   "/sdcard/models/"
#define MAX_MODEL_PATH 256

#define REG_MAX   16
#define REG_CH    (4 * REG_MAX)
#define MAX_BOXES 20
#define CONF_THRESH 0.30f

// -----------------------------------------------------------------------------
// DEBUG CONFIG (runtime logging knobs)
// -----------------------------------------------------------------------------
#define DBG_LOG_TENSORS        1   // tensor types/dims/quant
#define DBG_LOG_INPUT_STATS    1   // stats on resize_crop and quantized input tensor
#define DBG_LOG_OUTPUT_STATS   1   // stats on output tensor (dequant)
#define DBG_LOG_OUTPUT_SAMPLES 1   // a few sample values (raw + dequant)
#define DBG_LOG_YOLO_SCAN      1   // scan for global max class probability
#define DBG_LOG_DETECTIONS     1   // dump decoded boxes
#define DBG_TOPK_CLASSES       5   // top-k class probes at best cell
#define DBG_DUMP_LIMIT_BOXES   10  // max printed boxes per frame

static const char *TAG = "PIPELINE";

// -----------------------------------------------------------------------------
// GLOBAL TIMESTAMP STORAGE (FIXED)
// -----------------------------------------------------------------------------
static std::string g_ts_compact;
static std::string g_ts_iso;
static uint32_t g_frame_seq = 0;

// -----------------------------------------------------------------------------
// YOLO STRUCT
// -----------------------------------------------------------------------------
typedef struct {
    float x, y, w, h;
    float score;
    int   cls;
} yolo_box_t;

// TensorFlow globals
static tflite::MicroInterpreter *g_interpreter = nullptr;
static TfLiteTensor *g_input_tensor  = nullptr;
static TfLiteTensor *g_output_tensor = nullptr;

// -----------------------------------------------------------------------------
// UTILS
// -----------------------------------------------------------------------------
static inline float dequant_i8(int8_t v, const TfLiteTensor *t)
{
    return (v - t->params.zero_point) * t->params.scale;
}

static inline float sigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

static inline int8_t quantize_u8_to_int8(uint8_t v, float scale, int zero_point)
{
    int q = (int)roundf(v / scale) + zero_point;
    q = std::max(-128, std::min(127, q));
    return (int8_t)q;
}

static const char *tf_type_str(TfLiteType t)
{
    switch (t) {
    case kTfLiteNoType:  return "NoType";
    case kTfLiteFloat32: return "Float32";
    case kTfLiteInt32:   return "Int32";
    case kTfLiteUInt8:   return "UInt8";
    case kTfLiteInt64:   return "Int64";
    case kTfLiteString:  return "String";
    case kTfLiteBool:    return "Bool";
    case kTfLiteInt16:   return "Int16";
    case kTfLiteComplex64:return "Complex64";
    case kTfLiteInt8:    return "Int8";
    case kTfLiteFloat16: return "Float16";
    case kTfLiteFloat64: return "Float64";
    case kTfLiteComplex128:return "Complex128";
    case kTfLiteUInt64:  return "UInt64";
    case kTfLiteResource:return "Resource";
    case kTfLiteVariant: return "Variant";
    case kTfLiteUInt32:  return "UInt32";
    case kTfLiteUInt16:  return "UInt16";
    case kTfLiteInt4:    return "Int4";
    default:             return "Unknown";
    }
}

static void log_tensor_info(const char *name, const TfLiteTensor *t)
{
    if (!t) {
        ESP_LOGE(TAG, "%s tensor is NULL", name);
        return;
    }

    ESP_LOGI(TAG, "%s: type=%s, bytes=%d", name, tf_type_str(t->type), (int)t->bytes);

    if (t->dims && t->dims->size > 0) {
        std::string ds;
        for (int i = 0; i < t->dims->size; i++) {
            char buf[32];
            snprintf(buf, sizeof(buf), "%s%d", (i ? "x" : ""), t->dims->data[i]);
            ds += buf;
        }
        ESP_LOGI(TAG, "%s: dims=%s", name, ds.c_str());
    } else {
        ESP_LOGW(TAG, "%s: dims missing", name);
    }

    // Quant params (valid for quantized tensors)
    ESP_LOGI(TAG, "%s: quant scale=%.10f zero_point=%d",
             name, (double)t->params.scale, t->params.zero_point);
}

static void log_rgb_stats_u8(const char *label, const uint8_t *rgb, int w, int h)
{
    if (!rgb || w <= 0 || h <= 0) return;

    const int n = w * h * 3;
    uint8_t mn = 255, mx = 0;
    uint64_t sum = 0;

    for (int i = 0; i < n; i++) {
        uint8_t v = rgb[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
    }

    double mean = (double)sum / (double)n;
    ESP_LOGI(TAG, "%s: RGB888 stats n=%d min=%u max=%u mean=%.2f",
             label, n, (unsigned)mn, (unsigned)mx, mean);
}

static void log_i8_stats(const char *label, const int8_t *buf, int n, float scale, int zp)
{
    if (!buf || n <= 0) return;

    int8_t mn = 127, mx = -128;
    int64_t sum = 0;

    for (int i = 0; i < n; i++) {
        int8_t v = buf[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
    }

    double mean_i8 = (double)sum / (double)n;
    double mean_deq = ((mean_i8 - zp) * scale);

    ESP_LOGI(TAG, "%s: INT8 stats n=%d min=%d max=%d mean_i8=%.2f mean_deq=%.6f",
             label, n, (int)mn, (int)mx, mean_i8, mean_deq);
}

static void log_output_dequant_stats(const TfLiteTensor *out)
{
    if (!out || out->type != kTfLiteInt8) {
        ESP_LOGW(TAG, "Output stats skipped (out NULL or not int8)");
        return;
    }

    const int8_t *d = out->data.int8;
    const int n = out->bytes; // int8 => bytes == element count

    float mn = 1e30f, mx = -1e30f;
    double sum = 0.0;

    for (int i = 0; i < n; i++) {
        float v = dequant_i8(d[i], out);
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
    }

    double mean = sum / (double)n;
    ESP_LOGI(TAG, "Output dequant stats: n=%d min=%.6f max=%.6f mean=%.6f", n, mn, mx, mean);
}

static void log_output_samples(const TfLiteTensor *out, int count)
{
    if (!out || out->type != kTfLiteInt8) return;
    const int8_t *d = out->data.int8;
    const int n = out->bytes;

    int c = std::min(count, n);
    std::string s;
    s.reserve(128);

    for (int i = 0; i < c; i++) {
        char buf[64];
        float deq = dequant_i8(d[i], out);
        snprintf(buf, sizeof(buf), " [%d]=%d(%.4f)", i, (int)d[i], deq);
        s += buf;
    }

    ESP_LOGI(TAG, "Output samples:%s", s.c_str());
}

// -----------------------------------------------------------------------------
// YOLO DIAGNOSTIC: global max class score (across all cells/classes)
// -----------------------------------------------------------------------------
static float log_max_class_score(const TfLiteTensor *out)
{
    float maxs = 0.0f;
    const int8_t *d = out->data.int8;

    const int N   = out->dims->data[1];
    const int C   = out->dims->data[2];
    const int CLS = C - REG_CH;

    for (int i = 0; i < N; i++) {
        for (int c = 0; c < CLS; c++) {
            float p = sigmoid(
                dequant_i8(d[i * C + REG_CH + c], out)
            );
            if (p > maxs)
                maxs = p;
        }
    }

    ESP_LOGI(TAG, "YOLO max class score = %.6f (CONF_THRESH=%.2f)", maxs, (double)CONF_THRESH);
    return maxs;
}

typedef struct {
    float score;
    int cell;
    int cls;
} best_score_t;

static best_score_t find_best_cell_class(const TfLiteTensor *out)
{
    best_score_t best{0.0f, -1, -1};

    if (!out || out->type != kTfLiteInt8 || !out->dims || out->dims->size < 3) {
        return best;
    }

    const int8_t *data = out->data.int8;
    const int N = out->dims->data[1];
    const int C = out->dims->data[2];
    const int CLS_CH = C - REG_CH;

    for (int i = 0; i < N; i++) {
        int base = i * C;
        for (int c = 0; c < CLS_CH; c++) {
            float p = sigmoid(dequant_i8(data[base + REG_CH + c], out));
            if (p > best.score) {
                best.score = p;
                best.cell = i;
                best.cls = c;
            }
        }
    }

    return best;
}

static void log_topk_classes_at_cell(const TfLiteTensor *out, int cell, int k)
{
    if (!out || out->type != kTfLiteInt8 || !out->dims || out->dims->size < 3) return;
    if (cell < 0) return;

    const int8_t *data = out->data.int8;
    const int C = out->dims->data[2];
    const int CLS_CH = C - REG_CH;

    std::vector<std::pair<float,int>> v;
    v.reserve(CLS_CH);

    int base = cell * C;
    for (int c = 0; c < CLS_CH; c++) {
        float p = sigmoid(dequant_i8(data[base + REG_CH + c], out));
        v.push_back({p, c});
    }

    std::partial_sort(v.begin(), v.begin() + std::min(k, (int)v.size()), v.end(),
                      [](auto &a, auto &b){ return a.first > b.first; });

    std::string s;
    for (int i = 0; i < k && i < (int)v.size(); i++) {
        char buf[64];
        snprintf(buf, sizeof(buf), " #%d cls=%d p=%.6f", i+1, v[i].second, (double)v[i].first);
        s += buf;
    }

    ESP_LOGI(TAG, "Top-%d classes at best cell %d:%s", k, cell, s.c_str());
}

// -----------------------------------------------------------------------------
// YOLOv8 DFL DECODE
// -----------------------------------------------------------------------------
static int decode_yolov8(const TfLiteTensor *out, yolo_box_t *boxes)
{
    const int8_t *data = out->data.int8;
    int N = out->dims->data[1];
    int C = out->dims->data[2];
    int CLS_CH = C - REG_CH;

    int grid = (int)roundf(sqrtf((float)N));
    float stride = (float)INPUT_W / grid;

    // Deep sanity logs (one-liners, per-frame safe)
    ESP_LOGI(TAG, "YOLO decode: N=%d C=%d CLS_CH=%d REG_CH=%d grid=%d stride=%.3f",
             N, C, CLS_CH, REG_CH, grid, (double)stride);

    if (grid * grid != N) {
        ESP_LOGW(TAG, "YOLO decode: N=%d is not a perfect square (grid=%d)", N, grid);
    }

    int found = 0;

    for (int i = 0; i < N && found < MAX_BOXES; i++) {
        int base = i * C;
        float dfl[4] = {0};

        for (int s = 0; s < 4; s++) {
            float sum = 0;
            float probs[REG_MAX];

            for (int k = 0; k < REG_MAX; k++) {
                probs[k] = expf(dequant_i8(
                    data[base + s * REG_MAX + k], out));
                sum += probs[k];
            }
            for (int k = 0; k < REG_MAX; k++)
                dfl[s] += k * (probs[k] / sum);
        }

        int best_cls = -1;
        float best_score = 0;

        for (int c = 0; c < CLS_CH; c++) {
            float p = sigmoid(dequant_i8(
                data[base + REG_CH + c], out));
            if (p > best_score) {
                best_score = p;
                best_cls = c;
            }
        }

        if (best_score < CONF_THRESH) continue;

        int gx = i % grid;
        int gy = i / grid;

        float cx = (gx + 0.5f) * stride;
        float cy = (gy + 0.5f) * stride;

        boxes[found++] = {
            cx - dfl[0] * stride,
            cy - dfl[1] * stride,
            (dfl[0] + dfl[2]) * stride,
            (dfl[1] + dfl[3]) * stride,
            best_score,
            best_cls
        };
    }

    return found;
}

// -----------------------------------------------------------------------------
// Model config / load
// -----------------------------------------------------------------------------
static bool read_model_from_config(char *out_full_path, size_t out_len)
{
    FILE *f = fopen(CONFIG_PATH, "r");
    if (!f) {
        ESP_LOGE(TAG, "Config file not found: %s", CONFIG_PATH);
        return false;
    }

    char line[256];
    char model[128] = {0};

    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "#define", 7) == 0 && strstr(line, "MODEL")) {
            char *q1 = strchr(line, '"');
            char *q2 = q1 ? strchr(q1 + 1, '"') : nullptr;
            if (q1 && q2 && q2 > q1 + 1) {
                size_t len = q2 - q1 - 1;
                strncpy(model, q1 + 1, len);
                model[len] = 0;
                break;
            }
        }
    }

    fclose(f);

    if (!model[0]) {
        ESP_LOGE(TAG, "MODEL not found in config");
        return false;
    }

    snprintf(out_full_path, out_len, "%s%s", MODEL_DIR, model);
    ESP_LOGI(TAG, "Model: %s", out_full_path);
    return true;
}

static bool init_model()
{
    char model_path[MAX_MODEL_PATH];

    if (!read_model_from_config(model_path, sizeof(model_path))) {
        ESP_LOGE(TAG, "Model config read failed");
        return false;
    }

    struct stat st;
    if (stat(model_path, &st) != 0) {
        ESP_LOGE(TAG, "Model not found: %s", model_path);
        return false;
    }

    uint8_t *model_buf = (uint8_t *)
        heap_caps_malloc(st.st_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!model_buf) {
        ESP_LOGE(TAG, "Model buffer alloc failed");
        return false;
    }

    FILE *f = fopen(model_path, "rb");
    fread(model_buf, 1, st.st_size, f);
    fclose(f);

    const tflite::Model *model = tflite::GetModel(model_buf);

    static tflite::MicroMutableOpResolver<64> resolver;
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddTransposeConv();
    resolver.AddMaxPool2D();
    resolver.AddAveragePool2D();
    resolver.AddAdd();
    resolver.AddMul();
    resolver.AddSub();
    resolver.AddMaximum();
    resolver.AddMinimum();
    resolver.AddPad();
    resolver.AddPadV2();
    resolver.AddQuantize();
    resolver.AddDequantize();
    resolver.AddRelu();
    resolver.AddRelu6();
    resolver.AddLogistic();
    resolver.AddSoftmax();
    resolver.AddReshape();
    resolver.AddStridedSlice();
    resolver.AddConcatenation();
    resolver.AddTranspose();
    resolver.AddResizeNearestNeighbor();
    resolver.AddResizeBilinear();
    resolver.AddFullyConnected();

    uint8_t *arena = (uint8_t *)
        heap_caps_malloc(2 * 1024 * 1024,
                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!arena) {
        ESP_LOGE(TAG, "Tensor arena alloc failed");
        return false;
    }

    static tflite::MicroInterpreter static_interpreter(
        model, resolver, arena, 2 * 1024 * 1024);

    if (static_interpreter.AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors failed");
        return false;
    }

    g_interpreter   = &static_interpreter;
    g_input_tensor  = static_interpreter.input(0);
    g_output_tensor = static_interpreter.output(0);

    ESP_LOGI(TAG, "Model initialized successfully");

#if DBG_LOG_TENSORS
    ESP_LOGI(TAG, "Heap free (8bit)=%lu  SPIRAM free=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    log_tensor_info("INPUT", g_input_tensor);
    log_tensor_info("OUTPUT", g_output_tensor);
#endif

    return true;
}

// -----------------------------------------------------------------------------
// No-crop resize helper (RGB888, NN)
// -----------------------------------------------------------------------------
static uint8_t *resize_rgb888_no_crop(
    const uint8_t *src,
    int src_w,
    int src_h,
    int dst_w,
    int dst_h)
{
    // Geometry-preserving, no-crop resize using letterboxing
    // Output is always dst_w × dst_h (e.g. 192×192)
    // Aspect ratio preserved, padding added where needed

    if (!src || src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0)
        return nullptr;

    uint8_t *dst = (uint8_t *)malloc(dst_w * dst_h * 3);
    if (!dst)
        return nullptr;

    // Fill with black (padding)
    memset(dst, 0, dst_w * dst_h * 3);

    // Uniform scale
    float scale = std::min(
        (float)dst_w / (float)src_w,
        (float)dst_h / (float)src_h
    );

    int scaled_w = (int)(src_w * scale);
    int scaled_h = (int)(src_h * scale);

    int pad_x = (dst_w - scaled_w) / 2;
    int pad_y = (dst_h - scaled_h) / 2;

    for (int y = 0; y < scaled_h; y++) {
        int sy = (int)(y / scale);
        if (sy >= src_h) sy = src_h - 1;

        uint8_t *row_dst = dst + ((y + pad_y) * dst_w + pad_x) * 3;

        for (int x = 0; x < scaled_w; x++) {
            int sx = (int)(x / scale);
            if (sx >= src_w) sx = src_w - 1;

            const uint8_t *p = &src[(sy * src_w + sx) * 3];
            uint8_t *q = row_dst + x * 3;

            q[0] = p[0];
            q[1] = p[1];
            q[2] = p[2];
        }
    }

    return dst;
}

// -----------------------------------------------------------------------------
// PIPELINE TASK
// -----------------------------------------------------------------------------
void pipeline_task(void *pv)
{
    ESP_LOGI(TAG, "Pipeline task started");
    mkdir("/sdcard/capture", 0775);

    while (!g_interpreter || !g_input_tensor || !g_output_tensor) {
        ESP_LOGW(TAG, "Waiting for model initialization...");
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // One-time sanity
#if DBG_LOG_TENSORS
    ESP_LOGI(TAG, "Pipeline ready: input type=%s output type=%s",
             tf_type_str(g_input_tensor->type), tf_type_str(g_output_tensor->type));
#endif

    while (true) {

        // -------------------------------------------------
        // Capture
        // -------------------------------------------------
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (fb->format != PIXFORMAT_JPEG) {
            esp_camera_fb_return(fb);
            continue;
        }

        // -------------------------------------------------
        // Preview JPEG (HTTP)
        // -------------------------------------------------
        httpd_update_last_frame(fb->buf, fb->len);

        // -------------------------------------------------
        // JPEG → RGB888
        // -------------------------------------------------
        uint8_t *rgb = nullptr;
        int w = 0, h = 0;

        if (jpeg_to_rgb888(fb->buf, fb->len, &rgb, &w, &h) != ESP_OK) {
            esp_camera_fb_return(fb);
            continue;
        }

        // -------------------------------------------------
        // A) Aspect-crop resize (MODEL INPUT)
        // -------------------------------------------------
        uint8_t *resize_crop = nullptr;
        resize_rgb888_aspect_crop(rgb, w, h, INPUT_W, &resize_crop);

        if (!resize_crop) {
            free(rgb);
            esp_camera_fb_return(fb);
            continue;
        }

        // -------------------------------------------------
        // B) No-crop resize (DATASET / AUDIT)
        // -------------------------------------------------
        uint8_t *resize_nocrop =
            resize_rgb888_no_crop(rgb, w, h, INPUT_W, INPUT_H);

        free(rgb);

        if (!resize_nocrop) {
            free(resize_crop);
            esp_camera_fb_return(fb);
            continue;
        }

        // -------------------------------------------------
        // Filenames
        // -------------------------------------------------
        char jpg_path[128];
        char ppm_crop_path[128];
        char ppm_nocrop_path[128];

        snprintf(jpg_path, sizeof(jpg_path),
                 "/sdcard/capture/frame_%06lu.jpg",
                 (unsigned long)g_frame_seq);

        snprintf(ppm_crop_path, sizeof(ppm_crop_path),
                 "/sdcard/capture/frame_%06lu_cropped.ppm",
                 (unsigned long)g_frame_seq);

        snprintf(ppm_nocrop_path, sizeof(ppm_nocrop_path),
                 "/sdcard/capture/frame_%06lu_rgb192.ppm",
                 (unsigned long)g_frame_seq);

        // -------------------------------------------------
        // Save original JPEG
        // -------------------------------------------------
        sdcard_write_file(jpg_path, fb->buf, fb->len);

        // -------------------------------------------------
        // Save preprocessed images
        // -------------------------------------------------
        ppm_write_rgb888(ppm_crop_path,
                         resize_crop,
                         INPUT_W,
                         INPUT_H);

        ppm_write_rgb888(ppm_nocrop_path,
                         resize_nocrop,
                         INPUT_W,
                         INPUT_H);

        // -------------------------------------------------
        // Input stats (pre-quant)
        // -------------------------------------------------
#if DBG_LOG_INPUT_STATS
        log_rgb_stats_u8("MODEL_INPUT_RGB (cropped)", resize_crop, INPUT_W, INPUT_H);
        log_rgb_stats_u8("AUDIT_RGB (nocrop)", resize_nocrop, INPUT_W, INPUT_H);
#endif

    // -------------------------------------------------
    // Quantize (MODEL INPUT – NON-CROPPED, GEOMETRY-DISTORTED)
    // -------------------------------------------------
    const int in_count = INPUT_W * INPUT_H * 3;

    for (int i = 0; i < in_count; i++) {
        g_input_tensor->data.int8[i] =
            quantize_u8_to_int8(
                resize_nocrop[i],
                g_input_tensor->params.scale,
                g_input_tensor->params.zero_point
            );
    }

#if DBG_LOG_INPUT_STATS
log_i8_stats("MODEL_INPUT_INT8",
            g_input_tensor->data.int8,
            in_count,
            g_input_tensor->params.scale,
            g_input_tensor->params.zero_point);
#endif

        free(resize_crop);
        free(resize_nocrop);

        // -------------------------------------------------
        // Inference
        // -------------------------------------------------
        int64_t t0 = esp_timer_get_time();
        TfLiteStatus st = g_interpreter->Invoke();
        int64_t t1 = esp_timer_get_time();

        ESP_LOGI(TAG, "Frame %06lu: Invoke()=%s time=%lld us",
                 (unsigned long)g_frame_seq,
                 (st == kTfLiteOk ? "kTfLiteOk" : "kTfLiteError"),
                 (long long)(t1 - t0));

        if (st != kTfLiteOk) {
            ESP_LOGE(TAG, "Invoke failed on frame %lu", (unsigned long)g_frame_seq);
            esp_camera_fb_return(fb);
            g_frame_seq++;
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        // -------------------------------------------------
        // Output logging
        // -------------------------------------------------
#if DBG_LOG_TENSORS
        // per-frame light sanity (dims can change only if model changed)
        if (g_output_tensor && g_output_tensor->dims && g_output_tensor->dims->size >= 3) {
            ESP_LOGI(TAG, "OUTPUT dims: [%d x %d x %d] type=%s",
                     g_output_tensor->dims->data[0],
                     g_output_tensor->dims->data[1],
                     g_output_tensor->dims->data[2],
                     tf_type_str(g_output_tensor->type));
        }
#endif

#if DBG_LOG_OUTPUT_STATS
        log_output_dequant_stats(g_output_tensor);
#endif

#if DBG_LOG_OUTPUT_SAMPLES
        log_output_samples(g_output_tensor, 16);
#endif

        // -------------------------------------------------
        // YOLO: deep scan and decode
        // -------------------------------------------------
#if DBG_LOG_YOLO_SCAN
        float maxp = log_max_class_score(g_output_tensor);
        best_score_t best = find_best_cell_class(g_output_tensor);
        ESP_LOGI(TAG, "Best cell/class: cell=%d cls=%d p=%.6f (global max=%.6f)",
                 best.cell, best.cls, (double)best.score, (double)maxp);

        // Log top-k at that cell to see distribution (helpful for wrong label maps)
        if (best.cell >= 0) {
            log_topk_classes_at_cell(g_output_tensor, best.cell, DBG_TOPK_CLASSES);
        }
#endif

        yolo_box_t boxes[MAX_BOXES];
        int n = decode_yolov8(g_output_tensor, boxes);

        ESP_LOGI(TAG, "Frame %06lu: Detections=%d (CONF_THRESH=%.2f)",
                 (unsigned long)g_frame_seq, n, (double)CONF_THRESH);

#if DBG_LOG_DETECTIONS
        if (n > 0) {
            int lim = std::min(n, DBG_DUMP_LIMIT_BOXES);
            for (int i = 0; i < lim; i++) {
                ESP_LOGI(TAG,
                         "Box[%d]: cls=%d score=%.6f  x=%.1f y=%.1f w=%.1f h=%.1f",
                         i,
                         boxes[i].cls,
                         (double)boxes[i].score,
                         (double)boxes[i].x,
                         (double)boxes[i].y,
                         (double)boxes[i].w,
                         (double)boxes[i].h);
            }
            if (n > lim) {
                ESP_LOGI(TAG, "Box dump truncated: printed %d / %d", lim, n);
            }
        } else {
            // Critical: show why there are no boxes (threshold vs dead model)
            // If best.score is near 0.0 always -> likely model mismatch / bad quant / wrong op set
            best_score_t best = find_best_cell_class(g_output_tensor);
            ESP_LOGW(TAG,
                     "No detections. Best p=%.6f at cell=%d cls=%d (threshold=%.2f).",
                     (double)best.score, best.cell, best.cls, (double)CONF_THRESH);
        }
#endif

        // -------------------------------------------------
        // Cleanup
        // -------------------------------------------------
        esp_camera_fb_return(fb);
        g_frame_seq++;

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// -----------------------------------------------------------------------------
// Internal: synchronize ESP32 system clock from modem time
// -----------------------------------------------------------------------------
static bool sync_system_time_from_modem(void)
{
    std::string ts_compact;
    std::string ts_iso;

    if (!modem_get_timestamp(ts_compact, ts_iso)) {
        ESP_LOGW(TAG, "Failed to get modem timestamp");
        return false;
    }

    // Expected ts_compact format:
    // "YYYYMMDD_HHMMSS"
    // Example: "20251216_215253"
    int yyyy, MM, dd, hh, mm, ss;

    if (sscanf(ts_compact.c_str(),
               "%4d%2d%2d_%2d%2d%2d",
               &yyyy, &MM, &dd,
               &hh, &mm, &ss) != 6) {
        ESP_LOGE(TAG,
                 "Failed to parse modem time (compact): %s",
                 ts_compact.c_str());
        return false;
    }

    struct tm tm {};
    tm.tm_year = yyyy - 1900;
    tm.tm_mon  = MM - 1;
    tm.tm_mday = dd;
    tm.tm_hour = hh;
    tm.tm_min  = mm;
    tm.tm_sec  = ss;

    time_t t = mktime(&tm);
    if (t < 0) {
        ESP_LOGE(TAG, "mktime failed");
        return false;
    }

    struct timeval tv {
        .tv_sec  = t,
        .tv_usec = 0
    };

    if (settimeofday(&tv, nullptr) != 0) {
        ESP_LOGE(TAG, "settimeofday failed");
        return false;
    }

    ESP_LOGI(TAG,
             "System time synchronized from modem: %s",
             ts_iso.c_str());

    return true;
}

// -----------------------------------------------------------------------------
// APP MAIN
// -----------------------------------------------------------------------------
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Boot");

    // -----------------------------------------------------------------
    // 1. Power rails
    // -----------------------------------------------------------------
    axp2101_init_pmic();
    axp2101_verify_settings();

    // -----------------------------------------------------------------
    // 2. SD card (REQUIRED for model + config)
    // -----------------------------------------------------------------
    if (sdcard_mount() != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed");
        return;
    }

    // -----------------------------------------------------------------
    // 3. Modem (time source)
    // -----------------------------------------------------------------
    if (modem_init_uart() == ESP_OK) {
        wait_for_modem();
    } else {
        ESP_LOGW(TAG, "Modem UART init failed");
    }

    sync_system_time_from_modem();   // local static helper

    // -----------------------------------------------------------------
    // 4. WiFi + HTTP preview
    // -----------------------------------------------------------------
    wifi_ap_start();
    http_server_start();

    // -----------------------------------------------------------------
    // 5. Camera
    // -----------------------------------------------------------------
    if (camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed");
        return;
    }

    // -----------------------------------------------------------------
    // 6. Load ML model (CRITICAL)
    // -----------------------------------------------------------------
    if (!init_model()) {
        ESP_LOGE(TAG, "Model initialization failed");
        return;
    }

    // -----------------------------------------------------------------
    // 7. Start deterministic pipeline
    // -----------------------------------------------------------------
    xTaskCreate(
        pipeline_task,
        "PIPELINE",
        12288,
        nullptr,
        6,
        nullptr
    );
}
