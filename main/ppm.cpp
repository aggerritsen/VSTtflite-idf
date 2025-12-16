// File: main/ppm.cpp

#include "ppm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "img_converters.h"

static const char *TAG = "PPM";

// -----------------------------------------------------------------------------
// RGB565 → RGB888 conversion
// -----------------------------------------------------------------------------
static void rgb565_to_rgb888(
    const uint8_t *src,
    uint8_t *dst,
    int pixel_count)
{
    for (int i = 0; i < pixel_count; i++) {
        uint16_t p = ((uint16_t *)src)[i];

        uint8_t r = (p >> 11) & 0x1F;
        uint8_t g = (p >> 5)  & 0x3F;
        uint8_t b =  p        & 0x1F;

        dst[i * 3 + 0] = (r << 3) | (r >> 2);
        dst[i * 3 + 1] = (g << 2) | (g >> 4);
        dst[i * 3 + 2] = (b << 3) | (b >> 2);
    }
}

// -----------------------------------------------------------------------------
// Write RGB888 buffer as binary PPM (P6)
// -----------------------------------------------------------------------------
esp_err_t ppm_write_rgb888(
    const char *path,
    const uint8_t *rgb,
    int width,
    int height)
{
    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s", path);
        return ESP_FAIL;
    }

    fprintf(f, "P6\n%d %d\n255\n", width, height);
    fwrite(rgb, 1, width * height * 3, f);
    fclose(f);

    ESP_LOGI(TAG, "Wrote %s (%dx%d RGB888)", path, width, height);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// JPEG → RGB888 (via RGB565, guaranteed API)
// -----------------------------------------------------------------------------
esp_err_t jpeg_to_rgb888(
    const uint8_t *jpeg,
    size_t jpeg_len,
    uint8_t **out_rgb,
    int *out_w,
    int *out_h)
{
    // You KNOW the file is 320×240 (per your setup)
    const int width  = 320;
    const int height = 240;
    const int pixels = width * height;

    uint8_t *rgb565 = (uint8_t *)malloc(pixels * 2);
    if (!rgb565) {
        ESP_LOGE(TAG, "RGB565 alloc failed");
        return ESP_ERR_NO_MEM;
    }

    if (!jpg2rgb565(jpeg, jpeg_len, rgb565, JPG_SCALE_NONE)) {
        ESP_LOGE(TAG, "jpg2rgb565 failed");
        free(rgb565);
        return ESP_FAIL;
    }

    uint8_t *rgb888 = (uint8_t *)malloc(pixels * 3);
    if (!rgb888) {
        free(rgb565);
        return ESP_ERR_NO_MEM;
    }

    rgb565_to_rgb888(rgb565, rgb888, pixels);
    free(rgb565);

    *out_rgb = rgb888;
    *out_w   = width;
    *out_h   = height;

    ESP_LOGI(TAG, "JPEG → RGB888 (%dx%d)", width, height);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Center crop RGB888
// -----------------------------------------------------------------------------
esp_err_t crop_rgb888_center(
    const uint8_t *src,
    int src_w,
    int src_h,
    int crop_w,
    int crop_h,
    uint8_t **out_rgb)
{
    if (crop_w > src_w || crop_h > src_h) {
        ESP_LOGE(TAG, "Invalid crop size");
        return ESP_ERR_INVALID_ARG;
    }

    int x0 = (src_w - crop_w) / 2;
    int y0 = (src_h - crop_h) / 2;

    uint8_t *dst = (uint8_t *)malloc(crop_w * crop_h * 3);
    if (!dst) {
        ESP_LOGE(TAG, "Crop alloc failed");
        return ESP_ERR_NO_MEM;
    }

    for (int y = 0; y < crop_h; y++) {
        memcpy(
            dst + y * crop_w * 3,
            src + ((y0 + y) * src_w + x0) * 3,
            crop_w * 3
        );
    }

    *out_rgb = dst;
    ESP_LOGI(TAG, "Center crop %dx%d", crop_w, crop_h);
    return ESP_OK;
}

esp_err_t resize_rgb888(
    const uint8_t *src,
    int src_w,
    int src_h,
    int dst_w,
    int dst_h,
    uint8_t **out_rgb)
{
    uint8_t *dst = (uint8_t *)malloc(dst_w * dst_h * 3);
    if (!dst) {
        ESP_LOGE("PPM", "Resize alloc failed");
        return ESP_ERR_NO_MEM;
    }

    for (int y = 0; y < dst_h; y++) {
        int src_y = y * src_h / dst_h;
        for (int x = 0; x < dst_w; x++) {
            int src_x = x * src_w / dst_w;

            const uint8_t *p =
                src + (src_y * src_w + src_x) * 3;

            uint8_t *q =
                dst + (y * dst_w + x) * 3;

            q[0] = p[0];
            q[1] = p[1];
            q[2] = p[2];
        }
    }

    *out_rgb = dst;
    ESP_LOGI("PPM", "Resize %dx%d → %dx%d",
             src_w, src_h, dst_w, dst_h);

    return ESP_OK;
}

esp_err_t resize_rgb888_aspect_crop(
    const uint8_t *src,
    int src_w,
    int src_h,
    int dst_size,              // e.g. 128
    uint8_t **out_rgb)
{
    // Step 1: scale so short side == dst_size
    float scale;
    int scaled_w, scaled_h;

    if (src_w < src_h) {
        scale = (float)dst_size / src_w;
    } else {
        scale = (float)dst_size / src_h;
    }

    scaled_w = (int)(src_w * scale + 0.5f);
    scaled_h = (int)(src_h * scale + 0.5f);

    // Temporary buffer
    uint8_t *scaled =
        (uint8_t *)malloc(scaled_w * scaled_h * 3);
    if (!scaled) return ESP_ERR_NO_MEM;

    // Nearest-neighbour resize (aspect preserved)
    for (int y = 0; y < scaled_h; y++) {
        int sy = (int)(y / scale);
        for (int x = 0; x < scaled_w; x++) {
            int sx = (int)(x / scale);

            const uint8_t *p =
                src + (sy * src_w + sx) * 3;
            uint8_t *q =
                scaled + (y * scaled_w + x) * 3;

            q[0] = p[0];
            q[1] = p[1];
            q[2] = p[2];
        }
    }

    // Step 2: center crop to dst_size × dst_size
    int x0 = (scaled_w - dst_size) / 2;
    int y0 = (scaled_h - dst_size) / 2;

    uint8_t *dst =
        (uint8_t *)malloc(dst_size * dst_size * 3);
    if (!dst) {
        free(scaled);
        return ESP_ERR_NO_MEM;
    }

    for (int y = 0; y < dst_size; y++) {
        memcpy(
            dst + y * dst_size * 3,
            scaled + ((y0 + y) * scaled_w + x0) * 3,
            dst_size * 3
        );
    }

    free(scaled);
    *out_rgb = dst;

    ESP_LOGI("PPM",
        "Aspect resize+crop %dx%d → %dx%d",
        src_w, src_h, dst_size, dst_size);

    return ESP_OK;
}

void improve_rgb888_contrast(
    uint8_t *rgb,
    int width,
    int height)
{
    int pixels = width * height;

    for (int i = 0; i < pixels; i++) {
        for (int c = 0; c < 3; c++) {
            int v = rgb[i * 3 + c];

            // simple contrast stretch around mid-gray
            v = (v - 128) * 11 / 10 + 128;

            if (v < 0)   v = 0;
            if (v > 255) v = 255;

            rgb[i * 3 + c] = (uint8_t)v;
        }
    }
}

esp_err_t rgb888_to_grayscale(
    const uint8_t *src,
    int width,
    int height,
    uint8_t **out_gray)
{
    uint8_t *gray = (uint8_t *)malloc(width * height);
    if (!gray) return ESP_ERR_NO_MEM;

    int pixels = width * height;
    for (int i = 0; i < pixels; i++) {
        uint8_t r = src[i * 3 + 0];
        uint8_t g = src[i * 3 + 1];
        uint8_t b = src[i * 3 + 2];

        // ITU-R BT.601 luminance
        gray[i] = (uint8_t)(
            (77 * r + 150 * g + 29 * b) >> 8
        );
    }

    *out_gray = gray;
    return ESP_OK;
}

esp_err_t pgm_write_gray(
    const char *path,
    const uint8_t *gray,
    int width,
    int height)
{
    FILE *f = fopen(path, "wb");
    if (!f) return ESP_FAIL;

    fprintf(f, "P5\n%d %d\n255\n", width, height);
    fwrite(gray, 1, width * height, f);
    fclose(f);

    ESP_LOGI("PPM", "Wrote %s (%dx%d GRAY)", path, width, height);
    return ESP_OK;
}
