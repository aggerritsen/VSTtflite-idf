// File: main/ppm.h
#pragma once

#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Write RGB888 buffer as binary PPM (P6)
// -----------------------------------------------------------------------------
esp_err_t ppm_write_rgb888(
    const char *path,
    const uint8_t *rgb,
    int width,
    int height
);

// -----------------------------------------------------------------------------
// JPEG → RGB888 (via RGB565 internally)
// -----------------------------------------------------------------------------
esp_err_t jpeg_to_rgb888(
    const uint8_t *jpeg,
    size_t jpeg_len,
    uint8_t **out_rgb,
    int *out_w,
    int *out_h
);

// -----------------------------------------------------------------------------
// Center crop RGB888
// -----------------------------------------------------------------------------
esp_err_t crop_rgb888_center(
    const uint8_t *src,
    int src_w,
    int src_h,
    int crop_w,
    int crop_h,
    uint8_t **out_rgb
);

// -----------------------------------------------------------------------------
// Resize RGB888 (nearest-neighbour)
// -----------------------------------------------------------------------------
esp_err_t resize_rgb888(
    const uint8_t *src,
    int src_w,
    int src_h,
    int dst_w,
    int dst_h,
    uint8_t **out_rgb
);


esp_err_t resize_rgb888_aspect_crop(
    const uint8_t *src,
    int src_w,
    int src_h,
    int dst_size,
    uint8_t **out_rgb
);

esp_err_t rgb888_to_grayscale(
    const uint8_t *src,
    int width,
    int height,
    uint8_t **out_gray
);

esp_err_t pgm_write_gray(
    const char *path,
    const uint8_t *gray,
    int width,
    int height
);

// -----------------------------------------------------------------------------
// Light contrast improvement (RGB888, in-place)
// -----------------------------------------------------------------------------
void improve_rgb888_contrast(
    uint8_t *rgb,
    int width,
    int height
);


#ifdef __cplusplus
}
#endif