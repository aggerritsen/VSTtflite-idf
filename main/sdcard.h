#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Mounts the SD card using SDMMC 1-bit mode.
 * @return ESP_OK on success.
 */
esp_err_t sdcard_mount(void);

/**
 * @brief Unmounts the SD card.
 * @return ESP_OK on success.
 */
esp_err_t sdcard_unmount(void);

/**
 * @brief Checks if the SD card is currently mounted.
 * @return true if mounted, false otherwise.
 */
bool sdcard_is_mounted(void);

/**
 * @brief Creates a directory on the SD card.
 * @param path The full path to the directory (e.g., "/sdcard/captures").
 * @return ESP_OK on success or if the directory already exists.
 */
esp_err_t sdcard_mkdir(const char *path);

/**
 * @brief Writes data to a file on the SD card.
 * @param path The full path to the file (e.g., "/sdcard/image.jpg").
 * @param data Pointer to the data buffer.
 * @param size Size of the data in bytes.
 * @return ESP_OK on success.
 */
esp_err_t sdcard_write_file(const char *path, const uint8_t *data, size_t size);

/**
 * @brief Diagnostic function to print the contents of a directory on the SD card.
 * @param path The directory path to list (e.g., "/sdcard").
 * @return ESP_OK on success.
 */
esp_err_t sdcard_print_directory_tree(const char *path);