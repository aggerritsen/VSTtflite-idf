// File: main/sdcard.cpp
// SD Card component implementation for LILYGO T-SIM7080G-S3

#include "sdcard.h"

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "driver/gpio.h" // Needed for gpio_num_t cast

#include <sys/stat.h>
#include <sys/unistd.h>
#include <errno.h>
#include <dirent.h> // Required for opendir, readdir, closedir
#include <string.h> // Required for strcmp

static const char *TAG = "SDCARD";
static const char *MOUNT_POINT = "/sdcard";

static sdmmc_card_t *s_sd_card = nullptr;

// --- CORRECT GPIO DEFINITIONS FOR LILYGO T-SIM7080G-S3 SD SLOT (SDMMC 1-bit) ---
#define SDCARD_PIN_CMD 	39
#define SDCARD_PIN_CLK 	38
#define SDCARD_PIN_D0 	40 		// 'DATA' is D0 in 1-bit mode
#define SDCARD_PIN_D1 	GPIO_NUM_NC 
#define SDCARD_PIN_D2 	GPIO_NUM_NC 
#define SDCARD_PIN_D3 	GPIO_NUM_NC 


// =========================================================================
// Mount SD card (SDMMC, 1-bit mode)
// =========================================================================
esp_err_t sdcard_mount(void)
{
	if (s_sd_card) {
		ESP_LOGI(TAG, "SD card already mounted");
		return ESP_OK;
	}

	ESP_LOGI(TAG, "Mounting SD card at '%s' (SDMMC, 1-bit) using PINS: CMD=39, CLK=38, D0=40", MOUNT_POINT);

	// 1. SDMMC Host Config (Specify 1-bit width)
	sdmmc_host_t host = SDMMC_HOST_DEFAULT();
	host.flags |= SDMMC_HOST_FLAG_1BIT;
	host.max_freq_khz = SDMMC_FREQ_DEFAULT; 

	// 2. SDMMC Slot Config 
	sdmmc_slot_config_t slot = {}; 	
	
	// --- Pin assignments with explicit casting ---
	slot.clk = (gpio_num_t)SDCARD_PIN_CLK;
	slot.cmd = (gpio_num_t)SDCARD_PIN_CMD;
	slot.d0 = (gpio_num_t)SDCARD_PIN_D0;
	
	// Unused pins set to GPIO_NUM_NC
	slot.d1 = SDCARD_PIN_D1;
	slot.d2 = SDCARD_PIN_D2;
	slot.d3 = SDCARD_PIN_D3;
	
	slot.cd = GPIO_NUM_NC; 		// Card Detect
	slot.wp = GPIO_NUM_NC; 		// Write Protect (Not wired on this board)
	slot.width = 1; 			// Enforce 1-bit width

	// Configure internal pull-ups for the required lines (CLK, CMD, D0)
	slot.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;


	esp_vfs_fat_mount_config_t mount_cfg = {
		.format_if_mount_failed = false, // Keep false, we assume a pre-formatted card
		.max_files = 5,
		.allocation_unit_size = 16 * 1024,
		.disk_status_check_enable = false,
		.use_one_fat = false
	};

	esp_err_t ret = esp_vfs_fat_sdmmc_mount(
		MOUNT_POINT,
		&host,
		&slot, 	
		&mount_cfg,
		&s_sd_card
	);

	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "SD mount failed (non-fatal): %s (Ensure card is present)", esp_err_to_name(ret));
		s_sd_card = nullptr;
		return ret;
	}

	ESP_LOGI(TAG, "SD mounted OK, card name: %s", s_sd_card->cid.name);
	return ESP_OK;
}


// =========================================================================
// List Directory Contents (Diagnostic)
// =========================================================================
esp_err_t sdcard_print_directory_tree(const char *path)
{
	if (!sdcard_is_mounted()) {
		ESP_LOGE(TAG, "Cannot read directory: SD card is not mounted.");
		return ESP_FAIL;
	}
	
	DIR *dir = opendir(path);
	if (!dir) {
		// Log the underlying errno for detailed debug if opendir fails
		ESP_LOGE(TAG, "Failed to open directory %s. errno: %d (%s)", 
				 path, errno, strerror(errno));
		return ESP_FAIL;
	}
	
	ESP_LOGI(TAG, "--- SD Card Contents (%s) ---", path);
	struct dirent *entry;
	int file_count = 0;
	while ((entry = readdir(dir)) != NULL) {
		// Skip current (.) and parent (..) directory entries
		if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
			continue;
		}

		const char *type;
		// Determine file type (DT_DIR = 4, DT_REG = 8)
		if (entry->d_type == DT_DIR) {
			type = "DIR";
		} else if (entry->d_type == DT_REG) {
			type = "FILE";
		} else {
			type = "OTHER";
		}

		ESP_LOGI(TAG, "	 [%s] %s", type, entry->d_name);
		file_count++;
	}
	
	if (file_count == 0) {
		ESP_LOGI(TAG, "	 (Directory is empty)");
	}
	ESP_LOGI(TAG, "--- End of SD Card Contents ---");

	closedir(dir);
	return ESP_OK;
}


// =========================================================================
// Unmount SD card
// =========================================================================
esp_err_t sdcard_unmount(void)
{
	if (!s_sd_card) {
		ESP_LOGW(TAG, "SD not mounted");
		return ESP_OK;
	}

	ESP_LOGI(TAG, "Unmounting SD card");
	esp_err_t ret = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_sd_card);
	s_sd_card = nullptr;
	return ret;
}

// =========================================================================
// Helpers
// =========================================================================
bool sdcard_is_mounted(void)
{
	return s_sd_card != nullptr;
}

esp_err_t sdcard_mkdir(const char *path)
{
	// 0777 is full permissions (rwxrwxrwx)
	if (::mkdir(path, 0777) == 0 || errno == EEXIST) {
		return ESP_OK;
	}
	ESP_LOGE(TAG, "mkdir('%s') failed errno=%d (%s)", path, errno, strerror(errno));
	return ESP_FAIL;
}

esp_err_t sdcard_write_file(const char *path, const uint8_t *data, size_t size)
{
	// "wb" -> Write, Binary mode
	FILE *f = ::fopen(path, "wb");
	if (!f) {
		// Log the underlying errno for detailed debug
		ESP_LOGE(TAG, "fopen('%s') failed. errno: %d (%s)", path, errno, strerror(errno));
		return ESP_FAIL;
	}

	size_t written = fwrite(data, 1, size, f);
	fclose(f);

	if (written != size) {
		// If short write, log the written count vs expected size
		ESP_LOGE(TAG, "short write (%u/%u) to %s",
					 (unsigned)written, (unsigned)size, path);
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Wrote %u bytes to %s", (unsigned)size, path);
	return ESP_OK;
}