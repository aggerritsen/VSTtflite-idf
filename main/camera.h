#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t camera_init(void);
esp_err_t camera_test_capture(void);

#ifdef __cplusplus
}
#endif
