#pragma once
#include "esp_err.h"

// I2C + PMIC init
esp_err_t pmu_init_i2c(void);
void pmu_init(void);

// Verification layers
bool pmu_verify_basic(void);     // returns true if ALL critical rails OK
