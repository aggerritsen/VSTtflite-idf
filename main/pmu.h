#ifndef PMU_H
#define PMU_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// I2C / Hardware configuration
// -----------------------------------------------------------------------------
#define AXP2101_I2C_ADDR        0x34
#define AXP2101_I2C_PORT        I2C_NUM_0

#define I2C_PMIC_SDA_PIN        15
#define I2C_PMIC_SCL_PIN        7
#define I2C_PMIC_FREQ_HZ        400000

// -----------------------------------------------------------------------------
// AXP2101 Register Addresses
// -----------------------------------------------------------------------------
#define AXP_REG_PMU_STATUS2     0x01
#define AXP_REG_FAULT_DCDC_OC   0x48

#define AXP_REG_DCDC_EN_CTRL    0x80
#define AXP_REG_DCDC1_VOLTAGE   0x82
#define AXP_REG_DCDC3_VOLTAGE   0x84

#define AXP_REG_LDO_ONOFF_0     0x90
#define AXP_REG_LDO_ONOFF_1     0x98

#define AXP_REG_ALDO1_VOLTAGE   0x92
#define AXP_REG_ALDO2_VOLTAGE   0x93
#define AXP_REG_ALDO3_VOLTAGE   0x94
#define AXP_REG_ALDO4_VOLTAGE   0x95

#define AXP_REG_BLDO1_VOLTAGE   0x96
#define AXP_REG_BLDO2_VOLTAGE   0x97

#define AXP_REG_DLDO1_VOLTAGE   0x99
#define AXP_REG_DLDO2_VOLTAGE   0x9A

// -----------------------------------------------------------------------------
// Public PMIC API
// -----------------------------------------------------------------------------
void axp2101_init_pmic(void);

esp_err_t axp2101_write_reg(uint8_t reg, uint8_t value);
esp_err_t axp2101_read_reg(uint8_t reg, uint8_t *value);

// -----------------------------------------------------------------------------
// Diagnostics API (merged from pmu_axp2101_diag.*)
// -----------------------------------------------------------------------------
void axp2101_dump_all_registers(void);
void axp2101_verify_settings(void);

#ifdef __cplusplus
}
#endif

#endif // PMU_H
