#include "pmu.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "AXP2101_PMIC"

// -----------------------------------------------------------------------------
// I2C init
// -----------------------------------------------------------------------------
static esp_err_t axp2101_i2c_init(void)
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_PMIC_SDA_PIN;
    conf.scl_io_num = I2C_PMIC_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_PMIC_FREQ_HZ;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(AXP2101_I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
        return err;
    }

    err = i2c_driver_install(AXP2101_I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return err;
    }

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// I2C register access (PUBLIC)
// -----------------------------------------------------------------------------
esp_err_t axp2101_write_reg(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP2101_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
        AXP2101_I2C_PORT, cmd, pdMS_TO_TICKS(1000));

    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(5));
    return ret;
}

esp_err_t axp2101_read_reg(uint8_t reg, uint8_t *value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP2101_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP2101_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
        AXP2101_I2C_PORT, cmd, pdMS_TO_TICKS(1000));

    i2c_cmd_link_delete(cmd);
    return ret;
}

// -----------------------------------------------------------------------------
// PMIC INITIALIZATION
// -----------------------------------------------------------------------------
void axp2101_init_pmic(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Initializing AXP2101 PMIC");

    if (axp2101_i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "PMIC I2C init failed");
        return;
    }

    axp2101_write_reg(AXP_REG_PMU_STATUS2, 0xFF);
    axp2101_write_reg(AXP_REG_FAULT_DCDC_OC, 0xFF);

    axp2101_write_reg(AXP_REG_DCDC1_VOLTAGE, 0x12); // 3.3V
    axp2101_write_reg(AXP_REG_DCDC3_VOLTAGE, 0x66); // 3.0V

    axp2101_write_reg(AXP_REG_ALDO1_VOLTAGE, 0x0D); // 1.8V
    axp2101_write_reg(AXP_REG_ALDO2_VOLTAGE, 0x17); // 2.8V
    axp2101_write_reg(AXP_REG_ALDO3_VOLTAGE, 0x1C); // 3.3V
    axp2101_write_reg(AXP_REG_ALDO4_VOLTAGE, 0x19); // 3.0V

    axp2101_write_reg(AXP_REG_BLDO1_VOLTAGE, 0x0D); // 1.8V
    axp2101_write_reg(AXP_REG_BLDO2_VOLTAGE, 0x1C); // 3.3V

    axp2101_write_reg(AXP_REG_DLDO1_VOLTAGE, 0x1C); // 3.3V
    axp2101_write_reg(AXP_REG_DLDO2_VOLTAGE, 0x12); // 2.3V

    axp2101_write_reg(AXP_REG_LDO_ONOFF_0, 0xFF);
    axp2101_write_reg(AXP_REG_LDO_ONOFF_1, 0x03);
    axp2101_write_reg(AXP_REG_DCDC_EN_CTRL, 0x05);

    ESP_LOGI(TAG, "AXP2101 PMIC init complete");
}

// -----------------------------------------------------------------------------
// Voltage helpers
// -----------------------------------------------------------------------------
static float dcdc1_voltage(uint8_t v)
{
    uint8_t vset = v & 0x1F;
    return (vset <= 0x13) ? 1.5f + vset * 0.1f : -1.0f;
}

static float dcdc3_voltage(uint8_t v)
{
    uint8_t x = v & 0x7F;
    if (x <= 0x46) return 0.50f + x * 0.01f;
    if (x <= 0x57) return 1.22f + (x - 0x47) * 0.02f;
    if (x <= 0x6B) return 1.60f + (x - 0x58) * 0.1f;
    return -1.0f;
}

static float ldo_voltage(uint8_t v)
{
    uint8_t vset = v & 0x1F;
    return (vset <= 0x1E) ? 0.5f + vset * 0.1f : -1.0f;
}

// -----------------------------------------------------------------------------
// Diagnostics (FULL MERGE)
// -----------------------------------------------------------------------------
void axp2101_dump_all_registers(void)
{
    ESP_LOGI(TAG, "\n=== AXP2101 REGISTER MATRIX 0x00-0xFF ===");

    const int cols = 8;
    uint8_t reg_val;
    char line[64];

    for (uint16_t r = 0; r <= 0xFF; r++) {
        if ((r % cols) == 0) {
            if (r > 0) ESP_LOGI(TAG, "%s", line);
            snprintf(line, sizeof(line), "0x%02X |", r);
        }

        if (axp2101_read_reg(r, &reg_val) == ESP_OK) {
            char tmp[8];
            snprintf(tmp, sizeof(tmp), " %02X ", reg_val);
            strcat(line, tmp);
        } else {
            strcat(line, " -- ");
        }

        if (r == 0xFF) ESP_LOGI(TAG, "%s", line);
    }
}

void axp2101_verify_settings(void)
{
    uint8_t r;

    axp2101_dump_all_registers();

    ESP_LOGI(TAG, "--- PMIC Rail Status and Detailed Voltage Checks ---");

    if (axp2101_read_reg(AXP_REG_FAULT_DCDC_OC, &r) == ESP_OK) {
        ESP_LOGI(TAG,
            "REG 0x%02X (DCDC OC Fault) Read: 0x%02X (Expected: 0x00) - Status: %s",
            AXP_REG_FAULT_DCDC_OC,
            r,
            r == 0x00 ? "CLEAR" : "FAULT ACTIVE");
    }

    if (axp2101_read_reg(AXP_REG_DCDC_EN_CTRL, &r) == ESP_OK) {
        ESP_LOGI(TAG,
            "REG 0x%02X (DCDC Enable) Read: 0x%02X (Expected: 0x05) - Status: %s",
            AXP_REG_DCDC_EN_CTRL,
            r,
            (r & 0x05) == 0x05 ? "Enabled DCDC1/3" : "DISABLED");
    }

#define CHECK_LDO(reg, exp, label, calc) \
    if (axp2101_read_reg(reg, &r) == ESP_OK) { \
        float v = calc(r); \
        ESP_LOGI(TAG, \
            "REG 0x%02X (%s) Read: 0x%02X (%.2fV) | Expected: 0x%02X (%.2fV) -> %s", \
            reg, label, r, v, exp, calc(exp), (r == exp) ? "MATCH" : "FAIL"); \
    }

    CHECK_LDO(AXP_REG_DCDC1_VOLTAGE, 0x12, "DCDC1 - ESP32S3", dcdc1_voltage);
    CHECK_LDO(AXP_REG_DCDC3_VOLTAGE, 0x66, "DCDC3 - Modem/GPS", dcdc3_voltage);

    CHECK_LDO(AXP_REG_ALDO1_VOLTAGE, 0x0D, "ALDO1 - CAM Core", ldo_voltage);
    CHECK_LDO(AXP_REG_ALDO2_VOLTAGE, 0x17, "ALDO2 - CAM I/O", ldo_voltage);
    CHECK_LDO(AXP_REG_ALDO3_VOLTAGE, 0x1C, "ALDO3 - SD Card", ldo_voltage);
    CHECK_LDO(AXP_REG_ALDO4_VOLTAGE, 0x19, "ALDO4 - CAM Analog", ldo_voltage);

    CHECK_LDO(AXP_REG_BLDO1_VOLTAGE, 0x0D, "BLDO1 - Level Shift", ldo_voltage);
    CHECK_LDO(AXP_REG_BLDO2_VOLTAGE, 0x1C, "BLDO2 - Modem/GPS", ldo_voltage);

    CHECK_LDO(AXP_REG_DLDO1_VOLTAGE, 0x1C, "DLDO1", ldo_voltage);
    CHECK_LDO(AXP_REG_DLDO2_VOLTAGE, 0x12, "DLDO2", ldo_voltage);

#undef CHECK_LDO

    if (axp2101_read_reg(AXP_REG_LDO_ONOFF_0, &r) == ESP_OK) {
        ESP_LOGI(TAG,
            "REG 0x%02X (ALDO/BLDO Enable) Read: 0x%02X (Expected: 0xFF) - Status: %s",
            AXP_REG_LDO_ONOFF_0,
            r,
            r == 0xFF ? "Enabled" : "DISABLED");
    }

    if (axp2101_read_reg(AXP_REG_LDO_ONOFF_1, &r) == ESP_OK) {
        ESP_LOGI(TAG,
            "REG 0x%02X (DLDO Enable) Read: 0x%02X (Expected: 0x03) - Status: %s",
            AXP_REG_LDO_ONOFF_1,
            r,
            r == 0x03 ? "Enabled" : "DISABLED");
    }

    ESP_LOGI(TAG, "--- Verification Complete ---");
}
