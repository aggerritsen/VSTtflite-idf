#include "pmu.h"

#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PMU";

// -----------------------------------------------------------------------------
// Hardware / I2C configuration (T-SIM7080G-S3 + AXP2101)
// -----------------------------------------------------------------------------
#define AXP_I2C_ADDR        0x34

#define PMU_I2C_PORT        I2C_NUM_0
#define PMU_I2C_SDA         GPIO_NUM_15
#define PMU_I2C_SCL         GPIO_NUM_7
#define PMU_I2C_FREQ_HZ     400000

// -----------------------------------------------------------------------------
// AXP2101 registers (localized, no global bleed)
// -----------------------------------------------------------------------------
#define AXP_REG_DCDC_EN_CTRL    0x80
#define AXP_REG_FAULT_DCDC_OC   0x48

#define AXP_REG_DCDC1_VOLTAGE   0x82
#define AXP_REG_DCDC3_VOLTAGE   0x84

#define AXP_REG_ALDO1_VOLTAGE   0x92
#define AXP_REG_ALDO2_VOLTAGE   0x93
#define AXP_REG_ALDO3_VOLTAGE   0x94
#define AXP_REG_ALDO4_VOLTAGE   0x95
#define AXP_REG_BLDO1_VOLTAGE   0x96
#define AXP_REG_BLDO2_VOLTAGE   0x97
#define AXP_REG_DLDO1_VOLTAGE   0x99
#define AXP_REG_DLDO2_VOLTAGE   0x9A

#define AXP_REG_LDO_ONOFF_0     0x90
#define AXP_REG_LDO_ONOFF_1     0x98

// -----------------------------------------------------------------------------
// Legacy I2C low-level helpers
// -----------------------------------------------------------------------------
static esp_err_t axp_write(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(PMU_I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    vTaskDelay(pdMS_TO_TICKS(5));
    return err;
}

static esp_err_t axp_read(uint8_t reg, uint8_t *val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(PMU_I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return err;
}

static bool pmu_read(uint8_t reg, uint8_t *val)
{
    return axp_read(reg, val) == ESP_OK;
}

// -----------------------------------------------------------------------------
// Voltage decode helpers (datasheet-correct)
// -----------------------------------------------------------------------------
static float get_dcdc1_voltage(uint8_t reg)
{
    uint8_t vset = reg & 0x1F;
    if (vset <= 0x13) {
        return 1.5f + (float)vset * 0.1f;
    }
    return -1.0f;
}

static float get_dcdc3_voltage(uint8_t reg)
{
    uint8_t vset = reg & 0x7F;

    if (vset <= 0x46) {
        return 0.50f + (float)vset * 0.01f;
    } else if (vset <= 0x57) {
        return 1.22f + (float)(vset - 0x47) * 0.02f;
    } else if (vset <= 0x6B) {
        return 1.60f + (float)(vset - 0x58) * 0.1f;
    }
    return -1.0f;
}

static float get_ldo_voltage(uint8_t reg)
{
    uint8_t vset = reg & 0x1F;
    if (vset <= 0x1E) {
        return 0.5f + (float)vset * 0.1f;
    }
    return -1.0f;
}

// -----------------------------------------------------------------------------
// Shared verification helper
// -----------------------------------------------------------------------------
static bool pmu_check_voltage(
    const char *name,
    uint8_t reg,
    float (*decode)(uint8_t),
    float expected_v,
    uint8_t expected_raw)
{
    uint8_t raw;
    if (!pmu_read(reg, &raw)) {
        ESP_LOGE(TAG, "%s: read failed", name);
        return false;
    }

    float actual = decode(raw);
    bool match = (raw == expected_raw);

    ESP_LOGI(TAG,
        "%-22s %4.2f V (reg=0x%02X, expected %.2f V) -> %s",
        name,
        actual,
        raw,
        expected_v,
        match ? "MATCH" : "FAIL"
    );

    return match;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
esp_err_t pmu_init_i2c(void)
{
    i2c_config_t cfg = {};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = PMU_I2C_SDA;
    cfg.scl_io_num = PMU_I2C_SCL;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = PMU_I2C_FREQ_HZ;

    esp_err_t err = i2c_param_config(PMU_I2C_PORT, &cfg);
    if (err != ESP_OK) return err;

    return i2c_driver_install(PMU_I2C_PORT, cfg.mode, 0, 0, 0);
}

void pmu_init(void)
{
    ESP_LOGI(TAG, "Configuring AXP2101 power rails");

    // Clear fault latches
    axp_write(AXP_REG_FAULT_DCDC_OC, 0xFF);

    // Core rails
    axp_write(AXP_REG_DCDC1_VOLTAGE, 0x12); // 3.3V ESP32
    axp_write(AXP_REG_DCDC3_VOLTAGE, 0x66); // 3.0V modem/GNSS

    // LDO rails
    axp_write(AXP_REG_ALDO1_VOLTAGE, 0x0D); // 1.8V CAM core
    axp_write(AXP_REG_ALDO2_VOLTAGE, 0x17); // 2.8V CAM IO
    axp_write(AXP_REG_ALDO3_VOLTAGE, 0x1C); // 3.3V SD
    axp_write(AXP_REG_ALDO4_VOLTAGE, 0x19); // 3.0V CAM analog

    axp_write(AXP_REG_BLDO1_VOLTAGE, 0x0D); // 1.8V level shift
    axp_write(AXP_REG_BLDO2_VOLTAGE, 0x1C); // 3.3V modem

    axp_write(AXP_REG_DLDO1_VOLTAGE, 0x1C); // 3.3V
    axp_write(AXP_REG_DLDO2_VOLTAGE, 0x12); // 2.3V

    // Enable LDOs
    axp_write(AXP_REG_LDO_ONOFF_0, 0xFF);
    axp_write(AXP_REG_LDO_ONOFF_1, 0x03);

    // Enable DCDC1 + DCDC3
    axp_write(AXP_REG_DCDC_EN_CTRL, 0x05);

    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "PMU configuration complete");
}

// -----------------------------------------------------------------------------
// BASIC verification (always safe to run)
// -----------------------------------------------------------------------------
bool pmu_verify_basic(void)
{
    ESP_LOGI(TAG, "=== PMU basic rail verification ===");

    bool ok = true;

    // ---------------------------------------------------------------------
    // ESP32S3 core domain
    // ---------------------------------------------------------------------
    ok &= pmu_check_voltage(
        "DCDC1 (ESP32)",
        AXP_REG_DCDC1_VOLTAGE,
        get_dcdc1_voltage,
        3.3f,
        0x12
    );

    // ---------------------------------------------------------------------
    // Modem / GNSS domain
    // ---------------------------------------------------------------------
    ok &= pmu_check_voltage(
        "DCDC3 (Modem/GPS)",
        AXP_REG_DCDC3_VOLTAGE,
        get_dcdc3_voltage,
        3.0f,
        0x66
    );

    ok &= pmu_check_voltage(
        "BLDO2 (Modem I/O)",
        AXP_REG_BLDO2_VOLTAGE,
        get_ldo_voltage,
        3.3f,
        0x1C
    );

    // ---------------------------------------------------------------------
    // Camera domain
    // ---------------------------------------------------------------------
    ok &= pmu_check_voltage(
        "ALDO1 (Camera Core)",
        AXP_REG_ALDO1_VOLTAGE,
        get_ldo_voltage,
        1.8f,
        0x0D
    );

    ok &= pmu_check_voltage(
        "ALDO2 (Camera I/O)",
        AXP_REG_ALDO2_VOLTAGE,
        get_ldo_voltage,
        2.8f,
        0x17
    );

    ok &= pmu_check_voltage(
        "ALDO4 (Camera Analog)",
        AXP_REG_ALDO4_VOLTAGE,
        get_ldo_voltage,
        3.0f,
        0x19
    );

    // ---------------------------------------------------------------------
    // SD Card domain
    // ---------------------------------------------------------------------
    ok &= pmu_check_voltage(
        "ALDO3 (SD Card)",
        AXP_REG_ALDO3_VOLTAGE,
        get_ldo_voltage,
        3.3f,
        0x1C
    );

    // ---------------------------------------------------------------------
    // Level shifting domain
    // ---------------------------------------------------------------------
    ok &= pmu_check_voltage(
        "BLDO1 (Level Shift)",
        AXP_REG_BLDO1_VOLTAGE,
        get_ldo_voltage,
        1.8f,
        0x0D
    );

    ESP_LOGI(
        TAG,
        "=== PMU basic verification: %s ===",
        ok ? "OK" : "FAILED"
    );

    return ok;
}
