# ESP-IDF Configuration Guide

This document summarizes all **custom and critical sdkconfig settings** required for the ESP32-S3 camera + ML pipeline project. It covers the full path from **boot → power rails → camera → SD → Wi-Fi → inference**, and explains **why specific non-default choices were made**.

### Target Environment
* **ESP-IDF:** 5.5.1
* **Hardware:** ESP32-S3
* **Features:** Camera + PSRAM + SDMMC + PMU + Modem + TFLite-Micro

---

## 🔧 1. Set the Correct Target (CRITICAL)

Before configuring anything, run:

```bash
idf.py set-target esp32s3

```

**Or in VS Code:** `F1` → `IDF: ESP Set Espressif Device Target` → `esp32s3`

---

## 🔥 2. Flash Configuration (Serial Flasher Config)**Menu:** `Serial flasher config`

* **Flash size:** 16 MB
* **Flash frequency:** 80 MHz
* **SPI mode:** DIO

**Resulting flags:**

```ini
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_ESPTOOLPY_FLASHMODE_DIO=y

```

**Rationale:** Matches common ESP32-S3 modules. Required for PSRAM + SD + large ML models.

---

## 🧠 3. PSRAM (Octal Mode, Mandatory for ML)**Menu:** `Component config` → `ESP PSRAM`

* **Enable:** External SPI-connected RAM
* **Mode:** Octal Mode PSRAM
* **PSRAM speed:** 80 MHz
* **Always-internal malloc threshold:** 16384 bytes

**Relevant flags:**

```ini
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384
CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=65536

```

**Why:** TFLite-Micro tensor arena (>2 MB) lives in PSRAM. JPEG decode buffers + RGB frames exceed internal SRAM. This config keeps ISR / Wi-Fi buffers internal for stability.

---

## 🖥️ 4. Console Output (USB Serial/JTAG)**Menu:** `Component config` → `ESP-STDIO`

* **Console output:** USB Serial/JTAG
* **UART:** Enabled for fallback

**Flags:**

```ini
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
CONFIG_ESP_CONSOLE_UART=y
CONFIG_ESP_CONSOLE_UART_NUM=0
CONFIG_ESP_CONSOLE_UART_BAUDRATE=115200

```

**Benefits:** No external USB-UART adapter needed. Provides JTAG + logs over one cable and frees GPIOs for camera / SD.

---

## 📜 5. Logging Configuration**Menu:** `Component config` → `Log`

* **Default log level:** Info
* **Dynamic per-tag control:** Enabled

```ini
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
CONFIG_LOG_DYNAMIC_LEVEL_CONTROL=y
CONFIG_BOOTLOADER_LOG_LEVEL_INFO=y

```

---

## 📦 6. Custom Partition Table (REQUIRED)**Menu:** `Partition Table`

```ini
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"

```

**Why:** Large FAT partition needed for ML models (`/sdcard/models`) and captured JPEG / PPM frames.

---

## 💾 7. SD Card (SDMMC, 1-bit mode)**Hardware (T-SIM7080G-S3):**

* CLK → GPIO 38
* CMD → GPIO 39
* D0 → GPIO 40
* Mode: 1-bit only

**FATFS tuning:**

```ini
CONFIG_FATFS_VOLUME_COUNT=2
CONFIG_FATFS_SECTOR_4096=y
CONFIG_FATFS_ALLOC_PREFER_EXTRAM=y

```

**Why:** 4-KB sectors improve SD performance. Allocation in PSRAM reduces SRAM pressure.

---

## ⚡ 8. CPU Frequency**Menu:** `Component config` → `ESP System Settings`

```ini
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_160=y

```

**Why:** Stable performance and lower thermals than 240 MHz.

---

## 🛌 9. Sleep & Power Management**Menu:** `Component config` → `ESP System Settings` → `Sleep`

```ini
CONFIG_ESP_SLEEP_DEEP_SLEEP_WAKEUP_DELAY=2000
CONFIG_PM_SLEEP_FUNC_IN_IRAM=y
CONFIG_PM_SLP_IRAM_OPT=y

```

---

## 🔌 10. I²C Legacy Driver (INTENTIONAL)**Configuration:**

```ini
CONFIG_SCCB_HARDWARE_I2C_DRIVER_LEGACY=y
CONFIG_SCCB_HARDWARE_I2C_PORT1=y

```

**Why legacy I²C is used:**
This project intentionally avoids the new ESP-IDF I²C v2 driver for Camera SCCB and PMU (AXP192 / AXP2101).

* Camera SCCB drivers are legacy-oriented.
* New driver can cause SCCB NACKs or init failures.
* Deterministic startup is prioritized over async flexibility.

---

## 📷 11. Camera Configuration**Enabled sensors:**

```ini
CONFIG_OV2640_SUPPORT=y
CONFIG_OV5640_SUPPORT=y
CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX=32768
CONFIG_CAMERA_TASK_STACK_SIZE=4096
CONFIG_CAMERA_CORE0=y

```

**JPEG decoder:**

```ini
CONFIG_JD_USE_ROM=y
```

**Why:** ROM JPEG decoder is faster and stable.

---

## 🧠 12. ML / TFLite-Micro Considerations* **Memory:** PSRAM + cache layout supports large tensor arena.
* **Optimization:** ESP-NN optimizations enabled.
* **Performance:** ~9.3–9.5 seconds per frame (YOLOv8n int8).

---

## 🧪 13. Final Checklist| Feature | Status |

| --- | --- |
| **Target** | esp32s3 |
| **Flash** | 16MB / 80MHz / DIO |
| **PSRAM** | Octal @ 80 MHz |
| **Console** | USB Serial/JTAG |
| **Logging** | Info |
| **Partition table** | Custom |
| **SDMMC** | 1-bit (38/39/40) |
| **I²C** | Legacy |
| **Camera** | OV2640 / OV5640 |
| **ML** | TFLite-Micro (int8) |
| **CPU** | 160 MHz |

---

✅ SummaryThis `sdkconfig` is purpose-built to maximize stability, avoid silent hardware failures, and support large ML workloads. Change settings only with intent.
