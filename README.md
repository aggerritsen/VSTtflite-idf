# VSTtflite-idf

## Hardware Pinout

![T-SIM7080G-S3 Pinout](docs/Lilygo T-SIM7080G-S3 PINOUT.jpg)

Full pinout description: [docs/T-SIM7080G-S3_Pinout.md](docs/T-SIM7080G-S3_Pinout.md)

## ESP-IDF Configuration

See: [docs/ESP_IDF_Configuration.md](docs/ESP_IDF_Configuration.md)

## Project Overview

This project demonstrates running TensorFlow Lite Micro (TFLM) on the ESP32‑S3 T‑SIM7080G‑S3 board with:

- SD card loading of `.tflite` models  
- PSRAM-based tensor arena allocation  
- SDMMC file system scanning  
- Diagnostics of RAM, model ops, and interpreter initialization  
- Logging heap usage during every major step  

The model is loaded from:

```
/sdcard/models/vespcv_swiftyolo_int8_vela.tflite
```

⚠️ The current model contains **`ethos-u` custom ops**, which are not supported on ESP32‑class microcontrollers. A new model without ARM Ethos-U acceleration must be generated before inference can run.

## Features

- PSRAM 8 MB auto‑detected & validated
- SD card mounted in 1‑bit SDMMC mode
- Model loaded into PSRAM
- Interpreter initialized with PSRAM tensor arena
- Automatic directory scanning: `/sdcard/images`
- Detailed heap printouts at every stage

## Current Status

✔ PSRAM OK  
✔ SDMMC OK  
✔ Model loaded OK  
✖ AllocateTensors fails due to unsupported custom ops (`ethos-u`)

## Partition Table

Custom `partitions.csv` is used:

```
nvs,      data, nvs,     0x9000,  0x6000
phy_init, data, phy,     0xf000,  0x1000
factory,  app,  factory, 0x10000, 0xC00000
storage,  data, 0x81,    0xC10000,0x2F0000
```

## Repository Structure

```
├─ main/
│   ├─ app_main.cpp
├─ docs/
│   ├─ ESP_IDF_Configuration.md
│   ├─ T-SIM7080G-S3_Pinout.md
│   ├─ Lilygo T-SIM7080G-S3 PINOUT.jpg
├─ partitions.csv
├─ README.md
```
