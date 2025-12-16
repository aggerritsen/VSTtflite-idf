# ESP32 Camera → SD → Preview → TensorFlow Lite Micro Pipeline  
*(YOLOv8n – full integer quant, 192×192)*


---

## 1. Generic description

### Hardware
This project runs on an **ESP32-S3–class board** with:
- **Integrated camera interface OV2640**
- **External PSRAM** (required for TFLite Micro tensor arena)
- **AXP2101 PMU** (power management IC)
- **SD card** (SPI or SDMMC, board-specific)
- **Cellular modem** (used for time synchronization)
- **Wi-Fi** (used for HTTP preview)

**Camera**
- Typical ESP32 camera module (e.g. OV2640 / OV5640)
- JPEG capture mode
- Native resolution used here: **320×240**

The pipeline is designed to be deterministic and observable, not minimal.

**HTTPD**
Realtime image preview is available via webserver:
- SSID: **CAM-S3** (no password)
- Webpage http://192.168.4.1

---

### Software stack

- **ESP-IDF**
  - FreeRTOS
  - esp_camera
  - Wi-Fi + LWIP
  - SD card drivers
- **TensorFlow Lite Micro**
  - INT8 input / INT8 output
  - Custom operator resolver
- **YOLOv8n TFLite model**
  - Input: 192×192 RGB
  - Full integer quantization
  - Exported externally (likely via Google Colab)

Additional project modules:
- `pmu.*` – AXP2101 power rails
- `camera.*` – camera init wrapper
- `sdcard.*` – SD mount & file IO
- `modem.*` – UART modem + timestamp
- `wifi.*` – AP mode
- `httpd.*` – live JPEG preview
- `ppm.*` – JPEG→RGB, resize, PPM saving

---

### Installation hints

1. Use **ESP-IDF with PSRAM enabled**
2. Enable:
   - `CONFIG_SPIRAM_USE_MALLOC`
   - `CONFIG_ESP32_CAMERA`
3. Ensure SD card is mounted **before** model load
4. Place on SD card:
```
/sdcard/config/config.txt
/sdcard/models/yolov8n_2025-07-15_192_full_integer_quant.tflite
```

---

## 2. Initialization sequence

The application startup is strictly ordered:

### 1️⃣ Power rails (PMU)
- Initialize AXP2101
- Enable required rails for:
- ESP32 core
- Camera
- SD card
- Modem

Purpose: avoid brownouts and undefined peripheral state.

---

### 2️⃣ SD card mount
- SD card is mandatory:
- Model file is loaded from SD
- Captured frames are written to SD
- If SD mount fails → **application aborts**

---

### 3️⃣ Modem & timestamp
- Modem initialized via UART
- Timestamp queried (e.g. `AT+CCLK?`)
- Parsed into:
- Compact format (`YYYYMMDD_HHMMSS`)
- ISO-like format
- ESP32 system time is set via `settimeofday()`

Purpose:
- Deterministic filenames
- Traceable frame history
- Dataset consistency

---

### 4️⃣ Wi-Fi + HTTP preview
- ESP32 runs as **Wi-Fi Access Point**
- HTTP server started
- Latest JPEG frame exposed for live preview

This is *non-blocking* and independent of inference.

---

### 5️⃣ Camera initialization
- Camera configured for JPEG output
- Fixed resolution
- If camera init fails → **application aborts**

---

### 6️⃣ Model loading
- Model filename read from `/sdcard/config/config.txt`
- Model loaded fully into PSRAM
- TensorFlow Lite Micro interpreter created
- Tensor arena allocated (~2 MB)
- Input and output tensors resolved

At this point the system is **ready to run inference**.

---

## 3. Pipeline sequence (runtime loop)

Each iteration of the pipeline follows the same deterministic sequence:

### ① Capture
- Grab JPEG frame from camera (`esp_camera_fb_get`)
- Validate format

---

### ② Timestamp
- Frame sequence counter incremented
- System time already synchronized at boot

---

### ③ Preview
- Raw JPEG pushed to HTTP server
- Enables live monitoring without blocking pipeline

---

### ④ Decode (JPEG → RGB)
- JPEG decoded into **RGB888**
- Original camera resolution preserved (e.g. 320×240)

---

### ⑤ Resize
Two parallel paths are generated:

#### A. Cropped (aspect-preserving, center crop)
- Resize + crop to **192×192**
- No distortion
- Content loss possible
- Used mainly for comparison and diagnostics

#### B. Non-cropped (letterboxed)
- Uniform scale
- Padding added (black bars)
- Full field of view preserved
- Geometry preserved
- Output size: **192×192 RGB888**

---

### ⑥ Save
Three artifacts are written to SD:
- Original JPEG
- Cropped PPM (RGB888)
- Non-cropped (letterboxed) PPM (RGB888)

This enables **offline verification** of preprocessing.

---

### ⑦ Quantize
- RGB888 → INT8
- Uses model-provided:
- scale
- zero-point
- Input tensor fully populated

---

### ⑧ Infer
- `Invoke()` called on TFLite Micro interpreter
- Execution time logged (~9–10 s)
- No dynamic allocation during inference

---

### ⑨ Decode (attempted)
- Output tensor inspected
- Statistics logged
- YOLO-style decode attempted (currently mismatched)

---

### ⑩ Repeat
- Frame buffer returned
- Delay
- Next frame captured

---

## 4. Logging, analysis, and why we cannot continue (yet)

### What logging already gives us

The current logging is **extensive and correct**:

#### Input side
- RGB statistics (min / max / mean)
- INT8 statistics (raw + dequantized)
- Confirms:
- Geometry correctness
- Quantization is active
- Input is stable frame-to-frame

#### Inference
- Interpreter status (`kTfLiteOk`)
- Execution time
- Output tensor shape and type

#### Output inspection
- Raw INT8 samples
- Dequantized value ranges
- Deterministic behavior across frames

---

### The key observation

```
OUTPUT dims: [1 x 8 x 756]
```

This tells us conclusively:

- Output is **NOT a YOLO grid**
- `N = 8` is not a square → no spatial grid
- The model outputs **proposal vectors**, not raw detection heads
- The model is **full integer quant**, likely with post-processing embedded

As a result:
- DFL decoding is invalid
- Grid/stride math is invalid
- Sigmoid application is likely wrong
- Bounding boxes and class scores are misinterpreted

The inference **is running correctly**, but the **decoder does not match the model export format**.

---

### Why we cannot continue decoding blindly

At this point, runtime logging has extracted **all possible information** without inspecting the model itself.

What we do **not** know (and cannot infer from logs alone):

1. Exact layout of the 756 values per proposal
2. Whether box coordinates are:
   - normalized or absolute
3. Whether class scores are:
   - logits
   - probabilities
   - already thresholded
4. Whether NMS is already applied
5. Where objectness ends and class scores begin

---

### Questions that must be answered to proceed

To implement the **correct decoder**, one of the following is required:

1. The **Colab export script / notebook**
2. A **Netron inspection** of the `.tflite` model
3. Documentation of the export pipeline used

Without this, any further decoding would be guesswork.

---

## Summary

- The hardware and pipeline are working correctly
- Preprocessing and geometry are now correct
- Inference executes deterministically
- Logging is already at a very high level
- The blocker is **model output semantics**, not implementation bugs

Once the model’s output format is confirmed, decoding can be completed cleanly.

---

## References

The following documents provide detailed background and implementation specifics for this project:

- [SDKCONFIG.md](docs/SDKCONFIG.md)
  Setting of _sdkconfig_ which can be set by:
  ```
  idf.py menuconfig
  ```

- [PMU_README.md](./docs/PMU_README.md)  
  Detailed description of the AXP2101 PMU, power rails, and power-up sequencing.

- [CAM_SD_INIT.md](./docs/CAM_SD_INIT.md)  
  Camera and SD card initialization details, constraints, and failure modes.

- [MODEM.md](./docs/MODEM.md)  
  Modem integration, UART handling, and timestamp acquisition/parsing.

- [T-SIM7080G-S3_Pinout.md](.docs/T-SIM7080G-S3_Pinout.md)
  Overview of pins and GPIO's

- [User-Manual-6032606.pdf](.docs/User-Manual-6032606.pdf)
  Documentation in T-SIM7080G-S3 board

- [Camera_OV2640.pdf](.docs/Camera_OV2640.pdf)
  Documentation on camera OV2640

- [AXP2101_Datasheet_V1.0_en_3832.pdf](.docs/AXP2101_Datasheet_V1.0_en_3832.pdf)  
  Documentation on AXP2101 Power module

- [README.md](./docs/README.md)  
  Lessons learned.

---
