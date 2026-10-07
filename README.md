# VisioSort AI: Edge AI Quality Inspection & Automated Sorting System

An end-to-end TinyML prototype designed for industrial quality control on a budget (<50€). The system performs on-device visual defect detection using an ESP32-CAM module and triggers an SG90 servo motor for real-time mechanical rejection.

---

## 📌 Architecture & Pipeline

```text
[OV2640 Sensor] ---> [RGB888 Framebuffer (DRAM)] ---> [Edge Impulse CNN Model]
                                                             |
                                                 [Confidence Threshold >= 0.70]
                                                             |
                                              [LEDC Hardware PWM (GPIO 13)]
                                                             |
                                                  [SG90 Sorting Arm]
```

1. **Image Acquisition:** The OV2640 camera sensor captures QVGA (320x240) frames, converted to RGB888 and rescaled for the classifier buffer.
2. **On-Device Inference:** Quantized Convolutional Neural Network (CNN) deployed via Edge Impulse C++ SDK (TensorFlow Lite for Microcontrollers), running on the Xtensa dual-core LX6 processor without cloud dependencies.
3. **Hardware Actuation:** Real-time pulse-width modulation (PWM) generated via the ESP32 `LEDC` hardware peripheral (50 Hz, 16-bit resolution) to actuate the SG90 micro-servo arm and reject defective items.

---

## ⚙️ Hardware Specifications & Pinout

* **Microcontroller:** AI Thinker ESP32-CAM (Xtensa dual-core 32-bit @ 240 MHz, 520 KB SRAM + 4 MB PSRAM)
* **Camera Sensor:** Omnivision OV2640 (2 Megapixel)
* **Actuator:** TowerPro SG90 Micro Servo (50 Hz PWM)
* **Operating Voltage:** 5V DC (External power recommended during camera and servo operation)

### Wiring Table

| Component | Pin ESP32-CAM | Function | Notes |
| :--- | :--- | :--- | :--- |
| **SG90 Signal** | `GPIO 13` | Hardware PWM | Configured via `ledcAttach` (Core v3.x) |
| **SG90 VCC** | `5V` (External) | Power | Decoupling cap recommended to prevent brownout |
| **SG90 GND** | `GND` | Common Ground | Must share GND with ESP32 |
| **OV2640 Bus** | `GPIO 0, 5, 18, 19, 21, 22, 23, 25, 26, 27, 32..39` | Video Data & SCCB | Standard AI-Thinker pinout |

---

## 📊 Measured Performance

* **Preprocessing Latency (DSP):** ~10–15 ms
* **Inference Latency (Classification):** ~350–430 ms
* **Total Latency:** **< 500 ms** per frame
* **Rejection Mechanism:** 90° deflection pulse with 350 ms actuation window
* **Total Bill of Materials (BOM):** < 50€

---

## 🚀 Getting Started

### Prerequisites
* Arduino IDE 2.x
* ESP32 Arduino Core (`v3.x`) by Espressif Systems
* Exported Edge Impulse Arduino library: `VisioSort-AI_inferencing`

### Installation & Flashing
1. Clone this repository:
   ```bash
   git clone [https://github.com/](https://github.com/)<your-username>/visiosort-edge-ai.git
   ```
2. In the Arduino IDE, add the library zip:
   * Navigate to `Sketch` -> `Include Library` -> `Add .ZIP Library...`
   * Select `models/ei-visiosort-ai-arduino.zip`.
3. Open `firmware/esp32_camera/esp32_camera.ino`.
4. Select the target configuration:
   * **Board:** `AI Thinker ESP32-CAM`
   * **Partition Scheme:** `Huge APP (3MB No OTA/1MB SPIFFS)`
   * **PSRAM:** `Enabled`
5. Connect your FTDI programmer (`GPIO 0` grounded to flash), verify, and upload.
