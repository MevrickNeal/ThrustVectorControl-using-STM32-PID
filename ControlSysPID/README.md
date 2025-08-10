
# Project NEAL – STM32 TVC Flight Computer
<img width="1000" height="389" alt="PJNtvc" src="https://github.com/user-attachments/assets/8b4568ba-5474-4956-b44a-8632b4037b5c" />

**Project NEAL** is an STM32-based thrust vector control (TVC) flight computer designed for small-scale rocketry.
The system integrates sensing, control algorithms, actuation, telemetry, and optional data logging into one compact and reliable unit.

---

## 1. Project Overview

The TVC flight computer performs the following functions:

* **Sensing**

  * Measures rocket orientation (Roll & Pitch) using an IMU.
  * Reads altitude, pressure, and temperature from a barometric sensor.

* **Stabilizing**

  * Runs a PID control loop to maintain vertical stability.

* **Actuating**

  * Commands two MG90 servos on a TVC gimbal to correct deviations.

* **Transmitting**

  * Sends live telemetry to a ground station via LoRa at 10 Hz.

* **Logging**

  * Records flight data to an SD card (if installed) for post-flight analysis.

---

## 2. System Status

### Actively Used Components

| Component                | Description                                  |
| ------------------------ | -------------------------------------------- |
| STM32 Blue Pill (F103C8) | Main processor and control loop              |
| MPU6050                  | IMU providing Roll & Pitch via Kalman filter |
| BME280                   | Altitude, temperature, and pressure sensor   |
| MG90 Servos (x2)         | TVC gimbal actuation                         |
| LoRa-02 Module           | Telemetry transmission                       |

### Optional / Not Installed

| Component         | Description                                       |
| ----------------- | ------------------------------------------------- |
| SD Card Reader    | Fully supported in code; bypassed if not detected |
| NEO-6M GPS Module | Removed due to loss; code excludes GPS functions  |

---

## 3. Hardware and Pin Mapping

### I²C Bus – Sensors

| Device  | VCC  | GND | SDA | SCL |
| ------- | ---- | --- | --- | --- |
| MPU6050 | 3.3V | GND | PB7 | PB6 |
| BME280  | 3.3V | GND | PB7 | PB6 |

### SPI Bus – Communications and Storage

| Device    | VCC  | GND | SCK (PA5) | MISO (PA6) | MOSI (PA7) | CS/NSS |
| --------- | ---- | --- | --------- | ---------- | ---------- | ------ |
| LoRa-02   | 3.3V | GND | PA5       | PA6        | PA7        | PA4    |
| SD Reader | 3.3V | GND | PA5       | PA6        | PA7        | PB0    |

### Individual Pins

| Component             | Pin | Purpose                |
| --------------------- | --- | ---------------------- |
| LoRa Reset            | PA3 | Reset signal           |
| LoRa Interrupt (DIO0) | PA2 | Interrupt signal       |
| Servo X               | PA0 | PWM control for X-axis |
| Servo Y               | PA1 | PWM control for Y-axis |

**Power Notes:**

* STM32, MPU6050, BME280, and LoRa module are powered from the STM32’s 3.3V output.
* MG90 servos are powered from an external 5V source (e.g., 4×AA battery pack).
* A common ground must be shared between the STM32 and the servo power source.

---

## 4. Software Dependencies

The following Arduino IDE libraries are required:

* Adafruit MPU6050
* Adafruit BME280 Library
* Adafruit Unified Sensor (dependency)
* PID by Brett Beauregard
* LoRa by Sandeep Mistry
* SD (bundled with STM32 board package)

---

## 5. Future Development

* Reintroduce GPS tracking capability.
* Optimize telemetry packet structure for efficiency.
* Implement flight mode switching for autonomous testing.


