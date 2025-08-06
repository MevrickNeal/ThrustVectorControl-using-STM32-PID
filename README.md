# ThrustVectorControl-using-STM32-PID
A Thrust Vector Control flight computer based on the STM32 Blue Pill.
Here is the remodeled `README.md` without any emojis, keeping it clean and professional:

---

# STM32 Thrust Vector Control (TVC) Flight Computer

## 1. Overview

This project aims to develop a **Thrust Vector Control (TVC) system** for a model rocket using a custom-built flight computer based on the **STM32F103C8T6 ("Blue Pill")**. The system ensures active stabilization during flight through real-time control, data logging, and telemetry.

### Key Features

* Real-time IMU-based attitude sensing.
* PID-based thrust vector control using dual servo motors.
* Live telemetry downlink via LoRa communication.
* Onboard SD card logging for post-flight analysis.
* Ground station data visualization using Serial Studio.

---

## 2. System Architecture

### Flight Unit

* **Sensors:** MPU6050 (IMU), BME280 (altitude), NEO-6M (GPS).
* **Controller:** STM32 Blue Pill running a PID loop.
* **Actuation:** Two MG90 servos for TVC gimbal movement.
* **Telemetry:** LoRa module transmits data to ground.
* **Logging:** SD card records sensor and flight data.

### Ground Unit

* **Microcontroller:** Arduino Nano.
* **Communication:** LoRa receiver.
* **Visualization:** USB serial output to Serial Studio on PC.

---

## 3. Hardware Components

| Component                 | Role            | Description                                 |
| ------------------------- | --------------- | ------------------------------------------- |
| STM32F103C8T6 (Blue Pill) | Flight Computer | Runs control algorithms and manages sensors |
| MPU6050                   | IMU Sensor      | Measures orientation and acceleration       |
| BME280                    | Altitude Sensor | Measures pressure and temperature           |
| NEO-6M GPS                | GPS Module      | Provides coordinates and UTC time           |
| LoRa-02 Module (x2)       | Telemetry       | Ensures bidirectional data link             |
| MG90 Servos (x2)          | Actuators       | Controls the thrust vector direction        |
| Micro SD Card Module      | Data Logger     | Stores sensor and flight data               |
| Arduino Nano              | Ground Station  | Receives and forwards telemetry             |
| Buzzer & LED              | Indicators      | Flight status signaling                     |

---

## 4. Software & Tools

| Purpose               | Tool/IDE      |
| --------------------- | ------------- |
| Flight Firmware       | STM32CubeIDE  |
| Ground Station Code   | Arduino IDE   |
| PC Visualization      | Serial Studio |
| Programming Interface | ST-Link V2    |

---

## 5. Wiring & Schematics

Schematics and wiring diagrams will be added soon as hardware connections are finalized.

---

## 6. Project Journal

A detailed build log and experimental data will be documented in a separate journal file.

---

## 7. Challenges & Solutions

This section will evolve as technical issues arise and solutions are implemented.
