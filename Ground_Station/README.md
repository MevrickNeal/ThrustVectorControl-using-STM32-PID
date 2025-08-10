
# Project NEAL – Ground Station

**Project NEAL Ground Station** is an Arduino-based mission control unit designed to monitor and control the STM32 TVC flight computer during rocket flights.
It provides real-time telemetry display, launch countdown capability, and optional data forwarding to a computer for visualization.

---

## 1. Project Overview

The ground station performs the following functions:

* **Receiving Telemetry**

  * Listens for LoRa packets sent from the flight computer.

* **Live Data Display**

  * Parses incoming telemetry and shows key flight parameters (attitude, altitude, velocity, etc.) on a local SSD1306 OLED.

* **Launch Control**

  * Can initiate a 10-second launch countdown via a serial command.

* **Data Forwarding**

  * Forwards all raw telemetry packets to the connected PC over serial for use in tools like *Serial Studio*.

* **System Diagnostics**

  * Runs a startup self-check to verify OLED presence and provide clear status feedback.

---

## 2. Hardware Components

| Component             | Role               | Notes                                    |
| --------------------- | ------------------ | ---------------------------------------- |
| Arduino Nano / Uno    | Main controller    | Handles data reception and display logic |
| LoRa-02 Module        | Telemetry receiver | 433 MHz communication                    |
| SSD1306 OLED (128×64) | Display module     | I²C interface for real-time flight data  |

---

## 3. Wiring Diagram (Arduino Nano/Uno)

### LoRa-02 Module (SPI)

| LoRa Pin | Arduino Pin | Purpose     |
| -------- | ----------- | ----------- |
| VCC      | 5V          | Power       |
| GND      | GND         | Ground      |
| SCK      | D13         | SPI Clock   |
| MISO     | D12         | Master In   |
| MOSI     | D11         | Master Out  |
| NSS      | D10         | Chip Select |
| RST      | D9          | Reset       |
| DIO0     | D2          | Interrupt   |

### SSD1306 OLED (I²C)

| OLED Pin | Arduino Pin | Purpose   |
| -------- | ----------- | --------- |
| VCC      | 5V          | Power     |
| GND      | GND         | Ground    |
| SDA      | A4          | I²C Data  |
| SCL      | A5          | I²C Clock |

---

## 4. Functionality

### Normal Operation

* On startup, displays the "Project NEAL" intro screen.
* Enters listening mode showing "Awaiting Telemetry" until a valid packet arrives.
* Parses LoRa telemetry and updates the OLED in real-time.
* Prints all incoming packets to the serial monitor at **115200 baud**.

### Countdown Mode

* Triggered by sending **`l`** or **`L`** via serial.
* Displays a large-font **10-second countdown timer**.
* Pauses telemetry reception during countdown.
* Returns to normal listening mode after countdown completion.

---

## 5. Software Dependencies

Install the following Arduino IDE libraries via Library Manager:

* **LoRa** by Sandeep Mistry
* **Adafruit GFX Library**
* **Adafruit SSD1306**

---

## 6. Telemetry Data Format

The ground station expects a comma-separated data string from the flight computer in the format:

```
roll,pitch,altitude,temp,pressure,net_accel,velocity
```

**Example:**

```
-5.72,10.15,123.45,28.50,1004.20,10.50,5.21
```



https://wokwi.com/projects/438572296306581505
for simulation learning
