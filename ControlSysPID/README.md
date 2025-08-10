Project NEAL - STM32 TVC Flight Computer
This document details the hardware, software, and functionality of the STM32-based Thrust Vector Control (TVC) flight computer.

1. Project Overview
The primary goal of this system is to serve as the "brain" for a model rocket with active thrust vector control. It is a complete, self-contained unit responsible for:

Sensing: Reading the rocket's orientation (Roll & Pitch) and atmospheric data (Altitude, Pressure, Temperature).

Stabilizing: Calculating the necessary adjustments to maintain a stable, vertical flight path using a PID control loop.

Actuating: Commanding two servos connected to a TVC mount to correct any deviations from the desired flight path.

Transmitting: Sending a continuous stream of flight data (telemetry) to a ground station via a LoRa radio module.

Logging: Recording all flight data to an onboard SD card to act as a "black box" flight recorder.

2. System Status
This section outlines the current implementation status of all hardware components.

Actively Used Components:
STM32 Blue Pill (F103C8): The main processor running the control loop.

MPU6050: Provides raw acceleration and gyroscope data, which is fused by a Kalman filter to get stable Roll and Pitch angles.

BME280: Provides altitude, temperature, and pressure readings.

MG90 Servos (x2): Actively controlled by the PID loop to adjust the TVC mount.

LoRa-02 Module: Actively transmits a full telemetry packet 10 times per second.

Bypassed / Optional Components:
SD Card Reader: The code includes full support for the SD card reader. However, the system is designed to be fault-tolerant. If an SD card is not detected at startup, the system will print a warning and continue to function perfectly without data logging. This allows for testing without requiring a card to be present.

GPS Module (NEO-6M): This component was part of the original plan but was lost. The current code does not include any GPS functionality. It can be added back in the future if a new module is acquired.

3. Hardware & Pin Diagram
All components are wired to the STM32 Blue Pill as follows.

I2C Bus (Sensors)
Pins: PB7 (SDA), PB6 (SCL)

Devices: MPU6050, BME280

Sensor

VCC Pin

GND Pin

SDA Pin

SCL Pin

MPU6050

3.3V

GND

PB7

PB6

BME280

3.3V

GND

PB7

PB6

SPI Bus (Communications & Storage)
Pins: PA5 (SCK), PA6 (MISO), PA7 (MOSI)

Devices: LoRa Module, SD Card Reader

Module

VCC Pin

GND Pin

SCK

MISO

MOSI

CS/NSS

LoRa-02

3.3V

GND

PA5

PA6

PA7

PA4

SD Reader

3.3V

GND

PA5

PA6

PA7

PB0

Individual Pins
Component

Pin

Purpose

LoRa Reset

PA3

Reset Pin

LoRa Interrupt

PA2

Interrupt Pin (DIO0)

Servo X

PA0

PWM Signal for X-axis

Servo Y

PA1

PWM Signal for Y-axis

CRITICAL: Power Connections
The STM32, MPU6050, BME280, and LoRa module are all powered from the 3.3V output of the ST-Link programmer.

The two MG90 servos are powered by a separate, external 5V power source (e.g., a 4xAA battery pack). The ground of this external source must be connected to a GND pin on the Blue Pill to create a common ground.

4. Software Dependencies
To compile this code, the following libraries must be installed in the Arduino IDE via the Library Manager:

Adafruit MPU6050

Adafruit BME280 Library

Adafruit Unified Sensor (Installed as a dependency)

PID by Brett Beauregard

LoRa by Sandeep Mistry

SD (Usually included with the STM32 board package)
