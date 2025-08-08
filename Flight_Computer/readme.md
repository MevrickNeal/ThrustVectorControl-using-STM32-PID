# Flight Computer

This folder contains the code for the Arduino Mega-based flight computer.

## Overview

The primary role of this system is to read sensor data, calculate the rocket's orientation and altitude, and display this information on a local OLED screen. It performs a self-check on startup and provides audible feedback. This code forms the foundation for the future Thrust Vector Control (TVC) system.

## Key Components
* **Controller:** Arduino Mega 2560
* **Motion Sensor:** MPU6050
* **Altitude Sensor:** BME280
* **Display:** SSD1306 OLED Screen
* **Audible Feedback:** Buzzer
