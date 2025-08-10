/*
 =================================================================
 Project:      Project NEAL - STM32 TVC Controller v1
 Author:       [LianMollick]
 Date:         August 10, 2025
 Description:  This is the core control loop for the TVC system.
               It reads sensor data, logs it to an SD card, and
               sends a telemetry packet via LoRa.
               **UPDATE: SD Card is now optional and will not halt the program on failure.**

 WIRING:
  --- MPU6050 & BME280 (I2C) ---
  - VCC -> 3.3V, GND -> GND
  - SDA -> PB7, SCL -> PB6

  --- Servos (CRITICAL: USE EXTERNAL 5V POWER) ---
  - Servo X Signal -> PA0, Servo Y Signal -> PA1
  - Servo Power (+) -> External 5V +
  - Servo Ground (-) -> External 5V - AND Blue Pill GND

  --- LoRa & SD Card (SPI Bus) ---
  - VCC -> 3.3V, GND -> GND
  - SCK -> PA5 (Shared by LoRa and SD)
  - MISO -> PA6 (Shared by LoRa and SD)
  - MOSI -> PA7 (Shared by LoRa and SD)
  
  --- LoRa-02 Module Specific ---
  - NSS -> PA4 (LoRa Chip Select)
  - RST -> PA3
  - DIO0 -> PA2
  
  --- SD Card Reader Specific ---
  - CS -> PB0 (SD Card Chip Select)
 =================================================================
*/

// --- Include necessary libraries ---
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <PID_v1.h>
#include <SPI.h>
#include <LoRa.h>
#include <SD.h>

// --- Self-Contained Kalman Filter Implementation ---
class SimpleKalmanFilter {
  private:
    float Q_angle = 0.001f;
    float Q_bias = 0.003f;
    float R_measure = 0.03f;
    float angle = 0.0f;
    float bias = 0.0f;
    float P[2][2] = {{0,0},{0,0}};

  public:
    SimpleKalmanFilter() {}

    float getAngle(float newAngle, float newRate, float dt) {
      float rate = newRate - bias;
      angle += dt * rate;
      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;
      float S = P[0][0] + R_measure;
      float K[2];
      K[0] = P[0][0] / S;
      K[1] = P[1][0] / S;
      float y = newAngle - angle;
      angle += K[0] * y;
      bias += K[1] * y;
      float P00_temp = P[0][0];
      float P01_temp = P[0][1];
      P[0][0] -= K[0] * P00_temp;
      P[0][1] -= K[0] * P01_temp;
      P[1][0] -= K[1] * P00_temp;
      P[1][1] -= K[1] * P01_temp;
      return angle;
    }

    void setAngle(float newAngle) {
      angle = newAngle;
    }
};

// --- Create objects for our hardware ---
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
Servo servoX;
Servo servoY;
SimpleKalmanFilter kalmanX;
SimpleKalmanFilter kalmanY;
File flightLogFile;

// --- Pin Definitions ---
const int SD_CS_PIN = PB0;

// --- Servo Configuration ---
const int SERVO_X_PIN = PA0;
const int SERVO_Y_PIN = PA1;
const int SERVO_CENTER = 90;
const int SERVO_MIN = 60;
const int SERVO_MAX = 120;
const int PID_OUTPUT_LIMIT = SERVO_CENTER - SERVO_MIN;

// --- PID Controller Setup ---
double Kp = 2.0, Ki = 0.2, Kd = 0.5;
double setpointX = 0, setpointY = 0;
double angleX, angleY;
double correctionX, correctionY;
PID pidX(&angleX, &correctionX, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&angleY, &correctionY, &setpointY, Kp, Ki, Kd, DIRECT);

// --- Calibration & State Variables ---
float angleX_offset = 0;
float angleY_offset = 0;
float vertical_velocity = 0;
bool sd_card_ok = false; // NEW: Flag to track SD card status

// --- Timing Variables ---
unsigned long timer;
unsigned long last_telemetry_send = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("TVC Controller Initializing...");

  if (!mpu.begin()) {
    Serial.println("FATAL: MPU6050 not found!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 OK!");
  
  if (!bme.begin(0x76)) {
    Serial.println("FATAL: BME280 not found!");
    while(1) delay(10);
  }
  Serial.println("BME280 OK!");
  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  LoRa.setPins(PA4, PA3, PA2);
  if (!LoRa.begin(433E6)) {
    Serial.println("FATAL: Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa OK!");

  // --- Initialize SD Card (now optional) ---
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("WARNING: SD Card failed! Continuing without logging.");
    sd_card_ok = false; // Set flag to false
  } else {
    Serial.println("SD Card OK!");
    sd_card_ok = true; // Set flag to true
    // Create log file and write header
    flightLogFile = SD.open("flightlog.csv", FILE_WRITE);
    if (flightLogFile) {
      flightLogFile.println("Roll,Pitch,Altitude,Temp,Pressure,NetAccel,Velocity");
      flightLogFile.close();
      Serial.println("Log file created.");
    } else {
      Serial.println("Error opening flightlog.csv");
    }
  }

  Serial.println("Calibrating MPU6050...");
  delay(1000);
  
  float totalAngleX = 0;
  float totalAngleY = 0;
  for (int i = 0; i < 100; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    totalAngleX += atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    totalAngleY += atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    delay(10);
  }
  angleX_offset = totalAngleX / 100;
  angleY_offset = totalAngleY / 100;
  Serial.print("Calibration complete. X Offset: "); Serial.print(angleX_offset);
  Serial.print(" | Y Offset: "); Serial.println(angleY_offset);

  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  servoX.write(SERVO_CENTER);
  servoY.write(SERVO_CENTER);
  delay(1000);

  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  pidY.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  
  kalmanX.setAngle(angleX_offset);
  kalmanY.setAngle(angleY_offset);
  
  timer = micros();
  Serial.println("Initialization complete. TVC active.");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float dt = (float)(micros() - timer) / 1000000.0;
  timer = micros();

  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float gyroX_rate = g.gyro.x * 180.0 / PI;
  float gyroY_rate = g.gyro.y * 180.0 / PI;

  float absoluteAngleX = kalmanX.getAngle(roll_acc, gyroX_rate, dt);
  float absoluteAngleY = kalmanY.getAngle(pitch_acc, gyroY_rate, dt);

  angleX = absoluteAngleX - angleX_offset;
  angleY = absoluteAngleY - angleY_offset;

  pidX.Compute();
  pidY.Compute();

  int servoX_pos = SERVO_CENTER + correctionX;
  int servoY_pos = SERVO_CENTER + correctionY;

  servoX_pos = constrain(servoX_pos, SERVO_MIN, SERVO_MAX);
  servoY_pos = constrain(servoY_pos, SERVO_MIN, SERVO_MAX);

  servoX.write(servoX_pos);
  servoY.write(servoY_pos);

  // --- Send Telemetry and Log Data ---
  if (millis() - last_telemetry_send > 100) {
    last_telemetry_send = millis();
    
    float altitude = bme.readAltitude(1013.25);
    float temperature = bme.readTemperature();
    float pressure = bme.readPressure() / 100.0F;
    float total_accel = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));
    float net_accel = total_accel - 9.81; 
    vertical_velocity += net_accel * dt;

    String telemetry_packet = "";
    telemetry_packet += String(angleX, 2);
    telemetry_packet += ",";
    telemetry_packet += String(angleY, 2);
    telemetry_packet += ",";
    telemetry_packet += String(altitude, 2);
    telemetry_packet += ",";
    telemetry_packet += String(temperature, 2);
    telemetry_packet += ",";
    telemetry_packet += String(pressure, 2);
    telemetry_packet += ",";
    telemetry_packet += String(net_accel, 2);
    telemetry_packet += ",";
    telemetry_packet += String(vertical_velocity, 2);
    
    // Send the packet via LoRa
    LoRa.beginPacket();
    LoRa.print(telemetry_packet);
    LoRa.endPacket();

    // Write to SD card ONLY if it was initialized correctly
    if (sd_card_ok) {
      flightLogFile = SD.open("flightlog.csv", FILE_WRITE);
      if (flightLogFile) {
        flightLogFile.println(telemetry_packet);
        flightLogFile.close();
      } else {
        Serial.println("Error writing to SD card!");
      }
    }

    Serial.println("TX: " + telemetry_packet);
  }
}
