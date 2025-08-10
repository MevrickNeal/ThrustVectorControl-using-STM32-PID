/*
 =================================================================
 Project:      Project NEAL - STM32 TVC Controller v1
 Author:       [LianMollick]
 Date:         August 10, 2025
 Description:  This is the core control loop for the TVC system.
               It reads orientation from the MPU6050, uses a PID
               controller to calculate corrections, and commands
               two servos within a limited range to stabilize.
               **UPDATE: Added startup calibration to set a 'zero' reference point.**

 WIRING:
  --- MPU6050 ---
  - VCC -> 3.3V on Blue Pill
  - GND -> GND on Blue Pill
  - SDA -> PB7 on Blue Pill
  - SCL -> PB6 on Blue Pill

  --- Servos (CRITICAL: USE EXTERNAL 5V POWER) ---
  - Servo X Signal -> PA0 on Blue Pill
  - Servo Y Signal -> PA1 on Blue Pill
  - Servo Power (+) -> Positive terminal of a separate 5V battery pack
  - Servo Ground (-) -> Negative terminal of battery pack AND a GND pin on the Blue Pill
 =================================================================
*/

// --- Include necessary libraries ---
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <PID_v1.h>

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
Servo servoX;
Servo servoY;
SimpleKalmanFilter kalmanX;
SimpleKalmanFilter kalmanY;

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

// --- NEW: Calibration Variables ---
float angleX_offset = 0;
float angleY_offset = 0;

// --- Filter Timing Variables ---
unsigned long timer;

void setup() {
  Serial.begin(115200);
  Serial.println("TVC Controller Initializing...");

  if (!mpu.begin()) {
    Serial.println("FATAL: MPU6050 not found!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 OK!");
  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // --- NEW: Startup Calibration Routine ---
  Serial.println("Calibrating MPU6050... Do not move the rocket.");
  delay(1000); // Wait a second
  
  float totalAngleX = 0;
  float totalAngleY = 0;
  int calibration_reads = 100;

  for (int i = 0; i < calibration_reads; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    totalAngleX += atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    totalAngleY += atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    delay(10);
  }
  
  angleX_offset = totalAngleX / calibration_reads;
  angleY_offset = totalAngleY / calibration_reads;

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
  
  // Initialize the Kalman Filters with the calibrated starting angle
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

  // Get the absolute filtered angle from the Kalman filter
  float absoluteAngleX = kalmanX.getAngle(roll_acc, gyroX_rate, dt);
  float absoluteAngleY = kalmanY.getAngle(pitch_acc, gyroY_rate, dt);

  // --- NEW: Apply the startup calibration offset to get the relative angle ---
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

  // Print the RELATIVE angle
  Serial.print("AngleX: "); Serial.print(angleX, 1);
  Serial.print(" | CorrectionX: "); Serial.print(correctionX, 1);
  Serial.print(" | AngleY: "); Serial.print(angleY, 1);
  Serial.print(" | CorrectionY: "); Serial.println(correctionY, 1);
}
