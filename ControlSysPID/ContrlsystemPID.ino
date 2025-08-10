/*
 =================================================================
 Project:      Project NEAL - STM32 TVC Controller v1
 Author:       [LianMollick]
 Date:         August 10, 2025
 Description:  This is the core control loop for the TVC system.
               It reads orientation from the MPU6050, uses a PID
               controller to calculate corrections, and commands
               two servos within a limited range to stabilize.

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

// --- Create objects for our hardware ---
Adafruit_MPU6050 mpu;
Servo servoX;
Servo servoY;

// --- Servo Configuration (USER DEFINED) ---
const int SERVO_X_PIN = PA0;
const int SERVO_Y_PIN = PA1;
const int SERVO_CENTER = 90; // Your defined center point
const int SERVO_MIN = 60;    // Your defined minimum angle
const int SERVO_MAX = 120;   // Your defined maximum angle
const int PID_OUTPUT_LIMIT = SERVO_CENTER - SERVO_MIN; // Max travel from center = 30 degrees

// --- PID Controller Setup ---
// These are the "tuning knobs" for your TVC. They will need adjustment.
// Kp is for Proportional control (how hard to push back)
// Ki is for Integral control (correcting for long-term drift)
// Kd is for Derivative control (preventing overshoot)
double Kp = 2.0, Ki = 0.2, Kd = 0.5;

// Variables to link the PID controller to our rocket's state
double setpointX = 0, setpointY = 0; // The goal is to stay at 0 degrees (vertical)
double angleX, angleY;               // The current angle from the MPU6050
double correctionX, correctionY;     // The correction value calculated by the PID

// Create two PID controller objects, one for each axis
PID pidX(&angleX, &correctionX, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&angleY, &correctionY, &setpointY, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);
  Serial.println("TVC Controller Initializing...");

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("FATAL: Failed to find MPU6050 chip!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 OK!");

  // Attach servos to their pins
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);

  // Center the servos on startup
  servoX.write(SERVO_CENTER);
  servoY.write(SERVO_CENTER);
  delay(1000); // Wait for servos to center

  // Initialize the PID controllers
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  
  // Set the PID output limits to match the servo's max travel from center
  pidX.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  pidY.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);

  Serial.println("Initialization complete. TVC active.");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- Calculate current angle (this is the PID input) ---
  // NOTE: Depending on how you mounted your sensor, you might need to
  // swap angleX/angleY or multiply one by -1 to get the correct response.
  angleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI; // Roll
  angleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI; // Pitch

  // --- Compute the PID correction ---
  pidX.Compute();
  pidY.Compute();

  // --- Calculate the final servo position ---
  int servoX_pos = SERVO_CENTER + correctionX;
  int servoY_pos = SERVO_CENTER + correctionY;

  // --- IMPORTANT: Constrain the final values to your mount's physical limits ---
  servoX_pos = constrain(servoX_pos, SERVO_MIN, SERVO_MAX);
  servoY_pos = constrain(servoY_pos, SERVO_MIN, SERVO_MAX);

  // --- Command the Servos ---
  servoX.write(servoX_pos);
  servoY.write(servoY_pos);

  // --- Print data for debugging ---
  Serial.print("AngleX: "); Serial.print(angleX, 1);
  Serial.print(" | CorrectionX: "); Serial.print(correctionX, 1);
  Serial.print(" | AngleY: "); Serial.print(angleY, 1);
  Serial.print(" | CorrectionY: "); Serial.println(correctionY, 1);

  delay(20); // Run the control loop 50 times per second
}
