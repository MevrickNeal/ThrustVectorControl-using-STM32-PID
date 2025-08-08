#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

// Create sensor objects
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme; 

void setup() {
  Serial.begin(115200);
  Serial.println("Flight Computer Sensor Test...");

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Check wiring!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  // Initialize BME280
  if (!bme.begin(0x76)) { // I2C address is usually 0x77 or 0x76
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1) delay(10);
  }
  Serial.println("BME280 Found!");
  Serial.println("------------------------------------");
}

void loop() {
  // Get MPU6050 data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- Print Angles ---
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  
  Serial.print("Roll: ");
  Serial.print(roll, 1);
  Serial.print(" deg, Pitch: ");
  Serial.print(pitch, 1);
  Serial.println(" deg");

  // --- Print BME280 Data ---
  Serial.print("Altitude: ");
  Serial.print(bme.readAltitude(1013.25)); // 1013.25 is standard sea level pressure
  Serial.print(" m, Temp: ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  Serial.println("------------------------------------");
  delay(1000); 
}
