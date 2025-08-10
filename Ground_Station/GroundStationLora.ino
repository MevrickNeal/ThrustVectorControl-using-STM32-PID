/*
 =================================================================
 Project:      Project NEAL - Ground Station (Optimized)
 Author:       [LianMollick]
 Date:         August 11, 2025
 Description:  This optimized code runs on an Arduino Nano/Uno.
               It includes a self-diagnosing I2C scanner to automatically
               detect the OLED display's address and will not halt if
               the display fails, allowing other systems to be tested.

 WIRING (Arduino Nano/Uno):
  --- LoRa-02 Module (SPI) ---
  - VCC -> 5V, GND -> GND
  - SCK -> D13, MISO -> D12, MOSI -> D11
  - NSS -> D10, RST -> D9, DIO0 -> D2

  --- SSD1306 OLED (I2C) ---
  - VCC -> 5V, GND -> GND
  - SDA -> A4, SCL -> A5
 =================================================================
*/

// --- Include necessary libraries ---
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED Display Definitions ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
// NOTE: We will auto-detect the address, so this is just a default.
#define SCREEN_I2C_ADDR 0x3C 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Global Variables ---
bool countdown_active = false;
unsigned long countdown_start_time = 0;
const int countdown_duration_seconds = 10;

float roll = 0.0, pitch = 0.0, altitude = 0.0, temp = 0.0;
float pressure = 0.0, accel = 0.0, velocity = 0.0;
bool packet_received = false;
bool display_ok = false; // NEW: Flag to track if display is working

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nProject NEAL Ground Station Initializing...");

  // --- NEW: I2C Bus Scanner and Auto-Detection ---
  Wire.begin();
  Serial.println("Scanning I2C bus...");
  byte found_address = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C Device Found at address: 0x");
      Serial.println(address, HEX);
      if (address == 0x3C || address == 0x3D) {
        found_address = address;
      }
    }
  }

  // --- Initialize Display with Auto-Detection ---
  if (found_address != 0) {
    Serial.print("Attempting to initialize display at found address 0x");
    Serial.println(found_address, HEX);
    if (display.begin(SSD1306_SWITCHCAPVCC, found_address)) {
      display_ok = true;
      Serial.println("SUCCESS: SSD1306 display is OK!");
    }
  } 
  
  if (!display_ok) {
    // Fallback if scanner fails or finds nothing
    Serial.println("Scanner found no display. Trying default addresses...");
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C) || display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
        display_ok = true;
        Serial.println("SUCCESS: SSD1306 display is OK!");
    } else {
        Serial.println("*************************************************");
        Serial.println("FATAL: SSD1306 allocation failed on all addresses.");
        Serial.println("Check VCC, GND, SDA, and SCL wiring!");
        Serial.println("Continuing without display...");
        Serial.println("*************************************************");
    }
  }

  // --- Show Project NEAL Intro Screen (only if display works) ---
  if (display_ok) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 15);
    display.println("Project");
    display.setCursor(35, 40);
    display.println("NEAL");
    display.display();
    delay(3000);
  }

  // --- Initialize LoRa Module ---
  LoRa.setPins(10, 9, 2); // NSS, RST, DIO0
  if (!LoRa.begin(433E6)) {
    Serial.println("FATAL: Starting LoRa failed!");
    if (display_ok) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("LoRa Init Failed!");
      display.display();
    }
    while (true);
  }
  Serial.println("Ground Station Active. Waiting for Telemetry...");
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    if ((input == 'l' || input == 'L') && !countdown_active) {
      Serial.println("COMMAND: 'l' received. Starting 10s countdown...");
      countdown_active = true;
      countdown_start_time = millis();
    }
  }

  if (countdown_active) {
    handleCountdown();
  } else {
    handleLoRa();
  }
}

void handleCountdown() {
  unsigned long elapsed_ms = millis() - countdown_start_time;
  int remaining_seconds = countdown_duration_seconds - (elapsed_ms / 1000);

  if (remaining_seconds >= 0) {
    if (display_ok) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.print("T-");
      display.setTextSize(4);
      if (remaining_seconds < 10) display.setCursor(52, 18);
      else display.setCursor(32, 18);
      display.print(remaining_seconds);
      display.display();
    }
    delay(50);
  } else {
    countdown_active = false;
    Serial.println("Countdown finished!");
  }
}

void handleLoRa() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    packet_received = true;
    String received_string = "";
    while (LoRa.available()) {
      received_string += (char)LoRa.read();
    }
    
    Serial.println(received_string); // Forward to PC
    
    sscanf(received_string.c_str(), "%f,%f,%f,%f,%f,%f,%f", 
           &roll, &pitch, &altitude, &temp, &pressure, &accel, &velocity);

    if (display_ok) {
      displayDataOnOLED();
    }
  } else {
    if (!packet_received && display_ok) {
      displayWaitingScreen();
    }
  }
}

void displayDataOnOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.print("R:"); display.print(roll, 1);
  display.setCursor(64, 0);
  display.print(" P:"); display.print(pitch, 1);

  display.setCursor(0, 12);
  display.print("Alt: "); display.print(altitude, 1); display.print("m");

  display.setCursor(0, 24);
  display.print("Vel: "); display.print(velocity, 1); display.print("m/s");

  display.setCursor(0, 36);
  display.print("Acc: "); display.print(accel, 1); display.print("m/s2");
  
  display.setCursor(0, 48);
  display.print("T:"); display.print(temp, 1); display.print("C");
  display.setCursor(64, 48);
  display.print(" P:"); display.print(pressure, 0);

  display.display();
}

void displayWaitingScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 15);
  display.println(" Awaiting Telemetry");
  display.setCursor(0, 35);
  display.println(" from Flight Computer");
  display.display();
  delay(500);
}
