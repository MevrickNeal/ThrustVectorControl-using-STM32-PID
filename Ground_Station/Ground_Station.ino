// =================================================================
//                    LIBRARIES & DEFINITIONS
// =================================================================

// Include necessary libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// --- Pin and Sensor Definitions ---
#define LED_PIN 6        // Status LED connected to digital pin 6
#define DHTPIN 2         // DHT22 data pin connected to digital pin 2
#define DHTTYPE DHT22    // We are using the DHT22 sensor

// --- OLED Display Definitions ---
#define SCREEN_I2C_ADDR 0x3C // Use 0x3C or 0x3D depending on your display
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RST_PIN -1      // Reset pin (-1 if not available)

// --- Create objects for our hardware ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST_PIN);
DHT dht(DHTPIN, DHTTYPE);

// =================================================================
//                        GLOBAL VARIABLES
// =================================================================

// --- State Management ---
bool dht_status_ok = false;      // Tracks if the DHT sensor initialized correctly
bool countdown_active = false;   // Is the countdown currently running?
bool countdown_finished = false; // Has the countdown completed?

// --- Timer Configuration ---
unsigned long countdown_start_time = 0;
const int countdown_duration_seconds = 30; // 30-second countdown

// =================================================================
//                         SETUP FUNCTION
// =================================================================
void setup()
{
  // Start serial communication for debugging purposes
  Serial.begin(9600);
  Serial.println("\nProject NEAL Initializing...");

  // Set up the LED pin
  pinMode(LED_PIN, OUTPUT);

  // Initialize the DHT sensor
  dht.begin();
  delay(2000); // A small delay to allow sensor to stabilize
  
  // Check if the sensor is working by taking a reading.
  if (isnan(dht.readTemperature()))
  {
    Serial.println("ERROR: Failed to read from DHT sensor!");
    dht_status_ok = false;
  }
  else
  {
    Serial.println("SUCCESS: DHT sensor is OK!");
    dht_status_ok = true;
  }

  // Initialize the SSD1306 display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR))
  {
    Serial.println(F("FATAL: SSD1306 allocation failed. Halting."));
    while (true); // Don't proceed, loop forever
  }

  // --- Show Project NEAL Intro Screen ---
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 15); // Centered text
  display.println("Project");
  display.setCursor(35, 40); // Centered text
  display.println("NEAL");
  display.display();
  Serial.println("Intro displayed. Waiting 3 seconds...");
  delay(3000);
}

// =================================================================
//                          MAIN LOOP
// =================================================================
void loop()
{
  // --- Step 1: Check for Serial Commands ---
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'l' || input == 'L') {
      Serial.println("COMMAND: 'l' received. Starting countdown...");
      countdown_active = true;
      countdown_finished = false; // Reset finished state
      countdown_start_time = millis();
    }
  }

  // --- Step 2: Handle Countdown Finished State ---
  if (countdown_finished) {
    // This state is reached after the countdown hits zero.
    // The screen will flash and then freeze with the final message.
    
    // Flash the message 3 times
    for (int i = 0; i < 3; i++) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(5, 20);
      display.println("To the Infinity");
      display.setCursor(25, 35);
      display.println("and BEYOND!");
      display.display();
      delay(400);
      display.clearDisplay();
      display.display();
      delay(200);
    }

    // Display the final, solid message
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(5, 20);
    display.println("To the Infinity");
    display.setCursor(25, 35);
    display.println("and BEYOND!");
    display.display();
    
    Serial.println("Countdown finished. System halted. Please reset.");
    while(true); // Halt the program until the device is reset
  }
  
  // --- Step 3: Handle Active Countdown State ---
  else if (countdown_active) {
    unsigned long elapsed_ms = millis() - countdown_start_time;
    int remaining_seconds = countdown_duration_seconds - (elapsed_ms / 1000);

    if (remaining_seconds >= 0) {
      // While countdown is running, display the remaining time
      display.clearDisplay();
      display.setTextSize(4); // Use a large font for the number
      display.setTextColor(SSD1306_WHITE);
      
      // Adjust cursor x-position to keep the number centered
      if (remaining_seconds < 10) {
        display.setCursor(52, 18); // Centered for single digit
      } else {
        display.setCursor(32, 18); // Centered for double digits
      }
      
      display.print(remaining_seconds);
      display.display();
      delay(50); // Short delay for smooth countdown display
      
    } else {
      // Countdown has just hit zero
      countdown_active = false;
      countdown_finished = true;
    }
  }

  // --- Step 4: Handle Normal Operation State ---
  else {
    // This is the default screen when no countdown is active.
    
    // Handle Status LED based on DHT health
    if (dht_status_ok) {
      digitalWrite(LED_PIN, HIGH); // Solid ON if OK
    } else {
      digitalWrite(LED_PIN, millis() % 1000 < 500 ? HIGH : LOW); // Blink if error
    }

    // Read Data from DHT sensor
    float temperature = dht.readTemperature();
    if (isnan(temperature)) {
      // This handles cases where the sensor fails after a successful startup
      temperature = -99.9; 
    }
    
    // Calculate system uptime
    unsigned long uptime_seconds = millis() / 1000;

    // Update Display with Uptime and Temperature
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

    // Display Uptime
    display.setCursor(0, 5);
    display.print(F("Uptime"));
    display.setCursor(0, 25);
    display.print(uptime_seconds);
    display.print(F("s"));

    // Display Temperature
    display.setCursor(0, 50);
    display.print(temperature, 1);
    display.print((char)247); // Degree symbol °
    display.print(F("C"));

    display.display();

    // IMPORTANT: Wait 2 seconds before the next sensor read.
    // Reading the DHT sensor too quickly causes errors.
    delay(2000);
  }
}
