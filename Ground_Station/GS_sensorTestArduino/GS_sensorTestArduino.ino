#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ===== OLED setup =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== DHT Sensor =====
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ===== Button pins =====
#define BTN_SELECT 7   // Green button
#define BTN_UP     8   // Red button
#define BTN_DOWN   9   // Blue button

// ===== Menu =====
const char* menuItems[] = {"Orientation", "Temperature", "Countdown"};
const int menuLength = sizeof(menuItems) / sizeof(menuItems[0]);
int menuIndex = 0;

// ===== States =====
enum Mode { MENU, ORIENTATION, TEMPERATURE, COUNTDOWN };
Mode currentMode = MENU;

// ===== Countdown =====
unsigned long countdownStart = 0;
int countdownTime = 30; // seconds

void setup() {
  Serial.begin(9600);

  // Buttons
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);

  // DHT
  dht.begin();

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }

  // Intro
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 15);
  display.println("Project");
  display.setCursor(35, 40);
  display.println("NEAL");
  display.display();
  delay(3000);

  drawMenu();
}

void loop() {
  handleButtons();
  handleSerial();

  if (currentMode == MENU) {
    // nothing extra
  }
  else if (currentMode == TEMPERATURE) {
    showTemperature();
  }
  else if (currentMode == COUNTDOWN) {
    showCountdown();
  }
}

// ===== BUTTON HANDLING =====
void handleButtons() {
  static unsigned long lastPress = 0;
  if (millis() - lastPress < 150) return; // debounce

  if (currentMode == MENU) {
    if (digitalRead(BTN_UP) == LOW) {
      menuIndex = (menuIndex - 1 + menuLength) % menuLength;
      drawMenu();
      lastPress = millis();
    }
    if (digitalRead(BTN_DOWN) == LOW) {
      menuIndex = (menuIndex + 1) % menuLength;
      drawMenu();
      lastPress = millis();
    }
    if (digitalRead(BTN_SELECT) == LOW) {
      if (menuIndex == 1) currentMode = TEMPERATURE;
      else if (menuIndex == 2) {
        currentMode = COUNTDOWN;
        countdownStart = millis();
      }
      lastPress = millis();
    }
  } else {
    // Return to menu when SELECT pressed
    if (digitalRead(BTN_SELECT) == LOW) {
      currentMode = MENU;
      drawMenu();
      lastPress = millis();
    }
  }
}

// ===== SERIAL HANDLING =====
void handleSerial() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 't') {
      currentMode = TEMPERATURE;
    }
    else if (cmd == 'l') {
      currentMode = COUNTDOWN;
      countdownStart = millis();
    }
    else if (cmd == 'm') {
      currentMode = MENU;
      drawMenu();
    }
  }
}

// ===== MENU DISPLAY =====
void drawMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  for (int i = 0; i < menuLength; i++) {
    if (i == menuIndex) {
      display.fillRect(0, i * 10, 128, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(0, i * 10);
    display.println(menuItems[i]);
  }
  display.display();
}

// ===== TEMPERATURE DISPLAY =====
void showTemperature() {
  float temp = dht.readTemperature();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.print("Temp: ");
  if (isnan(temp)) {
    display.print("--");
  } else {
    display.print(temp, 1);
    display.write(247); // degree symbol
    display.print("C");
  }
  display.display();
}

// ===== COUNTDOWN DISPLAY =====
void showCountdown() {
  int secondsLeft = countdownTime - (millis() - countdownStart) / 1000;
  if (secondsLeft < 0) secondsLeft = 0;

  display.clearDisplay();
  display.setTextSize(4);
  display.setTextColor(SSD1306_WHITE);
  int x = (SCREEN_WIDTH - (secondsLeft < 10 ? 24 : 48)) / 2; // center text
  display.setCursor(x, 20);
  display.print(secondsLeft);
  display.display();

  if (secondsLeft == 0) {
    // Auto return to menu after countdown ends
    delay(1000);
    currentMode = MENU;
    drawMenu();
  }
}
