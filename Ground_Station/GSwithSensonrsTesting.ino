#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Button pins (corrected to match your wiring)
#define BTN_SELECT 7   // Green button
#define BTN_UP     8   // Red button
#define BTN_DOWN   9   // Blue button

// Menu variables
const char* menuItems[] = {"Orientation", "Temperature", "Countdown"};
const int menuLength = sizeof(menuItems) / sizeof(menuItems[0]);
int menuIndex = 0;

void setup() {
  Serial.begin(9600);

  // Buttons
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }

  // Intro screen
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
}

void handleButtons() {
  static unsigned long lastPress = 0;
  if (millis() - lastPress < 150) return; // debounce

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
    Serial.print("Selected: ");
    Serial.println(menuItems[menuIndex]);
    lastPress = millis();
  }
}

void handleSerial() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'o') menuIndex = 0;
    else if (cmd == 't') menuIndex = 1;
    else if (cmd == 'l') menuIndex = 2;
    drawMenu();
    Serial.print("Menu set to: ");
    Serial.println(menuItems[menuIndex]);
  }
}

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
