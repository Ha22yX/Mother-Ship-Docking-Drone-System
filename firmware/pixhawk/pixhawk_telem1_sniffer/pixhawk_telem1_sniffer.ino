#include <Arduino.h>

// Pixhawk 6C TELEM1:
// Pin 2 (TX) -> ESP32-S3 IO10 (RX)
// Pin 3 (RX) -> ESP32-S3 IO11 (TX)
static const int PIN_RX = 10;
static const int PIN_TX = 11;

HardwareSerial FC(1);

void setup() {
  Serial.begin(115200);
  delay(500);

  FC.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);

  Serial.println("ESP32-S3 <-> Pixhawk TELEM1 sniff test");
  Serial.println("Expect hex bytes if wiring and baud are correct.");
}

void loop() {
  while (FC.available()) {
    uint8_t b = FC.read();
    if (b < 16) Serial.print('0');
    Serial.print(b, HEX);
    Serial.print(' ');
  }

  static uint32_t t = 0;
  if (millis() - t > 200) {
    t = millis();
    Serial.println();
  }
}

