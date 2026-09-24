#include <Arduino.h>

// Requires MAVLink C headers in include path.
// For okalachev/mavlink-arduino library, include MAVLink.h
// https://github.com/okalachev/mavlink-arduino
#include <MAVLink.h>

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

  Serial.println("ESP32-S3 <-> Pixhawk TELEM1 MAVLink parser");
}

void loop() {
  static mavlink_message_t msg;
  static mavlink_status_t status;
  static uint32_t lastStat = 0;
  static uint32_t bytesIn = 0;
  static uint32_t msgsIn = 0;
  static uint32_t stx_fe = 0;
  static uint32_t stx_fd = 0;
  static uint32_t dumpCount = 0;

  while (FC.available()) {
    uint8_t c = FC.read();
    bytesIn++;
    if (c == 0xFE) stx_fe++;
    if (c == 0xFD) stx_fd++;
    if (dumpCount < 64) {
      if (c < 16) Serial.print('0');
      Serial.print(c, HEX);
      Serial.print(' ');
      dumpCount++;
      if (dumpCount == 64) Serial.println();
    }
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      msgsIn++;
      Serial.print("MSG ID=");
      Serial.print(msg.msgid);
      Serial.print(" SYS=");
      Serial.print(msg.sysid);
      Serial.print(" COMP=");
      Serial.print(msg.compid);
      Serial.print(" LEN=");
      Serial.println(msg.len);
    }
  }

  if (millis() - lastStat > 1000) {
    lastStat = millis();
    Serial.print("RX bytes=");
    Serial.print(bytesIn);
    Serial.print(" msgs=");
    Serial.print(msgsIn);
    Serial.print(" stx(0xFE)=");
    Serial.print(stx_fe);
    Serial.print(" stx(0xFD)=");
    Serial.println(stx_fd);
  }
}

