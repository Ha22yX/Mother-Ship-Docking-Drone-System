#include <Arduino.h>
#include <HardwareSerial.h>

// ESP32-S3 <-> OpenMV UART (SoftwareSerial)
// IO6 -> OpenMV P5 (RX)
// IO7 -> OpenMV P4 (TX)
#define OPENMV_RX 6   // ESP32 receives OpenMV TX
#define OPENMV_TX 7   // ESP32 sends to OpenMV RX

HardwareSerial OpenMVSerial(1);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("ESP32-S3 OpenMV UART Test Start");

  OpenMVSerial.begin(19200, SERIAL_8N1, OPENMV_RX, OPENMV_TX);
  Serial.println("UART1 initialized. Waiting for OpenMV data...");
}

void loop() {
  while (OpenMVSerial.available()) {
    char c = OpenMVSerial.read();
    if (c >= 32 && c <= 126) {
      Serial.write(c);
    } else if (c == '\n' || c == '\r') {
      Serial.write(c);
    }
  }
}

