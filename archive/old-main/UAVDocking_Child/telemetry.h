#pragma once

#include <Arduino.h>

struct Telemetry {
  bool valid = false;
  uint32_t lastMs = 0;
  double lat = 0;
  double lon = 0;
  float altAmsl = 0;
  float relAlt = 0;
  float vx = 0;
  float vy = 0;
  float vz = 0;
  float ax = 0;
  float ay = 0;
  float az = 0;
  float vel = 0;
  float cog = 0;
  float eph = 0;
  float epv = 0;
  float airspeed = 0;
  float groundspeed = 0;
  float climb = 0;
  int16_t heading = 0;
  uint16_t throttle = 0;
  float battVolt = 0;
  float battCurrent = 0;
  int8_t battRemaining = -1;
  float imuTemp = 0;
  float yawRad = 0;
  uint8_t fixType = 0;
  uint8_t sats = 0;
  uint8_t gps2Fix = 0;
  uint8_t gps2Sats = 0;
  float gps2Eph = 0;
  float gps2Epv = 0;
};

extern Telemetry self;

