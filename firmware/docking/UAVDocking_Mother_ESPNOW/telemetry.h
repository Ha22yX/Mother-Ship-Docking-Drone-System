#pragma once

#include <Arduino.h>

struct TelemetryData {
  uint32_t last_update_ms;
  int32_t lat_e7;
  int32_t lon_e7;
  float alt_m;
  float rel_alt_m;
  float vx_mps;
  float vy_mps;
  float vz_mps;
  float groundspeed_mps;
  float climb_mps;
  float heading_deg;
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float imu_acc_x;
  float imu_acc_y;
  float imu_acc_z;
  float imu_gyro_x;
  float imu_gyro_y;
  float imu_gyro_z;
  float hdop;
  float vdop;
  uint8_t fix_type;
  uint8_t sats;
  uint8_t gps2_fix;
  uint8_t gps2_sats;
  float gps2_hdop;
  float gps2_vdop;
  float battery_v;
  float battery_a;
  float battery_remaining;
  float imu_temp_c;
  uint8_t magnet_on;
};

void telemetryReset(TelemetryData &t);
float telemetryLat(const TelemetryData &t);
float telemetryLon(const TelemetryData &t);

