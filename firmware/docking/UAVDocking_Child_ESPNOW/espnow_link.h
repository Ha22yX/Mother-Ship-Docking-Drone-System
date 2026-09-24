#pragma once

#include <Arduino.h>
#include <esp_now.h>

#include "telemetry.h"

struct TelemetryPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t node_id;
  uint16_t payload_len;
  uint32_t seq;
  uint32_t time_ms;
  TelemetryData data;
} __attribute__((packed));

enum CommandType : uint8_t {
  CMD_NONE = 0,
  CMD_ARM = 1,
  CMD_TAKEOFF_1M = 2,
  CMD_LAND = 3,
  CMD_POSCTL = 4,
  CMD_MAGNET_TOGGLE = 5,
  CMD_FOLLOW_ON = 6,
  CMD_FOLLOW_OFF = 7,
  CMD_FOLLOW_SETPOINT = 8,
};

struct CommandPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t node_id;
  uint16_t payload_len;
  uint32_t seq;
  uint32_t time_ms;
  uint8_t cmd;
  int32_t lat_e7;
  int32_t lon_e7;
  float yaw_deg;
  float rel_alt_m;
} __attribute__((packed));

void espnowSetup(uint8_t self_node_id);
bool espnowSendTelemetry(const TelemetryData &data);
bool espnowGetRemote(TelemetryData &out, uint32_t &age_ms);
bool espnowSendCommand(uint8_t cmd, int32_t lat_e7, int32_t lon_e7, float yaw_deg, float rel_alt_m);
bool espnowGetCommand(uint8_t &cmd, uint32_t &seq, int32_t &lat_e7, int32_t &lon_e7, float &yaw_deg, float &rel_alt_m);
uint32_t espnowGetRxCount();
uint32_t espnowGetTxCount();

