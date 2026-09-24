#pragma once

#include <Arduino.h>

// Pixhawk TELEM1:
// Pin 2 (TX) -> ESP32-S3 IO10 (RX)
// Pin 3 (RX) -> ESP32-S3 IO11 (TX)
static const int PIN_RX = 10;
static const int PIN_TX = 11;
static const uint32_t FC_BAUD = 115200;

static const char *AP_SSID __attribute__((unused)) = "UAVDocking";
static const char *AP_PASS __attribute__((unused)) = "UAVDocking";
static const char *MOTHER_HOST __attribute__((unused)) = "http://192.168.1.1";

// MAVLink IDs
static const uint8_t SYS_ID = 255;   // GCS ID
static const uint8_t COMP_ID = 190;  // MAV_COMP_ID_GCS
static const uint8_t TARGET_SYS = 1;
static const uint8_t TARGET_COMP = 1;

static const uint16_t SETPOINT_TYPE_MASK = 0x0DF8; // position only
static const uint16_t SETPOINT_TYPE_MASK_POS_YAW = 0x09F8; // position + yaw

// RTK (RTCM) UDP port
static const uint16_t RTCM_UDP_PORT = 14550;

