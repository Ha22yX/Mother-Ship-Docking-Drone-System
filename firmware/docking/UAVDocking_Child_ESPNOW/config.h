#pragma once

#include <Arduino.h>

// Pixhawk TELEM2 (115200 8N1)
static const int PIN_RX = 10; // Pixhawk TX -> ESP32 RX
static const int PIN_TX = 11; // Pixhawk RX -> ESP32 TX
static const uint32_t FC_BAUD = 115200;

// ESP-NOW
static const uint8_t WIFI_CHANNEL = 1;
static const bool ESPNOW_USE_BROADCAST = true;
static const uint8_t ESPNOW_PEER_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint32_t ESPNOW_SEND_INTERVAL_MS = 200;

// Node IDs
static const uint8_t NODE_ID_MOTHER = 1;
static const uint8_t NODE_ID_CHILD = 2;

