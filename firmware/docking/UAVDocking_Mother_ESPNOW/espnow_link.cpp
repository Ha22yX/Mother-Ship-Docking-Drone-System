#include "espnow_link.h"

#include <WiFi.h>
#include <esp_wifi.h>

#include "config.h"

static TelemetryData s_remote;
static uint32_t s_remote_rx_ms = 0;
static uint32_t s_rx_count = 0;
static uint32_t s_tx_count = 0;
static uint32_t s_seq = 0;
static uint32_t s_cmd_seq = 0;
static uint8_t s_self_node = 0;
static uint8_t s_last_cmd = CMD_NONE;
static uint32_t s_last_cmd_seq = 0;
static int32_t s_last_cmd_lat = 0;
static int32_t s_last_cmd_lon = 0;
static float s_last_cmd_yaw = 0.0f;
static float s_last_cmd_alt = 0.0f;

static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  if (status == ESP_NOW_SEND_SUCCESS) {
    s_tx_count++;
  }
}

static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  (void)info;
  if (len == (int)sizeof(TelemetryPacket)) {
    const TelemetryPacket *pkt = reinterpret_cast<const TelemetryPacket *>(incomingData);
    if (pkt->magic != 0x55445731 || pkt->version != 1) {
      return;
    }
    if (pkt->node_id == s_self_node) {
      return;
    }
    s_remote = pkt->data;
    s_remote_rx_ms = millis();
    s_rx_count++;
    return;
  }
  if (len == (int)sizeof(CommandPacket)) {
    const CommandPacket *pkt = reinterpret_cast<const CommandPacket *>(incomingData);
    if (pkt->magic != 0x55445743 || pkt->version != 1) {
      return;
    }
    if (pkt->node_id == s_self_node) {
      return;
    }
    s_last_cmd = pkt->cmd;
    s_last_cmd_seq = pkt->seq;
    s_last_cmd_lat = pkt->lat_e7;
    s_last_cmd_lon = pkt->lon_e7;
    s_last_cmd_yaw = pkt->yaw_deg;
    s_last_cmd_alt = pkt->rel_alt_m;
    s_rx_count++;
  }
}

void espnowSetup(uint8_t self_node_id) {
  s_self_node = self_node_id;
  telemetryReset(s_remote);
  s_remote_rx_ms = 0;

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init failed");
    return;
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, ESPNOW_PEER_MAC, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESPNOW] add peer failed");
  } else {
    Serial.println("[ESPNOW] peer added");
  }
}

bool espnowSendTelemetry(const TelemetryData &data) {
  TelemetryPacket pkt = {};
  pkt.magic = 0x55445731; // "UDW1"
  pkt.version = 1;
  pkt.node_id = s_self_node;
  pkt.payload_len = sizeof(TelemetryData);
  pkt.seq = ++s_seq;
  pkt.time_ms = millis();
  pkt.data = data;
  return esp_now_send(ESPNOW_PEER_MAC, reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt)) == ESP_OK;
}

bool espnowSendCommand(uint8_t cmd, int32_t lat_e7, int32_t lon_e7, float yaw_deg, float rel_alt_m) {
  CommandPacket pkt = {};
  pkt.magic = 0x55445743; // "UDWC"
  pkt.version = 1;
  pkt.node_id = s_self_node;
  pkt.payload_len = sizeof(pkt.cmd) + sizeof(pkt.lat_e7) + sizeof(pkt.lon_e7) + sizeof(pkt.yaw_deg) + sizeof(pkt.rel_alt_m);
  pkt.seq = ++s_cmd_seq;
  pkt.time_ms = millis();
  pkt.cmd = cmd;
  pkt.lat_e7 = lat_e7;
  pkt.lon_e7 = lon_e7;
  pkt.yaw_deg = yaw_deg;
  pkt.rel_alt_m = rel_alt_m;
  return esp_now_send(ESPNOW_PEER_MAC, reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt)) == ESP_OK;
}

bool espnowGetRemote(TelemetryData &out, uint32_t &age_ms) {
  if (s_remote_rx_ms == 0) {
    return false;
  }
  out = s_remote;
  age_ms = millis() - s_remote_rx_ms;
  return true;
}

bool espnowGetCommand(uint8_t &cmd, uint32_t &seq, int32_t &lat_e7, int32_t &lon_e7, float &yaw_deg, float &rel_alt_m) {
  if (s_last_cmd_seq == 0) {
    return false;
  }
  cmd = s_last_cmd;
  seq = s_last_cmd_seq;
  lat_e7 = s_last_cmd_lat;
  lon_e7 = s_last_cmd_lon;
  yaw_deg = s_last_cmd_yaw;
  rel_alt_m = s_last_cmd_alt;
  return true;
}

uint32_t espnowGetRxCount() {
  return s_rx_count;
}

uint32_t espnowGetTxCount() {
  return s_tx_count;
}

