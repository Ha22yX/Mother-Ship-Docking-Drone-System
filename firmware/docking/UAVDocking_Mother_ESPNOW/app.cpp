#include "app.h"

#include <WiFi.h>
#include <esp_wifi.h>

#include "config.h"
#include "espnow_link.h"
#include "mavlink_iface.h"
#include "telemetry.h"
#include "web_server.h"

HardwareSerial FC(1);

static TelemetryData s_local;
static TelemetryData s_remote;
static uint32_t s_last_send = 0;
static uint32_t s_last_log = 0;
static uint32_t s_last_heartbeat = 0;
static uint32_t s_takeoff_pos_switch_ms = 0;
static bool s_child_follow_enabled = false;
static uint32_t s_last_follow_send = 0;
static float s_child_follow_alt_m = 1.0f;
static float s_child_follow_fwd_m = 0.0f;
static float s_child_follow_right_m = 0.0f;

static void wifiSetup() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS, WIFI_CHANNEL, 0, 4);
  esp_wifi_set_ps(WIFI_PS_NONE);
  Serial.print("[WiFi] AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void appSetup() {
  Serial.begin(115200);
  delay(300);

  telemetryReset(s_local);
  telemetryReset(s_remote);

  FC.begin(FC_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  mavlinkInit(FC);
  mavlinkRequestIntervals();

  wifiSetup();
  espnowSetup(NODE_ID_MOTHER);
  webSetTelemetrySources(&s_local, &s_remote);
  webSetTakeoffCallback([]() {
    mavlinkSendTakeoff(2.0f);
    s_takeoff_pos_switch_ms = millis() + 3000;
  });
  webSetArmCallback([](bool arm) {
    if (arm) {
      mavlinkSendTakeoffMode();
      delay(50);
      mavlinkSendArmDisarm(true);
    } else {
      mavlinkSendLandMode();
    }
  });
  webSetLandCallback([]() { mavlinkSendLandMode(); });
  webSetChildArmCallback([]() { espnowSendCommand(CMD_ARM, 0, 0, 0.0f, 0.0f); });
  webSetChildTakeoffCallback([]() { espnowSendCommand(CMD_TAKEOFF_1M, 0, 0, 0.0f, 0.0f); });
  webSetChildLandCallback([]() { espnowSendCommand(CMD_LAND, 0, 0, 0.0f, 0.0f); });
  webSetChildMagnetToggleCallback([]() { espnowSendCommand(CMD_MAGNET_TOGGLE, 0, 0, 0.0f, 0.0f); });
  webSetChildFollowCallback([](bool enable) {
    s_child_follow_enabled = enable;
    espnowSendCommand(enable ? CMD_FOLLOW_ON : CMD_FOLLOW_OFF, 0, 0, 0.0f, 0.0f);
  });
  webSetChildAltAdjustCallback([](float delta_m) {
    s_child_follow_alt_m += delta_m;
    if (s_child_follow_alt_m < 0.2f) {
      s_child_follow_alt_m = 0.2f;
    }
  });
  webSetChildOffsetAdjustCallback([](float forward_m, float right_m) {
    s_child_follow_fwd_m += forward_m;
    s_child_follow_right_m += right_m;
  });
  webSetup();

  Serial.print("[ESPNOW] MAC: ");
  Serial.println(WiFi.macAddress());
}

void appLoop() {
  mavlinkPoll();
  s_local = mavlinkGetTelemetry();

  uint32_t now = millis();
  if (now - s_last_heartbeat >= 1000) {
    s_last_heartbeat = now;
    mavlinkSendHeartbeat();
  }
  if (now - s_last_send >= ESPNOW_SEND_INTERVAL_MS) {
    s_last_send = now;
    espnowSendTelemetry(s_local);
  }
  webSetFollowStatus(s_child_follow_alt_m, s_child_follow_fwd_m, s_child_follow_right_m);
  if (s_child_follow_enabled && (now - s_last_follow_send) >= 200) {
    s_last_follow_send = now;
    if (s_local.lat_e7 != 0 && s_local.lon_e7 != 0) {
      const float yaw_rad = s_local.heading_deg * 0.01745329252f;
      const float north = s_child_follow_fwd_m * cosf(yaw_rad) - s_child_follow_right_m * sinf(yaw_rad);
      const float east = s_child_follow_fwd_m * sinf(yaw_rad) + s_child_follow_right_m * cosf(yaw_rad);
      const float dlat = north / 111111.0f;
      const float dlon = east / (111111.0f * cosf(s_local.lat_e7 / 1e7f * 0.01745329252f));
      const int32_t lat_e7 = s_local.lat_e7 + (int32_t)(dlat * 1e7f);
      const int32_t lon_e7 = s_local.lon_e7 + (int32_t)(dlon * 1e7f);
      espnowSendCommand(CMD_FOLLOW_SETPOINT, lat_e7, lon_e7, s_local.heading_deg, s_child_follow_alt_m);
    }
  }
  if (s_takeoff_pos_switch_ms != 0 && now >= s_takeoff_pos_switch_ms) {
    s_takeoff_pos_switch_ms = 0;
    mavlinkSendPositionMode();
  }

  uint32_t age_ms = 0;
  if (espnowGetRemote(s_remote, age_ms)) {
    // mark age using local time base for UI display
    s_remote.last_update_ms = now - age_ms;
  }

  if (now - s_last_log > 1000) {
    s_last_log = now;
    Serial.print("[MAV] rx_bytes=");
    Serial.print(mavlinkGetRxBytes());
    Serial.print(" msgs=");
    Serial.print(mavlinkGetMsgCount());
    Serial.print(" stx_v2=");
    Serial.print(mavlinkGetStxV2());
    Serial.print(" stx_v1=");
    Serial.print(mavlinkGetStxV1());
    Serial.print(" | ESPNOW rx=");
    Serial.print(espnowGetRxCount());
    Serial.print(" tx=");
    Serial.println(espnowGetTxCount());
  }

  webLoop();
}

