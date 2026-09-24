#include "app.h"

#include <WiFi.h>
#include <esp_wifi.h>

#include "config.h"
#include "espnow_link.h"
#include "mavlink_iface.h"
#include "telemetry.h"

HardwareSerial FC(1);

static TelemetryData s_local;
static TelemetryData s_remote;
static uint32_t s_last_send = 0;
static uint32_t s_last_log = 0;
static uint32_t s_last_heartbeat = 0;
static uint32_t s_takeoff_pos_switch_ms = 0;
static uint32_t s_last_cmd_seq = 0;
static bool s_magnet_on = false;
static bool s_follow_enabled = false;
static int32_t s_follow_lat_e7 = 0;
static int32_t s_follow_lon_e7 = 0;
static float s_follow_yaw_deg = 0.0f;
static float s_follow_alt_m = 1.0f;
static uint32_t s_last_follow_setpoint = 0;

static const int MAGNET_PIN = 12;

static void wifiSetup() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);
}

void appSetup() {
  Serial.begin(115200);
  delay(300);

  telemetryReset(s_local);
  telemetryReset(s_remote);

  pinMode(MAGNET_PIN, OUTPUT);
  digitalWrite(MAGNET_PIN, LOW);

  FC.begin(FC_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  mavlinkInit(FC);
  mavlinkRequestIntervals();

  wifiSetup();
  espnowSetup(NODE_ID_CHILD);

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
    s_local.magnet_on = s_magnet_on ? 1 : 0;
    espnowSendTelemetry(s_local);
  }

  uint8_t cmd = CMD_NONE;
  uint32_t cmd_seq = 0;
  int32_t cmd_lat = 0;
  int32_t cmd_lon = 0;
  float cmd_yaw = 0.0f;
  float cmd_alt = 0.0f;
  if (espnowGetCommand(cmd, cmd_seq, cmd_lat, cmd_lon, cmd_yaw, cmd_alt) && cmd_seq != s_last_cmd_seq) {
    s_last_cmd_seq = cmd_seq;
    if (cmd == CMD_ARM) {
      mavlinkSendTakeoffMode();
      delay(50);
      mavlinkSendArmDisarm(true);
    } else if (cmd == CMD_TAKEOFF_1M) {
      mavlinkSendTakeoffMode();
      delay(50);
      mavlinkSendArmDisarm(true);
      mavlinkSendTakeoff(1.0f);
      s_takeoff_pos_switch_ms = millis() + 3000;
    } else if (cmd == CMD_LAND) {
      mavlinkSendLandMode();
    } else if (cmd == CMD_POSCTL) {
      mavlinkSendPositionMode();
    } else if (cmd == CMD_MAGNET_TOGGLE) {
      s_magnet_on = !s_magnet_on;
      digitalWrite(MAGNET_PIN, s_magnet_on ? HIGH : LOW);
    } else if (cmd == CMD_FOLLOW_ON) {
      s_follow_enabled = true;
      s_last_follow_setpoint = 0;
      mavlinkSendPositionMode();
    } else if (cmd == CMD_FOLLOW_OFF) {
      s_follow_enabled = false;
    } else if (cmd == CMD_FOLLOW_SETPOINT) {
      s_follow_lat_e7 = cmd_lat;
      s_follow_lon_e7 = cmd_lon;
      s_follow_yaw_deg = cmd_yaw;
      s_follow_alt_m = cmd_alt;
    }
  }

  uint32_t age_ms = 0;
  if (espnowGetRemote(s_remote, age_ms)) {
    s_remote.last_update_ms = now - age_ms;
  }

  if (s_takeoff_pos_switch_ms != 0 && now >= s_takeoff_pos_switch_ms) {
    s_takeoff_pos_switch_ms = 0;
    mavlinkSendPositionMode();
  }
  if (s_follow_enabled && s_follow_lat_e7 != 0 && s_follow_lon_e7 != 0) {
    if (now - s_last_follow_setpoint >= 200) {
      s_last_follow_setpoint = now;
      mavlinkSendSetpointGlobalRelAltYaw(s_follow_lat_e7, s_follow_lon_e7, s_follow_alt_m, s_follow_yaw_deg);
    }
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
}

