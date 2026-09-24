#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <MAVLink.h>

// Pixhawk TELEM1:
// Pin 2 (TX) -> ESP32-S3 IO10 (RX)
// Pin 3 (RX) -> ESP32-S3 IO11 (TX)
static const int PIN_RX = 10;
static const int PIN_TX = 11;
static const uint32_t FC_BAUD = 115200;

// WiFi (from existing setup)
static const char *WIFI_SSID = "UAVDocking";
static const char *WIFI_PASS = "UAVDocking";

// MAVLink IDs
static const uint8_t SYS_ID = 255;   // GCS ID
static const uint8_t COMP_ID = 190;  // MAV_COMP_ID_GCS
static const uint8_t TARGET_SYS = 1;
static const uint8_t TARGET_COMP = 1;

HardwareSerial FC(1);
WebServer server(80);

struct RcState {
  uint16_t roll = 1500;
  uint16_t pitch = 1500;
  uint16_t throttle = 1000;
  uint16_t yaw = 1500;
  bool overrideEnabled = true;
  uint32_t lastUpdateMs = 0;
};

static RcState rc;
static uint32_t lastHeartbeatMs = 0;
static uint32_t lastRcSendMs = 0;

static void sendMavlink(const mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  FC.write(buf, len);
}

static void sendHeartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
      SYS_ID, COMP_ID, &msg,
      MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
      MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
  sendMavlink(msg);
}

static void sendArmDisarm(bool arm) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      SYS_ID, COMP_ID, &msg,
      TARGET_SYS, TARGET_COMP,
      MAV_CMD_COMPONENT_ARM_DISARM,
      0,
      arm ? 1.0f : 0.0f,
      0, 0, 0, 0, 0, 0);
  sendMavlink(msg);
}

static void sendRcOverride(const RcState &s) {
  mavlink_message_t msg;
  if (!s.overrideEnabled) {
    mavlink_msg_rc_channels_override_pack(
        SYS_ID, COMP_ID, &msg,
        TARGET_SYS, TARGET_COMP,
        0, 0, 0, 0, 0, 0, 0, 0);
  } else {
    mavlink_msg_rc_channels_override_pack(
        SYS_ID, COMP_ID, &msg,
        TARGET_SYS, TARGET_COMP,
        s.roll, s.pitch, s.throttle, s.yaw,
        0, 0, 0, 0);
  }
  sendMavlink(msg);
}

static uint16_t clampPwm(int v) {
  if (v < 1000) return 1000;
  if (v > 2000) return 2000;
  return (uint16_t)v;
}

static void handleRoot() {
  const char *page =
      "<!doctype html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>RC Web</title>"
      "<style>body{font-family:Arial;padding:12px}label{display:block;margin:8px 0}"
      "input{width:100%}button{margin:4px;padding:10px}</style>"
      "</head><body>"
      "<h2>Web RC Controller</h2>"
      "<div>"
      "<button onclick=\"cmd('/arm')\">ARM</button>"
      "<button onclick=\"cmd('/disarm')\">DISARM</button>"
      "<button onclick=\"cmd('/ovr?on=1')\">Override ON</button>"
      "<button onclick=\"cmd('/ovr?on=0')\">Override OFF</button>"
      "</div>"
      "<label>Roll <span id='rval'></span><input id='roll' type='range' min='1000' max='2000' value='1500'></label>"
      "<label>Pitch <span id='pval'></span><input id='pitch' type='range' min='1000' max='2000' value='1500'></label>"
      "<label>Throttle <span id='tval'></span><input id='throttle' type='range' min='1000' max='2000' value='1000'></label>"
      "<label>Yaw <span id='yval'></span><input id='yaw' type='range' min='1000' max='2000' value='1500'></label>"
      "<script>"
      "const r=document.getElementById('roll'),p=document.getElementById('pitch'),t=document.getElementById('throttle'),y=document.getElementById('yaw');"
      "function cmd(p){fetch(p).then(()=>{});}"
      "function send(){fetch(`/set?roll=${r.value}&pitch=${p.value}&throttle=${t.value}&yaw=${y.value}`)}"
      "function upd(){document.getElementById('rval').textContent=r.value;"
      "document.getElementById('pval').textContent=p.value;"
      "document.getElementById('tval').textContent=t.value;"
      "document.getElementById('yval').textContent=y.value;}"
      "r.oninput=p.oninput=t.oninput=y.oninput=()=>{upd();send();};"
      "setInterval(send,200);upd();"
      "</script></body></html>";
  server.send(200, "text/html", page);
}

static void handleSet() {
  if (server.hasArg("roll")) rc.roll = clampPwm(server.arg("roll").toInt());
  if (server.hasArg("pitch")) rc.pitch = clampPwm(server.arg("pitch").toInt());
  if (server.hasArg("throttle")) rc.throttle = clampPwm(server.arg("throttle").toInt());
  if (server.hasArg("yaw")) rc.yaw = clampPwm(server.arg("yaw").toInt());
  rc.lastUpdateMs = millis();
  server.send(200, "text/plain", "OK");
}

static void handleOverride() {
  if (server.hasArg("on")) {
    rc.overrideEnabled = (server.arg("on").toInt() != 0);
  }
  server.send(200, "text/plain", "OK");
}

static void handleArm() {
  sendArmDisarm(true);
  server.send(200, "text/plain", "OK");
}

static void handleDisarm() {
  sendArmDisarm(false);
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  FC.begin(FC_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/ovr", handleOverride);
  server.on("/arm", handleArm);
  server.on("/disarm", handleDisarm);
  server.begin();

  rc.lastUpdateMs = millis();
}

void loop() {
  server.handleClient();

  if (millis() - lastHeartbeatMs > 1000) {
    lastHeartbeatMs = millis();
    sendHeartbeat();
  }

  if (millis() - rc.lastUpdateMs > 1000) {
    rc.roll = 1500;
    rc.pitch = 1500;
    rc.yaw = 1500;
    rc.throttle = 1000;
  }

  if (millis() - lastRcSendMs > 100) {
    lastRcSendMs = millis();
    sendRcOverride(rc);
  }
}


