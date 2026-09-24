#include "app.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "commands.h"
#include "config.h"
#include "mavlink_iface.h"
#include "telemetry.h"
#include "web_server.h"

static uint32_t lastHeartbeat = 0;
static uint32_t lastSetpoint = 0;
static WiFiUDP rtkUdp;
static bool rtkUdpReady = false;
static bool holdInit = false;
static double holdLat = 0;
static double holdLon = 0;
static float holdRelAlt = 0;

static bool motherTargetActive = false;
static double motherTargetLat = 0;
static double motherTargetLon = 0;
static float motherTargetRelAlt = 0;

static uint32_t childCmdSeq = 0;
static String childCmdMode = "goto";
static double childTargetLat = 0;
static double childTargetLon = 0;
static float childTargetRelAlt = 3.0f;
static float childTargetYawDeg = 0.0f;
static float childOffsetNorth = 0.0f;
static float childOffsetEast = 0.0f;
static float childAltOffset = 0.0f;

void cmdMotherHover(float relAlt) {
  if (!mother.valid) return;
  motherTargetLat = mother.lat;
  motherTargetLon = mother.lon;
  motherTargetRelAlt = relAlt;
  motherTargetActive = true;
  mavlinkSendStatusText(relAlt >= 1.5f ? "Mother Hover 2m" : "Mother Hover 1m");
}

void cmdMotherArm(bool arm) {
  mavlinkSendArmDisarm(arm);
  motherTargetActive = false;
  if (mother.valid) {
    holdLat = mother.lat;
    holdLon = mother.lon;
    holdRelAlt = mother.relAlt;
    holdInit = true;
  }
  mavlinkSendStatusText(arm ? "Mother ARM" : "Mother DISARM");
}

void cmdMotherOffboard() {
  mavlinkSendOffboardMode();
  mavlinkSendStatusText("Offboard requested");
}

void cmdMotherRtl() {
  if (!mother.valid) return;
  motherTargetLat = mother.lat;
  motherTargetLon = mother.lon;
  motherTargetRelAlt = 0.0f;
  motherTargetActive = true;
  mavlinkSendStatusText("Mother RTL");
}

static void sendChildDock(double lat, double lon, float relAlt) {
  childTargetLat = lat;
  childTargetLon = lon;
  childTargetRelAlt = relAlt;
  childTargetYawDeg = mother.heading;
  childCmdMode = "dock";
  childCmdSeq++;
}

void cmdChildDock() {
  if (!mother.valid) return;
  const double latRad = mother.lat * DEG_TO_RAD;
  const double dLat = childOffsetNorth / 111111.0;
  const double dLon = childOffsetEast / (111111.0 * cos(latRad));
  const double lat = mother.lat + dLat;
  const double lon = mother.lon + dLon;
  const float relAlt = 3.0f + childAltOffset;
  sendChildDock(lat, lon, relAlt);
  mavlinkSendStatusText("Child Dock");
}

void cmdChildRtl() {
  childCmdMode = "rtl";
  childCmdSeq++;
}

void cmdChildAlt(float delta) {
  childAltOffset += delta;
  cmdChildDock();
}

void cmdChildMove(float dn, float de) {
  childOffsetNorth += dn;
  childOffsetEast += de;
  cmdChildDock();
}

String buildChildCmdResponse() {
  String resp = "seq=" + String(childCmdSeq) +
                ";cmd=" + childCmdMode +
                ";lat=" + String(childTargetLat, 7) +
                ";lon=" + String(childTargetLon, 7) +
                ";alt=" + String(childTargetRelAlt, 2) +
                ";yaw=" + String(childTargetYawDeg, 1) + ";";
  return resp;
}

void appSetup() {
  Serial.begin(115200);
  delay(200);

  mavlinkSetup();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASS);

  rtkUdp.begin(RTCM_UDP_PORT);
  rtkUdpReady = true;

  webSetup();
  mavlinkRequestIntervals();

  Serial.println("UAVDocking Mother boot");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void appLoop() {
  webLoop();

  if (millis() - lastHeartbeat > 1000) {
    lastHeartbeat = millis();
    mavlinkSendHeartbeat();
  }

  mavlinkLoop();

  if (!holdInit && mother.valid) {
    holdLat = mother.lat;
    holdLon = mother.lon;
    holdRelAlt = mother.relAlt;
    holdInit = true;
  }

  if (rtkUdpReady) {
    int packetSize = rtkUdp.parsePacket();
    while (packetSize > 0) {
      uint8_t buf[512];
      int len = rtkUdp.read(buf, sizeof(buf));
      if (len > 0) {
        mavlinkSendRtcm(buf, (size_t)len);
      }
      packetSize = rtkUdp.parsePacket();
    }
  }

  if (mother.valid && (millis() - lastSetpoint > 200)) {
    lastSetpoint = millis();
    if (motherTargetActive) {
      mavlinkSendSetpointGlobalRelAlt(motherTargetLat, motherTargetLon, motherTargetRelAlt);
    } else if (holdInit) {
      mavlinkSendSetpointGlobalRelAlt(holdLat, holdLon, holdRelAlt);
    }
  }

  static uint32_t lastLog = 0;
  if (millis() - lastLog > 1000) {
    lastLog = millis();
    Serial.print("GPS fix=");
    Serial.print(mother.fixType);
    Serial.print(" sats=");
    Serial.print(mother.sats);
    Serial.print(" lat=");
    Serial.print(mother.lat, 7);
    Serial.print(" lon=");
    Serial.print(mother.lon, 7);
    Serial.print(" relAlt=");
    Serial.print(mother.relAlt, 2);
    Serial.print(" heading=");
    Serial.println(mother.heading);
  }
}

