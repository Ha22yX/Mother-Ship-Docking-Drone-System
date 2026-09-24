#include "app.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "mavlink_iface.h"
#include "net_client.h"
#include "config.h"
#include "telemetry.h"

static uint32_t lastHeartbeat = 0;
static uint32_t lastSetpoint = 0;
static uint32_t lastStatusPush = 0;
static uint32_t lastCmdPoll = 0;

static uint32_t lastCmdSeq = 0;
static bool targetActive = false;
static double targetLat = 0;
static double targetLon = 0;
static float targetRelAlt = 3.0f;
static bool homeSet = false;
static double homeLat = 0;
static double homeLon = 0;
static WiFiUDP rtkUdp;
static bool rtkUdpReady = false;

enum RtlStage {
  RTL_IDLE = 0,
  RTL_CLIMB,
  RTL_RETURN,
  RTL_DESCEND
};
static RtlStage rtlStage = RTL_IDLE;

enum DockStage {
  DOCK_IDLE = 0,
  DOCK_CLIMB,
  DOCK_MOVE,
  DOCK_YAW
};
static DockStage dockStage = DOCK_IDLE;
static double dockTargetLat = 0;
static double dockTargetLon = 0;
static float dockTargetRelAlt = 3.0f;
static float dockTargetYawRad = 0.0f;

void appSetup() {
  Serial.begin(115200);
  delay(200);

  mavlinkSetup();
  netSetup();
  mavlinkRequestIntervals();

  Serial.println("UAVDocking Child boot");
}

void appLoop() {
  netEnsureConnected();
  if (!rtkUdpReady && WiFi.status() == WL_CONNECTED) {
    rtkUdp.begin(RTCM_UDP_PORT);
    rtkUdpReady = true;
  }

  if (millis() - lastHeartbeat > 1000) {
    lastHeartbeat = millis();
    mavlinkSendHeartbeat();
  }

  mavlinkLoop();
  if (!homeSet && self.valid) {
    homeLat = self.lat;
    homeLon = self.lon;
    homeSet = true;
  }

  if (millis() - lastCmdPoll > 500) {
    lastCmdPoll = millis();
    String cmd;
    uint32_t seq = 0;
    double lat = 0;
    double lon = 0;
    float relAlt = 0;
    float yawDeg = 0;
    if (netPollCommand(cmd, seq, lat, lon, relAlt, yawDeg)) {
      if (seq > lastCmdSeq) {
        lastCmdSeq = seq;
        if (cmd == "goto") {
          targetLat = lat;
          targetLon = lon;
          targetRelAlt = relAlt;
          targetActive = true;
          rtlStage = RTL_IDLE;
          dockStage = DOCK_IDLE;
        } else if (cmd == "rtl") {
          if (homeSet) {
            rtlStage = RTL_CLIMB;
            targetActive = true;
            targetLat = self.lat;
            targetLon = self.lon;
            targetRelAlt = 4.0f;
            dockStage = DOCK_IDLE;
          }
        } else if (cmd == "dock") {
          dockTargetLat = lat;
          dockTargetLon = lon;
          dockTargetRelAlt = relAlt;
          dockTargetYawRad = yawDeg * DEG_TO_RAD;
          dockStage = DOCK_CLIMB;
          rtlStage = RTL_IDLE;
          targetActive = true;
        }
      }
    }
  }

  if (millis() - lastStatusPush > 500) {
    lastStatusPush = millis();
    netPushStatus();
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

  if (rtlStage != RTL_IDLE && self.valid) {
    const float relAlt = self.relAlt;
    if (rtlStage == RTL_CLIMB) {
      targetLat = self.lat;
      targetLon = self.lon;
      targetRelAlt = 4.0f;
      if (relAlt >= 3.8f) {
        rtlStage = RTL_RETURN;
      }
    } else if (rtlStage == RTL_RETURN) {
      targetLat = homeLat;
      targetLon = homeLon;
      targetRelAlt = 4.0f;
      const double latRad = self.lat * DEG_TO_RAD;
      const double dLat = (homeLat - self.lat) * 111111.0;
      const double dLon = (homeLon - self.lon) * 111111.0 * cos(latRad);
      const double dist = sqrt(dLat * dLat + dLon * dLon);
      if (dist < 1.0) {
        rtlStage = RTL_DESCEND;
      }
    } else if (rtlStage == RTL_DESCEND) {
      targetLat = homeLat;
      targetLon = homeLon;
      targetRelAlt = 0.0f;
      if (relAlt <= 0.3f) {
        rtlStage = RTL_IDLE;
        targetActive = false;
      }
    }
  }

  if (dockStage != DOCK_IDLE && self.valid) {
    if (dockStage == DOCK_CLIMB) {
      targetLat = self.lat;
      targetLon = self.lon;
      targetRelAlt = dockTargetRelAlt;
      if (self.relAlt >= (dockTargetRelAlt - 0.2f)) {
        dockStage = DOCK_MOVE;
      }
    } else if (dockStage == DOCK_MOVE) {
      targetLat = dockTargetLat;
      targetLon = dockTargetLon;
      targetRelAlt = dockTargetRelAlt;
      const double latRad = self.lat * DEG_TO_RAD;
      const double dLat = (dockTargetLat - self.lat) * 111111.0;
      const double dLon = (dockTargetLon - self.lon) * 111111.0 * cos(latRad);
      const double dist = sqrt(dLat * dLat + dLon * dLon);
      if (dist < 0.5) {
        dockStage = DOCK_YAW;
      }
    } else if (dockStage == DOCK_YAW) {
      targetLat = dockTargetLat;
      targetLon = dockTargetLon;
      targetRelAlt = dockTargetRelAlt;
      float yawErr = dockTargetYawRad - self.yawRad;
      while (yawErr > PI) yawErr -= 2.0f * PI;
      while (yawErr < -PI) yawErr += 2.0f * PI;
      if (fabs(yawErr) < (10.0f * DEG_TO_RAD)) {
        dockStage = DOCK_IDLE;
      }
    }
  }

  if (targetActive && (millis() - lastSetpoint > 200)) {
    lastSetpoint = millis();
    if (dockStage == DOCK_YAW) {
      mavlinkSendSetpointGlobalRelAltYaw(targetLat, targetLon, targetRelAlt, dockTargetYawRad);
    } else {
      mavlinkSendSetpointGlobalRelAlt(targetLat, targetLon, targetRelAlt);
    }
  }

  static uint32_t lastLog = 0;
  if (millis() - lastLog > 1000) {
    lastLog = millis();
    Serial.print("GPS fix=");
    Serial.print(self.fixType);
    Serial.print(" sats=");
    Serial.print(self.sats);
    Serial.print(" lat=");
    Serial.print(self.lat, 7);
    Serial.print(" lon=");
    Serial.print(self.lon, 7);
    Serial.print(" relAlt=");
    Serial.print(self.relAlt, 2);
    Serial.print(" heading=");
    Serial.println((int)(self.yawRad * 57.2958f));
  }
}

