#include "net_client.h"

#include <HTTPClient.h>
#include <WiFi.h>

#include "config.h"
#include "telemetry.h"

void netSetup() {
  netEnsureConnected();
}

void netEnsureConnected() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.println("WiFi connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 5000) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP=");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect timeout.");
  }
}

void netPushStatus() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!self.valid) return;

  String url = String(MOTHER_HOST) + "/api/child?";
  url += "lat=" + String(self.lat, 7);
  url += "&lon=" + String(self.lon, 7);
  url += "&alt=" + String(self.altAmsl, 2);
  url += "&rel=" + String(self.relAlt, 2);
  url += "&vx=" + String(self.vx, 2);
  url += "&vy=" + String(self.vy, 2);
  url += "&vz=" + String(self.vz, 2);
  url += "&ax=" + String(self.ax, 2);
  url += "&ay=" + String(self.ay, 2);
  url += "&az=" + String(self.az, 2);
  url += "&vel=" + String(self.vel, 2);
  url += "&cog=" + String(self.cog, 1);
  url += "&heading=" + String(self.heading);
  url += "&air=" + String(self.airspeed, 2);
  url += "&gnd=" + String(self.groundspeed, 2);
  url += "&climb=" + String(self.climb, 2);
  url += "&eph=" + String(self.eph, 2);
  url += "&epv=" + String(self.epv, 2);
  url += "&bv=" + String(self.battVolt, 2);
  url += "&ba=" + String(self.battCurrent, 2);
  url += "&bp=" + String(self.battRemaining);
  url += "&temp=" + String(self.imuTemp, 2);
  url += "&fix=" + String(self.fixType);
  url += "&sats=" + String(self.sats);
  url += "&gps2fix=" + String(self.gps2Fix);
  url += "&gps2sats=" + String(self.gps2Sats);
  url += "&eph2=" + String(self.gps2Eph, 2);
  url += "&epv2=" + String(self.gps2Epv, 2);

  WiFiClient client;
  HTTPClient http;
  http.begin(client, url);
  http.GET();
  http.end();
}

static String getField(const String &src, const char *key) {
  String pat = String(key) + "=";
  int start = src.indexOf(pat);
  if (start < 0) return "";
  start += pat.length();
  int end = src.indexOf(';', start);
  if (end < 0) end = src.length();
  return src.substring(start, end);
}

bool netPollCommand(String &cmd, uint32_t &seq, double &lat, double &lon, float &relAlt, float &yawDeg) {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClient client;
  HTTPClient http;
  String url = String(MOTHER_HOST) + "/api/cmd";
  http.begin(client, url);
  int code = http.GET();
  if (code == 200) {
    String body = http.getString();
    String seqStr = getField(body, "seq");
    String cmdStr = getField(body, "cmd");
    String latStr = getField(body, "lat");
    String lonStr = getField(body, "lon");
    String altStr = getField(body, "alt");
    String yawStr = getField(body, "yaw");
    cmd = cmdStr;
    seq = (uint32_t)seqStr.toInt();
    lat = latStr.toDouble();
    lon = lonStr.toDouble();
    relAlt = altStr.toFloat();
    yawDeg = yawStr.toFloat();
    http.end();
    return true;
  }
  http.end();
  return false;
}

