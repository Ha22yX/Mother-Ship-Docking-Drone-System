#include "web_server.h"

#include <Arduino.h>
#include <WebServer.h>
#include <math.h>

#include "commands.h"
#include "config.h"
#include "telemetry.h"

static WebServer server(80);

static float safeFloat(float v) {
  if (isnan(v) || isinf(v)) return 0.0f;
  return v;
}

static void handleRoot() {
  const char *page =
      "<!doctype html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>UAV Docking</title>"
      "<style>body{font-family:Arial;padding:12px}button{margin:4px;padding:10px}"
      "table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:6px;font-size:12px}</style>"
      "</head><body>"
      "<h2>UAV Docking Debug</h2>"
      "<div>"
      "<button onclick=\"cmd('/api/mother/arm')\">Mother ARM</button>"
      "<button onclick=\"cmd('/api/mother/disarm')\">Mother DISARM</button>"
      "<button onclick=\"cmd('/api/mother/offboard')\">Offboard</button>"
      "<button onclick=\"cmd('/api/mother/rtl')\">Mother RTL (Land)</button>"
      "</div>"
      "<div>"
      "<button onclick=\"cmd('/api/mother/hover1')\">Mother Hover 1m</button>"
      "<button onclick=\"cmd('/api/mother/hover2')\">Mother Hover 2m</button>"
      "<button onclick=\"cmd('/api/child/dock')\">Child Dock (XY match, Z=3m)</button>"
      "<button onclick=\"cmd('/api/child/rtl')\">Child RTL</button>"
      "</div>"
      "<div>"
      "<button onclick=\"cmd('/api/child/alt?delta=0.2')\">Child Alt +</button>"
      "<button onclick=\"cmd('/api/child/alt?delta=-0.2')\">Child Alt -</button>"
      "<button onclick=\"cmd('/api/child/move?dn=0.2&de=0')\">Child North</button>"
      "<button onclick=\"cmd('/api/child/move?dn=-0.2&de=0')\">Child South</button>"
      "<button onclick=\"cmd('/api/child/move?dn=0&de=0.2')\">Child East</button>"
      "<button onclick=\"cmd('/api/child/move?dn=0&de=-0.2')\">Child West</button>"
      "</div>"
      "<h3>Status</h3>"
      "<table><tr><th></th><th>Lat</th><th>Lon</th><th>Alt</th><th>RelAlt</th>"
      "<th>Vx</th><th>Vy</th><th>Vz</th><th>Ax</th><th>Ay</th><th>Az</th>"
      "<th>Vel</th><th>CoG</th><th>Heading</th><th>Airspeed</th><th>GndSpd</th><th>Climb</th>"
      "<th>GPS</th><th>Sats</th><th>EPH</th><th>EPV</th><th>Diff</th>"
      "<th>GPS2</th><th>GPS2 Sats</th><th>EPH2</th><th>EPV2</th>"
      "<th>Bat(V)</th><th>Bat(A)</th><th>Bat(%)</th><th>IMU T</th><th>Age(ms)</th></tr>"
      "<tr><th>Mother</th>"
      "<td id='m_lat'></td><td id='m_lon'></td><td id='m_alt'></td><td id='m_rel'></td>"
      "<td id='m_vx'></td><td id='m_vy'></td><td id='m_vz'></td>"
      "<td id='m_ax'></td><td id='m_ay'></td><td id='m_az'></td>"
      "<td id='m_vel'></td><td id='m_cog'></td><td id='m_heading'></td>"
      "<td id='m_air'></td><td id='m_gnd'></td><td id='m_climb'></td>"
      "<td id='m_fix'></td><td id='m_sats'></td><td id='m_eph'></td><td id='m_epv'></td><td id='m_diff'></td>"
      "<td id='m_gps2fix'></td><td id='m_gps2sats'></td><td id='m_eph2'></td><td id='m_epv2'></td>"
      "<td id='m_bv'></td><td id='m_ba'></td><td id='m_bp'></td><td id='m_temp'></td><td id='m_age'></td>"
      "</tr>"
      "<tr><th>Child</th>"
      "<td id='c_lat'></td><td id='c_lon'></td><td id='c_alt'></td><td id='c_rel'></td>"
      "<td id='c_vx'></td><td id='c_vy'></td><td id='c_vz'></td>"
      "<td id='c_ax'></td><td id='c_ay'></td><td id='c_az'></td>"
      "<td id='c_vel'></td><td id='c_cog'></td><td id='c_heading'></td>"
      "<td id='c_air'></td><td id='c_gnd'></td><td id='c_climb'></td>"
      "<td id='c_fix'></td><td id='c_sats'></td><td id='c_eph'></td><td id='c_epv'></td><td id='c_diff'></td>"
      "<td id='c_gps2fix'></td><td id='c_gps2sats'></td><td id='c_eph2'></td><td id='c_epv2'></td>"
      "<td id='c_bv'></td><td id='c_ba'></td><td id='c_bp'></td><td id='c_temp'></td><td id='c_age'></td>"
      "</tr></table>"
      "<script>"
      "function cmd(p){fetch(p).then(()=>{});}"
      "function upd(){fetch('/api/status').then(r=>r.json()).then(d=>{"
      "let m=d.mother,c=d.child;"
      "q('m_lat',m.lat);q('m_lon',m.lon);q('m_alt',m.alt);q('m_rel',m.rel);"
      "q('m_vx',m.vx);q('m_vy',m.vy);q('m_vz',m.vz);q('m_ax',m.ax);q('m_ay',m.ay);q('m_az',m.az);"
      "q('m_vel',m.vel);q('m_cog',m.cog);q('m_heading',m.heading);q('m_air',m.air);q('m_gnd',m.gnd);q('m_climb',m.climb);"
      "q('m_fix',m.fix);q('m_sats',m.sats);q('m_eph',m.eph);q('m_epv',m.epv);q('m_diff',m.diff);"
      "q('m_gps2fix',m.gps2fix);q('m_gps2sats',m.gps2sats);q('m_eph2',m.eph2);q('m_epv2',m.epv2);"
      "q('m_bv',m.bv);q('m_ba',m.ba);q('m_bp',m.bp);q('m_temp',m.temp);q('m_age',m.age);"
      "q('c_lat',c.lat);q('c_lon',c.lon);q('c_alt',c.alt);q('c_rel',c.rel);"
      "q('c_vx',c.vx);q('c_vy',c.vy);q('c_vz',c.vz);q('c_ax',c.ax);q('c_ay',c.ay);q('c_az',c.az);"
      "q('c_vel',c.vel);q('c_cog',c.cog);q('c_heading',c.heading);q('c_air',c.air);q('c_gnd',c.gnd);q('c_climb',c.climb);"
      "q('c_fix',c.fix);q('c_sats',c.sats);q('c_eph',c.eph);q('c_epv',c.epv);q('c_diff',c.diff);"
      "q('c_gps2fix',c.gps2fix);q('c_gps2sats',c.gps2sats);q('c_eph2',c.eph2);q('c_epv2',c.epv2);"
      "q('c_bv',c.bv);q('c_ba',c.ba);q('c_bp',c.bp);q('c_temp',c.temp);q('c_age',c.age);"
      "});}"
      "function q(id,v){document.getElementById(id).textContent=v;}"
      "setInterval(upd,500);upd();"
      "</script></body></html>";
  server.send(200, "text/html", page);
}

static void handleStatus() {
  String json = "{";
  json += "\"mother\":{";
  json += "\"lat\":" + String(mother.lat, 7) + ",";
  json += "\"lon\":" + String(mother.lon, 7) + ",";
  json += "\"alt\":" + String(safeFloat(mother.altAmsl), 2) + ",";
  json += "\"rel\":" + String(safeFloat(mother.relAlt), 2) + ",";
  json += "\"vx\":" + String(safeFloat(mother.vx), 2) + ",";
  json += "\"vy\":" + String(safeFloat(mother.vy), 2) + ",";
  json += "\"vz\":" + String(safeFloat(mother.vz), 2) + ",";
  json += "\"ax\":" + String(safeFloat(mother.ax), 2) + ",";
  json += "\"ay\":" + String(safeFloat(mother.ay), 2) + ",";
  json += "\"az\":" + String(safeFloat(mother.az), 2) + ",";
  json += "\"vel\":" + String(safeFloat(mother.vel), 2) + ",";
  json += "\"cog\":" + String(safeFloat(mother.cog), 1) + ",";
  json += "\"heading\":" + String(mother.heading) + ",";
  json += "\"air\":" + String(safeFloat(mother.airspeed), 2) + ",";
  json += "\"gnd\":" + String(safeFloat(mother.groundspeed), 2) + ",";
  json += "\"climb\":" + String(safeFloat(mother.climb), 2) + ",";
  json += "\"fix\":\"" + String(fixTypeName(mother.fixType)) + "\",";
  json += "\"sats\":" + String(mother.sats) + ",";
  json += "\"eph\":" + String(safeFloat(mother.eph), 2) + ",";
  json += "\"epv\":" + String(safeFloat(mother.epv), 2) + ",";
  json += "\"diff\":" + String(mother.fixType >= 4 ? 1 : 0) + ",";
  json += "\"gps2fix\":\"" + String(fixTypeName(mother.gps2Fix)) + "\",";
  json += "\"gps2sats\":" + String(mother.gps2Sats) + ",";
  json += "\"eph2\":" + String(safeFloat(mother.gps2Eph), 2) + ",";
  json += "\"epv2\":" + String(safeFloat(mother.gps2Epv), 2) + ",";
  json += "\"bv\":" + String(safeFloat(mother.battVolt), 2) + ",";
  json += "\"ba\":" + String(safeFloat(mother.battCurrent), 2) + ",";
  json += "\"bp\":" + String(mother.battRemaining) + ",";
  json += "\"temp\":" + String(safeFloat(mother.imuTemp), 2) + ",";
  json += "\"age\":" + String(telemetryAgeMs(mother));
  json += "},";
  json += "\"child\":{";
  json += "\"lat\":" + String(child.lat, 7) + ",";
  json += "\"lon\":" + String(child.lon, 7) + ",";
  json += "\"alt\":" + String(safeFloat(child.altAmsl), 2) + ",";
  json += "\"rel\":" + String(safeFloat(child.relAlt), 2) + ",";
  json += "\"vx\":" + String(safeFloat(child.vx), 2) + ",";
  json += "\"vy\":" + String(safeFloat(child.vy), 2) + ",";
  json += "\"vz\":" + String(safeFloat(child.vz), 2) + ",";
  json += "\"ax\":" + String(safeFloat(child.ax), 2) + ",";
  json += "\"ay\":" + String(safeFloat(child.ay), 2) + ",";
  json += "\"az\":" + String(safeFloat(child.az), 2) + ",";
  json += "\"vel\":" + String(safeFloat(child.vel), 2) + ",";
  json += "\"cog\":" + String(safeFloat(child.cog), 1) + ",";
  json += "\"heading\":" + String(child.heading) + ",";
  json += "\"air\":" + String(safeFloat(child.airspeed), 2) + ",";
  json += "\"gnd\":" + String(safeFloat(child.groundspeed), 2) + ",";
  json += "\"climb\":" + String(safeFloat(child.climb), 2) + ",";
  json += "\"fix\":\"" + String(fixTypeName(child.fixType)) + "\",";
  json += "\"sats\":" + String(child.sats) + ",";
  json += "\"eph\":" + String(safeFloat(child.eph), 2) + ",";
  json += "\"epv\":" + String(safeFloat(child.epv), 2) + ",";
  json += "\"diff\":" + String(child.fixType >= 4 ? 1 : 0) + ",";
  json += "\"gps2fix\":\"" + String(fixTypeName(child.gps2Fix)) + "\",";
  json += "\"gps2sats\":" + String(child.gps2Sats) + ",";
  json += "\"eph2\":" + String(safeFloat(child.gps2Eph), 2) + ",";
  json += "\"epv2\":" + String(safeFloat(child.gps2Epv), 2) + ",";
  json += "\"bv\":" + String(safeFloat(child.battVolt), 2) + ",";
  json += "\"ba\":" + String(safeFloat(child.battCurrent), 2) + ",";
  json += "\"bp\":" + String(child.battRemaining) + ",";
  json += "\"temp\":" + String(safeFloat(child.imuTemp), 2) + ",";
  json += "\"age\":" + String(telemetryAgeMs(child));
  json += "}";
  json += "}";
  server.send(200, "application/json", json);
}

static void handleMotherHover1() {
  cmdMotherHover(1.0f);
  server.send(200, "text/plain", "OK");
}

static void handleMotherHover2() {
  cmdMotherHover(2.0f);
  server.send(200, "text/plain", "OK");
}

static void handleMotherArm() {
  cmdMotherArm(true);
  server.send(200, "text/plain", "OK");
}

static void handleMotherDisarm() {
  cmdMotherArm(false);
  server.send(200, "text/plain", "OK");
}

static void handleMotherOffboard() {
  cmdMotherOffboard();
  server.send(200, "text/plain", "OK");
}

static void handleMotherRtl() {
  cmdMotherRtl();
  server.send(200, "text/plain", "OK");
}

static void handleChildDock() {
  cmdChildDock();
  server.send(200, "text/plain", "OK");
}

static void handleChildRtl() {
  cmdChildRtl();
  server.send(200, "text/plain", "OK");
}

static void handleChildAlt() {
  if (server.hasArg("delta")) {
    cmdChildAlt(server.arg("delta").toFloat());
  }
  server.send(200, "text/plain", "OK");
}

static void handleChildMove() {
  float dn = 0.0f;
  float de = 0.0f;
  if (server.hasArg("dn")) {
    dn = server.arg("dn").toFloat();
  }
  if (server.hasArg("de")) {
    de = server.arg("de").toFloat();
  }
  cmdChildMove(dn, de);
  server.send(200, "text/plain", "OK");
}

static void handleChildStatus() {
  if (server.hasArg("lat")) child.lat = server.arg("lat").toDouble();
  if (server.hasArg("lon")) child.lon = server.arg("lon").toDouble();
  if (server.hasArg("alt")) child.altAmsl = server.arg("alt").toFloat();
  if (server.hasArg("rel")) child.relAlt = server.arg("rel").toFloat();
  if (server.hasArg("vx")) child.vx = server.arg("vx").toFloat();
  if (server.hasArg("vy")) child.vy = server.arg("vy").toFloat();
  if (server.hasArg("vz")) child.vz = server.arg("vz").toFloat();
  if (server.hasArg("ax")) child.ax = server.arg("ax").toFloat();
  if (server.hasArg("ay")) child.ay = server.arg("ay").toFloat();
  if (server.hasArg("az")) child.az = server.arg("az").toFloat();
  if (server.hasArg("vel")) child.vel = server.arg("vel").toFloat();
  if (server.hasArg("cog")) child.cog = server.arg("cog").toFloat();
  if (server.hasArg("heading")) child.heading = (int16_t)server.arg("heading").toInt();
  if (server.hasArg("air")) child.airspeed = server.arg("air").toFloat();
  if (server.hasArg("gnd")) child.groundspeed = server.arg("gnd").toFloat();
  if (server.hasArg("climb")) child.climb = server.arg("climb").toFloat();
  if (server.hasArg("eph")) child.eph = server.arg("eph").toFloat();
  if (server.hasArg("epv")) child.epv = server.arg("epv").toFloat();
  if (server.hasArg("bv")) child.battVolt = server.arg("bv").toFloat();
  if (server.hasArg("ba")) child.battCurrent = server.arg("ba").toFloat();
  if (server.hasArg("bp")) child.battRemaining = (int8_t)server.arg("bp").toInt();
  if (server.hasArg("temp")) child.imuTemp = server.arg("temp").toFloat();
  if (server.hasArg("fix")) child.fixType = (uint8_t)server.arg("fix").toInt();
  if (server.hasArg("sats")) child.sats = (uint8_t)server.arg("sats").toInt();
  if (server.hasArg("gps2fix")) child.gps2Fix = (uint8_t)server.arg("gps2fix").toInt();
  if (server.hasArg("gps2sats")) child.gps2Sats = (uint8_t)server.arg("gps2sats").toInt();
  if (server.hasArg("eph2")) child.gps2Eph = server.arg("eph2").toFloat();
  if (server.hasArg("epv2")) child.gps2Epv = server.arg("epv2").toFloat();
  child.valid = true;
  child.lastMs = millis();
  server.send(200, "text/plain", "OK");
}

static void handleChildCmd() {
  String resp = buildChildCmdResponse();
  server.send(200, "text/plain", resp);
}

void webSetup() {
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/mother/arm", handleMotherArm);
  server.on("/api/mother/disarm", handleMotherDisarm);
  server.on("/api/mother/offboard", handleMotherOffboard);
  server.on("/api/mother/rtl", handleMotherRtl);
  server.on("/api/mother/hover1", handleMotherHover1);
  server.on("/api/mother/hover2", handleMotherHover2);
  server.on("/api/child/dock", handleChildDock);
  server.on("/api/child/rtl", handleChildRtl);
  server.on("/api/child/alt", handleChildAlt);
  server.on("/api/child/move", handleChildMove);
  server.on("/api/child", handleChildStatus);
  server.on("/api/cmd", handleChildCmd);
  server.begin();
}

void webLoop() {
  server.handleClient();
}

