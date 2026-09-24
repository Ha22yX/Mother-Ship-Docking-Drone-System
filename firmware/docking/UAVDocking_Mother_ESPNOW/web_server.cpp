#include "web_server.h"

#include <WebServer.h>
#include <WiFi.h>

static WebServer server(80);
static const TelemetryData *s_local = nullptr;
static const TelemetryData *s_remote = nullptr;
static void (*s_takeoff_cb)() = nullptr;
static void (*s_arm_cb)(bool arm) = nullptr;
static void (*s_land_cb)() = nullptr;
static void (*s_child_arm_cb)() = nullptr;
static void (*s_child_takeoff_cb)() = nullptr;
static void (*s_child_land_cb)() = nullptr;
static void (*s_child_magnet_cb)() = nullptr;
static void (*s_child_follow_cb)(bool enable) = nullptr;
static bool s_child_follow_enabled = false;
static void (*s_child_alt_cb)(float delta_m) = nullptr;
static void (*s_child_offset_cb)(float forward_m, float right_m) = nullptr;
static float s_follow_alt_m = 1.0f;
static float s_follow_fwd_m = 0.0f;
static float s_follow_right_m = 0.0f;

static float safeFloat(float v) {
  if (isnan(v) || isinf(v)) {
    return 0.0f;
  }
  return v;
}

static void appendTelemetryJson(String &out, const char *name, const TelemetryData &t) {
  uint32_t now = millis();
  uint32_t age = (t.last_update_ms == 0) ? 0 : (now - t.last_update_ms);
  out += "\"";
  out += name;
  out += "\":{";
  out += "\"lat\":" + String(telemetryLat(t), 7) + ",";
  out += "\"lon\":" + String(telemetryLon(t), 7) + ",";
  out += "\"alt\":" + String(safeFloat(t.alt_m), 2) + ",";
  out += "\"relAlt\":" + String(safeFloat(t.rel_alt_m), 2) + ",";
  out += "\"vx\":" + String(safeFloat(t.vx_mps), 2) + ",";
  out += "\"vy\":" + String(safeFloat(t.vy_mps), 2) + ",";
  out += "\"vz\":" + String(safeFloat(t.vz_mps), 2) + ",";
  out += "\"gnd\":" + String(safeFloat(t.groundspeed_mps), 2) + ",";
  out += "\"climb\":" + String(safeFloat(t.climb_mps), 2) + ",";
  out += "\"hdg\":" + String(safeFloat(t.heading_deg), 1) + ",";
  out += "\"roll\":" + String(safeFloat(t.roll_deg), 1) + ",";
  out += "\"pitch\":" + String(safeFloat(t.pitch_deg), 1) + ",";
  out += "\"yaw\":" + String(safeFloat(t.yaw_deg), 1) + ",";
  out += "\"acc\":[" + String(safeFloat(t.imu_acc_x), 2) + "," +
         String(safeFloat(t.imu_acc_y), 2) + "," +
         String(safeFloat(t.imu_acc_z), 2) + "],";
  out += "\"gyro\":[" + String(safeFloat(t.imu_gyro_x), 2) + "," +
         String(safeFloat(t.imu_gyro_y), 2) + "," +
         String(safeFloat(t.imu_gyro_z), 2) + "],";
  out += "\"hdop\":" + String(safeFloat(t.hdop), 2) + ",";
  out += "\"vdop\":" + String(safeFloat(t.vdop), 2) + ",";
  out += "\"fix\":" + String(t.fix_type) + ",";
  out += "\"sats\":" + String(t.sats) + ",";
  out += "\"gps2Fix\":" + String(t.gps2_fix) + ",";
  out += "\"gps2Sats\":" + String(t.gps2_sats) + ",";
  out += "\"gps2Hdop\":" + String(safeFloat(t.gps2_hdop), 2) + ",";
  out += "\"gps2Vdop\":" + String(safeFloat(t.gps2_vdop), 2) + ",";
  out += "\"batV\":" + String(safeFloat(t.battery_v), 2) + ",";
  out += "\"batA\":" + String(safeFloat(t.battery_a), 2) + ",";
  out += "\"batRem\":" + String(safeFloat(t.battery_remaining), 0) + ",";
  out += "\"temp\":" + String(safeFloat(t.imu_temp_c), 1) + ",";
  out += "\"magnet\":" + String((int)t.magnet_on) + ",";
  out += "\"ageMs\":" + String(age);
  out += "}";
}

static String htmlPage() {
  String html;
  html.reserve(4000);
  html += "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>UAV Docking - ESPNOW</title>";
  html += "<style>body{font-family:Arial;margin:16px;background:#111;color:#eee}";
  html += "table{border-collapse:collapse;width:100%;margin-bottom:16px}";
  html += "th,td{border:1px solid #333;padding:6px;font-size:14px;text-align:left}";
  html += "h2{margin:12px 0}</style></head><body>";
  html += "<button onclick='arm(true)'>Mother ARM</button>";
  html += "<button onclick='land()'>Mother LAND</button>";
  html += "<button onclick='takeoff()'>Mother Takeoff 2m</button>";
  html += "<br><br>";
  html += "<button onclick='childArm()'>Child ARM</button>";
  html += "<button onclick='childTakeoff()'>Child Takeoff 1m</button>";
  html += "<button onclick='childLand()'>Child LAND</button>";
  html += "<button id='magnetBtn' onclick='childMagnet()'>Child Magnet Toggle</button>";
  html += "<button id='followBtn' onclick='childFollowToggle()'>Child Follow XY OFF</button>";
  html += "<br><br>";
  html += "<button onclick='childAlt(0.5)'>Child Alt +0.5m</button>";
  html += "<button onclick='childAlt(-0.5)'>Child Alt -0.5m</button>";
  html += "<button onclick='childMove(0.02,0)'>Child Forward +0.02m</button>";
  html += "<button onclick='childMove(-0.02,0)'>Child Back -0.02m</button>";
  html += "<button onclick='childMove(0,-0.02)'>Child Left -0.02m</button>";
  html += "<button onclick='childMove(0,0.02)'>Child Right +0.02m</button>";
  html += "<h2>Mother UAV</h2><table id='mother'></table>";
  html += "<h2>Child UAV</h2><table id='child'></table>";
  html += "<script>";
  html += "function takeoff(){fetch('/api/takeoff').catch(()=>{});}";
  html += "function arm(v){fetch('/api/arm?en='+(v?1:0)).catch(()=>{});}";
  html += "function land(){fetch('/api/land').catch(()=>{});}";
  html += "function childArm(){fetch('/api/child/arm').catch(()=>{});}";
  html += "function childTakeoff(){fetch('/api/child/takeoff').catch(()=>{});}";
  html += "function childLand(){fetch('/api/child/land').catch(()=>{});}";
  html += "function childMagnet(){fetch('/api/child/magnet').catch(()=>{});}";
  html += "function childFollowToggle(){";
  html += "const b=document.getElementById('followBtn');if(!b)return;";
  html += "const on=(b.dataset.on==='1');const next=on?0:1;";
  html += "fetch('/api/child/follow?en='+next).catch(()=>{});}";
  html += "function childAlt(d){fetch('/api/child/alt?d='+d).catch(()=>{});}";
  html += "function childMove(f,r){fetch('/api/child/offset?f='+f+'&r='+r).catch(()=>{});}";
  html += "function row(k,v){return `<tr><th>${k}</th><td>${v}</td></tr>`;}";
  html += "function fmt(n, d){return (n===null||n===undefined)?'-':Number(n).toFixed(d);}";
  html += "function fixName(f){";
  html += "switch(f){case 0:return 'NO_GPS';case 1:return 'NO_FIX';case 2:return '2D';case 3:return '3D';";
  html += "case 4:return 'DGPS';case 5:return 'RTK_FLOAT';case 6:return 'RTK_FIXED';case 7:return 'STATIC';";
  html += "case 8:return 'PPP';default:return 'UNK';}}";
  html += "function fmt3(a){if(!a||a.length<3)return '-';return fmt(a[0],2)+' , '+fmt(a[1],2)+' , '+fmt(a[2],2);}";  
  html += "function update(t, id){";
  html += "if(!t){document.getElementById(id).innerHTML='';return;}";
  html += "let s='';";
  html += "s+=row('Lat/Lon', fmt(t.lat,7)+', '+fmt(t.lon,7));";
  html += "s+=row('Alt/RelAlt (m)', fmt(t.alt,2)+' / '+fmt(t.relAlt,2));";
  html += "s+=row('Vel (m/s)', fmt(t.vx,2)+' , '+fmt(t.vy,2)+' , '+fmt(t.vz,2));";
  html += "s+=row('Ground/Climb (m/s)', fmt(t.gnd,2)+' / '+fmt(t.climb,2));";
  html += "s+=row('Heading (deg)', fmt(t.hdg,1));";
  html += "s+=row('Attitude (deg)', fmt(t.roll,1)+' / '+fmt(t.pitch,1)+' / '+fmt(t.yaw,1));";
  html += "s+=row('GPS Fix/Sats', t.fix+' ('+fixName(t.fix)+') / '+t.sats);";
  html += "s+=row('HDOP/VDOP', fmt(t.hdop,2)+' / '+fmt(t.vdop,2));";
  html += "s+=row('GPS2 Fix/Sats', t.gps2Fix+' ('+fixName(t.gps2Fix)+') / '+t.gps2Sats);";
  html += "s+=row('GPS2 HDOP/VDOP', fmt(t.gps2Hdop,2)+' / '+fmt(t.gps2Vdop,2));";
  html += "s+=row('IMU Acc (m/s^2)', fmt3(t.acc));";
  html += "s+=row('IMU Gyro (rad/s)', fmt3(t.gyro));";
  html += "s+=row('Battery (V, A, %)', fmt(t.batV,2)+' , '+fmt(t.batA,2)+' , '+fmt(t.batRem,0));";
  html += "s+=row('IMU Temp (C)', fmt(t.temp,1));";
  html += "s+=row('Magnet', t.magnet? 'ON':'OFF');";
  html += "s+=row('Age (ms)', t.ageMs);";
  html += "document.getElementById(id).innerHTML=s;}";
  html += "function updateMagnetBtn(t){";
  html += "const b=document.getElementById('magnetBtn');";
  html += "if(!b||!t){return;}";
  html += "const on=!!t.magnet;";
  html += "b.textContent=on?'Child Magnet ON':'Child Magnet OFF';";
  html += "b.style.background=on?'#2e7d32':'#444';";
  html += "b.style.color='#fff';";
  html += "}";
  html += "function updateFollowBtn(en){";
  html += "const b=document.getElementById('followBtn');if(!b)return;";
  html += "b.dataset.on=en?'1':'0';";
  html += "b.textContent=en?'Child Follow XY ON':'Child Follow XY OFF';";
  html += "b.style.background=en?'#1565c0':'#444';";
  html += "b.style.color='#fff';";
  html += "}";
  html += "function updateFollowInfo(j){";
  html += "const b=document.getElementById('followBtn');if(!b)return;";
  html += "const info=' alt='+j.follow_alt.toFixed(2)+'m fwd='+j.follow_fwd.toFixed(2)+'m right='+j.follow_right.toFixed(2)+'m';";
  html += "b.title=info;";
  html += "}";
  html += "async function tick(){";
  html += "try{const r=await fetch('/api/status');const j=await r.json();";
  html += "update(j.mother,'mother');update(j.child,'child');updateMagnetBtn(j.child);updateFollowBtn(j.follow);updateFollowInfo(j);";
  html += "}catch(e){console.log(e);}setTimeout(tick,500);}tick();";
  html += "</script></body></html>";
  return html;
}

void webSetTelemetrySources(const TelemetryData *local, const TelemetryData *remote) {
  s_local = local;
  s_remote = remote;
}

void webSetTakeoffCallback(void (*cb)()) {
  s_takeoff_cb = cb;
}

void webSetArmCallback(void (*cb)(bool arm)) {
  s_arm_cb = cb;
}

void webSetLandCallback(void (*cb)()) {
  s_land_cb = cb;
}

void webSetChildArmCallback(void (*cb)()) {
  s_child_arm_cb = cb;
}

void webSetChildTakeoffCallback(void (*cb)()) {
  s_child_takeoff_cb = cb;
}

void webSetChildLandCallback(void (*cb)()) {
  s_child_land_cb = cb;
}

void webSetChildMagnetToggleCallback(void (*cb)()) {
  s_child_magnet_cb = cb;
}

void webSetChildFollowCallback(void (*cb)(bool enable)) {
  s_child_follow_cb = cb;
}

void webSetChildAltAdjustCallback(void (*cb)(float delta_m)) {
  s_child_alt_cb = cb;
}

void webSetChildOffsetAdjustCallback(void (*cb)(float forward_m, float right_m)) {
  s_child_offset_cb = cb;
}

void webSetFollowStatus(float alt_m, float forward_m, float right_m) {
  s_follow_alt_m = alt_m;
  s_follow_fwd_m = forward_m;
  s_follow_right_m = right_m;
}

void webSetup() {
  server.on("/", []() { server.send(200, "text/html", htmlPage()); });
  server.on("/api/arm", []() {
    String en = server.arg("en");
    bool arm = (en == "1");
    if (s_arm_cb) {
      s_arm_cb(arm);
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/land", []() {
    if (s_land_cb) {
      s_land_cb();
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/child/arm", []() {
    if (s_child_arm_cb) {
      s_child_arm_cb();
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/child/takeoff", []() {
    if (s_child_takeoff_cb) {
      s_child_takeoff_cb();
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/child/land", []() {
    if (s_child_land_cb) {
      s_child_land_cb();
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/child/magnet", []() {
    if (s_child_magnet_cb) {
      s_child_magnet_cb();
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/child/follow", []() {
    String en = server.arg("en");
    bool enable = (en == "1");
    s_child_follow_enabled = enable;
    if (s_child_follow_cb) {
      s_child_follow_cb(enable);
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/child/alt", []() {
    float d = server.arg("d").toFloat();
    if (s_child_alt_cb) {
      s_child_alt_cb(d);
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/child/offset", []() {
    float f = server.arg("f").toFloat();
    float r = server.arg("r").toFloat();
    if (s_child_offset_cb) {
      s_child_offset_cb(f, r);
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/takeoff", []() {
    if (s_takeoff_cb) {
      s_takeoff_cb();
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/status", []() {
    String out = "{";
    if (s_local) {
      appendTelemetryJson(out, "mother", *s_local);
    } else {
      out += "\"mother\":{}";
    }
    out += ",";
    if (s_remote) {
      appendTelemetryJson(out, "child", *s_remote);
    } else {
      out += "\"child\":{}";
    }
    out += ",\"follow\":" + String(s_child_follow_enabled ? 1 : 0);
    out += ",\"follow_alt\":" + String(s_follow_alt_m, 2);
    out += ",\"follow_fwd\":" + String(s_follow_fwd_m, 2);
    out += ",\"follow_right\":" + String(s_follow_right_m, 2);
    out += "}";
    server.send(200, "application/json", out);
  });
  server.begin();
}

void webLoop() {
  server.handleClient();
}

