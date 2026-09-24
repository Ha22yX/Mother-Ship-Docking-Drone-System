#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>

// ESP32-S3 <-> OpenMV UART
// IO6 -> OpenMV P5 (RX)
// IO7 -> OpenMV P4 (TX)
#define OPENMV_RX 6
#define OPENMV_TX 7

// WiFi credentials
static const char *WIFI_SSID = "pronfs2";
static const char *WIFI_PASS = "ridiculous1";

// Web server
static const uint16_t HTTP_PORT = 80;
WebServer server(HTTP_PORT);

HardwareSerial OpenMVSerial(1);

// Latest AprilTag pose (camera -> tag)
struct TagPose {
  float tx;
  float ty;
  float tz;
  float rx;
  float ry;
  float rz;
  bool valid;
  uint32_t last_ms;
};

static TagPose latest = {0, 0, 0, 0, 0, 0, false, 0};

static String lineBuf;
static String usbBuf;

static IPAddress activeIp() {
  if (WiFi.status() == WL_CONNECTED) {
    return WiFi.localIP();
  }
  return WiFi.softAPIP();
}

static void printIpPort() {
  IPAddress ip = activeIp();
  Serial.print("IP: ");
  Serial.print(ip);
  Serial.print("  PORT: ");
  Serial.println(HTTP_PORT);
}

static bool parsePoseLine(const String &line, TagPose &out) {
  // Accepts key=value pairs like:
  // tx=..,ty=..,tz=..,rx=..,ry=..,rz=..
  // or rx_deg/ry_deg/rz_deg in degrees.
  // Fallback: CSV-like numbers in order tx,ty,tz,rx,ry,rz.
  bool hasKey = false;
  bool hasTx = false, hasTy = false, hasTz = false;
  bool hasRx = false, hasRy = false, hasRz = false;
  bool hasRxDeg = false, hasRyDeg = false, hasRzDeg = false;
  float rxDeg = 0, ryDeg = 0, rzDeg = 0;

  int start = 0;
  while (start < (int)line.length()) {
    int end = line.indexOf(',', start);
    if (end < 0) end = line.length();
    String token = line.substring(start, end);
    int eq = token.indexOf('=');
    if (eq > 0) {
      hasKey = true;
      String k = token.substring(0, eq);
      String v = token.substring(eq + 1);
      k.trim();
      v.trim();
      float fv = v.toFloat();
      if (k == "tx") { out.tx = fv; hasTx = true; }
      else if (k == "ty") { out.ty = fv; hasTy = true; }
      else if (k == "tz") { out.tz = fv; hasTz = true; }
      else if (k == "rx") { out.rx = fv; hasRx = true; }
      else if (k == "ry") { out.ry = fv; hasRy = true; }
      else if (k == "rz") { out.rz = fv; hasRz = true; }
      else if (k == "rx_deg") { rxDeg = fv; hasRxDeg = true; }
      else if (k == "ry_deg") { ryDeg = fv; hasRyDeg = true; }
      else if (k == "rz_deg") { rzDeg = fv; hasRzDeg = true; }
    }
    start = end + 1;
  }

  if (hasKey) {
    if ((!hasRx && hasRxDeg) || (!hasRy && hasRyDeg) || (!hasRz && hasRzDeg)) {
      const float kDeg = 0.01745329252f;
      if (!hasRx && hasRxDeg) out.rx = rxDeg * kDeg;
      if (!hasRy && hasRyDeg) out.ry = ryDeg * kDeg;
      if (!hasRz && hasRzDeg) out.rz = rzDeg * kDeg;
    }
    if (hasTx && hasTy && hasTz) {
      out.valid = true;
      out.last_ms = millis();
      return true;
    }
  }

  // Fallback: parse numeric CSV in order
  float vals[6];
  int found = 0;
  String num;
  bool inNum = false;

  for (size_t i = 0; i < line.length(); ++i) {
    char c = line[i];
    bool isNumChar = (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E';
    if (isNumChar) {
      num += c;
      inNum = true;
    } else {
      if (inNum) {
        if (found < 6) {
          vals[found++] = num.toFloat();
        }
        num = "";
        inNum = false;
      }
    }
  }
  if (inNum && found < 6) {
    vals[found++] = num.toFloat();
  }

  if (found >= 3) {
    out.tx = vals[0];
    out.ty = vals[1];
    out.tz = vals[2];
    out.rx = (found > 3) ? vals[3] : 0.0f;
    out.ry = (found > 4) ? vals[4] : 0.0f;
    out.rz = (found > 5) ? vals[5] : 0.0f;
    out.valid = true;
    out.last_ms = millis();
    return true;
  }
  return false;
}

static String htmlPage() {
  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>AprilTag 3D Viewer</title>";
  html += "<style>";
  html += "body{font-family:Arial;margin:0;background:#0b0b0b;color:#eee;}";
  html += "#wrap{display:flex;flex-direction:column;align-items:center;gap:12px;padding:12px;}";
  html += "#canvas{background:#111;border:1px solid #333;border-radius:8px;}";
  html += ".row{display:flex;gap:16px;flex-wrap:wrap;}";
  html += ".card{background:#141414;border:1px solid #2a2a2a;border-radius:8px;padding:10px;min-width:200px;}";
  html += ".title{font-weight:700;margin-bottom:6px;}";
  html += "</style></head><body>";
  html += "<div id='wrap'>";
  html += "<canvas id='canvas' width='640' height='420'></canvas>";
  html += "<div class='row'>";
  html += "<div class='card'><div class='title'>Camera Pos (x,y,z)</div><div id='pos'>-</div></div>";
  html += "<div class='card'><div class='title'>Tag Angles (rx,ry,rz)</div><div id='ang'>-</div></div>";
  html += "<div class='card'><div class='title'>Status</div><div id='stat'>-</div></div>";
  html += "</div></div>";
  html += "<script>";
  html += "const cv=document.getElementById('canvas');";
  html += "const ctx=cv.getContext('2d');";
  html += "const posEl=document.getElementById('pos');";
  html += "const angEl=document.getElementById('ang');";
  html += "const statEl=document.getElementById('stat');";
  html += "const cam={yaw:0.7,pitch:0.45,dist:2.2,drag:false,px:0,py:0};";
  html += "const POS_SCALE=1.0;";
  html += "const FLIP_X=false, FLIP_Y=true, FLIP_Z=true;";
  html += "const POS_EMA_ALPHA=0.2;";
  html += "const TAG_ON_WALL=true;";
  html += "let posEma=null;";
  html += "function camTransform(p){";
  html += "  let x=p.x, y=p.y, z=p.z;";
  html += "  const cy=Math.cos(cam.yaw), sy=Math.sin(cam.yaw);";
  html += "  let x1=cy*x+sy*z, z1=-sy*x+cy*z, y1=y;";
  html += "  const cp=Math.cos(cam.pitch), sp=Math.sin(cam.pitch);";
  html += "  let y2=cp*y1-sp*z1, z2=sp*y1+cp*z1, x2=x1;";
  html += "  z2+=cam.dist;";
  html += "  return {x:x2,y:y2,z:z2};";
  html += "}";
  html += "function proj(p){";
  html += "  const z=p.z<0.1?0.1:p.z;";
  html += "  const f=300/z;";
  html += "  return {x:cv.width/2+p.x*f,y:cv.height/2-p.y*f};";
  html += "}";
  html += "function mm(a,b){";
  html += "  return [";
  html += "    [a[0][0]*b[0][0]+a[0][1]*b[1][0]+a[0][2]*b[2][0],a[0][0]*b[0][1]+a[0][1]*b[1][1]+a[0][2]*b[2][1],a[0][0]*b[0][2]+a[0][1]*b[1][2]+a[0][2]*b[2][2]],";
  html += "    [a[1][0]*b[0][0]+a[1][1]*b[1][0]+a[1][2]*b[2][0],a[1][0]*b[0][1]+a[1][1]*b[1][1]+a[1][2]*b[2][1],a[1][0]*b[0][2]+a[1][1]*b[1][2]+a[1][2]*b[2][2]],";
  html += "    [a[2][0]*b[0][0]+a[2][1]*b[1][0]+a[2][2]*b[2][0],a[2][0]*b[0][1]+a[2][1]*b[1][1]+a[2][2]*b[2][1],a[2][0]*b[0][2]+a[2][1]*b[1][2]+a[2][2]*b[2][2]]";
  html += "  ];";
  html += "}";
  html += "function transpose3(R){";
  html += "  return [[R[0][0],R[1][0],R[2][0]],[R[0][1],R[1][1],R[2][1]],[R[0][2],R[1][2],R[2][2]]];";
  html += "}";
  html += "function applyR(R,v){";
  html += "  return [";
  html += "    R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],";
  html += "    R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],";
  html += "    R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]";
  html += "  ];";
  html += "}";
  html += "function applyFlipVec(v){";
  html += "  const fx=FLIP_X?-1:1, fy=FLIP_Y?-1:1, fz=FLIP_Z?-1:1;";
  html += "  return [v[0]*fx, v[1]*fy, v[2]*fz];";
  html += "}";
  html += "function applyFlipR(R){";
  html += "  const fx=FLIP_X?-1:1, fy=FLIP_Y?-1:1, fz=FLIP_Z?-1:1;";
  html += "  const F=[[fx,0,0],[0,fy,0],[0,0,fz]];";
  html += "  return mm(mm(F,R),F);";
  html += "}";
  html += "function rotMatrix(rx,ry,rz){";
  html += "  const cx=Math.cos(rx), sx=Math.sin(rx);";
  html += "  const cy=Math.cos(ry), sy=Math.sin(ry);";
  html += "  const cz=Math.cos(rz), sz=Math.sin(rz);";
  html += "  const Rx=[[1,0,0],[0,cx,-sx],[0,sx,cx]];";
  html += "  const Ry=[[cy,0,sy],[0,1,0],[-sy,0,cy]];";
  html += "  const Rz=[[cz,-sz,0],[sz,cz,0],[0,0,1]];";
  html += "  return mm(mm(Rz,Ry),Rx);";
  html += "}";
  html += "function drawLine(a,b,c){";
  html += "  const p1=proj(camTransform(a));";
  html += "  const p2=proj(camTransform(b));";
  html += "  ctx.strokeStyle=c; ctx.lineWidth=1.2;";
  html += "  ctx.beginPath(); ctx.moveTo(p1.x,p1.y); ctx.lineTo(p2.x,p2.y); ctx.stroke();";
  html += "}";
  html += "function drawAxes(){";
  html += "  drawLine({x:0,y:0,z:0},{x:0.4,y:0,z:0},'#ff5555');";
  html += "  drawLine({x:0,y:0,z:0},{x:0,y:0.4,z:0},'#55ff55');";
  html += "  drawLine({x:0,y:0,z:0},{x:0,y:0,z:0.4},'#5599ff');";
  html += "}";
  html += "function drawGrid(){";
  html += "  const n=6, step=0.1, col='#2a2a2a';";
  html += "  for(let i=-n;i<=n;i++){";
  html += "    drawLine({x:i*step,y:-n*step,z:0},{x:i*step,y:n*step,z:0},col);";
  html += "    drawLine({x:-n*step,y:i*step,z:0},{x:n*step,y:i*step,z:0},col);";
  html += "  }";
  html += "}";
  html += "function drawTagPlane(){";
  html += "  const hs=0.05;"; // 100mm tag -> 0.1m plane
  html += "  let pts=[];";
  html += "  if(TAG_ON_WALL){";
  html += "    pts=[[0,-hs,-hs],[0,hs,-hs],[0,hs,hs],[0,-hs,hs]];";
  html += "  }else{";
  html += "    pts=[[-hs,-hs,0],[hs,-hs,0],[hs,hs,0],[-hs,hs,0]];";
  html += "  }";
  html += "  const p2=pts.map(p=>proj(camTransform({x:p[0],y:p[1],z:p[2]})));";
  html += "  ctx.strokeStyle='#333'; ctx.lineWidth=1;";
  html += "  ctx.beginPath(); ctx.moveTo(p2[0].x,p2[0].y);";
  html += "  for(let i=1;i<p2.length;i++) ctx.lineTo(p2[i].x,p2[i].y);";
  html += "  ctx.closePath(); ctx.stroke();";
  html += "}";
  html += "function render(p){";
  html += "  ctx.clearRect(0,0,cv.width,cv.height);";
  html += "  ctx.fillStyle='#111'; ctx.fillRect(0,0,cv.width,cv.height);";
  html += "  drawGrid();";
  html += "  drawAxes();";
  html += "  drawTagPlane();";
  html += "  if(!p.valid) return;";
  html += "  const R_tag_in_cam=rotMatrix(p.rx,p.ry,p.rz);";
  html += "  let R_cam_in_tag=transpose3(R_tag_in_cam);";
  html += "  let camPos=applyR(R_cam_in_tag,[-p.tx,-p.ty,-p.tz]);";
  html += "  camPos=applyFlipVec(camPos);";
  html += "  camPos=[camPos[0]*POS_SCALE,camPos[1]*POS_SCALE,camPos[2]*POS_SCALE];";
  html += "  if(posEma===null){ posEma=camPos; }";
  html += "  else{";
  html += "    posEma=[";
  html += "      posEma[0]*(1-POS_EMA_ALPHA)+camPos[0]*POS_EMA_ALPHA,";
  html += "      posEma[1]*(1-POS_EMA_ALPHA)+camPos[1]*POS_EMA_ALPHA,";
  html += "      posEma[2]*(1-POS_EMA_ALPHA)+camPos[2]*POS_EMA_ALPHA";
  html += "    ];";
  html += "  }";
  html += "  camPos=posEma;";
  html += "  R_cam_in_tag=applyFlipR(R_cam_in_tag);";
  html += "  const ex=applyR(R_cam_in_tag,[0.4,0,0]);";
  html += "  const ey=applyR(R_cam_in_tag,[0,0.4,0]);";
  html += "  const ez=applyR(R_cam_in_tag,[0,0,0.4]);";
  html += "  drawLine({x:camPos[0],y:camPos[1],z:camPos[2]},{x:camPos[0]+ex[0],y:camPos[1]+ex[1],z:camPos[2]+ex[2]},'#ff8888');";
  html += "  drawLine({x:camPos[0],y:camPos[1],z:camPos[2]},{x:camPos[0]+ey[0],y:camPos[1]+ey[1],z:camPos[2]+ey[2]},'#88ff88');";
  html += "  drawLine({x:camPos[0],y:camPos[1],z:camPos[2]},{x:camPos[0]+ez[0],y:camPos[1]+ez[1],z:camPos[2]+ez[2]},'#88aaff');";
  html += "  const L=0.48, H=0.16;";
  html += "  const cornersCam=[[-H,-H,L],[H,-H,L],[H,H,L],[-H,H,L]];";
  html += "  const corners=cornersCam.map(c=>applyR(R_cam_in_tag,c)).map(c=>[camPos[0]+c[0],camPos[1]+c[1],camPos[2]+c[2]]);";
  html += "  const fr=[camPos,corners[0],camPos,corners[1],camPos,corners[2],camPos,corners[3],corners[0],corners[1],corners[1],corners[2],corners[2],corners[3],corners[3],corners[0]];";
  html += "  for(let i=0;i<fr.length;i+=2){";
  html += "    drawLine({x:fr[i][0],y:fr[i][1],z:fr[i][2]},{x:fr[i+1][0],y:fr[i+1][1],z:fr[i+1][2]},'#777');";
  html += "  }";
  html += "}";
  html += "async function tick(){";
  html += "  try{";
  html += "    const r=await fetch('/data');";
  html += "    const d=await r.json();";
  html += "    posEl.textContent=`${d.tx.toFixed(3)}, ${d.ty.toFixed(3)}, ${d.tz.toFixed(3)}`;";
  html += "    angEl.textContent=`${d.rx.toFixed(3)}, ${d.ry.toFixed(3)}, ${d.rz.toFixed(3)}`;";
  html += "    statEl.textContent=d.valid?`OK (${d.age_ms} ms)`:'NO DATA';";
  html += "    render(d);";
  html += "  }catch(e){ statEl.textContent='ERR'; }";
  html += "}";
  html += "cv.addEventListener('mousedown',e=>{cam.drag=true;cam.px=e.clientX;cam.py=e.clientY;});";
  html += "window.addEventListener('mouseup',()=>{cam.drag=false;});";
  html += "window.addEventListener('mousemove',e=>{";
  html += "  if(!cam.drag) return;";
  html += "  const dx=e.clientX-cam.px, dy=e.clientY-cam.py;";
  html += "  cam.px=e.clientX; cam.py=e.clientY;";
  html += "  cam.yaw+=dx*0.005; cam.pitch+=dy*0.005;";
  html += "  if(cam.pitch>1.4) cam.pitch=1.4; if(cam.pitch<-1.4) cam.pitch=-1.4;";
  html += "});";
  html += "cv.addEventListener('wheel',e=>{";
  html += "  e.preventDefault();";
  html += "  cam.dist+=e.deltaY*0.002;";
  html += "  if(cam.dist<0.6) cam.dist=0.6; if(cam.dist>6) cam.dist=6;";
  html += "},{passive:false});";
  html += "setInterval(tick,100); tick();";
  html += "</script></body></html>";
  return html;
}

static void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

static void handleData() {
  uint32_t age = latest.valid ? (millis() - latest.last_ms) : 0;
  String json = "{";
  json += "\"tx\":" + String(latest.tx, 4) + ",";
  json += "\"ty\":" + String(latest.ty, 4) + ",";
  json += "\"tz\":" + String(latest.tz, 4) + ",";
  json += "\"rx\":" + String(latest.rx, 4) + ",";
  json += "\"ry\":" + String(latest.ry, 4) + ",";
  json += "\"rz\":" + String(latest.rz, 4) + ",";
  json += "\"valid\":" + String(latest.valid ? "true" : "false") + ",";
  json += "\"age_ms\":" + String(age);
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("ESP32-S3 OpenMV AprilTag Web Viewer");

  OpenMVSerial.begin(19200, SERIAL_8N1, OPENMV_RX, OPENMV_TX);
  Serial.println("UART1 initialized for OpenMV.");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  const uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < 10000) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connect failed. Starting AP: AprilTag (open)");
    WiFi.mode(WIFI_AP);
    IPAddress apIP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, gateway, subnet);
    WiFi.softAP("AprilTag");
  }

  printIpPort();

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("HTTP server started.");
}

void loop() {
  server.handleClient();

  // USB Serial command: "ip"
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      usbBuf.trim();
      if (usbBuf.equalsIgnoreCase("ip")) {
        printIpPort();
      }
      usbBuf = "";
    } else {
      usbBuf += c;
    }
  }

  // Read OpenMV lines
  while (OpenMVSerial.available()) {
    char c = OpenMVSerial.read();
    if (c == '\n' || c == '\r') {
      if (lineBuf.length() > 0) {
        TagPose parsed = latest;
        if (parsePoseLine(lineBuf, parsed)) {
          latest = parsed;
        }
        lineBuf = "";
      }
    } else if (c >= 32 && c <= 126) {
      lineBuf += c;
      if (lineBuf.length() > 200) {
        lineBuf = "";
      }
    }
  }
}


