#include <Arduino.h>
#include <MAVLink.h>

// --- UWB (Tag) wiring on ESP32-S3 ---
// GPIO4  -> UWB RX (Pin3)
// GPIO5  <- UWB TX (Pin4)
static const int UWB_RX = 5;
static const int UWB_TX = 4;
static const uint32_t UWB_BAUD = 115200;

// --- Pixhawk TELEM1 wiring ---
// Pin 2 (TX) -> ESP32-S3 IO10 (RX)
// Pin 3 (RX) -> ESP32-S3 IO11 (TX)
static const int FC_RX = 10;
static const int FC_TX = 11;
static const uint32_t FC_BAUD = 57600; // set to your TELEM1 baud

HardwareSerial UWB(1);
HardwareSerial FC(2);

// Anchor layout: square side 35.5 inches
static const float INCH_TO_M = 0.0254f;
static const float SIDE_M = 35.5f * INCH_TO_M;
static const float HALF = SIDE_M * 0.5f;

// Anchor coordinates (meters), midpoints of 4 edges
// Physical order clockwise: 1 -> 2 -> 3 -> 4
// Mapping to UWB IDs (an0..an3):
//   an0 = Anchor 1 (right midpoint)
//   an1 = Anchor 2 (bottom midpoint)
//   an2 = Anchor 3 (left midpoint)
//   an3 = Anchor 4 (top midpoint)
static const float AX[4] = {+HALF, 0.0f, -HALF, 0.0f};
static const float AY[4] = {0.0f, -HALF, 0.0f, +HALF};

// Filter
static const int FILTER_N = 3;
float distHist[4][FILTER_N] = {{0}};
int distCount[4] = {0, 0, 0, 0};
int distIndex[4] = {0, 0, 0, 0};
float distM[4] = {-1, -1, -1, -1};

// Control parameters
static float kp_xy = 0.6f;
static float kp_z = 0.6f;
static float max_v = 0.6f; // m/s
static float desired_z = 1.0f; // meters above anchor plane

static bool followEnabled = false;
static uint32_t lastUwbMs = 0;
static uint32_t lastSetpointMs = 0;

// MAVLink IDs
static const uint8_t SYS_ID = 255;
static const uint8_t COMP_ID = 190;
static const uint8_t TARGET_SYS = 1;
static const uint8_t TARGET_COMP = 1;

float medianN(float *v, int n) {
  float a[5];
  for (int i = 0; i < n; ++i) a[i] = v[i];
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (a[j] < a[i]) {
        float t = a[i];
        a[i] = a[j];
        a[j] = t;
      }
    }
  }
  return a[n / 2];
}

float filterDistance(int id, float v) {
  distHist[id][distIndex[id]] = v;
  distIndex[id] = (distIndex[id] + 1) % FILTER_N;
  if (distCount[id] < FILTER_N) distCount[id]++;
  return medianN(distHist[id], distCount[id]);
}

void sendAT(const char *cmd) {
  UWB.print(cmd);
  UWB.print("\r\n");
  Serial.print("[CMD] ");
  Serial.println(cmd);
}

bool parseDistanceLine(const String &line, int &id, float &m) {
  if (!line.startsWith("an")) return false;
  int colon = line.indexOf(':');
  int mpos = line.indexOf('m');
  if (colon < 0 || mpos < 0) return false;
  id = line.substring(2, colon).toInt();
  m = line.substring(colon + 1, mpos).toFloat();
  if (id < 0 || id > 3) return false;
  return true;
}

bool solve2D(float &x, float &y) {
  if (distM[0] <= 0 || distM[1] <= 0 || distM[2] <= 0 || distM[3] <= 0)
    return false;

  float x0 = AX[0], y0 = AY[0], d0 = distM[0];
  float A11 = 0, A12 = 0, A22 = 0;
  float B1 = 0, B2 = 0;
  for (int i = 1; i < 4; ++i) {
    float xi = AX[i], yi = AY[i], di = distM[i];
    float ai = 2.0f * (xi - x0);
    float bi = 2.0f * (yi - y0);
    float ci = (xi * xi + yi * yi - di * di) -
               (x0 * x0 + y0 * y0 - d0 * d0);
    A11 += ai * ai;
    A12 += ai * bi;
    A22 += bi * bi;
    B1 += ai * ci;
    B2 += bi * ci;
  }
  float det = A11 * A22 - A12 * A12;
  if (fabs(det) < 1e-6f) return false;
  x = (B1 * A22 - B2 * A12) / det;
  y = (A11 * B2 - A12 * B1) / det;
  return true;
}

bool solve3DWithMirror(float &x, float &y, float &z) {
  if (!solve2D(x, y)) return false;
  float zsum = 0.0f;
  int zcount = 0;
  for (int i = 0; i < 4; ++i) {
    float dx = x - AX[i];
    float dy = y - AY[i];
    float z2 = distM[i] * distM[i] - dx * dx - dy * dy;
    if (z2 > 0) {
      zsum += sqrtf(z2);
      zcount++;
    }
  }
  if (zcount == 0) return false;
  z = zsum / zcount; // positive mirror
  return true;
}

void sendMavlink(const mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  FC.write(buf, len);
}

void sendHeartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
      SYS_ID, COMP_ID, &msg,
      MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
      MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
  sendMavlink(msg);
}

void sendArmDisarm(bool arm) {
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

void sendSetModeOffboard() {
  mavlink_message_t msg;
  mavlink_msg_set_mode_pack(
      SYS_ID, COMP_ID, &msg,
      TARGET_SYS, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
      PX4_CUSTOM_MAIN_MODE_OFFBOARD);
  sendMavlink(msg);
}

void sendVelocityBodyNED(float vx, float vy, float vz) {
  // Use velocity only in body frame
  uint16_t type_mask = (1 << 0) | (1 << 1) | (1 << 2) | // ignore position
                       (1 << 6) | (1 << 7) | (1 << 8) | // ignore accel
                       (1 << 9) | (1 << 10) | (1 << 11); // ignore yaw/yaw_rate
  mavlink_message_t msg;
  mavlink_msg_set_position_target_local_ned_pack(
      SYS_ID, COMP_ID, &msg,
      millis(), TARGET_SYS, TARGET_COMP,
      MAV_FRAME_BODY_NED, type_mask,
      0, 0, 0,
      vx, vy, vz,
      0, 0, 0,
      0, 0);
  sendMavlink(msg);
}

String readLineUwb() {
  static String line;
  while (UWB.available()) {
    char c = (char)UWB.read();
    if (c == '\r' || c == '\n') {
      if (line.length() > 0) {
        String out = line;
        line = "";
        return out;
      }
    } else if (c >= 32 && c <= 126) {
      line += c;
      if (line.length() > 120) {
        String out = line;
        line = "";
        return out;
      }
    }
  }
  return "";
}

String readLineUSB() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (line.length() > 0) {
        String out = line;
        line = "";
        return out;
      }
    } else {
      line += c;
      if (line.length() > 80) {
        String out = line;
        line = "";
        return out;
      }
    }
  }
  return "";
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  start        - arm + offboard + follow");
  Serial.println("  stop         - stop follow (zero velocity)");
  Serial.println("  kp <v>       - set kp_xy");
  Serial.println("  kz <v>       - set kp_z");
  Serial.println("  vmax <v>     - set max velocity");
  Serial.println("  z <v>        - set desired z (meters)");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  UWB.begin(UWB_BAUD, SERIAL_8N1, UWB_RX, UWB_TX);
  FC.begin(FC_BAUD, SERIAL_8N1, FC_RX, FC_TX);

  Serial.println("ESP32 UWB follow -> Pixhawk (Offboard)");
  printHelp();

  sendAT("AT+anchor_tag=0");
  delay(100);
  sendAT("AT+interval=5");
  delay(100);
  sendAT("AT+switchdis=1");
}

void loop() {
  // Read UWB distances
  String line = readLineUwb();
  if (line.length() > 0) {
    int id;
    float m;
    if (parseDistanceLine(line, id, m)) {
      distM[id] = filterDistance(id, m);
      lastUwbMs = millis();
    }
  }

  // USB commands
  String cmd = readLineUSB();
  if (cmd.length() > 0) {
    cmd.trim();
    if (cmd == "start") {
      followEnabled = true;
      sendArmDisarm(true);
      sendSetModeOffboard();
      Serial.println("[CFG] follow start");
    } else if (cmd == "stop") {
      followEnabled = false;
      sendVelocityBodyNED(0, 0, 0);
      Serial.println("[CFG] follow stop");
    } else if (cmd.startsWith("kp ")) {
      kp_xy = cmd.substring(3).toFloat();
      Serial.print("[CFG] kp=");
      Serial.println(kp_xy);
    } else if (cmd.startsWith("kz ")) {
      kp_z = cmd.substring(3).toFloat();
      Serial.print("[CFG] kz=");
      Serial.println(kp_z);
    } else if (cmd.startsWith("vmax ")) {
      max_v = cmd.substring(5).toFloat();
      Serial.print("[CFG] vmax=");
      Serial.println(max_v);
    } else if (cmd.startsWith("z ")) {
      desired_z = cmd.substring(2).toFloat();
      Serial.print("[CFG] z=");
      Serial.println(desired_z);
    } else if (cmd == "help") {
      printHelp();
    }
  }

  // Send heartbeat at 1 Hz
  static uint32_t lastHb = 0;
  if (millis() - lastHb > 1000) {
    lastHb = millis();
    sendHeartbeat();
  }

  // Follow control at 10 Hz
  if (followEnabled && (millis() - lastSetpointMs) > 100) {
    lastSetpointMs = millis();

    float x, y, z;
    if (solve3DWithMirror(x, y, z)) {
      // Map to body NED: body right = +x, body forward = +y
      float vx = kp_xy * y;  // forward
      float vy = kp_xy * x;  // right
      float vz = -kp_z * (z - desired_z); // down

      if (vx > max_v) vx = max_v;
      if (vx < -max_v) vx = -max_v;
      if (vy > max_v) vy = max_v;
      if (vy < -max_v) vy = -max_v;
      if (vz > max_v) vz = max_v;
      if (vz < -max_v) vz = -max_v;

      // If data is stale, stop
      if (millis() - lastUwbMs > 1000) {
        vx = vy = vz = 0;
      }
      sendVelocityBodyNED(vx, vy, vz);
    } else {
      sendVelocityBodyNED(0, 0, 0);
    }
  }
}

