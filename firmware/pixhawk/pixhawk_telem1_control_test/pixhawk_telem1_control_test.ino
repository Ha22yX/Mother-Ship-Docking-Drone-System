#include <Arduino.h>
#include <MAVLink.h>

// Pixhawk TELEM1:
// Pin 2 (TX) -> ESP32-S3 IO10 (RX)
// Pin 3 (RX) -> ESP32-S3 IO11 (TX)
static const int PIN_RX = 10;
static const int PIN_TX = 11;

HardwareSerial FC(1);

static const uint8_t SYS_ID = 255;  // GCS ID
static const uint8_t COMP_ID = 190; // MAV_COMP_ID_GCS
static const uint8_t TARGET_SYS = 1;
static const uint8_t TARGET_COMP = 1;

static bool heartbeatEnabled = true;
static uint32_t lastHeartbeat = 0;

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

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  arm        - send ARM");
  Serial.println("  disarm     - send DISARM");
  Serial.println("  hb on/off  - enable/disable heartbeat");
  Serial.println("  help       - show this help");
}

String readLineFromUsb() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        String out = line;
        line = "";
        return out;
      }
    } else {
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

void setup() {
  Serial.begin(115200);
  delay(300);
  FC.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);

  Serial.println("ESP32-S3 <-> Pixhawk TELEM1 control test");
  printHelp();
}

void loop() {
  // Send heartbeat at 1 Hz
  if (heartbeatEnabled && (millis() - lastHeartbeat) > 1000) {
    lastHeartbeat = millis();
    sendHeartbeat();
    Serial.println("[TX] HEARTBEAT");
  }

  // Handle USB commands
  String cmd = readLineFromUsb();
  if (cmd.length() > 0) {
    cmd.trim();
    cmd.toLowerCase();
    if (cmd == "arm") {
      sendArmDisarm(true);
      Serial.println("[TX] ARM");
    } else if (cmd == "disarm") {
      sendArmDisarm(false);
      Serial.println("[TX] DISARM");
    } else if (cmd == "hb on") {
      heartbeatEnabled = true;
      Serial.println("[CFG] heartbeat on");
    } else if (cmd == "hb off") {
      heartbeatEnabled = false;
      Serial.println("[CFG] heartbeat off");
    } else if (cmd == "help") {
      printHelp();
    } else {
      Serial.println("Unknown command. Type 'help'.");
    }
  }

  // Parse incoming MAVLink and show COMMAND_ACK
  static mavlink_message_t msg;
  static mavlink_status_t status;
  while (FC.available()) {
    uint8_t c = FC.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&msg, &ack);
        Serial.print("[ACK] cmd=");
        Serial.print(ack.command);
        Serial.print(" result=");
        Serial.println(ack.result);
      } else if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
        mavlink_gps_raw_int_t gps;
        mavlink_msg_gps_raw_int_decode(&msg, &gps);
        Serial.print("[GPS_RAW_INT] lat=");
        Serial.print(gps.lat / 1e7, 7);
        Serial.print(" lon=");
        Serial.print(gps.lon / 1e7, 7);
        Serial.print(" alt=");
        Serial.print(gps.alt / 1000.0, 2);
        Serial.print("m sats=");
        Serial.print(gps.satellites_visible);
        Serial.print(" fix=");
        Serial.println(gps.fix_type);
      } else if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t gp;
        mavlink_msg_global_position_int_decode(&msg, &gp);
        Serial.print("[GLOBAL_POS] lat=");
        Serial.print(gp.lat / 1e7, 7);
        Serial.print(" lon=");
        Serial.print(gp.lon / 1e7, 7);
        Serial.print(" alt=");
        Serial.print(gp.alt / 1000.0, 2);
        Serial.print("m rel=");
        Serial.print(gp.relative_alt / 1000.0, 2);
        Serial.print("m vx=");
        Serial.print(gp.vx / 100.0, 2);
        Serial.print(" vy=");
        Serial.print(gp.vy / 100.0, 2);
        Serial.print(" vz=");
        Serial.println(gp.vz / 100.0, 2);
      }
    }
  }
}

