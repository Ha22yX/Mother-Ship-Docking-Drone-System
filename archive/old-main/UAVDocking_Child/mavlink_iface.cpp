#include "mavlink_iface.h"

#include "config.h"
#include "telemetry.h"

static HardwareSerial FC(1);

static void sendMavlink(const mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  FC.write(buf, len);
}

void mavlinkSetup() {
  FC.begin(FC_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
}

void mavlinkSendHeartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
      SYS_ID, COMP_ID, &msg,
      MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
      MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
  sendMavlink(msg);
}

void mavlinkRequestIntervals() {
  mavlink_message_t msg;
  const struct Req {
    uint16_t msgId;
    float hz;
  } reqs[] = {
      {MAVLINK_MSG_ID_GPS_RAW_INT, 2.0f},
      {MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5.0f},
      {MAVLINK_MSG_ID_HIGHRES_IMU, 5.0f},
      {MAVLINK_MSG_ID_ATTITUDE, 5.0f},
      {MAVLINK_MSG_ID_VFR_HUD, 2.0f},
      {MAVLINK_MSG_ID_SYS_STATUS, 1.0f},
      {MAVLINK_MSG_ID_GPS2_RAW, 1.0f},
  };
  for (const auto &r : reqs) {
    const float intervalUs = (r.hz > 0) ? (1e6f / r.hz) : -1.0f;
    mavlink_msg_command_long_pack(
        SYS_ID, COMP_ID, &msg,
        TARGET_SYS, TARGET_COMP,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        r.msgId,
        intervalUs,
        0, 0, 0, 0, 0);
    sendMavlink(msg);
  }
}

void mavlinkSendSetpointGlobalRelAlt(double lat, double lon, float relAlt) {
  mavlink_message_t msg;
  const int32_t latInt = (int32_t)(lat * 1e7);
  const int32_t lonInt = (int32_t)(lon * 1e7);
  mavlink_msg_set_position_target_global_int_pack(
      SYS_ID, COMP_ID, &msg,
      millis(),
      TARGET_SYS, TARGET_COMP,
      MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
      SETPOINT_TYPE_MASK,
      latInt, lonInt, relAlt,
      0, 0, 0,
      0, 0, 0,
      0, 0);
  sendMavlink(msg);
}

void mavlinkSendSetpointGlobalRelAltYaw(double lat, double lon, float relAlt, float yawRad) {
  mavlink_message_t msg;
  const int32_t latInt = (int32_t)(lat * 1e7);
  const int32_t lonInt = (int32_t)(lon * 1e7);
  mavlink_msg_set_position_target_global_int_pack(
      SYS_ID, COMP_ID, &msg,
      millis(),
      TARGET_SYS, TARGET_COMP,
      MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
      SETPOINT_TYPE_MASK_POS_YAW,
      latInt, lonInt, relAlt,
      0, 0, 0,
      0, 0, 0,
      yawRad, 0);
  sendMavlink(msg);
}

void mavlinkSendRtcm(const uint8_t *data, size_t len) {
  if (!data || len == 0) return;
  static uint8_t seq = 0;
  size_t offset = 0;
  const size_t chunkSize = 180;
  while (offset < len) {
    size_t n = len - offset;
    if (n > chunkSize) n = chunkSize;
    uint8_t flags = (uint8_t)(seq & 0x1F);
    if (len > chunkSize) {
      uint8_t fragId = (uint8_t)((offset / chunkSize) & 0x07);
      flags |= (uint8_t)(fragId << 5);
    }
    mavlink_message_t msg;
    uint8_t payload[180] = {0};
    memcpy(payload, data + offset, n);
    mavlink_msg_gps_rtcm_data_pack(
        SYS_ID, COMP_ID, &msg,
        flags,
        (uint8_t)n,
        payload);
    sendMavlink(msg);
    offset += n;
    seq++;
  }
}

void mavlinkLoop() {
  static mavlink_message_t msg;
  static mavlink_status_t status;
  while (FC.available()) {
    uint8_t c = FC.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
        mavlink_gps_raw_int_t gps;
        mavlink_msg_gps_raw_int_decode(&msg, &gps);
        self.lat = gps.lat / 1e7;
        self.lon = gps.lon / 1e7;
        self.altAmsl = gps.alt / 1000.0f;
        self.fixType = gps.fix_type;
        self.sats = gps.satellites_visible;
        self.eph = gps.eph / 100.0f;
        self.epv = gps.epv / 100.0f;
        self.vel = gps.vel / 100.0f;
        self.cog = gps.cog / 100.0f;
        self.valid = true;
        self.lastMs = millis();
      } else if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t gp;
        mavlink_msg_global_position_int_decode(&msg, &gp);
        self.lat = gp.lat / 1e7;
        self.lon = gp.lon / 1e7;
        self.altAmsl = gp.alt / 1000.0f;
        self.relAlt = gp.relative_alt / 1000.0f;
        self.vx = gp.vx / 100.0f;
        self.vy = gp.vy / 100.0f;
        self.vz = gp.vz / 100.0f;
        self.valid = true;
        self.lastMs = millis();
      } else if (msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
        mavlink_highres_imu_t imu;
        mavlink_msg_highres_imu_decode(&msg, &imu);
        self.ax = imu.xacc;
        self.ay = imu.yacc;
        self.az = imu.zacc;
        self.imuTemp = imu.temperature;
      } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(&msg, &att);
        self.yawRad = att.yaw;
      } else if (msg.msgid == MAVLINK_MSG_ID_VFR_HUD) {
        mavlink_vfr_hud_t vfr;
        mavlink_msg_vfr_hud_decode(&msg, &vfr);
        self.airspeed = vfr.airspeed;
        self.groundspeed = vfr.groundspeed;
        self.heading = vfr.heading;
        self.throttle = vfr.throttle;
        self.climb = vfr.climb;
      } else if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
        mavlink_sys_status_t sys;
        mavlink_msg_sys_status_decode(&msg, &sys);
        self.battVolt = sys.voltage_battery / 1000.0f;
        self.battCurrent = sys.current_battery / 100.0f;
        self.battRemaining = sys.battery_remaining;
      } else if (msg.msgid == MAVLINK_MSG_ID_GPS2_RAW) {
        mavlink_gps2_raw_t gps2;
        mavlink_msg_gps2_raw_decode(&msg, &gps2);
        self.gps2Fix = gps2.fix_type;
        self.gps2Sats = gps2.satellites_visible;
        self.gps2Eph = gps2.eph / 100.0f;
        self.gps2Epv = gps2.epv / 100.0f;
      }
    }
  }
}

