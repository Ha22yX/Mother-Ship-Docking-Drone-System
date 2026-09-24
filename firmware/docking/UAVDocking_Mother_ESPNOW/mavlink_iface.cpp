#include "mavlink_iface.h"

static HardwareSerial *s_fc = nullptr;
static TelemetryData s_telemetry;
static uint32_t s_rx_bytes = 0;
static uint32_t s_msg_count = 0;
static uint32_t s_stx_v2 = 0;
static uint32_t s_stx_v1 = 0;

static const uint8_t SYS_ID = 255;  // GCS ID
static const uint8_t COMP_ID = 190; // MAV_COMP_ID_GCS
static const uint8_t TARGET_SYS = 1;
static const uint8_t TARGET_COMP = 1;

static void sendMavlink(const mavlink_message_t &msg) {
  if (!s_fc) {
    return;
  }
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  s_fc->write(buf, len);
}

static float safe_div100(uint16_t v) {
  if (v == 0xFFFF) {
    return 0.0f;
  }
  return v / 100.0f;
}

void mavlinkInit(HardwareSerial &serial) {
  s_fc = &serial;
  telemetryReset(s_telemetry);
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
      {MAVLINK_MSG_ID_ATTITUDE, 5.0f},
      {MAVLINK_MSG_ID_HIGHRES_IMU, 5.0f},
      {MAVLINK_MSG_ID_VFR_HUD, 2.0f},
      {MAVLINK_MSG_ID_BATTERY_STATUS, 1.0f},
      {MAVLINK_MSG_ID_SYS_STATUS, 1.0f},
      {MAVLINK_MSG_ID_SCALED_IMU2, 1.0f},
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

void mavlinkSendTakeoff(float rel_alt_m) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      SYS_ID, COMP_ID, &msg,
      TARGET_SYS, TARGET_COMP,
      MAV_CMD_NAV_TAKEOFF,
      0,
      0, 0, 0, 0,
      0, 0,
      rel_alt_m);
  sendMavlink(msg);
}

void mavlinkSendArmDisarm(bool arm) {
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

void mavlinkSendTakeoffMode() {
  // PX4 main mode AUTO (4) + sub mode TAKEOFF (2)
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      SYS_ID, COMP_ID, &msg,
      TARGET_SYS, TARGET_COMP,
      MAV_CMD_DO_SET_MODE,
      0,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
      4,  // PX4_CUSTOM_MAIN_MODE_AUTO
      2,  // PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF
      0, 0, 0, 0);
  sendMavlink(msg);
}

void mavlinkSendLandMode() {
  // Prefer explicit LAND command; also request AUTO.LAND mode for PX4.
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      SYS_ID, COMP_ID, &msg,
      TARGET_SYS, TARGET_COMP,
      MAV_CMD_DO_SET_MODE,
      0,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
      4,  // PX4_CUSTOM_MAIN_MODE_AUTO
      6,  // PX4_CUSTOM_SUB_MODE_AUTO_LAND
      0, 0, 0, 0);
  sendMavlink(msg);

  mavlink_msg_command_long_pack(
      SYS_ID, COMP_ID, &msg,
      TARGET_SYS, TARGET_COMP,
      MAV_CMD_NAV_LAND,
      0,
      0, 0, 0, 0,
      0, 0,
      0);
  sendMavlink(msg);
}

void mavlinkSendPositionMode() {
  // PX4 main mode POSCTL (1)
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      SYS_ID, COMP_ID, &msg,
      TARGET_SYS, TARGET_COMP,
      MAV_CMD_DO_SET_MODE,
      0,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
      1,  // PX4_CUSTOM_MAIN_MODE_POSCTL
      0,
      0, 0, 0, 0);
  sendMavlink(msg);
}

const TelemetryData &mavlinkGetTelemetry() {
  return s_telemetry;
}

uint32_t mavlinkGetRxBytes() {
  return s_rx_bytes;
}

uint32_t mavlinkGetMsgCount() {
  return s_msg_count;
}

uint32_t mavlinkGetStxV2() {
  return s_stx_v2;
}

uint32_t mavlinkGetStxV1() {
  return s_stx_v1;
}

void mavlinkPoll() {
  if (!s_fc) {
    return;
  }
  static mavlink_message_t msg;
  static mavlink_status_t status;
  while (s_fc->available()) {
    uint8_t c = s_fc->read();
    s_rx_bytes++;
    if (c == 0xFD) {
      s_stx_v2++;
    } else if (c == 0xFE) {
      s_stx_v1++;
    }
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      s_msg_count++;
      uint32_t now = millis();
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
          mavlink_gps_raw_int_t gps;
          mavlink_msg_gps_raw_int_decode(&msg, &gps);
          s_telemetry.lat_e7 = gps.lat;
          s_telemetry.lon_e7 = gps.lon;
          s_telemetry.alt_m = gps.alt / 1000.0f;
          s_telemetry.hdop = safe_div100(gps.eph);
          s_telemetry.vdop = safe_div100(gps.epv);
          s_telemetry.fix_type = gps.fix_type;
          s_telemetry.sats = gps.satellites_visible;
          s_telemetry.last_update_ms = now;
          break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
          mavlink_global_position_int_t gp;
          mavlink_msg_global_position_int_decode(&msg, &gp);
          s_telemetry.lat_e7 = gp.lat;
          s_telemetry.lon_e7 = gp.lon;
          s_telemetry.alt_m = gp.alt / 1000.0f;
          s_telemetry.rel_alt_m = gp.relative_alt / 1000.0f;
          s_telemetry.vx_mps = gp.vx / 100.0f;
          s_telemetry.vy_mps = gp.vy / 100.0f;
          s_telemetry.vz_mps = gp.vz / 100.0f;
          if (gp.hdg != 0xFFFF) {
            s_telemetry.heading_deg = gp.hdg / 100.0f;
          }
          s_telemetry.last_update_ms = now;
          break;
        }
        case MAVLINK_MSG_ID_VFR_HUD: {
          mavlink_vfr_hud_t hud;
          mavlink_msg_vfr_hud_decode(&msg, &hud);
          s_telemetry.groundspeed_mps = hud.groundspeed;
          s_telemetry.climb_mps = hud.climb;
          s_telemetry.heading_deg = hud.heading;
          s_telemetry.last_update_ms = now;
          break;
        }
        case MAVLINK_MSG_ID_ATTITUDE: {
          mavlink_attitude_t att;
          mavlink_msg_attitude_decode(&msg, &att);
          s_telemetry.roll_deg = att.roll * 57.2957795f;
          s_telemetry.pitch_deg = att.pitch * 57.2957795f;
          s_telemetry.yaw_deg = att.yaw * 57.2957795f;
          s_telemetry.last_update_ms = now;
          break;
        }
        case MAVLINK_MSG_ID_HIGHRES_IMU: {
          mavlink_highres_imu_t imu;
          mavlink_msg_highres_imu_decode(&msg, &imu);
          s_telemetry.imu_acc_x = imu.xacc;
          s_telemetry.imu_acc_y = imu.yacc;
          s_telemetry.imu_acc_z = imu.zacc;
          s_telemetry.imu_gyro_x = imu.xgyro;
          s_telemetry.imu_gyro_y = imu.ygyro;
          s_telemetry.imu_gyro_z = imu.zgyro;
          s_telemetry.last_update_ms = now;
          break;
        }
        case MAVLINK_MSG_ID_BATTERY_STATUS: {
          mavlink_battery_status_t bat;
          mavlink_msg_battery_status_decode(&msg, &bat);
          if (bat.voltages[0] != 0xFFFF) {
            s_telemetry.battery_v = bat.voltages[0] / 1000.0f;
          }
          if (bat.current_battery != -1) {
            s_telemetry.battery_a = bat.current_battery / 100.0f;
          }
          if (bat.battery_remaining != -1) {
            s_telemetry.battery_remaining = bat.battery_remaining;
          }
          s_telemetry.last_update_ms = now;
          break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS: {
          mavlink_sys_status_t sys;
          mavlink_msg_sys_status_decode(&msg, &sys);
          if (sys.voltage_battery != UINT16_MAX) {
            s_telemetry.battery_v = sys.voltage_battery / 1000.0f;
          }
          if (sys.current_battery != INT16_MAX) {
            s_telemetry.battery_a = sys.current_battery / 100.0f;
          }
          if (sys.battery_remaining != -1) {
            s_telemetry.battery_remaining = sys.battery_remaining;
          }
          s_telemetry.last_update_ms = now;
          break;
        }
        case MAVLINK_MSG_ID_SCALED_IMU2: {
          mavlink_scaled_imu2_t imu;
          mavlink_msg_scaled_imu2_decode(&msg, &imu);
          s_telemetry.imu_temp_c = imu.temperature / 100.0f;
          s_telemetry.last_update_ms = now;
          break;
        }
        case MAVLINK_MSG_ID_GPS2_RAW: {
          mavlink_gps2_raw_t gps2;
          mavlink_msg_gps2_raw_decode(&msg, &gps2);
          s_telemetry.gps2_fix = gps2.fix_type;
          s_telemetry.gps2_sats = gps2.satellites_visible;
          s_telemetry.gps2_hdop = safe_div100(gps2.eph);
          s_telemetry.gps2_vdop = safe_div100(gps2.epv);
          s_telemetry.last_update_ms = now;
          break;
        }
        default:
          break;
      }
    }
  }
}

