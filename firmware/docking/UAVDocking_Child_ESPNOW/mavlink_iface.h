#pragma once

#include <Arduino.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <MAVLink.h>
#pragma GCC diagnostic pop

#include "telemetry.h"

void mavlinkInit(HardwareSerial &serial);
void mavlinkPoll();
const TelemetryData &mavlinkGetTelemetry();
uint32_t mavlinkGetRxBytes();
uint32_t mavlinkGetMsgCount();
uint32_t mavlinkGetStxV2();
uint32_t mavlinkGetStxV1();
void mavlinkSendHeartbeat();
void mavlinkRequestIntervals();
void mavlinkSendTakeoff(float rel_alt_m);
void mavlinkSendArmDisarm(bool arm);
void mavlinkSendTakeoffMode();
void mavlinkSendLandMode();
void mavlinkSendPositionMode();
void mavlinkSendSetpointGlobalRelAlt(int32_t lat_e7, int32_t lon_e7, float rel_alt_m);
void mavlinkSendSetpointGlobalRelAltYaw(int32_t lat_e7, int32_t lon_e7, float rel_alt_m, float yaw_deg);

