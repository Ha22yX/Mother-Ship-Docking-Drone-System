#pragma once

#include <Arduino.h>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif
#include <MAVLink.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

void mavlinkSetup();
void mavlinkLoop();
void mavlinkSendHeartbeat();
void mavlinkRequestIntervals();
void mavlinkSendSetpointGlobalRelAlt(double lat, double lon, float relAlt);
void mavlinkSendSetpointGlobalRelAltYaw(double lat, double lon, float relAlt, float yawRad);
void mavlinkSendRtcm(const uint8_t *data, size_t len);

