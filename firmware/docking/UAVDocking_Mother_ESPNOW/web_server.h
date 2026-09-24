#pragma once

#include "telemetry.h"

void webSetup();
void webLoop();
void webSetTelemetrySources(const TelemetryData *local, const TelemetryData *remote);
void webSetTakeoffCallback(void (*cb)());
void webSetArmCallback(void (*cb)(bool arm));
void webSetLandCallback(void (*cb)());
void webSetChildArmCallback(void (*cb)());
void webSetChildTakeoffCallback(void (*cb)());
void webSetChildLandCallback(void (*cb)());
void webSetChildMagnetToggleCallback(void (*cb)());
void webSetChildFollowCallback(void (*cb)(bool enable));
void webSetChildAltAdjustCallback(void (*cb)(float delta_m));
void webSetChildOffsetAdjustCallback(void (*cb)(float forward_m, float right_m));
void webSetFollowStatus(float alt_m, float forward_m, float right_m);

