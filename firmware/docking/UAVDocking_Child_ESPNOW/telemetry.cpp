#include "telemetry.h"

void telemetryReset(TelemetryData &t) {
  memset(&t, 0, sizeof(TelemetryData));
}

float telemetryLat(const TelemetryData &t) {
  return t.lat_e7 / 1e7f;
}

float telemetryLon(const TelemetryData &t) {
  return t.lon_e7 / 1e7f;
}

