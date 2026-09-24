#include "telemetry.h"

Telemetry mother;
Telemetry child;

const char *fixTypeName(uint8_t fixType) {
  switch (fixType) {
    case 0: return "NO_GPS";
    case 1: return "NO_FIX";
    case 2: return "2D_FIX";
    case 3: return "3D_FIX";
    case 4: return "DGPS";
    case 5: return "RTK_FLOAT";
    case 6: return "RTK_FIXED";
    default: return "UNKNOWN";
  }
}

uint32_t telemetryAgeMs(const Telemetry &t) {
  if (!t.valid) return 0xFFFFFFFF;
  return millis() - t.lastMs;
}

