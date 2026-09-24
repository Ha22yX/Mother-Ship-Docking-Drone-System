#pragma once

#include <Arduino.h>

void netSetup();
void netEnsureConnected();
void netPushStatus();
bool netPollCommand(String &cmd, uint32_t &seq, double &lat, double &lon, float &relAlt, float &yawDeg);

