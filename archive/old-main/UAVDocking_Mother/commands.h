#pragma once

#include <Arduino.h>

void cmdMotherHover(float relAlt);
void cmdMotherArm(bool arm);
void cmdMotherOffboard();
void cmdMotherRtl();
void cmdChildDock();
void cmdChildRtl();
void cmdChildAlt(float delta);
void cmdChildMove(float dn, float de);
String buildChildCmdResponse();

