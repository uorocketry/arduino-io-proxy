#pragma once

#include <ArduinoComm.pb.h>

void initAnalogInput(const RocketryProto_AnalogInputInit &message);
void sendAnalogInputState();
