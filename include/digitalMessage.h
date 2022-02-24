#pragma once

#include "ArduinoComm.pb.h"

struct DigitalInfo
{
    uint8_t pin;
    bool activated;
};

void initDigitalOutput(const RocketryProto_DigitalOutputInit &message);
void controlDigitalOutput(const RocketryProto_DigitalOutputControl &message);
void sendDigitalOutputState();