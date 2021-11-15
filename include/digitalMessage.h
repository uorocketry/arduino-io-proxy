#pragma once

#include "ArduinoComm.pb.h"

struct DigitalInfo
{
    uint8_t pin;
    bool activated;
};

void initDigital(const RocketryProto_DigitalInit &message);
void controlDigital(const RocketryProto_DigitalControl &message);
void sendDigitalState();