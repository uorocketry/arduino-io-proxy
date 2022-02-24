#pragma once

#include "ArduinoComm.pb.h"
#include "Servo.h"

struct ServoOutputInfo
{
    Servo servo;
    uint8_t pin{};
    int safePosition{};
    int currentPosition{};
};

void initServoOutput(const RocketryProto_ServoOutputInit &message);
void controlServoOutput(const RocketryProto_ServoOutputControl &message);
void sendServoOutputState();
