#pragma once

#include "ArduinoComm.pb.h"
#include "Servo.h"

struct ServoInfo
{
    Servo servo;
    uint8_t pin;
    int safePosition;
    int currentPosition;
};

void initServo(const RocketryProto_ServoInit &message);
void controlServo(const RocketryProto_ServoControl &message);
void sendServoState();
