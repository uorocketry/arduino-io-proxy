#pragma once

#include "ArduinoComm.pb.h"

enum DCMotorDirection
{
    Forward,
    Reverse,
    Stopped
};

struct DCMotorInfo
{
    uint8_t limitSwitchMinPin;
    uint8_t limitSwitchMaxPin;
    uint8_t potentiometerPin;
    uint8_t motorForwardPin;
    uint8_t motorReversePin;
    uint8_t motorPower;

    bool active = false;
    int targetPosition = 0;
    int lastPosition = 0;
    DCMotorDirection direction = DCMotorDirection::Stopped;
    bool minLimitSwitch = 0;
    bool maxLimitSwitch = false;
};

void initDCMotor(const RocketryProto_DCMotorInit &message);
void controlDCMotor(const RocketryProto_DCMotorControl &message);
void sendDCMotorState();
void dcMotorControlLoop();