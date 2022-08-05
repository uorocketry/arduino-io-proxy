#pragma once

#include "ArduinoComm.pb.h"

enum class StepperMotorDirection
{
    Forward,
    Reverse,
    Stopped
};

void initStepper();
void controlStepperMotor(const RocketryProto_StepperMotorControl &message);
void sendStepperMotorState();
void stepperMotorControlLoop();