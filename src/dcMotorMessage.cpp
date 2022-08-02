#include "dcMotorMessage.h"
#include "Arduino.h"
#include "ArduinoComm.pb.h"
#include "main.h"
#include "utils.h"
#include <logging.h>
#include <pb_encode.h>

constexpr int maxMotorCount = 2;
DCMotorInfo dcMotors[maxMotorCount];
uint16_t motorCount = 0;

DCMotorInfo *findDCMotor(uint8_t forwardPin, uint8_t reversePin)
{
    for (uint16_t i = 0; i < motorCount; i++)
    {
        DCMotorInfo &dcMotor = dcMotors[i];
        if (dcMotor.motorForwardPin == forwardPin && dcMotor.motorReversePin == reversePin)
        {
            return &dcMotor;
        }
    }

    return nullptr;
}

void forward(DCMotorInfo &info)
{
    info.direction = DCMotorDirection::Forward;
    analogWrite(info.motorForwardPin, info.motorPower);
    analogWrite(info.motorReversePin, 0);
}

void reverse(DCMotorInfo &info)
{
    info.direction = DCMotorDirection::Reverse;
    analogWrite(info.motorForwardPin, 0);
    analogWrite(info.motorReversePin, info.motorPower);
}

void stop(DCMotorInfo &info)
{
    info.direction = DCMotorDirection::Stopped;
    analogWrite(info.motorForwardPin, 0);
    analogWrite(info.motorReversePin, 0);
}

void controlDCMotor(uint8_t forwardPin, uint8_t reversePin, int targetPosition)
{
    DCMotorInfo *motor = findDCMotor(forwardPin, reversePin);

    if (motor != nullptr)
    {
        sendEventMessage(RocketryProto_EventTypes_DC_MOTOR_CONTROL, forwardPin);

        motor->targetPosition = targetPosition;
        motor->active = true;

        if (motor->lastPosition < targetPosition)
        {
            forward(*motor);
        }
        else if (motor->lastPosition == targetPosition)
        {
            stop(*motor);
        }
        else
        {
            reverse(*motor);
        }
    }
    else
    {
        sendErrorMessage(RocketryProto_ErrorTypes_PIN_NOT_INITIALIZED, motor->motorForwardPin);
    }
}

void initDCMotor(const RocketryProto_DCMotorInit &motorInit)
{
    DCMotorInfo *motor = findDCMotor(motorInit.motorForwardPin, motorInit.motorReversePin);

    if (motor == nullptr)
    {
        // Ignore if we don't have any more free servos
        if (maxMotorCount == motorCount)
        {
            return;
        }

        dcMotors[motorCount].limitSwitchMinPin = motorInit.limitSwitchMinPin;
        dcMotors[motorCount].limitSwitchMaxPin = motorInit.limitSwitchMaxPin;
        dcMotors[motorCount].potentiometerPin = motorInit.potentiometerPin;
        dcMotors[motorCount].motorForwardPin = motorInit.motorForwardPin;
        dcMotors[motorCount].motorReversePin = motorInit.motorReversePin;
        dcMotors[motorCount].motorPower = motorInit.motorPower;

        pinMode(dcMotors[motorCount].limitSwitchMinPin, INPUT);
        pinMode(dcMotors[motorCount].limitSwitchMaxPin, INPUT);

        pinMode(dcMotors[motorCount].potentiometerPin, INPUT_PULLUP); // todo: do we want pull up

        pinMode(dcMotors[motorCount].motorForwardPin, OUTPUT);
        pinMode(dcMotors[motorCount].motorReversePin, OUTPUT);

        motorCount++;

        // todo: new message type
        sendEventMessage(RocketryProto_EventTypes_DC_MOTOR_INIT, motorInit.motorForwardPin);
    }
}

void controlDCMotor(const RocketryProto_DCMotorControl &dcMotorControl)
{
    controlDCMotor(dcMotorControl.pinForward, dcMotorControl.pinReverse, dcMotorControl.position);
}

void dcMotorControlLoop()
{
    for (uint16_t i = 0; i < motorCount; i++)
    {
        int position = analogRead(dcMotors[i].potentiometerPin);
        dcMotors[i].minLimitSwitch = digitalRead(dcMotors[i].limitSwitchMinPin);
        dcMotors[i].maxLimitSwitch = digitalRead(dcMotors[i].limitSwitchMaxPin);

        if (dcMotors[i].active)
        {
            DCMotorDirection direction = dcMotors[i].direction;

            if ((direction == DCMotorDirection::Reverse && dcMotors[i].minLimitSwitch) ||
                (direction == DCMotorDirection::Forward && dcMotors[i].maxLimitSwitch) ||
                (position < dcMotors[i].targetPosition && direction == DCMotorDirection::Reverse) ||
                (position > dcMotors[i].targetPosition && direction == DCMotorDirection::Forward))
            {
                dcMotors[i].active = false;
                stop(dcMotors[i]);
            }
        }

        dcMotors[i].lastPosition = position;
    }
}

void sendDCMotorState()
{
    for (uint16_t i = 0; i < motorCount; i++)
    {
        const DCMotorInfo &info = dcMotors[i];

        RocketryProto_ArduinoOut msg = RocketryProto_ArduinoOut_init_zero;
        msg.which_data = RocketryProto_ArduinoOut_dcMotorState_tag;

        RocketryProto_DCMotorState &state = msg.data.dcMotorState;
        state.motorForwardPin = info.motorForwardPin;
        state.motorReversePin = info.motorReversePin;
        state.position = info.lastPosition;
        state.minLimitSwitch = info.minLimitSwitch;
        state.maxLimitSwitch = info.maxLimitSwitch;
        switch (info.direction)
        {
        case DCMotorDirection::Stopped:
            state.direction = 0;
            break;
        case DCMotorDirection::Forward:
            state.direction = 1;
            break;
        case DCMotorDirection::Reverse:
            state.direction = -1;
            break;
        }

        pb_ostream_t sizestream = {nullptr};
        pb_encode(&sizestream, RocketryProto_ArduinoOut_fields, &msg);

        auto *buf = new uint8_t[sizestream.bytes_written];

        pb_ostream_t outputStream = pb_ostream_from_buffer(buf, sizestream.bytes_written);
        pb_encode(&outputStream, RocketryProto_ArduinoOut_fields, &msg);

        cobsPacketSerial.send(buf, sizestream.bytes_written);

        delete[] buf;
    }
}