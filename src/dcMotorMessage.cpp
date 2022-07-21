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
    analogWrite(info.motorForwardPin, info.motorPower);
    analogWrite(info.motorReversePin, 0);
}

void reverse(DCMotorInfo &info)
{
    analogWrite(info.motorForwardPin, 0);
    analogWrite(info.motorReversePin, info.motorPower);
}

void stop(DCMotorInfo &info)
{
    analogWrite(info.motorForwardPin, 0);
    analogWrite(info.motorReversePin, 0);
}

void controlDCMotor(uint8_t forwardPin, uint8_t reversePin, int position)
{
    DCMotorInfo *motor = findDCMotor(forwardPin, reversePin);

    if (motor != nullptr)
    {
        sendEventMessage(RocketryProto_EventTypes_DC_MOTOR_CONTROL, forwardPin);

        motor->targetPosition = position;
        motor->active = true;

        if (motor->lastPosition < position) {
            forward(*motor);
            motor->direction = DCMotorDirection::Closing;
        } else if (motor->lastPosition == position) {
            stop(*motor);
        } else {
            reverse(*motor);
            motor->direction = DCMotorDirection::Opening;
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

        pinMode(dcMotors[motorCount].potentiometerPin, INPUT); // todo: do we want pull up

        pinMode(dcMotors[motorCount].motorForwardPin, INPUT);
        pinMode(dcMotors[motorCount].motorReversePin, INPUT);

        motorCount++;

        //todo: new message type
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
        if (dcMotors[i].active) {
            int position = analogRead(dcMotors[i].potentiometerPin);

            DCMotorDirection direction = dcMotors[i].direction;
            bool limitMin = digitalRead(dcMotors[i].limitSwitchMinPin);
            bool limitMax = digitalRead(dcMotors[i].limitSwitchMaxPin);

            if ((direction == DCMotorDirection::Opening && limitMin)
                || (direction == DCMotorDirection::Closing && limitMax)
                || (position < dcMotors[i].targetPosition && position > dcMotors[i].lastPosition)
                || (position > dcMotors[i].targetPosition && position < dcMotors[i].lastPosition))
            {
                dcMotors[i].active = false;
                stop(dcMotors[i]);
            }

            dcMotors[i].lastPosition = position;
        }
    }
}

void sendDCMotorState()
{
    for (uint16_t i = 0; i < motorCount; i++)
    {
        const DCMotorInfo &info = dcMotors[i];

        RocketryProto_ArduinoOut msg = RocketryProto_ArduinoOut_init_zero;
        msg.which_data = RocketryProto_ArduinoOut_servoState_tag;

        RocketryProto_DCMotorState &state = msg.data.dcMotorState;
        state.motorForwardPin = info.motorForwardPin;
        state.motorReversePin = info.motorReversePin;
        state.position = info.lastPosition;

        pb_ostream_t sizestream = {nullptr};
        pb_encode(&sizestream, RocketryProto_ArduinoOut_fields, &msg);

        auto *buf = new uint8_t[sizestream.bytes_written];

        pb_ostream_t outputStream = pb_ostream_from_buffer(buf, sizestream.bytes_written);
        pb_encode(&outputStream, RocketryProto_ArduinoOut_fields, &msg);

        cobsPacketSerial.send(buf, sizestream.bytes_written);

        delete[] buf;
    }
}