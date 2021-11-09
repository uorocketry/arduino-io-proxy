#include "servoMessage.h"
#include "Arduino.h"
#include "utils.h"
#include <logging.h>

// Servo Information
constexpr int maxServoCount = 12;
ServoInfo servos[maxServoCount];
uint16_t servoCount = 0;

ServoInfo *findServo(int pin)
{
    for (uint16_t i = 0; i < servoCount; i++)
    {
        ServoInfo &servo = servos[i];
        if (servo.pin == pin)
        {
            return &servo;
        }
    }

    return nullptr;
}

void controlServo(uint8_t pin, int position)
{
    ServoInfo *servo = findServo(pin);

    if (servo != nullptr)
    {
        sendEventMessage(RocketryProto_EventTypes_SERVO_CONTROL, pin);

        servo->servo.write(position);
    }
    else
    {
        sendErrorMessage(RocketryProto_ErrorTypes_PIN_NOT_INITIALIZED, pin);
    }
}

void initServo(const RocketryProto_ServoInit &servoInit)
{
    ServoInfo *servo = findServo(servoInit.pin);

    if (servo == nullptr)
    {
        // Ignore if we don't have any more free servos
        if (maxServoCount == servoCount)
        {
            return;
        }

        servos[servoCount].pin = static_cast<uint8_t>(servoInit.pin);
        servos[servoCount].safePosition = static_cast<int>(servoInit.safePosition);
        servos[servoCount].servo.attach(servoInit.pin);

        servoCount++;

        sendEventMessage(RocketryProto_EventTypes_SERVO_INIT, servoInit.pin);
    }
}

void controlServo(const RocketryProto_ServoControl &servoControl)
{
    controlServo(servoControl.pin, servoControl.position);
}