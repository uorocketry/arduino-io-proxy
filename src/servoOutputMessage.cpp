#include "servoOutputMessage.h"
#include "Arduino.h"
#include "ArduinoComm.pb.h"
#include "main.h"
#include "utils.h"
#include <logging.h>
#include <pb_encode.h>

// Servo Information
constexpr int maxServoCount = 12;
ServoOutputInfo servos[maxServoCount];
uint16_t servoCount = 0;

ServoOutputInfo *findServo(int pin)
{
    for (uint16_t i = 0; i < servoCount; i++)
    {
        ServoOutputInfo &servo = servos[i];
        if (servo.pin == pin)
        {
            return &servo;
        }
    }

    return nullptr;
}

void controlServo(uint8_t pin, int position)
{
    ServoOutputInfo *servo = findServo(pin);

    if (servo != nullptr)
    {
        sendEventMessage(RocketryProto_EventTypes_SERVO_OUTPUT_CONTROL, pin);

        servo->servo.write(position);
        servo->currentPosition = position;
    }
    else
    {
        sendErrorMessage(RocketryProto_ErrorTypes_PIN_NOT_INITIALIZED, pin);
    }
}

void initServoOutput(const RocketryProto_ServoOutputInit &message)
{
    ServoOutputInfo *servo = findServo(message.pin);

    if (servo == nullptr)
    {
        // Ignore if we don't have any more free servos
        if (maxServoCount == servoCount)
        {
            sendErrorMessage(RocketryProto_ErrorTypes_OUT_OF_PINS);
            return;
        }

        servos[servoCount].pin = static_cast<uint8_t>(message.pin);
        servos[servoCount].safePosition = static_cast<int>(message.safePosition);
        servos[servoCount].servo.attach(message.pin);
        servos[servoCount].currentPosition = -1;

        servoCount++;

        sendEventMessage(RocketryProto_EventTypes_SERVO_OUTPUT_INIT, message.pin);
    }
}

void controlServoOutput(const RocketryProto_ServoOutputControl &message)
{
    controlServo(message.pin, message.position);
}

void sendServoOutputState()
{
    for (uint16_t i = 0; i < servoCount; i++)
    {
        const ServoOutputInfo &info = servos[i];

        RocketryProto_ArduinoOut msg = RocketryProto_ArduinoOut_init_zero;
        msg.which_data = RocketryProto_ArduinoOut_servoOutputState_tag;

        RocketryProto_ServoOutputState &state = msg.data.servoOutputState;
        state.pin = info.pin;
        state.position = info.currentPosition;

        pb_ostream_t sizestream = {nullptr};
        pb_encode(&sizestream, RocketryProto_ArduinoOut_fields, &msg);

        auto *buf = new uint8_t[sizestream.bytes_written];

        pb_ostream_t outputStream = pb_ostream_from_buffer(buf, sizestream.bytes_written);
        pb_encode(&outputStream, RocketryProto_ArduinoOut_fields, &msg);

        cobsPacketSerial.send(buf, sizestream.bytes_written);

        delete[] buf;
    }
}