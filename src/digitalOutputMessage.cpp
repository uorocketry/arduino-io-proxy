#include "digitalOutputMessage.h"
#include "Arduino.h"
#include "ArduinoComm.pb.h"
#include "main.h"
#include "pb_encode.h"
#include "utils.h"
#include <logging.h>

// Servo Information
constexpr int maxDigitalPins = 20;
DigitalOutputInfo digitalPins[maxDigitalPins];
uint16_t digitalCount = 0;

DigitalOutputInfo *findDigital(int pin)
{
    for (uint16_t i = 0; i < digitalCount; i++)
    {
        DigitalOutputInfo &digital = digitalPins[i];
        if (digital.pin == pin)
        {
            return &digital;
        }
    }

    return nullptr;
}

void controlDigital(uint8_t pin, bool activate)
{
    DigitalOutputInfo *digital = findDigital(pin);

    if (digital == nullptr)
    {
        sendErrorMessage(RocketryProto_ErrorTypes_PIN_NOT_INITIALIZED, pin);
        return;
    }

    sendEventMessage(RocketryProto_EventTypes_DIGITAL_OUTPUT_CONTROL, pin);

    digitalWrite(pin, activate);
    digital->activated = activate;
}

void initDigitalOutput(const RocketryProto_DigitalOutputInit &message)
{
    uint8_t pin = message.pin;

    if (findDigital(pin) == nullptr)
    {
        // Ignore if we don't have any more free servos
        if (maxDigitalPins == digitalCount)
        {
            return;
        }

        pinMode(pin, OUTPUT);

        digitalPins[digitalCount].pin = pin;
        digitalPins[digitalCount].activated = false;
        digitalCount++;

        sendEventMessage(RocketryProto_EventTypes_DIGITAL_OUTPUT_INIT, pin);
    }
}

void controlDigitalOutput(const RocketryProto_DigitalOutputControl &message)
{
    controlDigital(message.pin, message.activate);
}

void sendDigitalOutputState()
{
    for (uint16_t i = 0; i < digitalCount; i++)
    {
        const DigitalOutputInfo &info = digitalPins[i];

        RocketryProto_ArduinoOut msg = RocketryProto_ArduinoOut_init_zero;
        msg.which_data = RocketryProto_ArduinoOut_digitalOutputState_tag;

        RocketryProto_DigitalOutputState &state = msg.data.digitalOutputState;
        state.pin = info.pin;
        state.activated = info.activated;

        pb_ostream_t sizestream = {nullptr};
        pb_encode(&sizestream, RocketryProto_ArduinoOut_fields, &msg);

        auto *buf = new uint8_t[sizestream.bytes_written];

        pb_ostream_t outputStream = pb_ostream_from_buffer(buf, sizestream.bytes_written);
        pb_encode(&outputStream, RocketryProto_ArduinoOut_fields, &msg);

        cobsPacketSerial.send(buf, sizestream.bytes_written);

        delete[] buf;
    }
}