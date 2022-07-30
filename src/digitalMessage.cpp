#include "digitalMessage.h"
#include "Arduino.h"
#include "ArduinoComm.pb.h"
#include "main.h"
#include "pb_encode.h"
#include "utils.h"
#include <logging.h>

// Servo Information
constexpr int maxDigitalPins = 20;
DigitalInfo digitalPins[maxDigitalPins];
uint16_t digitalCount = 0;

DigitalInfo *findDigital(int pin)
{
    for (uint16_t i = 0; i < digitalCount; i++)
    {
        DigitalInfo &digital = digitalPins[i];
        if (digital.pin == pin)
        {
            return &digital;
        }
    }

    return nullptr;
}

void controlDigital(uint8_t pin, bool activate)
{
    DigitalInfo *digital = findDigital(pin);

    if (digital == nullptr)
    {
        sendErrorMessage(RocketryProto_ErrorTypes_PIN_NOT_INITIALIZED, pin);
        return;
    }

    sendEventMessage(RocketryProto_EventTypes_DIGITAL_CONTROL, pin);

    digitalWrite(pin, activate);
    digital->activated = activate;
}

void initDigital(const RocketryProto_DigitalInit &digitalInit)
{
    uint8_t pin = digitalInit.pin;

    if (findDigital(pin) == nullptr)
    {
        // Ignore if we don't have any more free servos
        if (maxDigitalPins == digitalCount)
        {
            return;
        }

        pinMode(pin, OUTPUT);
        digitalWrite(pin, digitalInit.safeState);

        digitalPins[digitalCount].pin = pin;
        digitalPins[digitalCount].activated = digitalInit.safeState;
        digitalCount++;

        sendEventMessage(RocketryProto_EventTypes_DIGITAL_INIT, pin);
    }
}

void controlDigital(const RocketryProto_DigitalControl &digitalControl)
{
    controlDigital(digitalControl.pin, digitalControl.activate);
}

void sendDigitalState()
{
    for (uint16_t i = 0; i < digitalCount; i++)
    {
        const DigitalInfo &info = digitalPins[i];

        RocketryProto_ArduinoOut msg = RocketryProto_ArduinoOut_init_zero;
        msg.which_data = RocketryProto_ArduinoOut_digitalState_tag;

        RocketryProto_DigitalState &state = msg.data.digitalState;
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