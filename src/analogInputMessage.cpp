#include "analogInputMessage.h"
#include "Arduino.h"
#include <logging.h>
#include <main.h>
#include <pb_encode.h>

struct AnalogInputInfo {
    uint8_t pin;
    uint32_t updateIntervalMilli;
    uint32_t lastUpdate;
};

constexpr int maxAnalogPins = 20;
AnalogInputInfo analogPins[maxAnalogPins];
uint16_t analogPinCount = 0;

AnalogInputInfo *findPin(int pin)
{
    for (uint16_t i = 0; i < analogPinCount; i++)
    {
        AnalogInputInfo &analog = analogPins[i];
        if (analog.pin == pin)
        {
            return &analog;
        }
    }

    return nullptr;
}



void initAnalogInput(const RocketryProto_AnalogInputInit &message)
{
    uint8_t pin = message.pin;
    bool pullUp = message.hasPullUp;
    uint32_t interval = message.updateIntervalMilli;

    if (findPin(pin) == nullptr)
    {
        // Ignore if we don't have any more free pins
        if (maxAnalogPins == analogPinCount)
        {
            sendErrorMessage(RocketryProto_ErrorTypes_OUT_OF_PINS);
            return;
        }

        if (pullUp) {
            pinMode(pin, INPUT_PULLUP);
        } else {
            pinMode(pin, INPUT);
        }

        analogPins[analogPinCount].pin = pin;
        analogPins[analogPinCount].updateIntervalMilli = interval;
        analogPins[analogPinCount].lastUpdate = 0;
        analogPinCount++;

        sendEventMessage(RocketryProto_EventTypes_ANALOG_INPUT_INIT, pin);
    }
}

void sendAnalogInputState()
{
    for (uint16_t i = 0; i < analogPinCount; i++)
    {
        const AnalogInputInfo &info = analogPins[i];
        if (millis() - info.lastUpdate < info.updateIntervalMilli) {
            return;
        }

        RocketryProto_ArduinoOut msg = RocketryProto_ArduinoOut_init_zero;
        msg.which_data = RocketryProto_ArduinoOut_analogInputState_tag;

        RocketryProto_AnalogInputState &state = msg.data.analogInputState;
        state.pin = info.pin;
        state.value = analogRead(info.pin);

        pb_ostream_t sizestream = {nullptr};
        pb_encode(&sizestream, RocketryProto_ArduinoOut_fields, &msg);

        auto *buf = new uint8_t[sizestream.bytes_written];

        pb_ostream_t outputStream = pb_ostream_from_buffer(buf, sizestream.bytes_written);
        pb_encode(&outputStream, RocketryProto_ArduinoOut_fields, &msg);

        cobsPacketSerial.send(buf, sizestream.bytes_written);

        delete[] buf;
    }
}