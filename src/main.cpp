#include "digitalOutputMessage.h"
#include "logging.h"
#include "servoOutputMessage.h"
#include "utils.h"
#include <Arduino.h>
#include <ArduinoComm.pb.h>
#include <main.h>
#include <pb_decode.h>

COBSPacketSerial cobsPacketSerial;
long lastOutputStateSend = 0;

void setup()
{
    cobsPacketSerial.begin(57600);

    cobsPacketSerial.setPacketHandler(&onPacketReceived);

    sendEventMessage(RocketryProto_EventTypes_RESET);
}

void loop()
{
    cobsPacketSerial.update();

    // Send outpu state each second
    if (millis() - lastOutputStateSend > 1000)
    {
        sendServoOutputState();
        sendDigitalOutputState();
        lastOutputStateSend = millis();
    }
}

void (*resetFunc)(void) = 0;

void onPacketReceived(const uint8_t *buffer, size_t size)
{
    if (size == 0)
        return;

    RocketryProto_ArduinoIn message = RocketryProto_ArduinoIn_init_zero;

    pb_istream_t stream = pb_istream_from_buffer(buffer, size);

    if (!pb_decode(&stream, RocketryProto_ArduinoIn_fields, &message))
    {
        sendErrorMessage(RocketryProto_ErrorTypes_DECODE_ERROR);
        return;
    }

    switch (message.which_data)
    {
    case RocketryProto_ArduinoIn_servoOutputInit_tag:
        initServoOutput(message.data.servoOutputInit);
        break;
    case RocketryProto_ArduinoIn_servoOutputControl_tag:
        controlServoOutput(message.data.servoOutputControl);
        break;
    case RocketryProto_ArduinoIn_digitalOutputInit_tag:
        initDigitalOutput(message.data.digitalOutputInit);
        break;
    case RocketryProto_ArduinoIn_digitalOutputControl_tag:
        controlDigitalOutput(message.data.digitalOutputControl);
        break;
    case RocketryProto_ArduinoIn_reset_tag:
        resetFunc();
        break;
    default:
        sendErrorMessage(RocketryProto_ErrorTypes_UNKNOWN_MESSAGE);
    }
}