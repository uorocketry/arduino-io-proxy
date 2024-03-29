#include "dcMotorMessage.h"
#include "digitalMessage.h"
#include "loadCell.h"
#include "logging.h"
#include "servoMessage.h"
#include "utils.h"
#include <Arduino.h>
#include <ArduinoComm.pb.h>
#include <main.h>
#include <pb_decode.h>

COBSPacketSerial cobsPacketSerial;
long lastStateSend = 0;

void setup()
{
    cobsPacketSerial.begin(57600);

    cobsPacketSerial.setPacketHandler(&onPacketReceived);

    loadCellInit();

    sendEventMessage(RocketryProto_EventTypes_RESET);
}

void loop()
{
    cobsPacketSerial.update();

    // Send state each second
    if (millis() - lastStateSend > 50)
    {
        sendServoState();
        sendDigitalState();
        sendDCMotorState();
        sendLoadCellState();
        lastStateSend = millis();
    }

    dcMotorControlLoop();
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
    case RocketryProto_ArduinoIn_servoInit_tag:
        initServo(message.data.servoInit);
        break;
    case RocketryProto_ArduinoIn_servoControl_tag:
        controlServo(message.data.servoControl);
        break;
    case RocketryProto_ArduinoIn_digitalInit_tag:
        initDigital(message.data.digitalInit);
        break;
    case RocketryProto_ArduinoIn_digitalControl_tag:
        controlDigital(message.data.digitalControl);
        break;
    case RocketryProto_ArduinoIn_dcMotorInit_tag:
        initDCMotor(message.data.dcMotorInit);
        break;
    case RocketryProto_ArduinoIn_dcMotorControl_tag:
        controlDCMotor(message.data.dcMotorControl);
        break;
    case RocketryProto_ArduinoIn_reset_tag:
        resetFunc();
        break;
    default:
        sendErrorMessage(RocketryProto_ErrorTypes_UNKNOWN_MESSAGE);
    }
}