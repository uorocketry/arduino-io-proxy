#include "logging.h"
#include "main.h"
#include <pb_encode.h>

void sendEventMessage(RocketryProto_EventTypes type) {
    sendEventMessage(type, 0);
}

void sendEventMessage(RocketryProto_EventTypes type, int32_t data) {
    RocketryProto_ArduinoOut msg = RocketryProto_ArduinoOut_init_zero;
    msg.which_data = RocketryProto_ArduinoOut_eventMessage_tag;

    RocketryProto_EventMessage &event = msg.data.eventMessage;
    event.type = type;
    event.data = data;

    pb_ostream_t sizestream = {nullptr};
    pb_encode(&sizestream, RocketryProto_ArduinoOut_fields, &msg);

    auto *buf = new uint8_t[sizestream.bytes_written];

    pb_ostream_t outputStream = pb_ostream_from_buffer(buf, sizestream.bytes_written);
    pb_encode(&outputStream, RocketryProto_ArduinoOut_fields, &msg);

    cobsPacketSerial.send(buf, sizestream.bytes_written);

    delete[] buf;
}

void sendErrorMessage(RocketryProto_ErrorTypes type) {
    sendErrorMessage(type, 0);
}

void sendErrorMessage(RocketryProto_ErrorTypes type, int32_t data) {
    RocketryProto_ArduinoOut msg = RocketryProto_ArduinoOut_init_zero;
    msg.which_data = RocketryProto_ArduinoOut_errorMessage_tag;

    RocketryProto_ErrorMessage &event = msg.data.errorMessage;
    event.type = type;
    event.data = data;

    pb_ostream_t sizestream = {nullptr};
    pb_encode(&sizestream, RocketryProto_ArduinoOut_fields, &msg);

    auto *buf = new uint8_t[sizestream.bytes_written];

    pb_ostream_t outputStream = pb_ostream_from_buffer(buf, sizestream.bytes_written);
    pb_encode(&outputStream, RocketryProto_ArduinoOut_fields, &msg);

    cobsPacketSerial.send(buf, sizestream.bytes_written);

    delete[] buf;
}