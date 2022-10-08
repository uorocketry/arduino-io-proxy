#include "loadCell.h"
#include "ArduinoComm.pb.h"
#include "main.h"
#include "pb_encode.h"
#include <Arduino.h>

constexpr uint8_t PDSCK = A5;
constexpr uint8_t DOUT = A4;

#define PORT_CLOCK PORTB
#define BIT_CLOCK B00100000

#define PIN_OUT PINB
#define BIT_OUT B00010000

// Single clock cycle delay
#define NOP __asm__ __volatile__("nop\n\t")

void sendLoadCellState(uint32_t val)
{
    RocketryProto_ArduinoOut msg = RocketryProto_ArduinoOut_init_zero;
    msg.which_data = RocketryProto_ArduinoOut_loadCellState_tag;

    RocketryProto_LoadCellState &state = msg.data.loadCellState;
    state.value = val;

    pb_ostream_t sizestream = {nullptr};
    pb_encode(&sizestream, RocketryProto_ArduinoOut_fields, &msg);

    auto *buf = new uint8_t[sizestream.bytes_written];

    pb_ostream_t outputStream = pb_ostream_from_buffer(buf, sizestream.bytes_written);
    pb_encode(&outputStream, RocketryProto_ArduinoOut_fields, &msg);

    cobsPacketSerial.send(buf, sizestream.bytes_written);

    delete[] buf;
}

void loadCellInit() {
    pinMode(PDSCK, OUTPUT);
    digitalWrite(PDSCK, LOW);

    pinMode(DOUT, INPUT);
}

void sendLoadCellState() {
    if (digitalRead(DOUT) == HIGH) {
        return;
    }

    uint32_t val = 0;

    cli();

    for (int i = 23; i >= 0; i--)
    {
        digitalWrite(PDSCK, HIGH);
        digitalWrite(PDSCK, LOW);

        val |= ((int32_t) digitalRead(DOUT)) << i;
    }

    // Clock one more time for channel A, gain 128
    digitalWrite(PDSCK, HIGH);
    digitalWrite(PDSCK, LOW);

    if (val & 0x800000) {
        val |= 0xFF000000;
    }

    sei();

    sendLoadCellState(val);
}