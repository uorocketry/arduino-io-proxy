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

void sendLoadCellState(int val)
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

    int32_t val = 0;

    cli();  // Disable interrupts

    for (int i = 23; i >= 0; i--)
    {
        PORT_CLOCK |= BIT_CLOCK;                // Clock high
        NOP; NOP; NOP; NOP; NOP;  NOP;  // Delay
        PORT_CLOCK &= ~BIT_CLOCK;               // Clock low

        val |= ((int32_t)(PIN_OUT & BIT_OUT)) << i;
    }

    // Clock one more time for channel A, gain 128
    PORT_CLOCK |= BIT_CLOCK;                // Clock high
    NOP; NOP; NOP; NOP; NOP;  NOP;  // Delay
    PORT_CLOCK &= ~BIT_CLOCK;               // Clock low

    sei(); // Enable interrupts

    sendLoadCellState(val);
}