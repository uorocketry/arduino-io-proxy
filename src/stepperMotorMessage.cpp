#include "stepperMotorMessage.h"

#include <AccelStepper.h>
#include <pb_encode.h>
#include "main.h"

constexpr uint8_t STEP_PIN = A4;
constexpr uint8_t DIRECTION_PIN = A5;

constexpr uint8_t MAX_LIMIT = A1;
constexpr uint8_t MIN_LIMIT = A2;

StepperMotorDirection stepperDirection = StepperMotorDirection::Stopped;

AccelStepper stepper (1, STEP_PIN, DIRECTION_PIN);


void controlStepperMotor(int speed) {
    stepper.move(100000 * (speed > 0 ? 1 : -1));
    stepperDirection = speed > 0 ? StepperMotorDirection::Forward : StepperMotorDirection::Reverse;
}

void controlStepperMotor(const RocketryProto_StepperMotorControl &message) {
    controlStepperMotor(message.speed);
}

void initStepper() {
    pinMode(MAX_LIMIT, INPUT_PULLUP);
    pinMode(MIN_LIMIT, INPUT_PULLUP);

    stepper.setMaxSpeed(100.0);
    stepper.setAcceleration(100.0);
}

void stepperMotorControlLoop() {
    if ((stepperDirection == StepperMotorDirection::Forward && !digitalRead(MAX_LIMIT)) || (stepperDirection == StepperMotorDirection::Reverse && !digitalRead(MIN_LIMIT)))
    {
        stepperDirection = StepperMotorDirection::Stopped;
    }

    if (stepperDirection != StepperMotorDirection::Stopped) {
        stepper.run();
    }
}

void sendStepperState()
{
    RocketryProto_ArduinoOut msg = RocketryProto_ArduinoOut_init_zero;
    msg.which_data = RocketryProto_ArduinoOut_dcMotorState_tag;

    RocketryProto_StepperMotorState &state = msg.data.stepperMotorState;
    state.maxLimit = MAX_LIMIT;
    state.minLimit = MIN_LIMIT;
    state.directionPin = DIRECTION_PIN;
    state.stepPin = STEP_PIN;
    state.speed = stepper.speed();

    switch (stepperDirection)
    {
    case StepperMotorDirection::Stopped:
        state.direction = 0;
        break;
    case StepperMotorDirection::Forward:
        state.direction = 1;
        break;
    case StepperMotorDirection::Reverse:
        state.direction = -1;
        break;
    }

    pb_ostream_t sizestream = {nullptr};
    pb_encode(&sizestream, RocketryProto_ArduinoOut_fields, &msg);

    auto *buf = new uint8_t[sizestream.bytes_written];

    pb_ostream_t outputStream = pb_ostream_from_buffer(buf, sizestream.bytes_written);
    pb_encode(&outputStream, RocketryProto_ArduinoOut_fields, &msg);

    cobsPacketSerial.send(buf, sizestream.bytes_written);

    delete[] buf;
}
