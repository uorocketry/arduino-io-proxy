#pragma once
#include "ArduinoComm.pb.h"
#include "Servo.h"
#include <Arduino.h>
#include <PacketSerial.h>

extern COBSPacketSerial cobsPacketSerial;

void onPacketReceived(const uint8_t *buffer, size_t size);