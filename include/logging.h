#pragma once
#include "Arduino.h"
#include <ArduinoComm.pb.h>

void sendEventMessage(RocketryProto_EventTypes type);
void sendEventMessage(RocketryProto_EventTypes type, int32_t data);
void sendErrorMessage(RocketryProto_ErrorTypes type);
void sendErrorMessage(RocketryProto_ErrorTypes type, int32_t data);