#pragma once
#include <ArduinoComm.pb.h>
#include "Arduino.h"

void sendEventMessage(RocketryProto_EventTypes type);
void sendEventMessage(RocketryProto_EventTypes type, int32_t data);
void sendErrorMessage(RocketryProto_ErrorTypes type);
void sendErrorMessage(RocketryProto_ErrorTypes type, int32_t data);