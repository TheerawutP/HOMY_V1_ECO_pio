#ifndef ELEVATOR_STORAGE_H
#define ELEVATOR_STORAGE_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <FS.h>
#include "elevatorTypes.h"

extern volatile uint8_t POS;
extern bool btwFloor;
extern bool emergency;
extern uint8_t MAX_FLOOR;
extern uint8_t lastTarget;
extern uint32_t FloorToFloor_MS;
extern direction_t lastDir;
extern TRANSIT transit;
extern state_t moving_state;

extern const char* STATUS_FILE;

void saveStatus();
void loadStatus();

#endif