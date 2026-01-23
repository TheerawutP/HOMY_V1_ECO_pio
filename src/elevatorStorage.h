#ifndef ELEVATOR_STORAGE_H
#define ELEVATOR_STORAGE_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <FS.h>
#include <Preferences.h>

extern Preferences preferences; 

extern volatile uint8_t POS;
extern bool btwFloor;




extern const char* STATUS_FILE;

void saveStatus();
void loadStatus();

#endif