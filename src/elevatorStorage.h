#ifndef ELEVATOR_STORAGE_H
#define ELEVATOR_STORAGE_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <FS.h>

extern volatile uint8_t POS;
extern bool btwFloor;

const char* STATUS_FILE = "/curr_status.txt";


void saveStatus() {
  StaticJsonDocument<64> doc;
  

  doc["p"] = POS;        
  doc["b"] = btwFloor;  

  File file = SPIFFS.open(STATUS_FILE, FILE_WRITE);
  if (!file) {
    Serial.println("Error: failed to save file");
    return;
  }

  if (serializeJson(doc, file) == 0) {
    Serial.println("Error: failed to write file");
  } else {
  }
  
  file.close(); 
}


void loadStatus() {
  if (!SPIFFS.exists(STATUS_FILE)) {
    Serial.println("file not found -> create default");
    POS = 1;          
    btwFloor = false; 
    saveStatus();   
    return;
  }

  File file = SPIFFS.open(STATUS_FILE, FILE_READ);
  if (!file) {
    Serial.println("Error: falied to read");
    return;
  }

  StaticJsonDocument<64> doc;
  DeserializationError error = deserializeJson(doc, file);

  if (error) {
    Serial.print("JSON Error: ");
    Serial.println(error.c_str());
    Serial.println("corrupted file -> use default");
    POS = 1;
    btwFloor = false;
  } else {

    POS = doc["p"];
    btwFloor = doc["b"];
    
    Serial.println("--- complete loading data ---");
    Serial.print("Position: "); Serial.println(POS);
    Serial.print("BtwFloor: "); Serial.println(btwFloor ? "TRUE" : "FALSE");
    Serial.println("-----------------------------");
  }

  file.close();
}

#endif