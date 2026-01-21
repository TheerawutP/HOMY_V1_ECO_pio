#include "elevatorStorage.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <FS.h>

const char *STATUS_FILE = "/curr_status.txt";

void setDefaults() {
    Serial.println("Setting Default Values...");
    POS = 1;                   
    btwFloor = false;
    lastTarget = 0;            
    lastDir = STAY;
    moving_state = IDLE;
    FloorToFloor_MS = 17000;    
    emergency = false;
    transit.floor = 1;
    transit.dir = STAY;
    
    saveStatus(); 
}

void saveStatus()
{
   
    StaticJsonDocument<768> doc;

    doc["POS"] = POS;
    doc["btwFloor"] = btwFloor;
    doc["lastTarget"] = lastTarget;
    doc["lastDir"] = (int)lastDir;          
    doc["moving_state"] = (int)moving_state;
    doc["FloorToFloor_MS"] = FloorToFloor_MS;
    doc["emergency"] = emergency;
    doc["MAX_FLOOR"] = MAX_FLOOR;
    doc["transit_floor"] = transit.floor;
    doc["transit_dir"] = (int)transit.dir;

    File file = SPIFFS.open(STATUS_FILE, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }
        
    if (serializeJson(doc, file) == 0) {
        Serial.println("Failed to write to file");
    }
    file.close();
}

void loadStatus()
{
    if (!SPIFFS.exists(STATUS_FILE))
    {
        Serial.println("Config file not found, creating new one...");
        setDefaults();
        return;
    }

    File file = SPIFFS.open(STATUS_FILE, FILE_READ);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        setDefaults(); 
        return;
    }

    StaticJsonDocument<768> doc;
    DeserializationError error = deserializeJson(doc, file);

    if (error)
    {
        Serial.print("Failed to read file, using defaults. Error: ");
        Serial.println(error.c_str());
        file.close();
        setDefaults(); 
        return;
    }

    POS = doc["POS"] | 1; 
    btwFloor = doc["btwFloor"] | false;
    lastTarget = doc["lastTarget"] | 1;
    lastDir = (direction_t)(doc["lastDir"] | (int)STAY);
    moving_state = (state_t)(doc["moving_state"] | (int)IDLE);
    emergency = doc["emergency"] | false;
    transit.floor = doc["transit_floor"] | 1;
    transit.dir = (direction_t)(doc["transit_dir"] | (int)STAY);

    if (doc.containsKey("FloorToFloor_MS")) {
        uint32_t ms = doc["FloorToFloor_MS"];
        if (ms >= 1000) {
            FloorToFloor_MS = ms;
        } else {
            FloorToFloor_MS = 17000;
        }
    } else {
        FloorToFloor_MS = 17000;
    }

    if (doc.containsKey("MAX_FLOOR")) {
        uint8_t mf = doc["MAX_FLOOR"];
        if (mf > 0) {
            MAX_FLOOR = mf;
        }
    }

    file.close();

    if (POS == 0 || (MAX_FLOOR > 0 && POS > MAX_FLOOR)) { 
        Serial.println("Invalid POS detected, resetting to 1");
        POS = 1; 
    }

    Serial.println("--- complete loading data ---");
    Serial.print("Position: ");
    Serial.println(POS);
    Serial.print("BtwFloor: ");
    Serial.println(btwFloor ? "TRUE" : "FALSE");
    Serial.print("Last Target: ");
    Serial.println(lastTarget);
    Serial.print("Last Direction: ");
    Serial.println(lastDir);
    Serial.print("Moving State: ");
    Serial.println(moving_state);
    Serial.print("FloorToFloor_MS: ");
    Serial.println(FloorToFloor_MS);
    Serial.print("Emergency: ");
    Serial.println(emergency ? "TRUE" : "FALSE");
    Serial.print("MAX_FLOOR: ");
    Serial.println(MAX_FLOOR);
    Serial.print("Transit Floor: ");
    Serial.println(transit.floor);
    Serial.print("Transit Direction: ");
    Serial.println(transit.dir);
    Serial.println("-----------------------------");
}