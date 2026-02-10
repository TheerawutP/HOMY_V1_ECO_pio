#include "elevatorStorage.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <FS.h>
#include <Preferences.h>
#include "elevatorTypes.h" 

extern status_t elevator;

void saveStatus() {
  preferences.begin("elevator", false);  
  
  preferences.putBytes("el_st", &elevator, sizeof(status_t));
  
  preferences.end();
  Serial.println(">> Status Saved!"); 
}

void loadStatus() {
  preferences.begin("elevator", true);  // true = Read-only
  
  size_t len = preferences.getBytes("el_st", &elevator, sizeof(status_t));
  
  preferences.end();

  if (len != sizeof(status_t)) {
    Serial.println(">> No saved status found (or struct changed). Using Defaults.");
    
    elevator.pos = 1;
    elevator.state = STATE_IDLE; 
    elevator.dir = DIR_NONE;    
    elevator.lastDir = DIR_NONE;
    elevator.target = 0;
    elevator.lastTarget = 0;
    elevator.isBrake = true;     
    elevator.btwFloor = false;
    elevator.hasChanged = true; 
  } else {
    Serial.println(">> Status Loaded Successfully.");
    
    if (elevator.state == STATE_RUNNING) {
        elevator.state = STATE_IDLE;
        elevator.isBrake = true;
    }
    
    elevator.hasChanged = true; 
    
    Serial.printf("   Loaded: Pos=%d, State=%d, Target=%d\n", elevator.pos, elevator.state, elevator.target);
  }
}