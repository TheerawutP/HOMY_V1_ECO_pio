#include "elevatorStorage.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <FS.h>
#include <Preferences.h>
#include "elevatorTypes.h"

const char *STATUS_FILE = "/curr_status.txt";

void saveStatus() {
  preferences.begin("elevator", false);  //write/read mode
  preferences.putUChar("POS", POS);    
  preferences.putBool("btw", btwFloor); 
  preferences.end();
}

void loadStatus() {
  preferences.begin("elevator", true);  //read-only mode
  POS = preferences.getUChar("POS", 1); // null return 1
  btwFloor = preferences.getBool("btw", false); //null return false
  preferences.end();
  
  Serial.printf("Loaded: POS=%d, Btw=%s\n", POS, btwFloor ? "TRUE" : "FALSE");
}