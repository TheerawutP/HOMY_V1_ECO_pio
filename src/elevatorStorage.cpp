#include "elevatorStorage.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <FS.h>
#include <Preferences.h>
#include "elevatorTypes.h"

const char *STATUS_FILE = "/curr_status.txt";

// void saveStatus()
// {
   
//     StaticJsonDocument<128> doc;

//     doc["POS"] = POS;
//     doc["btwFloor"] = btwFloor;

//     File file = SPIFFS.open(STATUS_FILE, FILE_WRITE);
//     if (!file) {
//         Serial.println("Failed to open file for writing");
//         return;
//     }
        
//     if (serializeJson(doc, file) == 0) {
//         Serial.println("Failed to write to file");
//     }
//     file.close();
// }

// void loadStatus()
// {
//     if (!SPIFFS.exists(STATUS_FILE))
//     {
//         Serial.println("Config file not found, creating new one...");
//         POS = 1;
//         btwFloor = false;
//         return;
//     }

//     File file = SPIFFS.open(STATUS_FILE, FILE_READ);
//     if (!file)
//     {
//         Serial.println("Failed to open file for reading");
//         return;
//     }

//     StaticJsonDocument<128> doc;
//     DeserializationError error = deserializeJson(doc, file);

//     if (error)
//     {
//         Serial.print("Failed to read file, using defaults. Error: ");
//         Serial.println(error.c_str());
//         file.close();
//         return;
//     }

//     POS = doc["POS"]; 
//     btwFloor = doc["btwFloor"];

//     file.close();

//     Serial.println("--- complete loading data ---");
//     Serial.print("Position: ");
//     Serial.println(POS);
//     Serial.print("BtwFloor: ");
//     Serial.println(btwFloor ? "TRUE" : "FALSE");

// }

void saveStatus() {
  preferences.begin("elevator", false); // เปิด namespace ชื่อ elevator, mode R/W
  preferences.putUChar("POS", POS);     // บันทึก uint8_t
  preferences.putBool("btw", btwFloor); // บันทึก bool
  preferences.end();
}

void loadStatus() {
  preferences.begin("elevator", true);  // mode Read Only
  POS = preferences.getUChar("POS", 1); // อ่านค่า ถ้าไม่มีให้คืนค่า 1
  btwFloor = preferences.getBool("btw", false); // อ่านค่า ถ้าไม่มีให้คืนค่า false
  preferences.end();
  
  Serial.printf("Loaded: POS=%d, Btw=%s\n", POS, btwFloor ? "TRUE" : "FALSE");
}