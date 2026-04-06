// #include <ModbusRTU.h>
#include <math.h>
#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>


#define WIFI_CHANNEL 11
#define SLAVE_ID 2
#define STATUS_LED 2

// #define modbus_RX 16
// #define modbus_TX 17
#define RX_DF 25
#define TX_DF 26
// #define DF_BUSY 27

#define L_up 19
#define L_down 18
#define L_stop 5
#define L_emerg 4

#define R_light 23
#define R_solenoid 22

//24v input
#define door_SS 32
// #define upButton 36
#define upButton 33
// #define downButton 39
#define downButton 27
//#define stopButton 34     F4:2D:C9:70:E3:FC
#define stopButton 14
// #define emergButton 35
#define emergButton 13

bool lastDFState = false;

volatile bool isDFPlaying = false;
uint32_t volumeLevel = 20;
HardwareSerial mySerial(1);
DFRobotDFPlayerMini myDFPlayer;

TimerHandle_t espnow_lost_timer;
QueueHandle_t espNowRxQueue;

volatile uint16_t parsing_data[16];
uint8_t currentCode = 0;

//----------------------------------------------------------------------------------------------------
// uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t masterAddress[] = { 0xEC, 0x64, 0xC9, 0x7C, 0xD8, 0x20 };  //change to master mac
// uint8_t masterAddress[] = { 0x1C, 0xC3, 0xAB, 0xBF, 0x82, 0xEC };
const uint8_t hopChannels[] = { 1, 6, 11, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13 };
const int numChannels = sizeof(hopChannels) / sizeof(hopChannels[0]);
uint8_t currentChannelIndex = 2;


typedef struct struct_message {
  uint8_t fromID;
  uint16_t commandFrame;
  uint16_t responseFrame;
  bool shouldResponse;
} struct_message;

struct_message recvData;
struct_message sendData;

void hopChannel() {
  currentChannelIndex = (currentChannelIndex + 1) % numChannels;
  uint8_t newChannel = hopChannels[currentChannelIndex];

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(newChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.print("Broadcast fail! Hopping to Channel: ");
  Serial.println(newChannel);
}

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  struct_message tempMsg;
  memcpy(&tempMsg, incomingData, sizeof(tempMsg));
  xQueueSend(espNowRxQueue, &tempMsg, 0);
  memcpy(&recvData, incomingData, sizeof(recvData));
  // Serial.print("commandFrame : ");
  // Serial.println(recvData.commandFrame, BIN);
}

void writeBit(uint16_t &value, uint8_t bit, bool state) {
  if (state) {
    value |= (1 << bit);  // set
  } else {
    value &= ~(1 << bit);  // clear
  }
}

inline void L_upButton(bool en) {
  if (en) {
    digitalWrite(L_up, HIGH);
  } else {
    digitalWrite(L_up, LOW);
  }
}

inline void L_downButton(bool en) {
  if (en) {
    digitalWrite(L_down, HIGH);
  } else {
    digitalWrite(L_down, LOW);
  }
}

inline void L_stopButton(bool en) {
  if (en) {
    digitalWrite(L_stop, HIGH);
  } else {
    digitalWrite(L_stop, LOW);
  }
}

inline void L_emergButton(bool en) {
  if (en) {
    digitalWrite(L_emerg, HIGH);
  } else {
    digitalWrite(L_emerg, LOW);
  }
}

inline void light(bool en) {
  if (en) {
    digitalWrite(R_light, HIGH);
  } else {
    digitalWrite(R_light, LOW);
  }
}

inline void solenoid(bool en) {
  if (en) {
    digitalWrite(R_solenoid, HIGH);
  } else {
    digitalWrite(R_solenoid, LOW);
  }
}

void vProcessing(void *pvParam) {
  static uint8_t lastPlayedCode = 255;
  struct_message msg;

  for (;;) {

    if (xQueueReceive(espNowRxQueue, &msg, portMAX_DELAY) == pdPASS) {

      uint16_t val = msg.commandFrame;
      Serial.print("commandFrame : ");
      Serial.println(val, BIN);

      solenoid(val & (1 << 0));
      L_upButton(val & (1 << 1));
      L_downButton(val & (1 << 2));
      L_stopButton(val & (1 << 3));
      L_emergButton(val & (1 << 4));
      light(val & (1 << 5));

      if ((val & (1 << 6)) != 0) {
        currentCode = (val >> 8) & 0x1F;
        if (currentCode != 0 && currentCode != lastPlayedCode) {
          myDFPlayer.playFolder(1, currentCode);
          lastPlayedCode = currentCode;
        } else if (currentCode == 0 && lastPlayedCode != 0) {
          myDFPlayer.stop();
          lastPlayedCode = 0;
        }
      }
    }
  }
}

void vWritePackage(void *pvParam) {
  uint16_t package = 0;
  uint16_t lastPackage = 0xFFFF;
  unsigned long lastSendTime = 0;

  const uint8_t DB_THRESH = 3;  // 3 x 20ms = 60ms stable before accepted
  uint8_t cnt_door = 0, cnt_up = 0, cnt_down = 0, cnt_stop = 0, cnt_emerg = 0;
  bool db_door = false, db_up = false, db_down = false, db_stop = false, db_emerg = false;
  bool isFirstPacket = true;

  for (;;) {

    bool r_door = !digitalRead(door_SS);
    bool r_up = !digitalRead(upButton);
    bool r_down = !digitalRead(downButton);
    bool r_stop = !digitalRead(stopButton);
    bool r_emerg = !digitalRead(emergButton);

    // Debounce door sensor
    if (r_door) {
      if (cnt_door < DB_THRESH) cnt_door++;
      db_door = (cnt_door >= DB_THRESH);
    } else {
      cnt_door = 0;
      db_door = false;
    }

    // Debounce up button
    if (r_up) {
      if (cnt_up < DB_THRESH) cnt_up++;
      db_up = (cnt_up >= DB_THRESH);
    } else {
      cnt_up = 0;
      db_up = false;
    }

    // Debounce down button
    if (r_down) {
      if (cnt_down < DB_THRESH) cnt_down++;
      db_down = (cnt_down >= DB_THRESH);
    } else {
      cnt_down = 0;
      db_down = false;
    }

    // Debounce stop button
    if (r_stop) {
      if (cnt_stop < DB_THRESH) cnt_stop++;
      db_stop = (cnt_stop >= DB_THRESH);
    } else {
      cnt_stop = 0;
      db_stop = false;
    }

    // Debounce emergency button
    if (r_emerg) {
      if (cnt_emerg < DB_THRESH) cnt_emerg++;
      db_emerg = (cnt_emerg >= DB_THRESH);
    } else {
      cnt_emerg = 0;
      db_emerg = false;
    }

    // Use debounced values
    writeBit(package, 0, db_door);
    writeBit(package, 1, db_up);
    writeBit(package, 2, db_down);
    writeBit(package, 3, db_stop);
    writeBit(package, 4, db_emerg);
    writeBit(package, 5, false);  // dfBusy

    // if (isFirstPacket) {
    //   esp_now_send(masterAddress, (uint8_t *)&sendData, sizeof(sendData));
    //   isFirstPacket = false;
    // }

    if (package != lastPackage || (millis() - lastSendTime >= 80)) {
      sendData.responseFrame = package;
      esp_err_t result = esp_now_send(masterAddress, (uint8_t *)&sendData, sizeof(sendData));

      if (result == ESP_OK) {
        if (package != lastPackage) {
          Serial.print(">> Sent to Master (State Changed)! Package: ");
          Serial.println(package, BIN);
        }
        lastPackage = package;
        lastSendTime = millis();
        // lastPackage = package;
        // xTimerReset(espnow_lost_timer, 0);
      } else {
        Serial.println(">> Send Fail!");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void reconnectToESPNow(TimerHandle_t xTimer) {
  Serial.println("Connection to Master lost for 15 seconds! System Restarting...");
  myDFPlayer.playFolder(1, 12);
  delay(2000);
  ESP.restart();
}

void setup() {

  Serial.begin(115200);
  espNowRxQueue = xQueueCreate(10, sizeof(struct_message));

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add master as peer");
    return;
  }

  mySerial.begin(9600, SERIAL_8N1, 25, 26);
  Serial.println(F("\nWait 3 seconds for DFPlayer to boot..."));
  delay(3000);
  Serial.println(F("Initializing DFPlayer..."));
  if (!myDFPlayer.begin(mySerial, true, false)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true)
      ;
  }

  Serial.println(F("DFPlayer Online!"));
  myDFPlayer.volume(volumeLevel);
  delay(500);


  // pinMode(R_safetySling, OUTPUT);
  pinMode(L_up, OUTPUT);
  pinMode(L_down, OUTPUT);
  pinMode(L_stop, OUTPUT);
  pinMode(L_emerg, OUTPUT);
  pinMode(R_light, OUTPUT);
  pinMode(R_solenoid, OUTPUT);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  pinMode(door_SS, INPUT_PULLUP);
  pinMode(upButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);
  pinMode(stopButton, INPUT_PULLUP);
  pinMode(emergButton, INPUT_PULLUP);


  Serial.print("My MAC is: ");
  Serial.println(WiFi.macAddress());


  Serial.println("Looking for Master...");
  sendData.fromID = SLAVE_ID;
  recvData.shouldResponse = false;
  struct_message waitMsg;

  // while (true) {
  //   // if (recvData.shouldResponse == true) {
  //   //   Serial.println("Master connected! Valid poll received.");
  //   //   sendData.responseFrame = 0;
  //   //   esp_now_send(masterAddress, (uint8_t *)&sendData, sizeof(sendData));
  //   //   myDFPlayer.playFolder(1, 10);

  //   if (xQueueReceive(espNowRxQueue, &waitMsg, pdMS_TO_TICKS(150)) == pdPASS) {

  //     // if (waitMsg.shouldResponse == true) {
  //       Serial.println("Master connected! Valid poll received.");
  //       sendData.responseFrame = 0;
  //       esp_now_send(masterAddress, (uint8_t *)&sendData, sizeof(sendData));
  //     // }
  //     break;

  //   } else {
  //     hopChannel();
  //     Serial.println("Still looking... (Waiting for Poll from Master)");
  //     // delay(150);
  //   }
  // }


  // espnow_lost_timer = xTimerCreate(
  //   "ESPNowWatchdog",
  //   pdMS_TO_TICKS(5000),
  //   pdFALSE,
  //   (void *)0,
  //   reconnectToESPNow);

  // if (espnow_lost_timer != NULL) {
  //   xTimerStart(espnow_lost_timer, 0);
  // }

  xTaskCreate(vWritePackage, "WritePackage", 4096, NULL, 3, NULL);
  xTaskCreate(vProcessing, "Processing", 4096, NULL, 3, NULL);
  digitalWrite(STATUS_LED, HIGH);
  Serial.println("ready to go!");
  myDFPlayer.playFolder(1, 10);
}

void loop() {
  // Serial.println(".");
  // Serial.println(recvData.commandFromMaster);
  // delay(1000);
}
