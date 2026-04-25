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
#define upButton 33
#define downButton 27
#define stopButton 14
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

// --- Network & Pairing Variables ---
#define MASTER_ID 100

// Start with Broadcast Address. It will auto-update when receiving Master's heartbeat.
uint8_t masterAddress[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

bool isPaired = false;
unsigned long lastHeartbeatTime = 0;

const uint8_t hopChannels[] = { 1, 6, 11, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13 };
const int numChannels = sizeof(hopChannels) / sizeof(hopChannels[0]);
uint8_t currentChannelIndex = 0;

typedef struct struct_message {
  uint8_t fromID;
  uint16_t commandFrame;
  uint16_t responseFrame;
  bool shouldResponse;
} struct_message;

struct_message recvData;
struct_message sendData;


// --- Function to change Wi-Fi channel ---
void hopChannel() {
  currentChannelIndex = (currentChannelIndex + 1) % numChannels;
  uint8_t newChannel = hopChannels[currentChannelIndex];

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(newChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.printf(">> Searching for Master... Hopped to Channel: %d\n", newChannel);
}

// --- Receive Callback (Handles Auto-Pairing & Data Processing) ---
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(struct_message)) return;

  struct_message tempMsg;
  memcpy(&tempMsg, incomingData, sizeof(tempMsg));

  // Automatic Pairing Logic using Master's Heartbeat
  if (tempMsg.fromID == MASTER_ID) {
    lastHeartbeatTime = millis();  // Reset heartbeat watchdog

    // Check if we need to pair or update Master MAC
    if (!isPaired || memcmp(masterAddress, esp_now_info->src_addr, 6) != 0) {
      Serial.print(">> Master found! MAC: ");
      for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", esp_now_info->src_addr[i]);
        if (i < 5) Serial.print(":");
      }
      Serial.println();

      // Delete old peer if exists
      if (esp_now_is_peer_exist(masterAddress)) {
        esp_now_del_peer(masterAddress);
      }

      // Update Master MAC
      memcpy(masterAddress, esp_now_info->src_addr, 6);

      // Register new Master peer specifically for Unicast
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, masterAddress, 6);
      peerInfo.channel = 0;  // Use current channel
      peerInfo.encrypt = false;
      peerInfo.ifidx = WIFI_IF_STA;  // Target the Station interface (Core v3.x.x fix)

      esp_err_t addStatus = esp_now_add_peer(&peerInfo);
      if (addStatus == ESP_OK) {
        isPaired = true;
        Serial.println(">> Auto-Pairing Successful!");
      } else {
        Serial.printf(">> Failed to add Master as peer. Error: 0x%X\n", addStatus);
      }
    }
  }

  // Send the received data to the processing queue
  xQueueSend(espNowRxQueue, &tempMsg, 0);
  memcpy(&recvData, incomingData, sizeof(recvData));
}

// --- Send Callback ---
void OnDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  // Can be used to track specific send failures if needed
}

void writeBit(uint16_t &value, uint8_t bit, bool state) {
  if (state) {
    value |= (1 << bit);  // set
  } else {
    value &= ~(1 << bit);  // clear
  }
}

inline void L_upButton(bool en) {
  digitalWrite(L_up, en ? HIGH : LOW);
}
inline void L_downButton(bool en) {
  digitalWrite(L_down, en ? HIGH : LOW);
}
inline void L_stopButton(bool en) {
  digitalWrite(L_stop, en ? HIGH : LOW);
}
inline void L_emergButton(bool en) {
  digitalWrite(L_emerg, en ? HIGH : LOW);
}
inline void light(bool en) {
  digitalWrite(R_light, en ? HIGH : LOW);
}
inline void solenoid(bool en) {
  digitalWrite(R_solenoid, en ? HIGH : LOW);
}

// --- Processing Task (Handles Relays and DFPlayer) ---
void vProcessing(void *pvParam) {
  static uint8_t lastPlayedCode = 255;
  struct_message msg;
  uint16_t lastCmd;
  for (;;) {
    if (xQueueReceive(espNowRxQueue, &msg, portMAX_DELAY) == pdPASS) {

      if (lastCmd != msg.commandFrame) {
        Serial.println("DataRecv! :");
        Serial.println(tempMsg.fromID);
        Serial.println(tempMsg.commandFrame, BIN);
        lastCmd = msg.commandFrame;
      }
      
      // Only process command frames from the Master
      if (msg.fromID == MASTER_ID && msg.commandFrame > 0) {
        uint16_t val = msg.commandFrame;
        // Serial.print("commandFrame : ");
        // Serial.println(val, BIN);

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
}

void vAutoPairing(void *pvParam) {
  const unsigned long HEARTBEAT_TIMEOUT = 8000;  // 8 seconds timeout before hopping
  for (;;) {

    if (!isPaired || (millis() - lastHeartbeatTime > HEARTBEAT_TIMEOUT)) {
      if (isPaired) {
        Serial.println(">> Master Lost! Restarting Auto-Pairing...");
      }
      isPaired = false;
      hopChannel();

      // Fast scan: wait 300ms on this channel.
      vTaskDelay(pdMS_TO_TICKS(400));
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

// --- Write Task (Handles Buttons and Network Transmission) ---
void vWritePackage(void *pvParam) {
  uint16_t package = 0;
  uint16_t lastPackage = 0xFFFF;
  unsigned long lastSendTime = 0;

  const uint8_t DB_THRESH = 3;  // 3 x 20ms = 60ms stable before accepted
  uint8_t cnt_door = 0, cnt_up = 0, cnt_down = 0, cnt_stop = 0, cnt_emerg = 0;
  bool db_door = false, db_up = false, db_down = false, db_stop = false, db_emerg = false;

  // const unsigned long HEARTBEAT_TIMEOUT = 8000; // 8 seconds timeout before hopping

  for (;;) {

    // if (!isPaired || (millis() - lastHeartbeatTime > HEARTBEAT_TIMEOUT)) {
    //   if (isPaired) {
    //     Serial.println(">> Master Lost! Restarting Auto-Pairing...");
    //   }
    //   isPaired = false;
    //   hopChannel();

    //   // Fast scan: wait 300ms on this channel.
    //   // Master broadcasts every 200ms, so 300ms is enough to catch it.
    //   vTaskDelay(pdMS_TO_TICKS(300));
    //   continue; // Skip sending and button reads until paired
    // }

    if (!isPaired) {
      vTaskDelay(pdMS_TO_TICKS(100));  // Sleep briefly and check again
      continue;                        // Skip reading sensors and sending data completely
    }

    // 2. Read Sensors & Debounce (Runs only when Paired)
    bool r_door = !digitalRead(door_SS);
    bool r_up = !digitalRead(upButton);
    bool r_down = !digitalRead(downButton);
    bool r_stop = !digitalRead(stopButton);
    bool r_emerg = !digitalRead(emergButton);


    // Debounce logic
    if (r_door) {
      if (cnt_door < DB_THRESH) cnt_door++;
      db_door = (cnt_door >= DB_THRESH);
    } else {
      cnt_door = 0;
      db_door = false;
    }
    if (r_up) {
      if (cnt_up < DB_THRESH) cnt_up++;
      db_up = (cnt_up >= DB_THRESH);
    } else {
      cnt_up = 0;
      db_up = false;
    }
    if (r_down) {
      if (cnt_down < DB_THRESH) cnt_down++;
      db_down = (cnt_down >= DB_THRESH);
    } else {
      cnt_down = 0;
      db_down = false;
    }
    if (r_stop) {
      if (cnt_stop < DB_THRESH) cnt_stop++;
      db_stop = (cnt_stop >= DB_THRESH);
    } else {
      cnt_stop = 0;
      db_stop = false;
    }
    if (r_emerg) {
      if (cnt_emerg < DB_THRESH) cnt_emerg++;
      db_emerg = (cnt_emerg >= DB_THRESH);
    } else {
      cnt_emerg = 0;
      db_emerg = false;
    }

    // Map debounced values to package frame
    writeBit(package, 0, db_door);
    writeBit(package, 1, db_up);
    writeBit(package, 2, db_down);
    writeBit(package, 3, db_stop);
    writeBit(package, 4, db_emerg);
    writeBit(package, 5, false);  // dfBusy

    // 3. Transmit Data to Master
    if (package != lastPackage || (millis() - lastSendTime >= 80)) {
      sendData.fromID = SLAVE_ID;
      sendData.responseFrame = package;
      sendData.commandFrame = 0;
      sendData.shouldResponse = false;

      esp_err_t result = esp_now_send(masterAddress, (uint8_t *)&sendData, sizeof(sendData));

      if (result == ESP_OK) {
        if (package != lastPackage) {
          // Serial.print(">> Sent to Master (State Changed)! Package: ");
          // Serial.println(package, BIN);
        }
        lastPackage = package;
        lastSendTime = millis();
      } else {
        // Serial.println(">> Send Fail!");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);
  espNowRxQueue = xQueueCreate(10, sizeof(struct_message));

  // Initialize Wi-Fi in Station mode and disconnect to clear radio state
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Disable Wi-Fi Power Saving to ensure it never misses a Heartbeat Broadcast
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Set initial channel
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(hopChannels[0], WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // --- DFPlayer Initialization ---
  mySerial.begin(9600, SERIAL_8N1, 25, 26);
  Serial.println(F("\nWait 3 seconds for DFPlayer to boot..."));
  delay(3000);
  Serial.println(F("Initializing DFPlayer..."));
  if (!myDFPlayer.begin(mySerial, true, false)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    // while (true); // Commenting out so it doesn't freeze the whole board if DF fails
  } else {
    Serial.println(F("DFPlayer Online!"));
    myDFPlayer.volume(volumeLevel);
    delay(500);
    myDFPlayer.playFolder(1, 10);
  }

  // --- GPIO Setup ---
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

  Serial.print("CABIN MAC is: ");
  Serial.println(WiFi.macAddress());
  Serial.println(">> CABIN Started. Waiting for Master Heartbeat...");

  // --- Start Tasks ---
  xTaskCreate(vAutoPairing, "reconnectESPnow", 2048, NULL, 3, NULL);
  xTaskCreate(vWritePackage, "WritePackage", 4096, NULL, 3, NULL);
  xTaskCreate(vProcessing, "Processing", 4096, NULL, 3, NULL);
  digitalWrite(STATUS_LED, HIGH);
  Serial.println("Ready to go!");
}

void loop() {
}
