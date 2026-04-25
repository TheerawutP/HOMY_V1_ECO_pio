// #include <WiFi.h>
// #include <esp_now.h>
// #include <esp_wifi.h>

// // #define STATUS_LED 2
// #define ALM_1 13
// #define ALM_2 14
// #define VTG_IN_1 32
// #define VTG_IN_2 33

// #define SLAVE_ID 5
// #define WIFI_CHANNEL 11

// volatile uint16_t package = 0;

// uint8_t masterAddress[] = { 0xEC, 0x64, 0xC9, 0x7C, 0xD8, 0x20 };  //sena
// // uint8_t masterAddress[] = { 0x1C, 0xC3, 0xAB, 0xBF, 0x82, 0xEC };  //test
// const uint8_t hopChannels[] = { 1, 6, 11, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13 };
// const int numChannels = sizeof(hopChannels) / sizeof(hopChannels[0]);
// uint8_t currentChannelIndex = 2;

// typedef struct struct_message {
//   uint8_t fromID;
//   uint16_t commandFrame;
//   uint16_t responseFrame;
//   bool shouldResponse;
// } struct_message;

// struct_message recvData;
// struct_message sendData;

// void hopChannel() {
//   currentChannelIndex = (currentChannelIndex + 1) % numChannels;
//   uint8_t newChannel = hopChannels[currentChannelIndex];

//   esp_wifi_set_promiscuous(true);
//   esp_wifi_set_channel(newChannel, WIFI_SECOND_CHAN_NONE);
//   esp_wifi_set_promiscuous(false);

//   Serial.print("Broadcast fail! Hopping to Channel: ");
//   Serial.println(newChannel);
// }

// void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
//   if (len != sizeof(struct_message)) return;

//   struct_message tempMsg;
//   memcpy(&tempMsg, incomingData, sizeof(tempMsg));

//   // --- Automatic Pairing Logic ---
//   if (tempMsg.fromID == 100) { // MASTER_ID
//     if (memcmp(esp_now_info->src_addr, masterAddress, 6) != 0) {
//       Serial.println(">> New Master detected! Updating MAC...");

//       // Remove old peer
//       esp_now_del_peer(masterAddress);

//       // Update master MAC
//       memcpy(masterAddress, esp_now_info->src_addr, 6);

//       // Add new peer
//       esp_now_peer_info_t peerInfo = {};
//       memcpy(peerInfo.peer_addr, masterAddress, 6);
//       peerInfo.channel = 0;
//       peerInfo.encrypt = false;
//       if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//         Serial.println(">> Failed to add new Master as peer");
//       } else {
//         Serial.println(">> Auto-paired with new Master.");
//       }
//     }
//   }

//   // memcpy(&tempMsg, incomingData, sizeof(tempMsg));
//   // xQueueSend(espNowRxQueue, &tempMsg, 0);
//   memcpy(&recvData, incomingData, sizeof(recvData));

//   // Serial.print("commandFrame : ");
//   // Serial.println(recvData.commandFrame, BIN);
// }

// void writeBit(uint16_t &value, uint8_t bit, bool state) {
//   if (state) {
//     value |= (1 << bit);  // set
//   } else {
//     value &= ~(1 << bit);  // clear
//   }
// }

// void vProcessing(void *pvParam) {
//   bool isDetected = false;
//   uint16_t tempPackage = 0;

//   for (;;) {
//     bool s1_active = (digitalRead(VTG_IN_1) == HIGH);
//     bool s2_active = (digitalRead(VTG_IN_2) == HIGH);

//     isDetected = (s1_active || s2_active);

//     if (isDetected) {
//       writeBit(tempPackage, 0, true);
//     } else {
//       writeBit(tempPackage, 0, false);
//     }

//     digitalWrite(ALM_1, s1_active ? HIGH : LOW);
//     digitalWrite(ALM_2, s2_active ? HIGH : LOW);

//     package = tempPackage;
//     vTaskDelay(pdMS_TO_TICKS(20));
//   }
// }

// void vWritePackage(void *pvParam) {
//   uint16_t lastPackage = 0xFFFF;
//   bool isFirstPacket = true;

//   unsigned long lastSendTime = 0;
//   const unsigned long SEND_INTERVAL = 200;
//   for (;;) {

//     if (isFirstPacket) {
//       sendData.fromID = SLAVE_ID;
//       sendData.responseFrame = package;
//       esp_now_send(masterAddress, (uint8_t *)&sendData, sizeof(sendData));
//       isFirstPacket = false;
//     }

//     if (package != lastPackage || (millis() - lastSendTime >= SEND_INTERVAL)) {

//       sendData.fromID = SLAVE_ID;
//       sendData.responseFrame = package;
//       esp_err_t result = esp_now_send(masterAddress, (uint8_t *)&sendData, sizeof(sendData));

//       if (result == ESP_OK) {
//         Serial.print(">> Sent to Master! Package: ");
//         Serial.println(package, BIN);
//         // lastPackage = package;
//       } else {
//         Serial.println(">> Send Fail!");
//       }
//       lastPackage = package;
//       lastSendTime = millis();
//     }

//     vTaskDelay(pdMS_TO_TICKS(20));
//   }
// }

// void setup() {
//   Serial.begin(115200);

//   WiFi.mode(WIFI_STA);
//   esp_wifi_set_promiscuous(true);
//   esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
//   esp_wifi_set_promiscuous(false);

//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW");
//     return;
//   }

//   esp_now_register_recv_cb(OnDataRecv);

//   esp_now_peer_info_t peerInfo = {};
//   memcpy(peerInfo.peer_addr, masterAddress, 6);
//   peerInfo.channel = 0;
//   peerInfo.encrypt = false;

//   if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//     Serial.println("Failed to add master as peer");
//     return;
//   }

//   Serial.print("My MAC is: ");
//   Serial.println(WiFi.macAddress());

//   pinMode(VTG_IN_1, INPUT_PULLUP);
//   pinMode(VTG_IN_2, INPUT_PULLUP);
//   pinMode(ALM_1, OUTPUT);
//   pinMode(ALM_2, OUTPUT);

//   xTaskCreate(vWritePackage, "WritePackage", 2048, NULL, 3, NULL);
//   xTaskCreate(vProcessing, "Processing", 4096, NULL, 3, NULL);
// }

// void loop() {
// }



#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// --- Pin Definitions ---
#define ALM_1 13
#define ALM_2 14
#define VTG_IN_1 32
#define VTG_IN_2 33

#define SLAVE_ID 5
#define MASTER_ID 100  // ID used by Master heartbeat

volatile uint16_t package = 0;

// --- Network & Pairing Variables ---
// Start with Broadcast Address. It will be updated automatically during pairing.
uint8_t masterAddress[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
bool isPaired = false;
unsigned long lastHeartbeatTime = 0;

// Channels to scan (1-13)
const uint8_t hopChannels[] = { 1, 6, 11, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13 };
const int numChannels = sizeof(hopChannels) / sizeof(hopChannels[0]);
uint8_t currentChannelIndex = 0;

typedef struct struct_message {
  uint8_t fromID;
  uint16_t commandFrame;
  uint16_t responseFrame;
  bool shouldResponse;
} struct_message;

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

// --- Receive Callback (Handles Auto-Pairing) ---
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(struct_message)) return;

  struct_message tempMsg;
  memcpy(&tempMsg, incomingData, sizeof(tempMsg));

  // Check if message is a Heartbeat from Master
  if (tempMsg.fromID == MASTER_ID) {
    lastHeartbeatTime = millis();  // Reset heartbeat timeout

    // If not paired yet, or Master MAC changed, update it
    if (!isPaired || memcmp(masterAddress, esp_now_info->src_addr, 6) != 0) {
      Serial.print(">> Master found! MAC: ");
      for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", esp_now_info->src_addr[i]);
        if (i < 5) Serial.print(":");
      }
      Serial.println();

      // Delete old peer (if exists)
      if (isPaired) {
        esp_now_del_peer(masterAddress);
      }

      // Save new Master MAC
      memcpy(masterAddress, esp_now_info->src_addr, 6);

      // --- NEW FIX: Explicitly configure peer Info ---
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, masterAddress, 6);
      peerInfo.channel = 0;  // 0 allows sending on current channel
      peerInfo.encrypt = false;
      peerInfo.ifidx = WIFI_IF_STA;  // MUST ADD THIS: Target the Station interface

      // Check if peer already exists in ESP-NOW memory, if yes, delete it first
      if (esp_now_is_peer_exist(masterAddress)) {
        esp_now_del_peer(masterAddress);
      }

      // Try adding the peer and catch the exact error code if it fails
      esp_err_t addStatus = esp_now_add_peer(&peerInfo);

      if (addStatus == ESP_OK) {
        isPaired = true;
        Serial.println(">> Auto-Pairing Successful!");
      } else {
        // Print the specific hex error code to serial for further debugging
        Serial.printf(">> Failed to add Master as peer. Error code: 0x%X\n", addStatus);
      }
    }
  }
}

// --- Send Callback ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Only for debug purposes
  // if(status != ESP_NOW_SEND_SUCCESS) {
  //   Serial.println(">> Packet Delivery Failed");
  // }
}

void writeBit(uint16_t &value, uint8_t bit, bool state) {
  if (state) value |= (1 << bit);
  else value &= ~(1 << bit);
}

// --- Task: Read Sensor & Update Package ---
void vProcessing(void *pvParam) {
  bool isDetected = false;
  uint16_t tempPackage = 0;

  for (;;) {
    bool s1_active = (digitalRead(VTG_IN_1) == HIGH);
    bool s2_active = (digitalRead(VTG_IN_2) == HIGH);

    isDetected = (s1_active || s2_active);

    writeBit(tempPackage, 0, isDetected);

    digitalWrite(ALM_1, s1_active ? HIGH : LOW);
    digitalWrite(ALM_2, s2_active ? HIGH : LOW);

    package = tempPackage;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// --- Task: Manage Connection & Send Data ---
void vWritePackage(void *pvParam) {
  uint16_t lastPackage = 0xFFFF;
  unsigned long lastSendTime = 0;
  const unsigned long SEND_INTERVAL = 200;

  for (;;) {
    // 1. Connection Monitoring & Channel Hopping
    // Master sends heartbeat every 2000ms. Wait 2500ms before giving up and hopping.
    if (!isPaired || (millis() - lastHeartbeatTime > 8000)) {
      isPaired = false;
      hopChannel();
      vTaskDelay(pdMS_TO_TICKS(300));  // Stay on this channel for 2.5s to listen for heartbeat
      continue;                         // Skip sending data until paired
    }

    // 2. Data Transmission (Only runs if Paired)
    if (package != lastPackage || (millis() - lastSendTime >= SEND_INTERVAL)) {
      sendData.fromID = SLAVE_ID;
      sendData.responseFrame = package;
      sendData.commandFrame = 0;
      sendData.shouldResponse = false;

      esp_err_t result = esp_now_send(masterAddress, (uint8_t *)&sendData, sizeof(sendData));

      if (result == ESP_OK) {
        Serial.printf(">> Sent Package to Master: %d\n", package);
        lastPackage = package;
        lastSendTime = millis();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize Wi-Fi in Station mode and disconnect to clear radio state
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Set initial channel
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(hopChannels[0], WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register Callbacks
  esp_now_register_recv_cb(OnDataRecv);
  // esp_now_register_send_cb(OnDataSent);

  // Note: We don't add the broadcast peer manually here.
  // It's not strictly necessary for receiving, and we will add the Unicast peer in OnDataRecv.

  Serial.print("VTG MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println(">> VTG Started. Waiting for Master Heartbeat...");

  pinMode(VTG_IN_1, INPUT_PULLUP);
  pinMode(VTG_IN_2, INPUT_PULLUP);
  pinMode(ALM_1, OUTPUT);
  pinMode(ALM_2, OUTPUT);

  xTaskCreate(vWritePackage, "WritePackage", 4096, NULL, 3, NULL);
  xTaskCreate(vProcessing, "Processing", 2048, NULL, 3, NULL);
}

void loop() {
  // FreeRTOS tasks handle everything
}
