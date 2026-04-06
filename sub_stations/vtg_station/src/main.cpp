#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// #define STATUS_LED 2
#define ALM_1 13
#define ALM_2 14
#define VTG_IN_1 32
#define VTG_IN_2 33

#define SLAVE_ID 5
#define WIFI_CHANNEL 11

volatile uint16_t package = 0;

uint8_t masterAddress[] = { 0xEC, 0x64, 0xC9, 0x7C, 0xD8, 0x20 };  //sena
// uint8_t masterAddress[] = { 0x1C, 0xC3, 0xAB, 0xBF, 0x82, 0xEC };  //test
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
  // struct_message tempMsg;

  // memcpy(&tempMsg, incomingData, sizeof(tempMsg));
  // xQueueSend(espNowRxQueue, &tempMsg, 0);
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

void vProcessing(void *pvParam) {
  bool isDetected = false;
  uint16_t tempPackage = 0;

  for (;;) {
    bool s1_active = (digitalRead(VTG_IN_1) == HIGH);
    bool s2_active = (digitalRead(VTG_IN_2) == HIGH);

    isDetected = (s1_active || s2_active);

    if (isDetected) {
      writeBit(tempPackage, 0, true);
    } else {
      writeBit(tempPackage, 0, false);
    }

    digitalWrite(ALM_1, s1_active ? HIGH : LOW);
    digitalWrite(ALM_2, s2_active ? HIGH : LOW);

    package = tempPackage;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void vWritePackage(void *pvParam) {
  uint16_t lastPackage = 0xFFFF;
  bool isFirstPacket = true;

  unsigned long lastSendTime = 0;
  const unsigned long SEND_INTERVAL = 200;
  for (;;) {

    if (isFirstPacket) {
      sendData.fromID = SLAVE_ID;
      sendData.responseFrame = package;
      esp_now_send(masterAddress, (uint8_t *)&sendData, sizeof(sendData));
      isFirstPacket = false;
    }

    if (package != lastPackage || (millis() - lastSendTime >= SEND_INTERVAL)) {

      sendData.fromID = SLAVE_ID;
      sendData.responseFrame = package;
      esp_err_t result = esp_now_send(masterAddress, (uint8_t *)&sendData, sizeof(sendData));

      if (result == ESP_OK) {
        Serial.print(">> Sent to Master! Package: ");
        Serial.println(package, BIN);
        // lastPackage = package;
      } else {
        Serial.println(">> Send Fail!");
      }
      lastPackage = package;
      lastSendTime = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);

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

  Serial.print("My MAC is: ");
  Serial.println(WiFi.macAddress());

  pinMode(VTG_IN_1, INPUT_PULLUP);
  pinMode(VTG_IN_2, INPUT_PULLUP);
  pinMode(ALM_1, OUTPUT);
  pinMode(ALM_2, OUTPUT);

  xTaskCreate(vWritePackage, "WritePackage", 2048, NULL, 3, NULL);
  xTaskCreate(vProcessing, "Processing", 4096, NULL, 3, NULL);
}

void loop() {
}
