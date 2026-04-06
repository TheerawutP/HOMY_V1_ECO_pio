#include <ModbusRTU.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define SLAVE_ID 4
// #define RX_PIN 16
// #define TX_PIN 17
#define WIFI_CHANNEL 11
#define STATUS_LED 2

#define buzzer 13
#define deviceNum 5  //amount of used ultrasonic ss
#define ALM 15       //output pin when detected
#define RANGE_START 2
#define RANGE_END 60
#define RANGE_END_AUX 60  //range for lateral ultrasonic ss
#define TIMEOUT 12000
#define DECROSSTALK 60  //period for each device trig-echo then move to another one

// uint8_t rx_pin[deviceNum] = { 33, 14, 18, 4, 25 };
// uint8_t tx_pin[deviceNum] = { 32, 27, 19, 13, 26 };
// uint8_t LED_pin[deviceNum] = { 23, 22, 21, 19, 18 };
uint8_t LED_pin[deviceNum] = { 18, 19, 21, 22, 23 };

uint8_t rx_pin[deviceNum] = { 36, 39, 34, 35, 32 };
uint8_t tx_pin[deviceNum] = { 33, 25, 26, 27, 14 };

uint32_t duration[deviceNum];
uint32_t distanceRaw[deviceNum];
uint32_t distance[deviceNum];

// ----- Median Filter Buffer -----
uint32_t medBuf[deviceNum][3];
uint8_t medIndex[deviceNum] = { 0 };

// ------- Median function -------
uint32_t median3(uint32_t a, uint32_t b, uint32_t c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}



// QueueHandle_t xProcessingQueue = NULL;

// ModbusRTU RTU_SLAVE;
volatile uint16_t package = 0;

TimerHandle_t espnow_lost_timer;
QueueHandle_t espNowRxQueue;


uint8_t masterAddress[] = { 0xEC, 0x64, 0xC9, 0x7C, 0xD8, 0x20 };  //change to master mac
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
  struct_message tempMsg;
  memcpy(&tempMsg, incomingData, sizeof(tempMsg));
  xQueueSend(espNowRxQueue, &tempMsg, 0);
  memcpy(&recvData, incomingData, sizeof(recvData));
  Serial.print("commandFrame : ");
  Serial.println(recvData.commandFrame, BIN);
}


// uint16_t cbWrite(TRegister *reg, uint16_t val) {
//   // if (lastSVal != val) {
//   //   lastSVal = val;
//   //   Serial.println(String("HregSet val:") + String(val));
//   // }
//   return val;
// }

// uint16_t cbRead(TRegister *reg, uint16_t val) {
//   reg->value = val;
//   return val;
// }

void writeBit(uint16_t &value, uint8_t bit, bool state) {
  if (state) {
    value |= (1 << bit);  // set
  } else {
    value &= ~(1 << bit);  // clear
  }
}

// void vModbusCom(void *pvParam) {

//   uint16_t parsing_data[16];

//   for (;;) {

//     // memset(parsing_data, 0, sizeof(parsing_data));

//     RTU_SLAVE.task();
//     uint16_t val = RTU_SLAVE.Hreg(1);
//     for (int i = 0; i < 16; i++) {
//       parsing_data[i] = (val >> i) & 0x01;
//     }
//     // xQueueSend(xProcessingQueue, parsing_data, 0);
//     vTaskDelay(pdMS_TO_TICKS(20));
//   }
// }

void vProcessing(void *pvParam) {
  uint16_t data[16];

  for (;;) {
    bool alert = false;
    uint16_t tempPackage = 0;

    for (int i = 0; i < deviceNum; i++) {

      // ------- Trigger -------
      digitalWrite(tx_pin[i], LOW);
      delayMicroseconds(2);
      digitalWrite(tx_pin[i], HIGH);
      delayMicroseconds(10);
      digitalWrite(tx_pin[i], LOW);

      // ------- Read Echo -------
      duration[i] = pulseIn(rx_pin[i], HIGH, TIMEOUT);

      if (duration[i] == 0) {
        distanceRaw[i] = 999;
      } else {
        distanceRaw[i] = (duration[i] * 0.0343) / 2;
      }

      // ------- Median Buffer Update -------
      medBuf[i][medIndex[i]] = distanceRaw[i];
      medIndex[i] = (medIndex[i] + 1) % 3;

      // ------- Median Filter -------
      distance[i] = median3(medBuf[i][0], medBuf[i][1], medBuf[i][2]);


      // ------- Range Check -------
      bool inRange;

      if (i != 4) {
        inRange = (distance[i] > RANGE_START && distance[i] < RANGE_END);
      } else {
        inRange = (distance[i] > RANGE_START && distance[i] < RANGE_END_AUX);
      }

      if (inRange) {
        writeBit(tempPackage, i + 1, true);
        alert = true;
      }

      digitalWrite(LED_pin[i], inRange ? HIGH : LOW);
      delay(DECROSSTALK);
    }

    // ------- Update ALM Bit (Bit 0) -------
    digitalWrite(ALM, alert ? HIGH : LOW);
    if (alert) {
      writeBit(tempPackage, 0, true);
    }
    package = tempPackage;


    digitalWrite(ALM, alert ? HIGH : LOW);
    digitalWrite(buzzer, alert ? HIGH : LOW);

    // ============================================================
    //  Plotter Output (distance[0]–distance[4])
    // ============================================================
    Serial.print(distance[0]);
    Serial.print(",");
    Serial.print(distance[1]);
    Serial.print(",");
    Serial.print(distance[2]);
    Serial.print(",");
    Serial.print(distance[3]);
    Serial.print(",");
    Serial.println(distance[4]);


    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void vWritePackage(void *pvParam) {
  // uint16_t package;
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
      lastSendTime = millis();
    }
    // if (recvData.shouldResponse) {
    //   sendData.responseFrame = package;
    //   esp_err_t result = esp_now_send(masterAddress, (uint8_t *)&sendData, sizeof(sendData));
    //   if (result == ESP_OK) {
    //     Serial.println("boardcast success!");
    //     Serial.println(package, BIN);
    //     recvData.shouldResponse = false;
    //     xTimerReset(espnow_lost_timer, 0);

    //   } else {
    //     Serial.println("boardcast fail!");
    //     // hopChannel();
    //   }
    // }
    if (package != lastPackage || (millis() - lastSendTime >= SEND_INTERVAL)) {
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

void reconnectToESPNow(TimerHandle_t xTimer) {
  Serial.println("Connection to Master lost for 15 seconds! System Restarting...");
  ESP.restart();
}


void setup() {
  Serial.begin(115200);
  espNowRxQueue = xQueueCreate(10, sizeof(struct_message));

  // Serial2.begin(38400, SERIAL_8E1, RX_PIN, TX_PIN);  // RX=16, TX=17
  // RTU_SLAVE.begin(&Serial2);
  // RTU_SLAVE.slave(SLAVE);


  // RTU_SLAVE.addHreg(0x0000, 0);
  // RTU_SLAVE.onSetHreg(0x0000, cbWrite);
  // RTU_SLAVE.onGetHreg(0x0000, cbRead);

  // RTU_SLAVE.addHreg(0x0001, 0);
  // RTU_SLAVE.onSetHreg(0x0001, cbWrite);
  // RTU_SLAVE.onGetHreg(0x0001, cbRead);
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

  pinMode(ALM, OUTPUT);
  pinMode(buzzer, OUTPUT);

  for (int i = 0; i < deviceNum; i++) {
    pinMode(rx_pin[i], INPUT);
    pinMode(tx_pin[i], OUTPUT);
    pinMode(LED_pin[i], OUTPUT);

    digitalWrite(tx_pin[i], LOW);
    digitalWrite(LED_pin[i], LOW);

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);
    // init median buffer
    medBuf[i][0] = medBuf[i][1] = medBuf[i][2] = 999;
  }

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

  //     if (waitMsg.shouldResponse == true) {
  //       Serial.println("Master connected! Valid poll received.");
  //       sendData.responseFrame = 0;
  //       esp_now_send(masterAddress, (uint8_t *)&sendData, sizeof(sendData));
  //     }
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

  xTaskCreate(vWritePackage, "WritePackage", 2048, NULL, 3, NULL);
  xTaskCreate(vProcessing, "Processing", 4096, NULL, 3, NULL);
  digitalWrite(STATUS_LED, HIGH);
}



void loop() {
}
