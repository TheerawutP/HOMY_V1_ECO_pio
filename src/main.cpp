#include <WiFi.h>
#include "Arduino.h"
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <AsyncJson.h>
#include "wifi_credentials.h"
#include "WebSocketsServer.h"
#include "DNSServer.h"
#include <PubSubClient.h>
#include "RCSwitch.h"
#include "FS.h"
#include "SPIFFS.h"
#include <ModbusMaster.h>
#include <Preferences.h>
#include "elevatorTypes.h"
#include "elevatorStorage.h"
#include <esp_now.h>

#define PIN_RX 16
#define PIN_TX 17
#define WIFI_READY 13
#define RFReceiver 23
#define floorSensor1 32
#define floorSensor2 33
#define R_UP 19        // Relay UP
#define R_DW 18        // Relay DOWN
#define R_POWER_CUT 15 // Relay 4
#define BRK 5          // brake
#define NoPower 25
#define safetySling 26
#define speedGovernor 14
#define EMO 21 // emergency stop

// #define CS 21 //direct to 3.3v
// #define RST_SYS 4
// #define MB_RX 26
// #define MB_TX 27
#define MASTER_ID 100

#define toFloor1 174744
#define toFloor2 174740
// #define POWER_CUT 174738
#define STOP 174737
#define INVERTER_DI_STOP 992
#define INVERTER_DI_EMO 5092

// remote set2
#define toFloor1_2 12418580
#define toFloor2_2 12418584
#define STOP_2 12418578
#define EM_remote_2 12418577

// remote set3
#define toFloor1_3 16003636
#define toFloor2_3 16003640
#define STOP_3 16003634
#define EM_remote_3 16003633

// remote set4
#define toFloor1_4 16751236
#define toFloor2_4 16751240
#define STOP_4 16751234
#define EM_remote_4 16751233

typedef enum
{
  BTN_TO_FLOOR_1,
  BTN_TO_FLOOR_2,
  BTN_TO_FLOOR_3,
  BTN_TO_FLOOR_4,
  BTN_TO_FLOOR_5,
  BTN_TO_FLOOR_6,
  BTN_STOP,
  BTN_EMERGENCY,
  BTN_UNKNOWN
} RF_ButtonType;

typedef struct
{
  unsigned long rfCode; 
  RF_ButtonType type;   
} RF_KeyMap;

const RF_KeyMap rfKeys[] = {
  // --- set 1 ---
  // {toFloor1, BTN_TO_FLOOR_1},
  // {toFloor2, BTN_TO_FLOOR_2},
  // {STOP, BTN_STOP},
  // {EM_remote_1, BTN_EMERGENCY}, 
  
  // --- set 2 ---
  {toFloor1_2, BTN_TO_FLOOR_1},
  {toFloor2_2, BTN_TO_FLOOR_2},
  {STOP_2, BTN_STOP},
  // {EM_remote_2, BTN_EMERGENCY},

  // --- set 3 ---
  {toFloor1_3, BTN_TO_FLOOR_1},
  {toFloor2_3, BTN_TO_FLOOR_2},
  {STOP_3, BTN_STOP},
  // {EM_remote_3, BTN_EMERGENCY},

    // --- set 4 ---
  {toFloor1_4, BTN_TO_FLOOR_1},
  {toFloor2_4, BTN_TO_FLOOR_2},
  {STOP_4, BTN_STOP}
  // {EM_remote_4, BTN_EMERGENCY},
};

const int numRfKeys = sizeof(rfKeys) / sizeof(RF_KeyMap);

#define SF_0000 0
#define SF_1001 1  // going up
#define SF_1002 2  // going dw
#define SF_1003 3  // reaching
#define SF_1004 4  // obstable under cabin
#define SF_1005 5  // beware pinch
#define SF_1006 6  // no power go to floor1
#define SF_1007 7  // overweight
#define SF_1008 8  // safety brake
#define SF_1009 9  // please close the door
#define SF_1010 10 // elevator ready to use
#define SF_1011 11 // cant connect to wifi
#define SF_1012 12 // system going to reset
#define SF_1013 13 // wait for modbus
#define SF_1014 14 // reach1
#define SF_1015 15 // reach2
#define SF_1016 16 // emerg stop

// both dir block
#define MODBUS_DIS_BIT (1 << 0) // 0x01
#define DOOR_OPEN_BIT (1 << 1)  // 0x02
#define EMERG_BIT (1 << 2)      // 0x04

// block go up
#define VTG_BIT (1 << 3) // 0x08

// block go down
#define SAFETY_BRAKE_BIT (1 << 4) // 0x10
#define VSG_BIT (1 << 5)          // 0x20

// block dir group (true = dont allowed)
#define BLOCK_UP_MASK (VTG_BIT | EMERG_BIT | DOOR_OPEN_BIT | MODBUS_DIS_BIT)
#define BLOCK_DOWN_MASK (SAFETY_BRAKE_BIT | VSG_BIT | EMERG_BIT | DOOR_OPEN_BIT | MODBUS_DIS_BIT)

// ms timings
// #define DEBOUNCE_MS 1000
// #define BRAKE_MS 2000
#define WAIT_TO_RUNNING_MS 600
// #define POWER_CUT_MS 3000

uint8_t torque_rated_up = 80;
uint8_t torque_rated_down = 40; // both in percentage

volatile uint32_t modbusDelayTime = 50;
volatile uint32_t modbusRetryTime = 20;

volatile uint8_t overSpeed_counter = 0;
volatile uint32_t lastTimeCount = 0;

// const uint32_t minSpeedPeriod = 1000;
const uint32_t upper_bound_speed_interval = 800;
const uint32_t lower_bound_speed_interval = 1000;
const uint32_t overSpeed_threshold = 2;

const uint8_t MIN_FLOOR = 1;
uint8_t MAX_FLOOR = 2;

volatile uint16_t writeFrame[5][16]; // slave id, write reg

const char *mqtt_broker = "kit.flinkone.com";
const int mqtt_port = 1883; // unencrypt
const char *KIT_topic = "kit";
const char *UT_case = "/UT_55555";
const char *system_status = "/sys_v2";
const char *elevator_status = "/ele_status";
const char *inverter_status = "/inv_status";
char mqtt_topic[64];

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
ModbusMaster node;
RCSwitch RF = RCSwitch();
Preferences preferences;

SemaphoreHandle_t mqttMutex; // Mutex to protect MQTT client
SemaphoreHandle_t modbusMutex;
SemaphoreHandle_t dataMutex;

QueueHandle_t xQueueCommand;

TaskHandle_t xOchestratorHandle;
TaskHandle_t xRFReceiverHandleHandle;
TaskHandle_t xPollingModbusHandle;
TaskHandle_t xPollingFloorSensor1Handle;
TaskHandle_t xPollingFloorSensor2Handle;
TaskHandle_t xPollingNoPowerHandle;
TaskHandle_t xPollingSafetySlingHandle;
TaskHandle_t xEmergeStopHandle;
TaskHandle_t xNoPowerLandingHandle;
TaskHandle_t xModbusTimeoutHandle;
TaskHandle_t xClearCommandHandle;
TaskHandle_t xPollingModbusMasterHandle;
TaskHandle_t xWriteStationHandle;
TaskHandle_t xPollingSpeedGovernor;

TimerHandle_t xStartRunningTimer;
TimerHandle_t xProcessDataHandle;

EventGroupHandle_t xRunningEventGroup;

AsyncWebServer server(80);
WebSocketsServer m_websocketserver = WebSocketsServer(81);

// WifiTool object
const int WAIT_FOR_WIFI_TIME_OUT = 6000;
const char *PARAM_MESSAGE = "message"; // message server receives from client
std::unique_ptr<DNSServer> dnsServer;
std::unique_ptr<AsyncWebServer> m_wifitools_server;
const byte DNS_PORT = 53;
bool restartSystem = false;
String temp_json_string = "";

status_t elevator = {
    .pos = 1,
    .state = STATE_IDLE,
    .dir = DIR_NONE,
    .lastDir = DIR_NONE,
    .target = 0,
    .lastTarget = 0,
    .isBrake = true,
    .btwFloor = false,
    .hasChanged = false};

cabin_t cabinState = {
    .isDoorClosed = false,
    .isAim2 = false,
    .isAim1 = false,
    .isUserStop = false,
    .isEmergStop = false,
    .isBusy = false};

inverter_t inverterState = {
    .running_hz = 0,
    .torque = 0,
    .digitalInput = 0};

vsg_t vsgState = {
    .isAlarm = {0},
    .shouldPause = false};

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t CABIN_MAC[6] = {0x94, 0x54, 0xC5, 0xB2, 0x2A, 0xEC};
uint8_t VSG_MAC[6] = {0x94, 0x54, 0xC5, 0xB1, 0x6A, 0x70};
typedef struct struct_message
{
  uint8_t fromID;
  uint16_t commandFrame;
  uint16_t responseFrame;
  bool shouldResponse;
} struct_message;

struct_message recvData;
struct_message sendData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&recvData, incomingData, sizeof(recvData));
  // if (!esp_now_is_peer_exist(mac))
  // {
  //   esp_now_peer_info_t peer = {};
  //   memcpy(peer.peer_addr, mac, 6);
  //   peer.encrypt = false;
  //   esp_now_add_peer(&peer);
  //   memcpy(stationMac[recvData.stationID], mac, 6);
  //   stationReady[recvData.stationID] = true;
  //   Serial.printf("Station %d peer registered\n", recvData.stationID);
  // }
}

// while sleep param

// other helpers

// movement helpers
// update elevator val helpers

inline void ROTATE(direction_t dir)
{
  if (dir == DIR_UP)
  {
    digitalWrite(R_UP, HIGH);
    digitalWrite(R_DW, LOW);
    Serial.println("Move UP");
  }
  else if (dir == DIR_DOWN)
  {
    digitalWrite(R_UP, LOW);
    digitalWrite(R_DW, HIGH);
    Serial.println("Move DOWN");
  }
}

inline void BRK_ON()
{
  digitalWrite(BRK, LOW);
  // Serial.println("Brake ON");
}

inline void BRK_OFF()
{
  digitalWrite(BRK, HIGH);
  // Serial.println("Brake OFF");
}

inline void M_STP()
{
  digitalWrite(R_UP, LOW);
  digitalWrite(R_DW, LOW);
  // Serial.println("Motor Stop");
}

void enableTransmit(bool &shouldWrite)
{
  shouldWrite = true;
}

void writeBit(uint16_t &value, uint8_t bit, bool state)
{
  if (state)
  {
    value |= (1 << bit); // set
  }
  else
  {
    value &= ~(1 << bit); // clear
  }
}

void writeFrameDFPlayer(uint8_t track, uint16_t &dataframe, bool isBusy, uint8_t startDFBits)
{
  uint8_t track4Bit = track & 0x1F;

  if (isBusy == true)
  {
    // writeBit(dataframe, startDFBits + 1, true);
    // writeBit(dataframe, startDFBits, true);

    // clear busy dfplayer
    dataframe |= (1 << (startDFBits + 1));

    // //write enable dfplayer
    dataframe |= (1 << startDFBits);

    // clear df 4-bit before overwrite
    dataframe &= ~(0x1F << (startDFBits + 2));

    // write num of track
    dataframe |= (track4Bit << (startDFBits + 2));
  }
}

void emoActivate()
{
  xEventGroupSetBits(xRunningEventGroup, EMERG_BIT);
  // xTaskNotify(xOchestratorHandle, EMERG_PRESSED, eSetValueWithOverwrite);
  M_STP();
  BRK_ON();
  digitalWrite(EMO, HIGH);

  // if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
  // {
  //   // writeFrameDFPlayer(SF_1016, cabinState.writtenFrame[1], cabinState.isBusy, 6);
  //   enableTransmit(cabinState.shouldWrite);
  //   writeBit(cabinState.writtenFrame[1], 4, true);
  //   writeBit(cabinState.writtenFrame[1], 2, false);
  //   writeBit(cabinState.writtenFrame[1], 1, false);
  //   xSemaphoreGive(dataMutex);
  // }
}

void emoDeactivate()
{
  xEventGroupClearBits(xRunningEventGroup, EMERG_BIT);
  // xTaskNotify(xOchestratorHandle, EMERG_RELEASED, eSetValueWithOverwrite);
  digitalWrite(EMO, LOW);
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
  {
    enableTransmit(cabinState.shouldWrite);
    writeBit(cabinState.writtenFrame[1], 4, false);
    xSemaphoreGive(dataMutex);
  }
}

void abortMotion()
{
  if (elevator.isBrake == true)
  {
    return;
  }

  M_STP();
  BRK_ON();

  elevator.state = STATE_IDLE;
  elevator.lastDir = elevator.dir;
  elevator.dir = DIR_NONE;
  elevator.target = 0;
  elevator.isBrake = true;

  Serial.println("Elevator Halted.");
}

bool isSafeToRun(direction_t dir)
{
  EventBits_t currentBits = xEventGroupGetBits(xRunningEventGroup);

  if (dir == DIR_UP)
  {
    if ((currentBits & BLOCK_UP_MASK) != 0)
    {
      Serial.printf("BLOCKED UP! Reason bits: 0x%X\n", (currentBits & BLOCK_UP_MASK));
      return false;
    }
  }
  else if (dir == DIR_DOWN)
  {
    if ((currentBits & BLOCK_DOWN_MASK) != 0)
    {
      Serial.printf("BLOCKED DOWN! Reason bits: 0x%X\n", (currentBits & BLOCK_DOWN_MASK));
      return false;
    }
  }

  return true;
}

// modbus helpers

bool readDataFrom(uint8_t slaveID, uint16_t startAddress, uint8_t numRead, uint16_t *hreg_row)
{
  node.begin(slaveID, Serial1);
  uint8_t result = node.readHoldingRegisters(startAddress, numRead);

  if (result == node.ku8MBSuccess)
  {
    for (int i = 0; i < numRead; i++)
    {
      hreg_row[i] = node.getResponseBuffer(i);
    }
    return true;
  }
  else
  {
    Serial.print("Error at ID ");
    Serial.print(slaveID);
    Serial.print(": ");
    Serial.println(result, HEX);
    return false;
  }
}

// background helpers

void setupMQTT()
{
  mqttClient.setServer(mqtt_broker, mqtt_port);
  // mqttClient.setCallback(callback);
  mqttClient.setBufferSize(1024);
}

boolean connectAttempt(String ssid, String password)
{
  boolean isWiFiConnected = false;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  if (ssid == "")
  {
    WiFi.begin();
    Serial.print(F("Connecting to last known WiFi..."));
  }
  else
  {
    int ssidSize = ssid.length() + 1;
    int passwordSize = password.length() + 1;
    char ssidArray[ssidSize] = {0};
    char passwordArray[passwordSize] = {0};
    ssid.toCharArray(ssidArray, ssidSize);
    password.toCharArray(passwordArray, passwordSize);
    WiFi.begin(ssidArray, passwordArray);

    Serial.print(F("Connecting to SSID: "));
    Serial.println(ssid);
  }

  unsigned long now = millis();
  while (WiFi.status() != WL_CONNECTED && millis() < now + WAIT_FOR_WIFI_TIME_OUT)
  {
    Serial.print(".");
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(F("\nWiFi connected"));
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
    isWiFiConnected = true;
  }
  else
  {
    Serial.println(F("\nWiFi connection failed."));
  }

  return isWiFiConnected;
}

void setUpAPService()
{
  Serial.println(F("Starting Access Point server."));

  // DNSServer dnsServer;
  // dnsServer.reset(new DNSServer());
  WiFi.mode(WIFI_AP);
  // WiFi.softAPConfig(IPAddress(172, 217, 28, 1), IPAddress(172, 217, 28, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP("Ximplex_LuckD");
  delay(1000);

  /* Setup the DNS server redirecting all the domains to the apIP */
  // dnsServer->setErrorReplyCode(DNSReplyCode::NoError);
  // dnsServer->start(DNS_PORT, "*", IPAddress(172, 217, 28, 1));

  // Serial.println("dns server config done");
}

void process()
{
  /// DNS
  // dnsServer->processNextRequest();
  // yield
  yield();
  delay(10);
  // Reset flag/timer
  if (restartSystem)
  {
    if (restartSystem + 1000 < millis())
    {
      ESP.restart();
    } // end if
  } // end if
}

void handleGetSavSecreteJson(AsyncWebServerRequest *request)
{
  String message;

  String m_SSID1_name;
  String m_SSID2_name;
  String m_SSID3_name;
  String m_PWD1_name;
  String m_PWD2_name;
  String m_PWD3_name;
  String m_temp_string;
  int params = request->params();
  for (int i = 0; i < params; i++)
  {
    AsyncWebParameter *p = request->getParam(i);
    if (p->isPost())
    {
      Serial.print(i);
      Serial.print(F("\t"));
      Serial.print(p->name().c_str());
      Serial.print(F("\t"));
      Serial.println(p->value().c_str());
      m_temp_string = p->name().c_str();
      if (m_temp_string == "ssid1")
      {
        m_SSID1_name = p->value().c_str();
      }
      else if (m_temp_string == "pass1")
      {
        m_PWD1_name = p->value().c_str();
      }
      else if (m_temp_string == "ssid2")
      {
        m_SSID2_name = p->value().c_str();
      }
      else if (m_temp_string == "pass2")
      {
        m_PWD2_name = p->value().c_str();
      }
      else if (m_temp_string == "ssid3")
      {
        m_SSID3_name = p->value().c_str();
      }
      else if (m_temp_string == "pass3")
      {
        m_PWD3_name = p->value().c_str();
      }
    }
  }
  if (request->hasParam(PARAM_MESSAGE, true))
  {
    message = request->getParam(PARAM_MESSAGE, true)->value();
    Serial.println(message);
  }
  else
  {
    message = "No message sent";
  }
  request->send(200, "text/HTML", "Credentials saved. Rebooting...");

  // {"SSID1":"myssid1xyz","PWD1":"mypwd1xyz",
  //     "SSID2":"myssid2xyz","PWD2":"mypwd2xyz",
  //     "SSID3":"myssid3xyz","PWD3":"mypwd3xyz"}

  String SSID_and_pwd_JSON = "";
  SSID_and_pwd_JSON += "{\"SSID1\":\"";
  SSID_and_pwd_JSON += m_SSID1_name;
  SSID_and_pwd_JSON += "\",\"PWD1\":\"";
  SSID_and_pwd_JSON += m_PWD1_name;

  SSID_and_pwd_JSON += "\",\"SSID2\":\"";
  SSID_and_pwd_JSON += m_SSID2_name;
  SSID_and_pwd_JSON += "\",\"PWD2\":\"";
  SSID_and_pwd_JSON += m_PWD2_name;

  SSID_and_pwd_JSON += "\",\"SSID3\":\"";
  SSID_and_pwd_JSON += m_SSID3_name;
  SSID_and_pwd_JSON += "\",\"PWD3\":\"";
  SSID_and_pwd_JSON += m_PWD3_name;

  SSID_and_pwd_JSON += "\"}";

  Serial.println("JSON string to write to file = ");
  Serial.println(SSID_and_pwd_JSON);

  Serial.print("m_SSID1_name = ");
  Serial.print(m_SSID1_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_PWD1_name = ");
  Serial.print(m_PWD1_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_SSID2_name = ");
  Serial.print(m_SSID2_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_PWD2_name = ");
  Serial.print(m_PWD2_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_SSID3_name = ");
  Serial.print(m_SSID3_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_PWD3_name = ");
  Serial.println(m_PWD3_name);

  File m_ssid_pwd_file_to_write_name = SPIFFS.open("/credentials.JSON", FILE_WRITE);

  if (!m_ssid_pwd_file_to_write_name)
  {
    Serial.println("There was an error opening the pwd/ssid file for writing");
    return;
  }

  Serial.println("Writing this JSON string to pwd/ssid file:");
  Serial.println(SSID_and_pwd_JSON);

  if (!m_ssid_pwd_file_to_write_name.println(SSID_and_pwd_JSON))
  {
    Serial.println("File write failed");
  }
  m_ssid_pwd_file_to_write_name.close();

  request->send(200, "text/html", "<h1>Restarting .....</h1>");
  restartSystem = millis();
}

void handleGetSavSecreteJsonNoReboot(AsyncWebServerRequest *request)
{
  String message;

  String m_SSID1_name;
  String m_SSID2_name;
  String m_SSID3_name;
  String m_PWD1_name;
  String m_PWD2_name;
  String m_PWD3_name;
  String m_temp_string;
  int params = request->params();
  for (int i = 0; i < params; i++)
  {
    AsyncWebParameter *p = request->getParam(i);
    if (p->isPost())
    {
      Serial.print(i);
      Serial.print(F("\t"));
      Serial.print(p->name().c_str());
      Serial.print(F("\t"));
      Serial.println(p->value().c_str());
      m_temp_string = p->name().c_str();
      if (m_temp_string == "ssid1")
      {
        m_SSID1_name = p->value().c_str();
      }
      else if (m_temp_string == "pass1")
      {
        m_PWD1_name = p->value().c_str();
      }
      else if (m_temp_string == "ssid2")
      {
        m_SSID2_name = p->value().c_str();
      }
      else if (m_temp_string == "pass2")
      {
        m_PWD2_name = p->value().c_str();
      }
      else if (m_temp_string == "ssid3")
      {
        m_SSID3_name = p->value().c_str();
      }
      else if (m_temp_string == "pass3")
      {
        m_PWD3_name = p->value().c_str();
      }
    }
  }
  if (request->hasParam(PARAM_MESSAGE, true))
  {
    message = request->getParam(PARAM_MESSAGE, true)->value();
    Serial.println(message);
  }
  else
  {
    message = "No message sent";
  }
  // request->send(200, "text/HTML", "bla bla bla bla bla xyz xyz xyz xyz xyz ");

  // {"SSID1":"myssid1xyz","PWD1":"mypwd1xyz",
  //     "SSID2":"myssid2xyz","PWD2":"mypwd2xyz",
  //     "SSID3":"myssid3xyz","PWD3":"mypwd3xyz"}

  String SSID_and_pwd_JSON = "";
  SSID_and_pwd_JSON += "{\"SSID1\":\"";
  SSID_and_pwd_JSON += m_SSID1_name;
  SSID_and_pwd_JSON += "\",\"PWD1\":\"";
  SSID_and_pwd_JSON += m_PWD1_name;

  SSID_and_pwd_JSON += "\",\"SSID2\":\"";
  SSID_and_pwd_JSON += m_SSID2_name;
  SSID_and_pwd_JSON += "\",\"PWD2\":\"";
  SSID_and_pwd_JSON += m_PWD2_name;

  SSID_and_pwd_JSON += "\",\"SSID3\":\"";
  SSID_and_pwd_JSON += m_SSID3_name;
  SSID_and_pwd_JSON += "\",\"PWD3\":\"";
  SSID_and_pwd_JSON += m_PWD3_name;

  SSID_and_pwd_JSON += "\"}";

  Serial.println("JSON string to write to file = ");
  Serial.println(SSID_and_pwd_JSON);

  Serial.print("m_SSID1_name = ");
  Serial.print(m_SSID1_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_PWD1_name = ");
  Serial.print(m_PWD1_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_SSID2_name = ");
  Serial.print(m_SSID2_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_PWD2_name = ");
  Serial.print(m_PWD2_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_SSID3_name = ");
  Serial.print(m_SSID3_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_PWD3_name = ");
  Serial.println(m_PWD3_name);

  File m_ssid_pwd_file_to_write_name = SPIFFS.open("/credentials.JSON", FILE_WRITE);

  if (!m_ssid_pwd_file_to_write_name)
  {
    Serial.println("There was an error opening the pwd/ssid file for writing");
    return;
  }

  Serial.println("Writing this JSON string to pwd/ssid file:");
  Serial.println(SSID_and_pwd_JSON);

  if (!m_ssid_pwd_file_to_write_name.println(SSID_and_pwd_JSON))
  {
    Serial.println("File write failed");
  }
  m_ssid_pwd_file_to_write_name.close();

  request->send(200, "text/html", "  <head> <meta http-equiv=\"refresh\" content=\"2; URL=wifi.html\" /> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> </head> <body> <h1> Credentials stored to flash on NanoStat. </h1>  </body>");
  restartSystem = millis();
}

void getWifiScanJson(AsyncWebServerRequest *request)
{
  String json = "{\"scan_result\":[";
  int n = WiFi.scanComplete();
  if (n == -2)
  {
    WiFi.scanNetworks(true);
  }
  else if (n)
  {
    for (int i = 0; i < n; ++i)
    {
      if (i)
        json += ",";
      json += "{";
      json += "\"RSSI\":" + String(WiFi.RSSI(i));
      json += ",\"SSID\":\"" + WiFi.SSID(i) + "\"";
      json += "}";
    }
    WiFi.scanDelete();
    if (WiFi.scanComplete() == -2)
    {
      WiFi.scanNetworks(true);
    }
  }
  json += "]}";
  request->send(200, "application/json", json);
  json = String();
}

bool readSSIDPWDfile(String m_pwd_filename_to_read)
{
  File m_pwd_file_to_read = SPIFFS.open(m_pwd_filename_to_read);

  if (!m_pwd_file_to_read)
  {
    Serial.println("Failed to open PWD file file for reading");
    return false;
  }

  if (!SPIFFS.exists(m_pwd_filename_to_read))
  {
    Serial.print(m_pwd_filename_to_read);
    Serial.println("  does not exist.");
    return false;
  }

  String m_pwd_file_string;
  while (m_pwd_file_to_read.available()) // read json from file
  {
    m_pwd_file_string += char(m_pwd_file_to_read.read());
  } // end while
  // Serial.print("m_pwd_file_string = ");
  // Serial.println(m_pwd_file_string);
  m_pwd_file_to_read.close();

  // parse
  StaticJsonDocument<1000> m_JSONdoc_from_pwd_file;
  DeserializationError m_error = deserializeJson(m_JSONdoc_from_pwd_file, m_pwd_file_string); // m_JSONdoc is now a json object
  if (m_error)
  {
    Serial.println("deserializeJson() failed with code ");
    Serial.println(m_error.c_str());
  }
  // m_JSONdoc_from_pwd_file is the JSON object now we can use it.
  String m_SSID1_name = m_JSONdoc_from_pwd_file["SSID1"];
  String m_SSID2_name = m_JSONdoc_from_pwd_file["SSID2"];
  String m_SSID3_name = m_JSONdoc_from_pwd_file["SSID3"];
  String m_PWD1_name = m_JSONdoc_from_pwd_file["PWD1"];
  String m_PWD2_name = m_JSONdoc_from_pwd_file["PWD1"];
  String m_PWD3_name = m_JSONdoc_from_pwd_file["PWD1"];

  // Try connecting:
  //****************************8
  if (connectAttempt(m_SSID1_name, m_PWD1_name))
  {
    return true;
  }
  Serial.println("Failed to connect.");
  if (connectAttempt(m_SSID2_name, m_PWD2_name))
  {
    return true;
  }
  Serial.println("Failed to connect.");

  if (connectAttempt(m_SSID3_name, m_PWD3_name))
  {
    return true;
  }
  Serial.println("Failed to connect.");

  return false;
}

void runWifiPortal()
{

  m_wifitools_server.reset(new AsyncWebServer(80));

  IPAddress myIP;
  myIP = WiFi.softAPIP();
  // myIP = WiFi.localIP();
  Serial.print(F("AP IP address: "));
  Serial.println(myIP);

  // Need to tell server to accept packets from any source with any header via http methods GET, PUT:
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, PUT");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "*");

  m_wifitools_server->serveStatic("/", SPIFFS, "/").setDefaultFile("wifi_index.html");

  // m_wifitools_server->on("/saveSecret/", HTTP_ANY, [&, this](AsyncWebServerRequest *request) {
  //   handleGetSavSecreteJson(request);
  // });

  m_wifitools_server->on("/saveSecret", HTTP_POST, [](AsyncWebServerRequest *request)
                         { handleGetSavSecreteJson(request); });

  // m_wifitools_server->on("/list", HTTP_ANY, [](AsyncWebServerRequest *request)
  //                        { handleFileList(request); });

  m_wifitools_server->on("/wifiScan.json", HTTP_GET, [](AsyncWebServerRequest *request)
                         { getWifiScanJson(request); });

  Serial.println(F("HTTP server started"));
  m_wifitools_server->begin();
  if (!MDNS.begin("keepintouch")) // see https://randomnerdtutorials.com/esp32-access-point-ap-web-server/
  {
    Serial.println("Error setting up MDNS responder !");
    while (1)
      ;
    {
      delay(1000);
    }
  }
  Serial.println("MDNS started.");
  // MDNS.begin("nanostat");
  while (1) // loop until user hits restart... Once credentials saved, won't end up here again unless wifi not connecting!
  {
    process();
  }
}

void runWifiPortal_after_connected_to_WIFI()
{
  // Don't run this after starting server or ESP32 will crash!!!
  server.on("/saveSecret/", HTTP_POST, [](AsyncWebServerRequest *request)
            { handleGetSavSecreteJsonNoReboot(request); });

  Serial.println(F("HTTP server started"));
  m_wifitools_server->begin();
  if (!MDNS.begin("keepintouch")) // see https://randomnerdtutorials.com/esp32-access-point-ap-web-server/
  {
    Serial.println("Error setting up MDNS responder !");
    while (1)
      ;
    {
      delay(1000);
    }
  }
  Serial.println("MDNS started.");
  // MDNS.begin("nanostat");
  while (1) // loop until user hits restart... Once credentials saved, won't end up here again unless wifi not connecting!
  {
    process();
  }
}

void handle_websocket_text(uint8_t *payload)
{
  // do something...
  Serial.printf("handle_websocket_text called for: %s\n", payload);

  // Parse JSON payload
  StaticJsonDocument<1000> m_JSONdoc_from_payload;
  DeserializationError m_error = deserializeJson(m_JSONdoc_from_payload, payload); // m_JSONdoc is now a json object
  if (m_error)
  {
    Serial.println("deserializeJson() failed with code ");
    Serial.println(m_error.c_str());
  }

  JsonObject m_JsonObject_from_payload = m_JSONdoc_from_payload.as<JsonObject>();

  for (JsonPair keyValue : m_JsonObject_from_payload)
  {
    String m_key_string = keyValue.key().c_str();

    if (m_key_string == "change_floor_number_to")
    {
      Serial.println("change_floor_number_to called");
      int m_new_floor_number = m_JSONdoc_from_payload["change_floor_number_to"];
      Serial.println(m_new_floor_number);
      // MAX_FLOOR = m_new_floor_number;
    }

    if (m_key_string == "change_up_duration_to")
    {
      Serial.println("change_up_duration_to called");
      int m_new_up_duration = m_JSONdoc_from_payload["change_up_duration_to"];
      Serial.println(m_new_up_duration);
      // FloorToFloor_MS = m_new_up_duration;
    }
  }
}

void sendWebsocketAlert(const char *alertType, const char *message)
{
  char alertBuf[128];
  snprintf(alertBuf, sizeof(alertBuf), "{\"alert\":\"%s\", \"msg\":\"%s\"}", alertType, message);

  m_websocketserver.broadcastTXT(alertBuf, strlen(alertBuf));
  Serial.printf(">> Sent Alert to UI: %s\n", message);
}

void onWebSocketEvent(uint8_t num,
                      WStype_t type,
                      uint8_t *payload,
                      size_t length)
{
  switch (type)
  {

  // Client has disconnected
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;

  // New client has connected
  case WStype_CONNECTED:
  {
    IPAddress ip = m_websocketserver.remoteIP(num);
    Serial.printf("[%u] Connection from ", num);
    Serial.println(ip.toString());
  }
  break;

  // Echo text message back to client
  case WStype_TEXT:
    // Serial.println(payload[0,length-1]); // this doesn't work....
    Serial.printf("[%u] Received text: %s\n", num, payload);
    // m_websocketserver.sendTXT(num, payload);
    // if (true == false) // later change to if message has certain format:
    // {
    //   m_websocket_send_rate = (float)atof((const char *)&payload[0]); // adjust data send rate used in loop
    // }
    handle_websocket_text(payload);

    break;

  // For everything else: do nothing
  case WStype_BIN:
  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
  default:
    break;
  }
}

void configureserver()
// configures server
{
  // Need to tell server to accept packets from any source with any header via http methods GET, PUT:
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, PUT");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "*");

  // Button #1
  server.addHandler(new AsyncCallbackJsonWebHandler("/on_Button_UP_pressed", [](AsyncWebServerRequest *request1, JsonVariant &json1)
                                                    {
                                                      const JsonObject &jsonObj1 = json1.as<JsonObject>();
                                                      if (jsonObj1["on"])
                                                      {
                                                        Serial.println("Up button pressed.");
                                                        Serial.println("------------------");
        userCommand_t userCommand;
        userCommand.target = 2;
        userCommand.type = moveToFloor;
        userCommand.from = FROM_WS;
        xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
                                                      }
                                                      request1->send(200, "OK"); }));

  // Button #2
  server.addHandler(new AsyncCallbackJsonWebHandler("/on_Button_DOWN_pressed", [](AsyncWebServerRequest *request2, JsonVariant &json2)
                                                    {
                                                      const JsonObject &jsonObj2 = json2.as<JsonObject>();
                                                      if (jsonObj2["on"])
                                                      {
                                                        Serial.println("Down button pressed.");
                                                        Serial.println("------------------");
        userCommand_t userCommand;
        userCommand.target = 1;
        userCommand.type = moveToFloor;
        userCommand.from = FROM_WS;
        xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
                                                      }
                                                      request2->send(200, "OK"); }));

  // Button #11
  // server.addHandler(new AsyncCallbackJsonWebHandler("/button11pressed", [](AsyncWebServerRequest *request2, JsonVariant &json2)
  //                                                   {
  //   const JsonObject &jsonObj2 = json2.as<JsonObject>();
  //   if (jsonObj2["on"])
  //   {
  //     Serial.println("Button 11 pressed. Running DPV sweep.");
  //     // digitalWrite(LEDPIN, HIGH);

  //   }
  //   request2->send(200, "OK"); }));
  // Button #3
  server.addHandler(new AsyncCallbackJsonWebHandler("/on_Button_STOP_pressed", [](AsyncWebServerRequest *request3, JsonVariant &json3)
                                                    {
                                                      const JsonObject &jsonObj3 = json3.as<JsonObject>();
                                                      if (jsonObj3["on"])
                                                      {
                                                        Serial.println("stop button pressed. Stopping all movement!");
                                                        Serial.println("------------------");
        emoDeactivate();
        userCommand_t userCommand;
        userCommand.target = 0;
        userCommand.type = userAbort;
        userCommand.from = FROM_WS;
        xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
                                                      }
                                                      request3->send(200, "OK"); }));

  // Button #4
  server.addHandler(new AsyncCallbackJsonWebHandler("/on_Button_EMERGENCY_pressed", [](AsyncWebServerRequest *request4, JsonVariant &json4)
                                                    {
                                                      const JsonObject &jsonObj4 = json4.as<JsonObject>();
                                                      if (jsonObj4["on"])
                                                      {
                                                        Serial.println("Emergency button pressed. Stopping all movement immediately!");
                                                        Serial.println("------------------");
                                                        // ws_cmd = true;
                                                        // ws_cmd_value = POWER_CUT;                                          
        bool emoState = digitalRead(EMO);
        emoState = !emoState;
        digitalWrite(EMO, emoState);
                                   
                                                      }
                                                      request4->send(200, "OK"); }));

  // Button #5
  // server.addHandler(new AsyncCallbackJsonWebHandler("/button5pressed", [](AsyncWebServerRequest *request5, JsonVariant &json5)
  //                                                   {
  //                                                     const JsonObject &jsonObj5 = json5.as<JsonObject>();
  //                                                     if (jsonObj5["on"])
  //                                                     {
  //                                                       Serial.println("Button 5 pressed. Running DC sweep.");
  //                                                       // digitalWrite(LEDPIN, HIGH);

  //                                                     }
  //                                                     request5->send(200, "OK"); }));
  // // Button #6
  // server.addHandler(new AsyncCallbackJsonWebHandler("/button6pressed", [](AsyncWebServerRequest *request6, JsonVariant &json6)
  //                                                   {
  //                                                     const JsonObject &jsonObj6 = json6.as<JsonObject>();
  //                                                     if (jsonObj6["on"])
  //                                                     {
  //                                                       Serial.println("Button 6 pressed. Running IV sweep.");
  //                                                       // digitalWrite(LEDPIN, HIGH);

  //                                                     }
  //                                                     request6->send(200, "OK"); }));

  // // Button #7
  // server.addHandler(new AsyncCallbackJsonWebHandler("/button7pressed", [](AsyncWebServerRequest *request7, JsonVariant &json7)
  //                                                   {
  //                                                     const JsonObject &jsonObj7 = json7.as<JsonObject>();
  //                                                     if (jsonObj7["on"])
  //                                                     {
  //                                                       Serial.println("Button 7 pressed. Running CAL sweep.");
  //                                                       // digitalWrite(LEDPIN, HIGH);

  //                                                     }
  //                                                     request7->send(200, "OK"); }));
  // // Button #8
  // server.addHandler(new AsyncCallbackJsonWebHandler("/button8pressed", [](AsyncWebServerRequest *request8, JsonVariant &json8)
  //                                                   {
  //                                                     const JsonObject &jsonObj8 = json8.as<JsonObject>();
  //                                                     if (jsonObj8["on"])
  //                                                     {
  //                                                       Serial.println("Button 8 pressed. Running MISC_MODE sweep.");
  //                                                       // digitalWrite(LEDPIN, HIGH);

  //                                                     }
  //                                                     request8->send(200, "OK"); }));
  // // Button #9
  // server.addHandler(new AsyncCallbackJsonWebHandler("/button9pressed", [](AsyncWebServerRequest *request9, JsonVariant &json9)
  //                                                   {
  //                                                     const JsonObject &jsonObj9 = json9.as<JsonObject>();
  //                                                     if (jsonObj9["on"])
  //                                                     {
  //                                                       Serial.println("Button 9 pressed.");
  //                                                       // digitalWrite(LEDPIN, HIGH);

  //                                                     }
  //                                                     request9->send(200, "OK"); }));
  // // Button #10
  // server.addHandler(new AsyncCallbackJsonWebHandler("/button10pressed", [](AsyncWebServerRequest *request10, JsonVariant &json10)
  //                                                   {
  //                                                     const JsonObject &jsonObj10 = json10.as<JsonObject>();
  //                                                     if (jsonObj10["on"])
  //                                                     {
  //                                                       Serial.println("Button 10 pressed.");
  //                                                       // digitalWrite(LEDPIN, HIGH);

  //                                                     }
  //                                                     request10->send(200, "OK"); }));

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  // server.on("/downloadfile", HTTP_GET, [](AsyncWebServerRequest *request)
  //           { request->send(SPIFFS, "/data.txt", "text/plain", true); });

  server.on("/rebootnanostat", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              // reboot the ESP32
              request->send(200, "text/HTML", "  <head> <meta http-equiv=\"refresh\" content=\"5; URL=index.html\" /> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> </head> <body> <h1> Rebooting! </h1>  </body>");
              delay(500);
              ESP.restart(); });

  server.onNotFound([](AsyncWebServerRequest *request)
                    {
                      if (request->method() == HTTP_OPTIONS)
                      {
                        request->send(200); // options request typically sent by client at beginning to make sure server can handle request
                      }
                      else
                      {
                        Serial.println("Not found");
                        request->send(404, "Not found");
                      } });

  // Send a POST request to <IP>/actionpage with a form field message set to <message>
  server.on("/actionpage.html", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              String message;
              Serial.println("actionpage.html, HTTP_POST actionpage received , processing....");
              Serial.printf("Total Parameters Received: %d\n", request->params());
              //**********************************************
              preferences.begin("my-config", false);
              // List all parameters int params = request->params();
              int params = request->params();
              for (int i = 0; i < params; i++)
              {
                AsyncWebParameter *p = request->getParam(i);
                if (p->isPost())
                {
                  // Serial.print(i);
                  // Serial.print(F("\t"));
                  // Serial.print(p->name().c_str());
                  // Serial.print(F("\t"));
                  // Serial.println(p->value().c_str());
                  String paramName = p->name();
                  String paramValue = p->value();

                  Serial.print("Param: ");
                  Serial.print(paramName);
                  Serial.print(" = ");
                  Serial.println(paramValue);

                  if (paramName == "force_state")
                  {
                    int stateVal = paramValue.toInt();

                    state_t newState = (state_t)stateVal;
                    elevator.state = newState;

                    Serial.printf(">> Manual Force State to: %d \n", stateVal);
                  }
                }
              }

              // preferences.end();
              // Serial.println("--- Updated Variables ---");
              // Serial.printf("Hour Meter Offset: %d\n", hour_meter_runtime_offset);
              //**********************************************

              if (request->hasParam(PARAM_MESSAGE, true))
              {
                message = request->getParam(PARAM_MESSAGE, true)->value();
                Serial.println(message);
              }
              else
              {
                message = "No message sent";
              }
              // request->send(200, "text/HTML", "Hello, POST: " + message);
              // request->send(200, "text/HTML", "Sweep data saved. Click <a href=\"/index.html\">here</a> to return to main page.");
              request->send(200, "text/HTML", "  <head> <meta http-equiv=\"refresh\" content=\"2; URL=index.html\" /> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> </head> <body> <h1> Settings saved! </h1> <p> Returning to main page. </p> </body>");
              // request->send(200, "OK");
            });

  // Wifitools stuff:
  // Save credentials:
  server.on("/saveSecret", HTTP_POST, [](AsyncWebServerRequest *request)
            { handleGetSavSecreteJsonNoReboot(request); });

  // Wifi scan:
  server.on("/wifiScan.json", HTTP_GET, [](AsyncWebServerRequest *request)
            { getWifiScanJson(request); });

  server.begin();
}

//
//
//

void getDir(uint8_t target, transitCommand_t *cmd)
{
  if (elevator.pos != target)
  {
    direction_t newDir = (target > elevator.pos) ? DIR_UP : DIR_DOWN;
    uint8_t newTarget = target;
    uint8_t trackNum = 0;
    uint8_t dirBit = 0;

    cmd->dir = newDir;
    cmd->target = newTarget;

    elevator.state = STATE_PENDING;
    elevator.dir = newDir;
    elevator.lastTarget = newTarget;
    elevator.target = newTarget;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      if (newDir == DIR_UP)
      {
        writeFrameDFPlayer(SF_1001, cabinState.writtenFrame[1], cabinState.isBusy, 6);
        enableTransmit(cabinState.shouldWrite);
        writeBit(cabinState.writtenFrame[1], 5, true);
        writeBit(cabinState.writtenFrame[1], 3, false);
        writeBit(cabinState.writtenFrame[1], 2, false);
        writeBit(cabinState.writtenFrame[1], 1, true);
      }
      else
      {
        writeFrameDFPlayer(SF_1002, cabinState.writtenFrame[1], cabinState.isBusy, 6);
        enableTransmit(cabinState.shouldWrite);
        writeBit(cabinState.writtenFrame[1], 5, true);
        writeBit(cabinState.writtenFrame[1], 3, false);
        writeBit(cabinState.writtenFrame[1], 2, true);
        writeBit(cabinState.writtenFrame[1], 1, false);
      }
      xSemaphoreGive(dataMutex);
    }
  }
  else
  {
    if (elevator.btwFloor == true)
    {
      direction_t newDir = DIR_NONE;
      uint8_t newTarget = target;
      uint8_t trackNum = 0;
      uint8_t dirBit = 0;

      if (elevator.lastTarget > elevator.pos)
        newDir = DIR_DOWN;
      if (elevator.lastTarget < elevator.pos)
        newDir = DIR_UP;

      cmd->dir = newDir;
      cmd->target = newTarget;

      elevator.state = STATE_PENDING;
      elevator.dir = newDir;
      elevator.target = newTarget;

      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
      {
        if (newDir == DIR_UP)
        {
          writeFrameDFPlayer(SF_1001, cabinState.writtenFrame[1], cabinState.isBusy, 6);
          enableTransmit(cabinState.shouldWrite);
          writeBit(cabinState.writtenFrame[1], 5, true);
          writeBit(cabinState.writtenFrame[1], 3, false);
          writeBit(cabinState.writtenFrame[1], 2, false);
          writeBit(cabinState.writtenFrame[1], 1, true);
        }
        else
        {
          writeFrameDFPlayer(SF_1002, cabinState.writtenFrame[1], cabinState.isBusy, 6);
          enableTransmit(cabinState.shouldWrite);
          writeBit(cabinState.writtenFrame[1], 5, true);
          writeBit(cabinState.writtenFrame[1], 3, false);
          writeBit(cabinState.writtenFrame[1], 2, true);
          writeBit(cabinState.writtenFrame[1], 1, false);
        }
        xSemaphoreGive(dataMutex);
      }
    }
    else
    {
      Serial.println("It's here");
    }
  }
}

void transit(transitCommand_t cmd)
{
  bool pinUp = digitalRead(R_UP);
  bool pinDw = digitalRead(R_DW);

  if (!pinUp && !pinDw)
  {
    Serial.println("start transit");
    BRK_OFF();
    ROTATE(cmd.dir);
    elevator.isBrake = false;
    elevator.btwFloor = true;
  }
}

// central state manager

void checkUpdatePos(uint8_t reachedFloorNum)
{
  if ((reachedFloorNum > 0) && (reachedFloorNum == elevator.target))
  {

    if (elevator.state == STATE_EMERGENCY)
    {
      Serial.println("Clear Emergency!!!");
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
      {
        enableTransmit(cabinState.shouldWrite);
        writeBit(cabinState.writtenFrame[1], 4, false);
        writeBit(cabinState.writtenFrame[1], 3, true);
        writeBit(cabinState.writtenFrame[1], 2, false);
        writeBit(cabinState.writtenFrame[1], 1, false);
        writeBit(cabinState.writtenFrame[1], 0, false);
        xSemaphoreGive(dataMutex);
      }
    }

    M_STP();
    BRK_ON();
    elevator.pos = reachedFloorNum;
    elevator.state = STATE_IDLE;
    elevator.dir = DIR_NONE;
    elevator.target = 0;
    elevator.isBrake = true;

    reachedFloorNum = 0;
    Serial.printf("Reached Target Floor %d: Stopping...\n", elevator.pos);
  }
}

void vOchestrator(void *pvParams)
{

  uint32_t ulNotificationValue;
  userCommand_t userCommand; // cmdType, source of command
  transitCommand_t command;  // dir + target
  elevatorEvent_t evtType;
  uint8_t reachedFloorNum = 0;

  const uint32_t SLOW_POLL_MS = 500;
  const uint32_t SLOW_RETRY_MS = 50;

  const uint32_t FAST_POLL_MS = 50;
  const uint32_t FAST_RETRY_MS = 20;

  const unsigned long IDLE_TIMEOUT = 10 * 60 * 1000;

  uint32_t lastCommandTime = 0;
  esp_err_t result;

  for (;;)
  {

    ////////////////////////////listen to all event that would happens///////////////////////////////////

    if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &ulNotificationValue, 0) == pdPASS)
    {
      // eventListener(ulNotificationValue, &evtType);

      evtType = (elevatorEvent_t)ulNotificationValue;

      switch (evtType)
      {

      case SAFETY_BRAKE:
        // xEventGroupSetBits(xRunningEventGroup, SAFETY_BRAKE_BIT);
        // emoActivate();
        // abortMotion();
        Serial.println("sling!!!");
        elevator.state = STATE_EMERGENCY;
        // abortMotion();
        M_STP();
        BRK_ON();

        if (elevator.btwFloor == false)
        {
          elevator.target = elevator.pos;
        }
        else
        {
          if (elevator.dir == DIR_UP)
          {

            elevator.target = elevator.pos + 1;
          }
          else
          {

            elevator.target = elevator.pos;
          }
        }

        elevator.dir = DIR_UP;
        command.dir = elevator.dir;
        command.target = elevator.target;

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
          writeFrameDFPlayer(SF_1008, cabinState.writtenFrame[1], cabinState.isBusy, 6);
          enableTransmit(cabinState.shouldWrite);
          writeBit(cabinState.writtenFrame[1], 4, true);
          writeBit(cabinState.writtenFrame[1], 3, false);
          writeBit(cabinState.writtenFrame[1], 2, false);
          writeBit(cabinState.writtenFrame[1], 1, false);
          writeBit(cabinState.writtenFrame[1], 0, true);
          xSemaphoreGive(dataMutex);
        }

        transit(command);
        sendWebsocketAlert("WARNING", "Safety Sling is detected. Elevator halted.");

        break;

      case DOOR_IS_OPEN:
        xEventGroupSetBits(xRunningEventGroup, DOOR_OPEN_BIT);

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
          if (elevator.state == STATE_RUNNING)
          {
            writeFrameDFPlayer(SF_1009, cabinState.writtenFrame[1], cabinState.isBusy, 6);
            enableTransmit(cabinState.shouldWrite);
            writeBit(cabinState.writtenFrame[1], 3, true);
            writeBit(cabinState.writtenFrame[1], 2, false);
            writeBit(cabinState.writtenFrame[1], 1, false);
          }
          xSemaphoreGive(dataMutex);
        }

        Serial.println("DOOR IS OPEN!!!");
        emoActivate();
        abortMotion();
        sendWebsocketAlert("WARNING", "Door is open. Elevator paused.");
        // elevator.state = STATE_PAUSED;

        break;

      case DOOR_IS_CLOSED:
        xEventGroupClearBits(xRunningEventGroup, DOOR_OPEN_BIT);

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
          // if (elevator.state == STATE_PAUSED)
          // {
          //   writeFrameDFPlayer(SF_0000, cabinState.writtenFrame[1], cabinState.isBusy, 6);
          //   enableTransmit(cabinState.shouldWrite);
          // }
          xSemaphoreGive(dataMutex);
        }

        Serial.println("DOOR IS CLOSED!!!");
        emoDeactivate();
        // elevator.state = STATE_PENDING;
        // xEventGroupClearBits(xRunningEventGroup, DOOR_OPEN_BIT);
        break;

      case VTG_ALARM:
        xEventGroupSetBits(xRunningEventGroup, VTG_BIT);
        emoActivate();
        abortMotion();
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
          writeFrameDFPlayer(SF_1005, cabinState.writtenFrame[1], cabinState.isBusy, 6);
          enableTransmit(cabinState.shouldWrite);
          writeBit(cabinState.writtenFrame[1], 3, true);
          writeBit(cabinState.writtenFrame[1], 2, false);
          writeBit(cabinState.writtenFrame[1], 1, false);
          xSemaphoreGive(dataMutex);
        }
        break;

      case VTG_CLEAR:
        xEventGroupClearBits(xRunningEventGroup, VTG_BIT);

        break;

      case VSG_ALARM:
        xEventGroupSetBits(xRunningEventGroup, VSG_BIT);
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
          if (elevator.dir == DIR_DOWN && elevator.state == STATE_RUNNING)
          {
            writeFrameDFPlayer(SF_1004, cabinState.writtenFrame[1], cabinState.isBusy, 6);
            enableTransmit(cabinState.shouldWrite);
          }
          xSemaphoreGive(dataMutex);
        }
        if (elevator.dir == DIR_DOWN)
        {
          M_STP();
          elevator.state = STATE_PAUSED;
          sendWebsocketAlert("WARNING", "VSG is detected. Elevator paused.");
        }
        break;

      case VSG_CLEAR:
        xEventGroupClearBits(xRunningEventGroup, VSG_BIT);
        if (elevator.dir != DIR_NONE)
        {
          elevator.state = STATE_PENDING;
        }
        break;

      case MODBUS_TIMEOUT:
        // xEventGroupSetBits(xRunningEventGroup, MODBUS_DIS_BIT);

        break;

      case EMERG_PRESSED:
        // xEventGroupSetBits(xRunningEventGroup, EMERG_BIT);
        emoActivate();

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
          writeFrameDFPlayer(SF_1016, cabinState.writtenFrame[1], cabinState.isBusy, 6);
          enableTransmit(cabinState.shouldWrite);
          writeBit(cabinState.writtenFrame[1], 4, true);
          writeBit(cabinState.writtenFrame[1], 3, false);
          writeBit(cabinState.writtenFrame[1], 2, false);
          writeBit(cabinState.writtenFrame[1], 1, false);
          xSemaphoreGive(dataMutex);
        }
        break;

        // case EMERG_RELEASED:
        //   xEventGroupClearBits(xRunningEventGroup, EMERG_BIT);

        //   break;

      case NO_POWER:

        elevator.state = STATE_EMERGENCY;
        elevator.target = MIN_FLOOR;
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
          writeFrameDFPlayer(SF_1006, cabinState.writtenFrame[1], cabinState.isBusy, 6);
          enableTransmit(cabinState.shouldWrite);
          writeBit(cabinState.writtenFrame[1], 2, true);
          writeBit(cabinState.writtenFrame[1], 1, false);
          xSemaphoreGive(dataMutex);
        }
        xTaskNotify(xNoPowerLandingHandle, 1, eSetValueWithOverwrite);

        break;

      case COMMAND_CLEAR:

        break;

      case FLOOR1_REACHED:
        elevator.pos = 1;
        elevator.btwFloor = false;
        checkUpdatePos(elevator.pos);

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
          writeFrameDFPlayer(SF_1014, cabinState.writtenFrame[1], cabinState.isBusy, 6);
          enableTransmit(cabinState.shouldWrite);
          writeBit(cabinState.writtenFrame[1], 2, false);
          writeBit(cabinState.writtenFrame[1], 1, false);
          xSemaphoreGive(dataMutex);
        }

        break;

      case FLOOR2_REACHED:
        elevator.pos = 2;
        elevator.btwFloor = false;
        checkUpdatePos(elevator.pos);

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
          writeFrameDFPlayer(SF_1015, cabinState.writtenFrame[1], cabinState.isBusy, 6);
          enableTransmit(cabinState.shouldWrite);
          writeBit(cabinState.writtenFrame[1], 2, false);
          writeBit(cabinState.writtenFrame[1], 1, false);
          xSemaphoreGive(dataMutex);
        }

        break;

      case POWER_RESTORED:
        BRK_ON();
        elevator.state = STATE_IDLE;
        elevator.isBrake = true;
        xQueueReset(xQueueCommand);
        break;

      case PAUSED_CLEARED:
        // elevator.state = STATE_PENDING;

        break;

      case OVERTORQUE:
        break;

      case OVERSPEED:
        Serial.println("OverSpeed!!!!");
        emoActivate();
        M_STP();
        BRK_ON();
        sendWebsocketAlert("WARNING", "Over Speed is detected. Elevator halted.");

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
          writeFrameDFPlayer(SF_1008, cabinState.writtenFrame[1], cabinState.isBusy, 6);
          enableTransmit(cabinState.shouldWrite);
          writeBit(cabinState.writtenFrame[1], 4, true);
          writeBit(cabinState.writtenFrame[1], 3, false);
          writeBit(cabinState.writtenFrame[1], 2, false);
          writeBit(cabinState.writtenFrame[1], 1, true);
          writeBit(cabinState.writtenFrame[1], 0, true);
          xSemaphoreGive(dataMutex);
        }
        break;

      default:
        break;
      }
    }

    /////////////////////////////////slow polling count//////////////////////////////////////////

    if ((millis() - lastCommandTime > IDLE_TIMEOUT) && (elevator.state == STATE_IDLE))
    {
      if (modbusDelayTime != SLOW_POLL_MS)
      {
        modbusDelayTime = SLOW_POLL_MS;
      }

      if (modbusRetryTime != SLOW_RETRY_MS)
      {
        modbusRetryTime = SLOW_RETRY_MS;
      }
    }

    ///////////////////////////////////state of elevator////////////////////////////////////////

    switch (elevator.state)
    {

    case STATE_IDLE:

      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
      {
        // writeFrameDFPlayer(SF_1013, cabinState.writtenFrame[1], cabinState.isBusy, 6);
        enableTransmit(cabinState.shouldWrite);
        writeBit(cabinState.writtenFrame[1], 3, true);
        xSemaphoreGive(dataMutex);
      }

      //////////////////////////////////////handle any commands from user////////////////////////////

      if (xQueueReceive(xQueueCommand, &userCommand, 0) == pdPASS)
      {
        commandType_t cmd = userCommand.type;
        uint8_t targetFloor = userCommand.target;

        lastCommandTime = millis();
        modbusDelayTime = FAST_POLL_MS;
        modbusRetryTime = FAST_RETRY_MS;

        if (cmd == moveToFloor)
        {

          getDir(targetFloor, &command);
        }
      }

      break;

    case STATE_PENDING:
      if (xTimerIsTimerActive(xStartRunningTimer) == pdFALSE)
      {
        xTimerStart(xStartRunningTimer, 0);
        Serial.println(">> Timer Started (Waiting for bit...)");
      }
      break;

    case STATE_RUNNING:
      if (isSafeToRun(command.dir))
      {
        transit(command);
      }
      else
      {
        elevator.state = STATE_IDLE;
      }

      if (xQueueReceive(xQueueCommand, &userCommand, 0) == pdPASS)
      {
        commandType_t cmd = userCommand.type;
        uint8_t targetFloor = userCommand.target;
        if (cmd == userAbort)
        {
          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
          {
            writeFrameDFPlayer(SF_1003, cabinState.writtenFrame[1], cabinState.isBusy, 6);
            enableTransmit(cabinState.shouldWrite);
            writeBit(cabinState.writtenFrame[1], 2, false);
            writeBit(cabinState.writtenFrame[1], 1, false);

            xSemaphoreGive(dataMutex);
          }

          abortMotion();
        }
      }
      break;

    case STATE_PAUSED:
      break;

    case STATE_EMERGENCY:
      // emergencyHandler(evtType);
      break;

    default:
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

// void vRFReceiver(void *pvParams)
// {
//   userCommand_t userCommand; // target, type, from
//   static unsigned long lastTimeCmd1 = 0;
//   static unsigned long lastTimeCmd2 = 0;
//   const unsigned long DEBOUNCE_DELAY = 3000;

//   for (;;)
//   {
//     int cmd = 0;
//     unsigned long now = millis();

//     if (RF.available())
//     {
//       cmd = RF.getReceivedValue();
//       RF.resetAvailable();
//     }

//     switch (cmd)
//     {
//     case toFloor1:
//       if (now - lastTimeCmd1 > DEBOUNCE_DELAY)
//       {
//         lastTimeCmd1 = now;

//         // if (elevator.pos == 0)
//         // {
//         //   POS = 1;
//         // }

//         Serial.println("received toFloor1 cmd");
//         if (elevator.state == STATE_IDLE)
//         {
//           userCommand.target = 1;
//           userCommand.type = moveToFloor;
//           userCommand.from = FROM_RF;
//           xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
//         }
//       }
//       else
//       {
//         Serial.println("toFloor1 Ignored (Debounce 5s)");
//       }
//       break;

//     case toFloor2:
//       if (now - lastTimeCmd2 > DEBOUNCE_DELAY)
//       {
//         lastTimeCmd2 = now;

//         Serial.println("received toFloor2 cmd");
//         if (elevator.state == STATE_IDLE)
//         {
//           userCommand.target = 2;
//           userCommand.type = moveToFloor;
//           userCommand.from = FROM_RF;
//           xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
//         }
//       }
//       else
//       {
//         Serial.println("toFloor2 Ignored (Debounce 5s)");
//       }
//       break;

//       // case POWER_CUT:
//       //   Serial.println("received POWER CUT! cmd");
//       //   strcpy(publish_status.cmd, "POWER_CUT!");
//       //   digitalWrite(R_POWER_CUT, LOW);
//       //   xTimerStart(xPowerCutTimer, 0);
//       //   break;

//     case STOP:
//       Serial.println("received STOP cmd");
//       emoDeactivate();
//       if (elevator.state == STATE_RUNNING)
//       {
//         userCommand.target = 0;
//         userCommand.type = userAbort;
//         userCommand.from = FROM_RF;
//         xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);

//         lastTimeCmd1 = 0;
//         lastTimeCmd2 = 0;
//       }
//       break;
//     }
//     vTaskDelay(pdMS_TO_TICKS(50));
//   }
// }

void vRFReceiver(void *pvParams)
{
  userCommand_t userCommand; 
  static unsigned long lastTimeCmd1 = 0;
  static unsigned long lastTimeCmd2 = 0;
  const unsigned long DEBOUNCE_DELAY = 3000; 

  for (;;)
  {
    unsigned long cmd = 0; 
    unsigned long now = millis();

    if (RF.available())
    {
      cmd = RF.getReceivedValue();
      RF.resetAvailable();
      
      if (cmd == 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
        continue; 
      }
      
      Serial.printf(">> RF Received Code: %lu\n", cmd);

      RF_ButtonType pressedBtn = BTN_UNKNOWN;
      for (int i = 0; i < numRfKeys; i++) {
        if (rfKeys[i].rfCode == cmd) {
          pressedBtn = rfKeys[i].type;
          break; 
        }
      }

      switch (pressedBtn)
      {
        case BTN_TO_FLOOR_1:
          if (now - lastTimeCmd1 > DEBOUNCE_DELAY)
          {
            lastTimeCmd1 = now;
            Serial.println(">> RF Command: TO FLOOR 1");
            if (elevator.state == STATE_IDLE)
            {
              userCommand.target = 1;
              userCommand.type = moveToFloor;
              userCommand.from = FROM_RF;
              xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
            }
          }
          break;

        case BTN_TO_FLOOR_2:
          if (now - lastTimeCmd2 > DEBOUNCE_DELAY)
          {
            lastTimeCmd2 = now;
            Serial.println(">> RF Command: TO FLOOR 2");
            if (elevator.state == STATE_IDLE)
            {
              userCommand.target = 2;
              userCommand.type = moveToFloor;
              userCommand.from = FROM_RF;
              xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
            }
          }
          break;

        case BTN_STOP:
          Serial.println(">> RF Command: STOP");
          emoDeactivate(); 
          if (elevator.state == STATE_RUNNING)
          {
            userCommand.target = 0;
            userCommand.type = userAbort;
            userCommand.from = FROM_RF;
            xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);

            lastTimeCmd1 = 0;
            lastTimeCmd2 = 0;
          }
          break;

        // case BTN_EMERGENCY:
        //   Serial.println(">> RF Command: EMERGENCY LOCK");
        //   // xTaskNotify(xOchestratorHandle, EMERG_PRESSED, eSetValueWithOverwrite);
        //   break;

        case BTN_UNKNOWN:
        default:
          break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// timer callbacks



void vStartRunning(TimerHandle_t xTimer)
{
  if (elevator.state == STATE_PENDING)
  {
    if (elevator.dir != DIR_NONE && elevator.target != 0)
    {
      Serial.println(">> Timer Done: Valid Direction -> GO RUNNING!");
      elevator.state = STATE_RUNNING;
    }
    else
    {
      Serial.println(">> Timer Done: No Direction -> GO IDLE");
      elevator.state = STATE_IDLE;
    }
  }
}

// polling threads

void vPollingModbusMaster(void *pvParams)
{
  uint8_t INVERTER_ID = 1;
  uint8_t CABIN_ID = 2;
  uint8_t HALL_2_ID = 3;
  uint8_t VSG_ID = 4;

  uint16_t FIRST_REG_INVERTER = 28672;
  uint16_t FIRST_REG_CABIN = 1;
  uint16_t FIRST_REG_HALL = 1;
  uint16_t FIRST_REG_VSG = 1;

  uint16_t NUM_READ_INVERTER = 10;
  uint16_t NUM_READ_CABIN = 1;
  uint16_t NUM_READ_HALL = 1;
  uint16_t NUM_READ_VSG = 1;
  modbusStation_t currStation = INVERTER_STA;
  uint16_t pollingData[5][16];
  uint8_t result;
  static bool lastDoorState = true;
  static bool lastVtgState = false;
  static bool lastVsgState = false;

  static bool lastEmergStop = false;

  for (;;)
  {

    // switch (currStation)
    // {
    // case INVERTER_STA:
    //   break;

    // case CABIN_STA:
    //   if (cabinState.shouldWrite == true)
    //   {
    //     node.begin(CABIN_ID, Serial1);
    //     // Serial.print("write to cabin: ");
    //     // Serial.println(cabinState.writtenFrame[1]);
    //     result = node.writeSingleRegister(0x0001, cabinState.writtenFrame[1]);

    //     if (result == node.ku8MBSuccess)
    //     {
    //       // Serial.println("write success");
    //       cabinState.shouldWrite = false;
    //       vTaskDelay(pdMS_TO_TICKS(20));
    //     }
    //     else
    //     {
    //       Serial.println("fail to write cabin");
    //       vTaskDelay(pdMS_TO_TICKS(10));
    //     }
    //   }
    //   break;

    // case VSG_STA:
    //   if (vsgState.shouldWrite == true)
    //   {
    //     node.begin(VSG_ID, Serial1);
    //     node.writeSingleRegister(0x0001, vsgState.writtenFrame[1]);
    //     vsgState.shouldWrite = false;
    //     vTaskDelay(pdMS_TO_TICKS(20));
    //   }
    //   break;

    // default:
    //   break;
    // }

    // --- 2. POLLING STATE MACHINE:
    if (xSemaphoreTake(modbusMutex, portMAX_DELAY) == pdTRUE)
    {

      switch (currStation)
      {
      case INVERTER_STA:
        if (readDataFrom(INVERTER_ID, 28672, 10, pollingData[INVERTER_ID]))
        {
          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
          {
            inverterState.running_hz = pollingData[INVERTER_ID][1];
            inverterState.torque = pollingData[INVERTER_ID][6];
            inverterState.digitalInput = pollingData[INVERTER_ID][7];
            xSemaphoreGive(dataMutex);
          }
        }
        currStation = CABIN_STA;
        break;

      case CABIN_STA:
        // if (readDataFrom(CABIN_ID, 0, 1, pollingData[CABIN_ID]))
        // {
        //   if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        //   {
        //     cabinState.isDoorClosed = pollingData[CABIN_ID][0] & (1 << 0);
        //     cabinState.isAim2 = pollingData[CABIN_ID][0] & (1 << 1);
        //     cabinState.isAim1 = pollingData[CABIN_ID][0] & (1 << 2);
        //     cabinState.isUserStop = pollingData[CABIN_ID][0] & (1 << 3);
        //     cabinState.isEmergStop = pollingData[CABIN_ID][0] & (1 << 4);
        //     cabinState.isBusy = pollingData[CABIN_ID][0] & (1 << 5);
        //     cabinState.vtgAlarm = pollingData[CABIN_ID][0] & (1 << 6);
        //     xSemaphoreGive(dataMutex);
        //   }
        // }

        // if (cabinState.isAim2)
        // {
        //   Serial.println("received toFloor1 cmd");
        //   if (elevator.state == STATE_IDLE)
        //   {
        //     userCommand_t userCommand; // target, type, from
        //     userCommand.target = 2;
        //     userCommand.type = moveToFloor;
        //     userCommand.from = FROM_CABIN;
        //     xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
        //   }
        // }

        // if (cabinState.isAim1)
        // {
        //   Serial.println("received toFloor1 cmd");
        //   if (elevator.state == STATE_IDLE)
        //   {
        //     userCommand_t userCommand; // target, type, from
        //     userCommand.target = 1;
        //     userCommand.type = moveToFloor;
        //     userCommand.from = FROM_CABIN;
        //     xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
        //   }
        // }

        // if (cabinState.isUserStop)
        // {
        //   Serial.println("received STOP cmd");
        //   emoDeactivate();
        //   if (elevator.state == STATE_RUNNING)
        //   {
        //     userCommand_t userCommand; // target, type, from
        //     userCommand.target = 0;
        //     userCommand.type = userAbort;
        //     userCommand.from = FROM_CABIN;
        //     xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
        //   }
        // }

        // if (cabinState.isDoorClosed != lastDoorState)
        // {

        //   if (cabinState.isDoorClosed == true)
        //   {

        //     xTaskNotify(xOchestratorHandle, DOOR_IS_CLOSED, eSetValueWithOverwrite);
        //   }
        //   else
        //   {
        //     if (elevator.state != STATE_EMERGENCY)
        //     {
        //       xTaskNotify(xOchestratorHandle, DOOR_IS_OPEN, eSetValueWithOverwrite);
        //     }
        //   }
        //   lastDoorState = cabinState.isDoorClosed;
        // }

        // if (cabinState.isEmergStop)
        // {
        //   emoActivate();
        // }

        // if (cabinState.isEmergStop != lastEmergStop)
        // {
        //   if (cabinState.isEmergStop == true)
        //   {
        //     xTaskNotify(xOchestratorHandle, EMERG_PRESSED, eSetValueWithOverwrite);
        //   }
        //   lastEmergStop = cabinState.isEmergStop;
        // }

        // if (cabinState.vtgAlarm != lastVtgState)
        // {
        //   if (cabinState.vtgAlarm == true && elevator.state != STATE_EMERGENCY)
        //   {
        //     xTaskNotify(xOchestratorHandle, VTG_ALARM, eSetValueWithOverwrite);
        //   }
        //   else
        //   {
        //     xTaskNotify(xOchestratorHandle, VTG_CLEAR, eSetValueWithOverwrite);
        //   }
        //   lastVtgState = cabinState.vtgAlarm;
        // }

        currStation = VSG_STA;
        break;

      case VSG_STA:
        // if (readDataFrom(VSG_ID, 0, 1, pollingData[VSG_ID]))
        // {
        //   if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        //   {
        //     vsgState.shouldPause = pollingData[VSG_ID][0] & (1 << 0);
        //     xSemaphoreGive(dataMutex);
        //   }
        // }

        // if (vsgState.shouldPause != lastVsgState)
        // {

        //   if (vsgState.shouldPause == true)
        //   {
        //     xTaskNotify(xOchestratorHandle, VSG_ALARM, eSetValueWithOverwrite);
        //   }
        //   else
        //   {
        //     xTaskNotify(xOchestratorHandle, VSG_CLEAR, eSetValueWithOverwrite);
        //   }

        //   lastVsgState = vsgState.shouldPause;
        // }

        currStation = INVERTER_STA;
        break;
      }
      xSemaphoreGive(modbusMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(modbusDelayTime));
  }
}

void vESP_NOW(void *pvParams)
{

  static bool lastDoorState = true;
  static bool lastVtgState = false;
  static bool lastVsgState = false;
  static bool lastEmergStop = false;

  uint8_t stationID;
  uint8_t pollFromStation;
  uint16_t commandFromMaster;
  uint16_t replyToMaster;
  
  bool isConnected_CABIN = false;
  bool isConnected_VSG = false;

  modbusStation_t currStation = CABIN_STA;

  esp_err_t result;

  for (;;)
  {

    /////////////////////////////////////write block////////////////////////////////////

    if (cabinState.shouldWrite == true)
    {

      sendData.fromID = MASTER_ID;
      sendData.commandFrame = cabinState.writtenFrame[1];
      sendData.responseFrame = 0;
      sendData.shouldResponse = false;

      result = esp_now_send(CABIN_MAC, (uint8_t *)&sendData, sizeof(sendData));
      if (result == ESP_OK)
      {
        cabinState.shouldWrite = false;
        // Serial.println("boardcast success!");
      }
    }

    /////////////////////////////////////polling block////////////////////////////////////

    switch (currStation)
    {
    case CABIN_STA:
      sendData.fromID = MASTER_ID;
      // sendData.commandFrame = 0;
      sendData.responseFrame = 0;
      sendData.shouldResponse = true;
      
      if(isConnected_CABIN == false){
      result = esp_now_send(CABIN_MAC, (uint8_t *)&sendData, sizeof(sendData));
      }
      // if (result == ESP_OK)
      // {
      //   // Serial.println("boardcast success!");
      //   recvData.pollFromStation = 0;
      // }
      currStation = VSG_STA;
      break;

    case VSG_STA:
      sendData.fromID = MASTER_ID;
      // sendData.commandFrame = 0;
      sendData.responseFrame = 0;
      sendData.shouldResponse = true;

      result = esp_now_send(VSG_MAC, (uint8_t *)&sendData, sizeof(sendData));
      // if (result == ESP_OK)
      // {
      //   // Serial.println("boardcast success!");
      // }
      currStation = CABIN_STA;
      break;

    default:
      break;
    }

    /////////////////////////////////////action on bits block////////////////////////////////////

    if (recvData.fromID == 2)
    { 
      isConnected_CABIN = true;
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
      {
        cabinState.isDoorClosed = recvData.responseFrame & (1 << 0);
        cabinState.isAim2 = recvData.responseFrame & (1 << 1);
        cabinState.isAim1 = recvData.responseFrame & (1 << 2);
        cabinState.isUserStop = recvData.responseFrame & (1 << 3);
        cabinState.isEmergStop = recvData.responseFrame & (1 << 4);
        cabinState.isBusy = recvData.responseFrame & (1 << 5);
        cabinState.vtgAlarm = recvData.responseFrame & (1 << 6);
        xSemaphoreGive(dataMutex);
      }
    }

    if (recvData.fromID == 4)
    {
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
      {
        vsgState.shouldPause = recvData.responseFrame & (1 << 0);
        xSemaphoreGive(dataMutex);
      }
    }

    if (cabinState.isAim2)
    {
      Serial.println("received toFloor1 cmd");
      if (elevator.state == STATE_IDLE)
      {
        userCommand_t userCommand; // target, type, from
        userCommand.target = 2;
        userCommand.type = moveToFloor;
        userCommand.from = FROM_CABIN;
        xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
      }
    }

    if (cabinState.isAim1)
    {
      Serial.println("received toFloor1 cmd");
      if (elevator.state == STATE_IDLE)
      {
        userCommand_t userCommand; // target, type, from
        userCommand.target = 1;
        userCommand.type = moveToFloor;
        userCommand.from = FROM_CABIN;
        xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
      }
    }

    if (cabinState.isUserStop)
    {
      Serial.println("received STOP cmd");
      emoDeactivate();
      if (elevator.state == STATE_RUNNING)
      {
        userCommand_t userCommand; // target, type, from
        userCommand.target = 0;
        userCommand.type = userAbort;
        userCommand.from = FROM_CABIN;
        xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
      }
    }

    if (cabinState.isDoorClosed != lastDoorState)
    {

      if (cabinState.isDoorClosed == false)
      {

        xTaskNotify(xOchestratorHandle, DOOR_IS_CLOSED, eSetValueWithOverwrite);
      }
      else
      {
        if (elevator.state != STATE_EMERGENCY)
        {
          xTaskNotify(xOchestratorHandle, DOOR_IS_OPEN, eSetValueWithOverwrite);
        }
      }
      lastDoorState = cabinState.isDoorClosed;
    }

    if (cabinState.isEmergStop != lastEmergStop)
    {
      if (cabinState.isEmergStop == true)
      {
        xTaskNotify(xOchestratorHandle, EMERG_PRESSED, eSetValueWithOverwrite);
      }
      lastEmergStop = cabinState.isEmergStop;
    }

    if (cabinState.vtgAlarm != lastVtgState)
    {
      if (cabinState.vtgAlarm == true && elevator.state != STATE_EMERGENCY)
      {
        xTaskNotify(xOchestratorHandle, VTG_ALARM, eSetValueWithOverwrite);
      }
      else
      {
        xTaskNotify(xOchestratorHandle, VTG_CLEAR, eSetValueWithOverwrite);
      }
      lastVtgState = cabinState.vtgAlarm;
    }

    if (vsgState.shouldPause != lastVsgState)
    {

      if (vsgState.shouldPause == true)
      {
        xTaskNotify(xOchestratorHandle, VSG_ALARM, eSetValueWithOverwrite);
      }
      else
      {
        xTaskNotify(xOchestratorHandle, VSG_CLEAR, eSetValueWithOverwrite);
      }

      lastVsgState = vsgState.shouldPause;
    }

    vTaskDelay(pdMS_TO_TICKS(modbusDelayTime));
  }
}

void vPollingFloorSensor1(void *pvParams) // first floor sensor
{
  uint8_t floorSensor1_counter = 0;
  const uint8_t STABLE_THRESHOLD = 20;
  bool hasNotified = false;

  for (;;)
  {
    bool raw_floorSensor1 = (digitalRead(floorSensor1) == LOW);
    if (raw_floorSensor1)
    {
      if (floorSensor1_counter < STABLE_THRESHOLD)
        floorSensor1_counter++;
    }
    else
    {
      floorSensor1_counter = 0;
      hasNotified = false;
    }

    bool isAtFloor1 = (floorSensor1_counter >= STABLE_THRESHOLD);

    if (isAtFloor1 == true && hasNotified == false)
    {
      xTaskNotify(xOchestratorHandle, FLOOR1_REACHED, eSetValueWithOverwrite);
      hasNotified = true;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void vPollingFloorSensor2(void *pvParams) // first floor sensor
{
  uint8_t floorSensor2_counter = 0;
  const uint8_t STABLE_THRESHOLD = 20;
  bool hasNotified = false;

  for (;;)
  {
    bool raw_floorSensor2 = (digitalRead(floorSensor2) == LOW);
    if (raw_floorSensor2)
    {
      if (floorSensor2_counter < STABLE_THRESHOLD)
        floorSensor2_counter++;
    }
    else
    {
      floorSensor2_counter = 0;
      hasNotified = false;
    }

    bool isAtFloor2 = (floorSensor2_counter >= STABLE_THRESHOLD);

    if (isAtFloor2 == true && hasNotified == false)
    {
      xTaskNotify(xOchestratorHandle, FLOOR2_REACHED, eSetValueWithOverwrite);
      hasNotified = true;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void vPollingNoPower(void *pvParams)
{
  uint8_t stable_counter = 0;
  const uint8_t THRESHOLD = 20;
  bool lastIsNoPower = false;

  for (;;)
  {
    bool raw_noPower = (digitalRead(NoPower) == LOW);

    if (raw_noPower)
    {
      if (stable_counter < THRESHOLD)
        stable_counter++;
    }
    else
    {
      if (stable_counter > 0)
        stable_counter--;
    }

    bool currentIsNoPower = (stable_counter >= THRESHOLD);
    bool currentIsPowerOK = (stable_counter == 0);

    if (currentIsNoPower && !lastIsNoPower)
    {
      Serial.println(">> Event: Power LOST!");
      xTaskNotify(xOchestratorHandle, NO_POWER, eSetValueWithOverwrite);
      lastIsNoPower = true;
    }

    else if (currentIsPowerOK && lastIsNoPower)
    {
      Serial.println(">> Event: Power RESTORED!");
      xTaskNotify(xOchestratorHandle, POWER_RESTORED, eSetValueWithOverwrite);
      lastIsNoPower = false;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void vPollingSafetySling(void *pvParams)
{
  uint8_t safetySling_counter = 0;
  const uint8_t STABLE_THRESHOLD = 20;
  bool hasNotified = false;

  for (;;)
  {
    bool raw_safetySling = (digitalRead(safetySling) == LOW);
    if (raw_safetySling)
    {
      if (safetySling_counter < STABLE_THRESHOLD)
        safetySling_counter++;
    }
    else
    {
      safetySling_counter = 0;
      hasNotified = false;
    }

    bool isSafetyActive = (safetySling_counter >= STABLE_THRESHOLD);

    if (isSafetyActive == true && hasNotified == false)
    {
      xTaskNotify(xOchestratorHandle, SAFETY_BRAKE, eSetValueWithOverwrite);
      hasNotified = true;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void vPollingSpeedGovernor(void *pvParams)
{
  bool lastPinState = digitalRead(speedGovernor);
  unsigned long lastEdgeTime = millis();

  for (;;)
  {
    bool currentPinState = digitalRead(speedGovernor);

    if (lastPinState == HIGH && currentPinState == LOW)
    {
      unsigned long now = millis();
      unsigned long diff = now - lastEdgeTime;
      lastEdgeTime = now;

      if (diff > 50)
      {
        Serial.print("diff is: ");
        Serial.println(diff);
        if (diff < upper_bound_speed_interval)
        {
          overSpeed_counter++;
        }
        else if (diff <= lower_bound_speed_interval)
        {
          overSpeed_counter = 0;
        }
        else
        {
          overSpeed_counter = 0;
        }

        if (overSpeed_counter >= overSpeed_threshold)
        {
          overSpeed_counter = 0;
          Serial.println(">> OVERSPEED DETECTED (via Polling) !!!");
          xTaskNotify(xOchestratorHandle, OVERSPEED, eSetValueWithOverwrite);
        }
      }
    }

    lastPinState = currentPinState;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
// safety task handlers

void vNoPowerLanding(void *pvParams)
{
  for (;;)
  {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0)
    {
      // Serial.println(">> EMERGENCY: Landing Sequence Started!");

      while (!(elevator.pos == MIN_FLOOR && elevator.btwFloor == false))
      {

        if (elevator.state != STATE_EMERGENCY)
          break;

        BRK_OFF();
        elevator.isBrake = false;
        Serial.println("Brake OFF !");
        vTaskDelay(pdMS_TO_TICKS(800));

        // if (elevator.pos == MIN_FLOOR && elevator.btwFloor == false) {
        //      Serial.println(">> Reached Floor 1!");
        //      break;
        // }

        if (elevator.state != STATE_EMERGENCY)
          break;

        BRK_ON();
        elevator.isBrake = true;

        Serial.println("Brake ON !");
        vTaskDelay(pdMS_TO_TICKS(1800));
      }

      BRK_ON();
      elevator.isBrake = true;
    }
    // vTaskDelay(10);
  }
}

void vClearCommand(void *pvParams)
{
  for (;;)
  {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0)
    {
      // Serial.println(">> Clear Command Received");
    }
  }
}

// other threads

void vReconnectTask(void *pvParams)
{
  for (;;)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      // Protect Check/Connect with Mutex
      if (xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE)
      {
        if (!mqttClient.connected())
        {
          Serial.print("MQTT connecting...");

          String clientId = "ESP32-" + String(random(0xffff), HEX); // Unique ID

          if (mqttClient.connect(clientId.c_str()))
          {
            Serial.println("connected");
            // Re-subscribe here if needed
          }
          else
          {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
          }
        }

        // IMPORTANT: loop() must be called frequently to maintain connection
        if (mqttClient.connected())
        {
          mqttClient.loop();
        }

        xSemaphoreGive(mqttMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
  }
}

void vPublishTask(void *pvParams)
{
  char jsonBuf[512];

  status_t lastPubState;
  memset(&lastPubState, 0, sizeof(status_t));

  unsigned long lastForcePubTime = 0;
  const unsigned long FORCE_PUB_INTERVAL = 5000;

  for (;;)
  {
    status_t currState;
    uint16_t hz = 0, tq = 0, di = 0;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      currState = elevator;
      hz = inverterState.running_hz;
      tq = inverterState.torque;
      di = inverterState.digitalInput;
      xSemaphoreGive(dataMutex);
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    bool stateChanged = false;
    if (currState.pos != lastPubState.pos ||
        currState.state != lastPubState.state ||
        currState.dir != lastPubState.dir ||
        currState.target != lastPubState.target ||
        currState.isBrake != lastPubState.isBrake ||
        currState.btwFloor != lastPubState.btwFloor)
    {
      stateChanged = true;
    }

    if (stateChanged || (millis() - lastForcePubTime > FORCE_PUB_INTERVAL))
    {
      snprintf(mqtt_topic, sizeof(mqtt_topic), "%s%s%s%s",
               KIT_topic, UT_case, system_status, elevator_status);

      snprintf(jsonBuf, sizeof(jsonBuf),
               "{"
               "\"floorValue\":%d,"
               "\"state\":%d,"
               "\"dir\":%d,"
               "\"targetFloor\":%d,"
               "\"isBrake\":%s,"
               "\"btwFloor\":%s,"
               "\"emo\":%s,"
               "\"inv_hz\":%d,"
               "\"inv_tq\":%d,"
               "\"inv_di\":%d"
               "}",
               currState.pos,
               currState.state,
               currState.dir,
               currState.target,
               currState.isBrake ? "true" : "false",
               currState.btwFloor ? "true" : "false",
               (digitalRead(EMO) == HIGH) ? "true" : "false",
               hz, tq, di);

      if (xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE)
      {
        if (mqttClient.connected())
        {
          mqttClient.publish(mqtt_topic, jsonBuf);
        }
        xSemaphoreGive(mqttMutex);
      }

      lastPubState = currState;
      lastForcePubTime = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vStatusLogger(void *pvParams)
{
  int lastPOS = 0;
  bool lastBtw = false;

  for (;;)
  {

    if (elevator.pos != lastPOS || elevator.btwFloor != lastBtw)
    {
      // if (elevator.pos != lastPOS)
      // {

      saveStatus();

      lastPOS = elevator.pos;
      lastBtw = elevator.btwFloor;
      // }
    }
    vTaskDelay(3000);
  }
}

void vUpdatePage(void *pvParams)
{
  static char jsonBuf[256];

  for (;;)
  {

    m_websocketserver.loop();

    // uint8_t pos;
    // status_t statusCopy;

    // pos = POS;

    // if (xSemaphoreTake(hasChangedMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    // {
    //   statusCopy = publish_status;
    //   xSemaphoreGive(hasChangedMutex);
    // }
    // else
    // {
    //   vTaskDelay(pdMS_TO_TICKS(100));
    //   continue;
    // }

    snprintf(
        jsonBuf,
        sizeof(jsonBuf),
        "{\"floorValue\":%d,"
        "\"state\":%d,"
        "\"up\":%s,"
        "\"down\":%s,"
        "\"targetFloor\":%d,"
        "\"btwFloor\":%s,"
        "\"emo\":%s}",
        elevator.pos,
        elevator.state,
        (elevator.dir == DIR_UP) ? "true" : "false",
        (elevator.dir == DIR_DOWN) ? "true" : "false",
        elevator.target,
        (elevator.btwFloor) ? "true" : "false",
        (digitalRead(EMO) == HIGH) ? "true" : "false" //
    );

    m_websocketserver.broadcastTXT(jsonBuf, strlen(jsonBuf));

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(38400, SERIAL_8E1, PIN_RX, PIN_TX);

  while (!Serial)
    ;
  Serial.println("Welcome to Ximplex LuckD");
  delay(500);

  // ############################### SPIFFS STARTUP #######################################
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  loadStatus();
  delay(500);

  RF.enableReceive(RFReceiver);
  delay(500);

  bool m_autoconnected_attempt_succeeded = false;
  m_autoconnected_attempt_succeeded = connectAttempt("", ""); // uses SSID/PWD stored in ESP32 secret memory.....
  if (!m_autoconnected_attempt_succeeded)
  {
    // try SSID/PWD from file...
    Serial.println("Failed to connect.");
    String m_filenametopass = "/credentials.JSON";
    m_autoconnected_attempt_succeeded = readSSIDPWDfile(m_filenametopass);
  }
  if (!m_autoconnected_attempt_succeeded)
  {
    setUpAPService();
    runWifiPortal();
  }

  MDNS.begin("ximplex_websocket");

  server.reset(); // try putting this in setup
  configureserver();

  m_websocketserver.begin();
  m_websocketserver.onEvent(onWebSocketEvent); // Start WebSocket server and assign callback

  delay(500);

  // wifiClient.setInsecure();
  Serial.println(WiFi.localIP());
  setupMQTT();

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed");
  }
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerBroadcast = {};
  memcpy(peerBroadcast.peer_addr, broadcastAddress, 6);
  peerBroadcast.channel = 0;
  peerBroadcast.encrypt = false;
  if (esp_now_add_peer(&peerBroadcast) != ESP_OK)
  {
    Serial.println("Failed to add broadcast peer");
  }

  esp_now_peer_info_t peerCabin = {};
  memcpy(peerCabin.peer_addr, CABIN_MAC, 6);
  peerCabin.channel = 0;
  peerCabin.encrypt = false;
  if (esp_now_add_peer(&peerCabin) != ESP_OK)
  {
    Serial.println("Failed to add Cabin peer");
  }

  esp_now_peer_info_t peerVsg = {};
  memcpy(peerVsg.peer_addr, VSG_MAC, 6);
  peerVsg.channel = 0;
  peerVsg.encrypt = false;
  if (esp_now_add_peer(&peerVsg) != ESP_OK)
  {
    Serial.println("Failed to add VSG peer");
  }

  pinMode(WIFI_READY, OUTPUT);
  pinMode(R_UP, OUTPUT);
  pinMode(R_DW, OUTPUT);
  pinMode(R_POWER_CUT, OUTPUT);
  pinMode(BRK, OUTPUT);
  pinMode(EMO, OUTPUT);

  pinMode(floorSensor1, INPUT_PULLUP); // normal pull-up by using external resistor
  pinMode(floorSensor2, INPUT_PULLUP); // normal pull-up by using external resistor
  // attachInterrupt(floorSensor1, ISR_LowerLim, FALLING);
  // attachInterrupt(floorSensor2, ISR_UpperLim, FALLING);

  pinMode(NoPower, INPUT_PULLUP); // normal pull-up by using external resistor
  // attachInterrupt(NP, ISR_Landing, FALLING);
  pinMode(safetySling, INPUT_PULLUP);

  pinMode(speedGovernor, INPUT_PULLUP);
  // attachInterrupt(speedGovernor, ISR_OverSpeed, FALLING);

  // attachInterrupt(safetySling, ISR_Safety, FALLING);
  // pinMode(CS, OUTPUT);
  // digitalWrite(CS, HIGH); // rf always waked up

  // pinMode(RST_SYS, INPUT_PULLUP);
  // attachInterrupt(RST_SYS, ISR_ResetSystem, FALLING);

  // digitalWrite(R_POWER_CUT, HIGH);

  xRunningEventGroup = xEventGroupCreate();

  if (xRunningEventGroup != NULL)
  {
    xEventGroupClearBits(xRunningEventGroup, 0xFFFFFF); // ล้างทุก Bit
  }
  else
  {
    Serial.println("Error: Cannot create Event Group!");
  }

  // xSemTransit = xSemaphoreCreateBinary();
  // xSemDoneTransit = xSemaphoreCreateBinary();
  // xSemLanding = xSemaphoreCreateBinary();

  // xTransitMutex = xSemaphoreCreateMutex();

  mqttMutex = xSemaphoreCreateMutex();
  modbusMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  // hasChangedMutex = xSemaphoreCreateMutex();

  xQueueCommand = xQueueCreate(10, sizeof(userCommand_t));
  // xQueueGetDirection = xQueueCreate(1, sizeof(uint8_t));

  xStartRunningTimer = xTimerCreate("startRunning", WAIT_TO_RUNNING_MS, pdFALSE, NULL, vStartRunning);

  xTaskCreate(vOchestrator, "Ochestrator", 4096, NULL, 5, &xOchestratorHandle);

  // xTaskCreate(vSafetySling, "SafetySling", 1536, NULL, 6, &xSafetySlingHandle);

  xTaskCreate(vNoPowerLanding, "NoPowerLanding", 1536, NULL, 4, &xNoPowerLandingHandle);
  xTaskCreate(vClearCommand, "ClearCommand", 1536, NULL, 4, &xClearCommandHandle);

  // xTaskCreate(vProcessData, "ProcessData", 4093, NULL, 5, &xProcessDataHandle);
  xTaskCreate(vPollingModbusMaster, "ModbusMaster", 4096, NULL, 5, &xPollingModbusMasterHandle);
  xTaskCreate(vESP_NOW, "ESP_NOW", 4096, NULL, 5, NULL);
  // xTaskCreate(vPollingModbus, "PollingModbus", 3072, NULL, 5, &xPollingModbusHandle);
  // xTaskCreate(vWriteStation, "WriteStation", 3072, NULL, 4, &xWriteStationHandle);

  xTaskCreate(vPollingFloorSensor1, "PollingFloorSensor", 2048, NULL, 4, &xPollingFloorSensor1Handle);
  xTaskCreate(vPollingFloorSensor2, "PollingFloorSensor2", 2048, NULL, 4, &xPollingFloorSensor2Handle);
  xTaskCreate(vPollingNoPower, "PollingNoPower", 2048, NULL, 4, &xPollingNoPowerHandle);
  xTaskCreate(vPollingSafetySling, "PollingSafetySling", 2048, NULL, 4, &xPollingSafetySlingHandle);
  xTaskCreate(vPollingSpeedGovernor, "PollingSpeedGovernor", 2048, NULL, 4, &xPollingSpeedGovernor);

  xTaskCreate(vReconnectTask, "ReconnectTask", 4096, NULL, 2, NULL);
  xTaskCreate(vPublishTask, "PublishTask", 4096, NULL, 2, NULL);
  xTaskCreate(vStatusLogger, "StatusLogger", 2048, NULL, 2, NULL);
  xTaskCreate(vUpdatePage, "UpdatePage", 4096, NULL, 2, NULL);
  xTaskCreate(vRFReceiver, "RFReceiver", 3072, NULL, 2, &xRFReceiverHandleHandle);

  delay(500);
  digitalWrite(WIFI_READY, HIGH);
  Serial.print("Heap free memory (in bytes)= ");
  Serial.println(ESP.getFreeHeap());
  Serial.println(F("Setup complete."));
  delay(500);

  M_STP();
  BRK_ON();
  emoDeactivate();

  Serial.print("wifi chan: ");
  Serial.println(WiFi.channel());
  Serial.print("My MAC is: ");
  Serial.println(WiFi.macAddress());
}

void loop()
{
  static unsigned long lastDebugTime = 0;
  const unsigned long DEBUG_INTERVAL = 2000;

  if (millis() - lastDebugTime > DEBUG_INTERVAL)
  {
    lastDebugTime = millis();

    status_t dbg_elevator;
    cabin_t dbg_cabin;
    inverter_t dbg_inverter;
    vsg_t dbg_vsg;
    EventBits_t dbg_events = 0;

    // --- BLOCK 1: Snapshot Data (Copy Thread-Safe) ---
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      dbg_elevator = elevator;
      dbg_cabin = cabinState;
      dbg_inverter = inverterState;
      dbg_vsg = vsgState;
      xSemaphoreGive(dataMutex);
    }

    // //   // Event Group (Safety Flags)
    if (xRunningEventGroup != NULL)
    {
      dbg_events = xEventGroupGetBits(xRunningEventGroup);
    }

    // //   Serial.println("\n--- [ SYSTEM DEBUGGER ] ---");
    // //   Serial.printf("Uptime: %lu ms | Heap: %u bytes\n", millis(), ESP.getFreeHeap());

    //   // --- BLOCK 2: Elevator Logic State ---
    Serial.println(">> [LOGIC STATE]");
    Serial.printf("  State: %d (%d)\n", dbg_elevator.state, dbg_elevator.state);
    Serial.printf("  Pos: %d | Target: %d | LastTarget: %d\n", elevator.pos, elevator.target, dbg_elevator.lastTarget);
    Serial.printf("  Dir: %d | Brake Logic: %s\n", dbg_elevator.dir, dbg_elevator.isBrake ? "LOCKED" : "RELEASED");
    Serial.print("btwFloor: ");
    Serial.println(elevator.btwFloor);
    // //   // --- BLOCK 3: Hardware I/O  ---
    // //   Serial.println(">> [HARDWARE I/O]");
    // //   Serial.printf("  Floor Sensors: FL1=%d, FL2=%d (0=Active)\n", digitalRead(floorSensor1), digitalRead(floorSensor2));
    // //   Serial.printf("  Power Monitor: %d (0=NoPower)\n", digitalRead(NoPower));
    // //   Serial.printf("  Relays: UP=%d, DW=%d, BRK=%d, EMO=%d\n", digitalRead(R_UP), digitalRead(R_DW), digitalRead(BRK), digitalRead(EMO));

    // --- BLOCK 4: Modbus Data (Slave Status) ---
    Serial.println(">> [MODBUS DATA]");
    Serial.printf("  [INV] Hz: %d | Torque: %d | DI: %d\n", dbg_inverter.running_hz, dbg_inverter.torque, dbg_inverter.digitalInput);
    Serial.printf("  [CABIN] DoorClosed: %d | Emerg: %d | Busy: %d\n", dbg_cabin.isDoorClosed, dbg_cabin.isEmergStop, dbg_cabin.isBusy);
    Serial.printf("  [VSG] PauseReq: %d | Alarm[0]: %d\n", dbg_vsg.shouldPause, dbg_vsg.isAlarm[0]);

    // // //   // --- BLOCK 5: Event Flags (Safety Blocks) ---
    Serial.println(">> [SAFETY FLAGS]");
    Serial.printf("  Raw Bits: 0x%06X\n", dbg_events);
    Serial.printf("  - Door Open: %d\n", (dbg_events & DOOR_OPEN_BIT) ? 1 : 0);
    Serial.printf("  - Modbus Dis: %d\n", (dbg_events & MODBUS_DIS_BIT) ? 1 : 0);
    Serial.printf("  - Emergency: %d\n", (dbg_events & EMERG_BIT) ? 1 : 0);

    Serial.println(digitalRead(27));
    Serial.println(digitalRead(14));
    Serial.println(overSpeed_counter);
    Serial.println("---------------------------");
  }
}
