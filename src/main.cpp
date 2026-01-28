// Libraries
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

// gpio
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
#define NP 25
#define CS 21
#define RST_SYS 4
#define MB_RX 26
#define MB_TX 27

// rf codes
#define toFloor1 174744
#define toFloor2 174740
#define POWER_CUT 174738
#define STOP 174737

#define GO_UP 5259521
#define GO_DW 5259528
#define GO_STOP 5259524

// ms timings
#define DEBOUNCE_MS 1000
#define BRAKE_MS 2000
#define WAIT_MS 300
#define POWER_CUT_MS 3000

#define Inverter_slaveID 1
#define H0_INV 28672
#define INV_NUM 19

const char *mqtt_broker = "kit.flinkone.com";
const int mqtt_port = 1883; // unencrypt
char *KIT_topic = "kit";
char *UT_case = "/UT_500";
char *system_status = "/sys_v2";
char *elevator_status = "/ele_status";
char *inverter_status = "/inv_status";
char mqtt_topic[64];

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
ModbusMaster node;
RCSwitch RF = RCSwitch();
Preferences preferences;

SemaphoreHandle_t mqttMutex; // Mutex to protect MQTT client
SemaphoreHandle_t hasChangedMutex;
SemaphoreHandle_t xTransitMutex;
SemaphoreHandle_t xSemTransit = NULL;
SemaphoreHandle_t xSemDoneTransit = NULL;
SemaphoreHandle_t xSemLanding = NULL;

QueueHandle_t xQueueTransit;
QueueHandle_t xQueueGetDirection;

TimerHandle_t xDisbrakeTimer;
TimerHandle_t xWaitTimer;
TimerHandle_t xStopTransitTimer;
TimerHandle_t xPowerCutTimer;

TaskHandle_t xLandingHandle;
TaskHandle_t xPublishHandle;

AsyncWebServer server(80);
WebSocketsServer m_websocketserver = WebSocketsServer(81);
int last_time_loop_called = millis();
int last_time_sent_websocket_server = millis();
float m_websocket_send_rate = 1.0; // Hz, how often to send a test point to websocket...
bool m_send_websocket_test_data_in_loop = false;

// WifiTool object
int WAIT_FOR_WIFI_TIME_OUT = 6000;
const char *PARAM_MESSAGE = "message"; // message server receives from client
std::unique_ptr<DNSServer> dnsServer;
std::unique_ptr<AsyncWebServer> m_wifitools_server;
const byte DNS_PORT = 53;
bool restartSystem = false;
String temp_json_string = "";

volatile uint8_t POS = -1;
volatile uint8_t defaultPOS = 1;
volatile uint8_t TARGET = 0;

uint8_t MAX_FLOOR = 2;
uint8_t MIN_FLOOR = 1;
uint8_t lastDiffTarget = 0;

uint32_t ws_cmd_value = 0;
uint32_t FloorToFloor_MS = 18500; // only for up dir

int hreg[8][32];

volatile unsigned long lastFloorISR_1 = 0;
volatile unsigned long lastFloorISR_2 = 0;
volatile unsigned long lastFloorISR_3 = 0;
volatile unsigned long lastLowerLim = 0;
volatile unsigned long lastUpperLim = 0;
volatile unsigned long lastNoPowerISR = 0;
volatile unsigned long lastResetSysISR = 0;

bool emergency = false;
bool btwFloor = false;
bool ws_cmd = false;
bool hasChanged = true;

direction_t lastDir = STAY;

status_t publish_status = {
    "NULL",
    true,
    STAY,
    IDLE,
    0,
    false,
    NORMAL,
    0};


state_t moving_state = IDLE;
read_state curr_slave = INV;
volatile TRANSIT transit;

//******************* END VARIABLE DECLARATIONS**************************

inline void ROTATE(direction_t dir)
{
  if (dir == UP)
  {
    digitalWrite(R_UP, HIGH);
    delay(100);
    digitalWrite(R_DW, LOW);
    Serial.println("Move UP");
  }
  else if (dir == DOWN)
  {
    digitalWrite(R_UP, LOW);
    delay(100);
    digitalWrite(R_DW, HIGH);
    Serial.println("Move DOWN");
  }
  delay(100);
}

inline void BRK_ON()
{
  digitalWrite(BRK, LOW);
  delay(100);
  Serial.println("Brake ON");
}

inline void BRK_OFF()
{
  digitalWrite(BRK, HIGH);
  Serial.println("Brake OFF");
}

inline void M_STP()
{
  digitalWrite(R_UP, LOW);
  // delay(100);
  digitalWrite(R_DW, LOW);
  // delay(100);
  Serial.println("Motor Stop");
}

inline void M_UP()
{
  digitalWrite(R_UP, HIGH);
  Serial.println("Move UP");
}

inline void M_DW()
{
  digitalWrite(R_DW, HIGH);
  Serial.println("Move Down");
}

String statusToJson_ELE(const status_t status)
{
  // Create a JSON document (adjust size if needed, 256 is usually enough for this struct)
  // Note: Use StaticJsonDocument<256> if you are on ArduinoJson v6
  // JsonDocument doc;
  StaticJsonDocument<256> doc;

  // Add string values
  doc["cmd"] = status.cmd;

  // Add booleans (Library automatically converts to true/false)
  doc["isBrake"] = status.isBrake;
  doc["btwFloor"] = status.btwFloor;

  // Add numbers
  doc["pos"] = status.pos;
  doc["targetFloor"] = status.targetFloor;

  // Add Enums (Must cast to int, or use a helper function to convert to String)
  doc["dir"] = (int)status.dir;
  doc["movingState"] = (int)status.state;
  doc["mode"] = (int)status.mode;

  // Serialize into a String
  String jsonOutput;
  serializeJson(doc, jsonOutput);

  return jsonOutput;
}

String statusToJson_INV(int *status)
{
  // JsonDocument doc;
  StaticJsonDocument<512> doc;

  doc["Running 0.01Hz"] = status[0];
  doc["Setting 0.01Hz"] = status[1];
  doc["Bus 0.1V"] = status[2];
  doc["Out V"] = status[3];

  doc["Out 0.01A"] = status[4];
  doc["Power 0.1kW"] = status[5];
  doc["Torque 0.1%"] = status[6];
  doc["DI input state"] = status[7];

  doc["D0 input state"] = status[8];
  doc["AI1 0.01V"] = status[9];
  doc["AI2 V/A"] = status[10];
  doc["Panal 0.1V"] = status[11];

  doc["Count"] = status[12];
  doc["Length"] = status[13];
  doc["Display Hz"] = status[14];
  doc["PID setting"] = status[15];

  doc["PID feedback"] = status[16];
  doc["PLC state"] = status[17];
  doc["HDI input 0.01Hz"] = status[18];
  doc["Feedback 0.01Hz"] = status[19];

  // Serialize into a String
  String jsonOutput;
  serializeJson(doc, jsonOutput);

  return jsonOutput;
}

void setupMQTT()
{
  mqttClient.setServer(mqtt_broker, mqtt_port);
  // mqttClient.setCallback(callback);
  mqttClient.setBufferSize(1024);
}

void reconnect()
{
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    // String clientId = "ESP32Client-0001";
    // if (mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
    if (mqttClient.connect("esp32"))
    {
      Serial.println("connected");
      // Serial.print("Client ID: ");
      // Serial.println(clientId);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
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
  WiFi.softAP("Ximplex_KIT");
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
      MAX_FLOOR = m_new_floor_number;
    }

    if (m_key_string == "change_up_duration_to")
    {
      Serial.println("change_up_duration_to called");
      int m_new_up_duration = m_JSONdoc_from_payload["change_up_duration_to"];
      Serial.println(m_new_up_duration);
      FloorToFloor_MS = m_new_up_duration;
    }
  }
}

void onWebSocketEvent(uint8_t num,
                      WStype_t type,
                      uint8_t *payload,
                      size_t length)
{
  // Serial.println("onWebSocketEvent called");
  // Figure out the type of WebSocket event
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
                                                        ws_cmd = true;
                                                        ws_cmd_value = toFloor2;
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
                                                        ws_cmd = true;
                                                        ws_cmd_value = toFloor1;
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
                                                        ws_cmd = true;
                                                        ws_cmd_value = STOP;
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
                                                        ws_cmd = true;
                                                        ws_cmd_value = POWER_CUT;
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

              //**********************************************

              // List all parameters int params = request->params();
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
                  // Serial.print(F("\t"))

                  // Serial.println(i,'/T',p->name().c_str(),'/T',p->value().c_str());
                  //  Serial.println(i,'/T',p->name().c_str(),'/T',p->value().c_str());
                  // Serial.println(i,'/T',p->name().c_str(),'/T',p->value().c_str());
                  // Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
                }
              }

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

void publishMqtt(const char *topic, const char *msg)
{
  if (xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE)
  {
    if (mqttClient.connected())
    {
      bool result = mqttClient.publish(topic, msg, true);
      if (result)
      {
        Serial.println("publish success");
      }
      else
      {
        Serial.println("publish fail");
      }
      // Serial.printf("Pub: %s -> %s\n", topic, msg);
    }
    xSemaphoreGive(mqttMutex);
  }
}

void doneTransit(uint8_t floor, bool btw, state_t state)
{
  POS = floor;
  btwFloor = btw;
  moving_state = state;
  TARGET = 0;

  publish_status.state = moving_state;
  publish_status.pos = POS;
  publish_status.btwFloor = false;
  publish_status.targetFloor = TARGET;
  hasChanged = true;
  M_STP();
}

void vStatusLogger(void *arg)
{
  int lastPOS = -1;
  bool lastBtw = false;

  for (;;)
  {
    // Check if either variable has changed
    if (POS != lastPOS || btwFloor != lastBtw)
    // if(POS != lastPOS)
    {

      saveStatus();

      lastPOS = POS;
      // lastBtw = btwFloor;
    }
    vTaskDelay(1000);
  }
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

void vUpdatePage(void *pvParams)
{
  static char jsonBuf[256];

  for (;;)
  {
    m_websocketserver.loop();

    uint8_t pos;
    status_t statusCopy;

    pos = POS;

    if (xSemaphoreTake(hasChangedMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
      statusCopy = publish_status;
      xSemaphoreGive(hasChangedMutex);
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    snprintf(
        jsonBuf,
        sizeof(jsonBuf),
        "{\"floorValue\":%d,"
        "\"Up\":%s,"
        "\"Down\":%s,"
        "\"BtwFloor\":%s,"
        "\"Moving\":%s,"
        "\"TargetFloor\":%d,"
        "\"Mode\":\"%s\"}",
        pos,
        (statusCopy.dir == UP) ? "true" : "false",
        (statusCopy.dir == DOWN) ? "true" : "false",
        statusCopy.btwFloor ? "true" : "false",
        (statusCopy.state == MOVING) ? "true" : "false",
        statusCopy.targetFloor,
        (statusCopy.mode == NORMAL) ? "NORMAL" : "EMERGENCY");

    m_websocketserver.broadcastTXT(jsonBuf, strlen(jsonBuf));

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

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
  for (;;)
  {
    if (xSemaphoreTake(hasChangedMutex, portMAX_DELAY) == pdTRUE)
    {
      if (hasChanged == true)
      {

        String status_payload = statusToJson_ELE(publish_status);

        snprintf(
            mqtt_topic,
            sizeof(mqtt_topic),
            "%s%s%s%s",
            KIT_topic,
            UT_case,
            system_status,
            elevator_status);

        publishMqtt(mqtt_topic, status_payload.c_str());
        hasChanged = false;
      }
      xSemaphoreGive(hasChangedMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void vPublishInverterTask(void *pvParams)
{
  for (;;)
  {
    if ((hreg[Inverter_slaveID][7] != 992) && (hreg[Inverter_slaveID][7] != 0))
    {
      String inverter_payload = statusToJson_INV(hreg[Inverter_slaveID]);

      snprintf(
          mqtt_topic,
          sizeof(mqtt_topic),
          "%s%s%s%s",
          KIT_topic,
          UT_case,
          system_status,
          inverter_status);

      publishMqtt(mqtt_topic, inverter_payload.c_str());
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vPollingTask(void *pvParams)
{
  for (;;)
  {
    uint32_t result;
    digitalWrite(WIFI_READY, HIGH);
    switch (curr_slave)
    {
    case INV:
      node.begin(Inverter_slaveID, Serial1);
      result = node.readHoldingRegisters(H0_INV, INV_NUM); // start hreg address, num of read
      if (result == node.ku8MBSuccess)
      {
        for (int i = 0; i < INV_NUM; i++)
        {
          hreg[Inverter_slaveID][i] = node.getResponseBuffer(i);
        }
      }
      else
      {
        Serial.println(result); // Check this code for timeouts (226) or invalid data (227)
      }
      // last_slave = INV;
      // curr_slave = PLC;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void vTransit(void *arg)
{
  for (;;)
  {
    if (xSemaphoreTake(xSemTransit, portMAX_DELAY) == pdTRUE)
    {
      if (xSemaphoreTake(xTransitMutex, portMAX_DELAY) == pdTRUE)
      {
        TARGET = transit.floor;
        xSemaphoreGive(xTransitMutex);
      }
      // BRK_OFF();
      Serial.println("start transit");
      moving_state = MOVING;
      ROTATE(transit.dir);

      if (transit.dir == UP)
        xTimerChangePeriod(xStopTransitTimer, pdMS_TO_TICKS(FloorToFloor_MS), 0);
      // publish_status.isBrake = false;
      publish_status.state = MOVING;
      hasChanged = true;
    }
  }
}

void vGetDirection(void *arg)
{
  uint8_t target;
  direction_t dir;

  for (;;)
  {
    if (xQueueReceive(xQueueGetDirection, &target, portMAX_DELAY) == pdTRUE)
    {

      moving_state = WAITING;

      if (POS != target)
      {
        dir = (target > POS) ? UP : DOWN;

        transit.floor = target;
        transit.dir = dir;

        lastDiffTarget = target;

        xTimerStart(xWaitTimer, 0);
      }
      else
      {
        if (btwFloor == true)
        {
          if (lastDiffTarget > POS) transit.dir = DOWN;
          if (lastDiffTarget < POS) transit.dir = UP;

          transit.floor = target;

          xTimerStart(xWaitTimer, 0);
        }
        else
        {
          if (btwFloor == false)
            Serial.println("It's here");
        }
      }

      publish_status.btwFloor = btwFloor;
      publish_status.targetFloor = transit.floor;
      publish_status.dir = transit.dir;
      hasChanged = true;
    }
  }
}

void vLanding(void *arg)
{
  for (;;)
  {
    if (xSemaphoreTake(xSemLanding, portMAX_DELAY) == pdTRUE)
    {
      if (POS == MIN_FLOOR && btwFloor == false)
      {
        // Serial.println("finish Safety landing");
        // BRK_OFF();
        // emergency = false;
        // publish_status.mode = NORMAL;
        // publish_status.isBrake = false;
        // hasChanged = true;
        vTaskSuspend(NULL);
        continue;
      }

      BRK_ON();
      TARGET = MIN_FLOOR;
      moving_state = MOVING;
      publish_status.state = moving_state;
      publish_status.targetFloor = TARGET;
      publish_status.isBrake = true;
      hasChanged = true;

      vTaskDelay(pdMS_TO_TICKS(2000));

      BRK_OFF();
      publish_status.isBrake = false;
      hasChanged = true;

      vTaskDelay(pdMS_TO_TICKS(500));

      xSemaphoreGive(xSemLanding);
    }
  }
}

void vWaitToTransit(TimerHandle_t xTimer)
{
  Serial.println("Wait time over, start transit");
  xSemaphoreGive(xSemTransit);
}

void vStopTransit(TimerHandle_t xTimer)
{
  M_STP();
  // BRK_ON();
  moving_state = IDLE;
  btwFloor = false;
  POS = publish_status.targetFloor; // update current position

  publish_status.pos = POS;
  publish_status.targetFloor = TARGET;
  publish_status.btwFloor = btwFloor;
  // publish_status.isBrake = true;
  publish_status.state = moving_state;
  TARGET = 0;

  hasChanged = true;
}

void vReceive(void *arg)
{
  int cmd_buf;

  static unsigned long lastTimeCmd1 = 0;
  static unsigned long lastTimeCmd2 = 0;
  const unsigned long DEBOUNCE_DELAY = 3000;

  for (;;)
  {
    int cmd = 0;
    bool commandReceived = false;
    unsigned long now = millis();

    if (RF.available())
    {
      cmd = RF.getReceivedValue();
      RF.resetAvailable();
      commandReceived = true;
    }

    if (ws_cmd == true)
    {
      ws_cmd = false;
      cmd = ws_cmd_value;
      commandReceived = true;
    }

    if (commandReceived && (emergency == false))
    {
      switch (cmd)
      {
      case toFloor1:
        if (now - lastTimeCmd1 > DEBOUNCE_DELAY)
        {
          lastTimeCmd1 = now;

          cmd_buf = 1;
          if (POS == 0)
          {
            POS = 1;
          }

          Serial.println("received toFloor1 cmd");
          strcpy(publish_status.cmd, "toFloor1");
          if (moving_state == IDLE)
            xQueueSend(xQueueGetDirection, &cmd_buf, (TickType_t)0);
        }
        else
        {
          Serial.println("toFloor1 Ignored (Debounce 5s)");
        }
        break;

      case toFloor2:
        if (now - lastTimeCmd2 > DEBOUNCE_DELAY)
        {
          lastTimeCmd2 = now;

          cmd_buf = 2;
          Serial.println("received toFloor2 cmd");
          strcpy(publish_status.cmd, "toFloor2");
          if (moving_state == IDLE)
            xQueueSend(xQueueGetDirection, &cmd_buf, (TickType_t)0);
        }
        else
        {
          Serial.println("toFloor2 Ignored (Debounce 5s)");
        }
        break;

      case POWER_CUT:
        Serial.println("received POWER CUT! cmd");
        strcpy(publish_status.cmd, "POWER_CUT!");
        digitalWrite(R_POWER_CUT, LOW);
        xTimerStart(xPowerCutTimer, 0);
        break;

      case STOP:
        if (moving_state != IDLE)
        {
          strcpy(publish_status.cmd, "STOP");
          Serial.println("received STOP cmd");
          xTimerStop(xStopTransitTimer, 0);
          
          if(moving_state == MOVING) btwFloor = true; 
          // if (POS != transit.floor)
          // {
          //   lastDiffTarget = transit.floor;
          // }
          // lastDir = transit.dir;

          M_STP();
          // BRK_ON();
          xQueueReset(xQueueGetDirection);
          TARGET = 0;
          moving_state = IDLE;

          publish_status.btwFloor = btwFloor;
          publish_status.targetFloor = 0;
          publish_status.state = IDLE;
          publish_status.isBrake = true;

          lastTimeCmd1 = 0;
          lastTimeCmd2 = 0;
        }
        break;
      }

      hasChanged = true;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void vCutPower(TimerHandle_t xTimer)
{
  digitalWrite(R_POWER_CUT, HIGH);
}

void ARDUINO_ISR_ATTR ISR_LowerLim()
{
  unsigned long now = millis();
  if (now - lastLowerLim < DEBOUNCE_MS)
    return; // debounce 50ms
  lastLowerLim = now;

  if (transit.dir == UP) return;
  
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  doneTransit(MIN_FLOOR, false, IDLE);

  if (emergency == true)
  {
    // Serial.println("finish command toLanding");
    emergency = false;
    // BRK_OFF();
    digitalWrite(R_POWER_CUT, HIGH);
    publish_status.mode = NORMAL;
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ARDUINO_ISR_ATTR ISR_UpperLim()
{
  unsigned long now = millis();
  if (now - lastUpperLim < DEBOUNCE_MS)
    return;
  lastUpperLim = now;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  doneTransit(MAX_FLOOR, false, IDLE);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ARDUINO_ISR_ATTR ISR_Landing()
{
  unsigned long now = millis();
  if (now - lastNoPowerISR < DEBOUNCE_MS)
    return;
  lastNoPowerISR = now;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  emergency = true;
  publish_status.mode = EMERGENCY;
  hasChanged = true;

  if ((POS != MIN_FLOOR) || (btwFloor == true))
  {
    xTaskResumeFromISR(xLandingHandle);
    xSemaphoreGiveFromISR(xSemLanding, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ARDUINO_ISR_ATTR ISR_ResetSystem()
{
  unsigned long now = millis();
  if (now - lastResetSysISR < DEBOUNCE_MS)
    return;
  lastResetSysISR = now;
  // Serial.println("Reset System, Back to Floor1");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (POS != 1)
  {
    transit.floor = 1;
    transit.dir = DOWN;
    publish_status.targetFloor = 1;
    publish_status.dir = DOWN;
    hasChanged = true;
    xSemaphoreGiveFromISR(xSemTransit, &xHigherPriorityTaskWoken);
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(38400, SERIAL_8E1, PIN_RX, PIN_TX);
  node.begin(Inverter_slaveID, Serial1);
  while (!Serial)
    ;

  Serial.println("Welcome to Ximplex_KIT");
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

  MDNS.begin("ximplex_ws");

  server.reset(); // try putting this in setup
  configureserver();

  m_websocketserver.begin();
  m_websocketserver.onEvent(onWebSocketEvent); // Start WebSocket server and assign callback

  delay(500);

  // wifiClient.setInsecure();
  Serial.println(WiFi.localIP());
  setupMQTT();

  pinMode(WIFI_READY, OUTPUT);
  pinMode(R_UP, OUTPUT);
  pinMode(R_DW, OUTPUT);
  pinMode(R_POWER_CUT, OUTPUT);
  pinMode(BRK, OUTPUT);

  pinMode(floorSensor1, INPUT); // normal pull-up by using external resistor
  pinMode(floorSensor2, INPUT); // normal pull-up by using external resistor
  attachInterrupt(floorSensor1, ISR_LowerLim, FALLING);
  // attachInterrupt(floorSensor2, ISR_UpperLim, FALLING);

  pinMode(NP, INPUT); // normal pull-up by using external resistor
  // attachInterrupt(NP, ISR_Landing, FALLING);

  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH); // rf always waked up

  pinMode(RST_SYS, INPUT_PULLUP);
  // attachInterrupt(RST_SYS, ISR_ResetSystem, FALLING);

  digitalWrite(R_POWER_CUT, HIGH);
  M_STP;

  xSemTransit = xSemaphoreCreateBinary();
  xSemDoneTransit = xSemaphoreCreateBinary();
  xSemLanding = xSemaphoreCreateBinary();

  xTransitMutex = xSemaphoreCreateMutex();
  mqttMutex = xSemaphoreCreateMutex();
  hasChangedMutex = xSemaphoreCreateMutex();

  xQueueGetDirection = xQueueCreate(1, sizeof(uint8_t));

  xWaitTimer = xTimerCreate("WaitTimer", WAIT_MS, pdFALSE, NULL, vWaitToTransit);
  xStopTransitTimer = xTimerCreate("StopTransitTimer", 100, pdFALSE, NULL, vStopTransit);
  xPowerCutTimer = xTimerCreate("PowerCutTimer", POWER_CUT_MS, pdFALSE, NULL, vCutPower);
  // xDisbrakeTimer = xTimerCreate("DisbrakeTimer", BRAKE_MS, pdFALSE, NULL, vDisbrake);

  xTaskCreate(vReconnectTask, "ReconnectTask", 4096, NULL, 3, NULL);
  xTaskCreate(vPublishTask, "PublishTask", 4096, NULL, 3, &xPublishHandle);
  xTaskCreate(vReceive, "Receive", 1024, NULL, 2, NULL);
  xTaskCreate(vGetDirection, "GetDirection", 1024, NULL, 3, NULL);
  xTaskCreate(vTransit, "Transit", 1024, NULL, 3, NULL);
  xTaskCreate(vLanding, "Landing", 2048, NULL, 4, &xLandingHandle);
  xTaskCreate(vStatusLogger, "StatusLogger", 4096, NULL, 2, NULL);
  xTaskCreate(vPublishInverterTask, "PublishInverter", 4096, NULL, 3, NULL);
  xTaskCreate(vPollingTask, "Polling", 4096, NULL, 3, NULL);
  xTaskCreate(vUpdatePage, "UpdatePage", 4096, NULL, 3, NULL);
  // xTaskCreate(vStopper, "Stopper", 1024, NULL, 4, NULL);
  delay(500);

  Serial.print("Heap free memory (in bytes)= ");
  Serial.println(ESP.getFreeHeap());
  Serial.println(F("Setup complete."));
  delay(500);
}

void loop()
{

  // if (RF.available())
  // {
  //   Serial.println(RF.getReceivedValue());
  //   Serial.println(RF.getReceivedBitlength());
  //   Serial.println(RF.getReceivedDelay());
  //   Serial.println(RF.getReceivedProtocol());
  //   RF.resetAvailable();
  // }

  Serial.print("curr_pos in fs: ");
  Serial.print(POS);

  if (btwFloor == true)
  {
    Serial.println(" (btwFloor)");
  }
  else
  {
    Serial.println(" ");
  }

  Serial.println("----------");
  vTaskDelay(5000);
}
