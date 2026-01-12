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

// #include <OTA.h>
#include <Update.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// gpio
#define PIN_RX 16
#define PIN_TX 15
#define WIFI_READY 22

#define RFReceiver 23
#define floorSensor1 32
#define floorSensor2 33
#define R_UP 19      // Relay UP
#define R_DW 18      // Relay DOWN
#define MOVING_DW 17 // Relay 4
#define BRK 5        // brake
#define NP 25
#define CS 21
#define RST_SYS 4

#define toFloor1 174744
#define toFloor2 174740
// #define toFloor3 174738
#define STOP 174737
#define DEBOUNCE_MS 200
#define BRAKE_MS 2000
#define WAIT_MS 1500
#define MAX_FLOOR 3
#define MIN_FLOOR 3
#define FloorToFloor_MS 5000
enum direction_t
{
  UP,
  DOWN,
  STAY
};

enum state_t
{
  IDLE,
  MOVING,
};
state_t moving_state = IDLE;

enum elevatorMode_t
{
  NORMAL,
  EMERGENCY
};

const char *mqtt_broker = "kit.flinkone.com";
const int mqtt_port = 1883; // unencrypt

// topics
char *KIT_topic = "kit";
char *UT_case = "/UT_0001";
char *system_status = "/sys_v2";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

SemaphoreHandle_t mqttMutex; // Mutex to protect MQTT client
SemaphoreHandle_t hasChangedMutex;
QueueHandle_t xQueueTransit;
QueueHandle_t xQueueGetDirection;
SemaphoreHandle_t xSemTransit = NULL;
SemaphoreHandle_t xSemDoneTransit = NULL;
SemaphoreHandle_t xSemLanding;
TimerHandle_t xDisbrakeTimer;
TimerHandle_t xWaitTimer;
TimerHandle_t xStopTransitTimer;
TaskHandle_t xLandingHandle;
TaskHandle_t xPublishHandle;
SemaphoreHandle_t xTransitMutex;

typedef struct
{
  char cmd[16];
  bool isBrake;
  direction_t dir;
  state_t state;
  uint8_t pos;
  bool btwFloor;
  elevatorMode_t mode;
  uint8_t targetFloor;
} status_t;

status_t publish_status = {
    "NULL",
    true,
    STAY,
    IDLE,
    0,
    false,
    NORMAL,
    0};

typedef struct
{
  uint8_t floor;
  direction_t dir;
} TRANSIT;

TRANSIT transit;
RCSwitch RF = RCSwitch();

bool hasChanged = true;

AsyncWebServer server(80); //
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
volatile uint8_t TARGET = -1;
volatile unsigned long lastFloorISR_1 = 0;
volatile unsigned long lastFloorISR_2 = 0;
volatile unsigned long lastFloorISR_3 = 0;
volatile unsigned long lastNoPowerISR = 0;
volatile unsigned long lastResetSysISR = 0;
bool emergency = false;
bool btwFloor = false;
direction_t lastDir = UP;

//******************* END VARIABLE DECLARATIONS**************************

inline void ROTATE(direction_t dir)
{
  if (dir == UP)
  {
    digitalWrite(R_UP, HIGH);
    digitalWrite(MOVING_DW, LOW);
    Serial.println("Move UP");
  }
  else if (dir == DOWN)
  {
    digitalWrite(R_DW, HIGH);
    digitalWrite(MOVING_DW, HIGH);
    Serial.println("Move DOWN");
  }
}

inline void BRK_ON()
{
  digitalWrite(BRK, LOW);
  Serial.println("Brake ON");
}

inline void BRK_OFF()
{
  digitalWrite(BRK, HIGH);
  Serial.println("Brake OFF");
}

// inline void M_RUN() {
//     digitalWrite(EN, HIGH);

//     Serial.println("Motor Run");
// }

inline void M_STP()
{
  digitalWrite(R_UP, LOW);
  digitalWrite(R_DW, LOW);
  digitalWrite(MOVING_DW, LOW);
  Serial.println("Motor Stop");
}

inline void M_UP()
{
  digitalWrite(R_UP, HIGH);
  digitalWrite(MOVING_DW, LOW);
  Serial.println("Move UP");
}

inline void M_DW()
{
  digitalWrite(R_DW, HIGH);
  digitalWrite(MOVING_DW, HIGH);
  Serial.println("Move Down");
}

inline int BrkState()
{
  return digitalRead(BRK);
}

String statusToJson(const status_t status)
{
  // Create a JSON document (adjust size if needed, 256 is usually enough for this struct)
  // Note: Use StaticJsonDocument<256> if you are on ArduinoJson v6
  JsonDocument doc;

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
  doc["state"] = (int)status.state;
  doc["mode"] = (int)status.mode;

  // Serialize into a String
  String jsonOutput;
  serializeJson(doc, jsonOutput);

  return jsonOutput;
}

void setupMQTT()
{
  mqttClient.setServer(mqtt_broker, mqtt_port);
  // mqttClient.setCallback(callback);
}

void handle_websocket_text(uint8_t *payload)
{
  // do something...
  Serial.printf("handle_websocket_text called for: %s\n", payload);

  // test JSON parsing...
  // char m_JSONMessage[] = "{\"Key1\":123,\"Key2\",345}";
  // StaticJsonDocument<1000> m_JSONdoc;
  // deserializeJson(m_JSONdoc, m_JSONMessage); // m_JSONdoc is now a json object
  // int m_key1_value = m_JSONdoc["Key1"];
  // Serial.println(m_key1_value);

  // Parse JSON payload
  StaticJsonDocument<1000> m_JSONdoc_from_payload;
  DeserializationError m_error = deserializeJson(m_JSONdoc_from_payload, payload); // m_JSONdoc is now a json object
  if (m_error)
  {
    Serial.println("deserializeJson() failed with code ");
    Serial.println(m_error.c_str());
  }
  // Serial.println(m_key2_value);
  // now to iterate over (unknown) keys, we have to cast the StaticJsonDocument object into a JsonObject:
  // see https://techtutorialsx.com/2019/07/09/esp32-arduinojson-printing-the-keys-of-the-jsondocument/
  JsonObject m_JsonObject_from_payload = m_JSONdoc_from_payload.as<JsonObject>();
  // Iterate and print to serial:
  //   uint8_t LMPgain_control_panel = 6; // Feedback resistor of TIA.
  // int num_adc_readings_to_average_control_panel = 1;
  // int sweep_param_delayTime_ms_control_panel = 50;
  // int cell_voltage_control_panel = 100;

  // if (true == false) // if key = "change_cell_voltage_to"
  // {
  //   m_websocket_send_rate = (float)atof((const char *)&payload[0]); // adjust data send rate used in loop
  // }
  // deserializeJson(m_JSONdoc, payload); // m_JSONdoc is now a json object that was payload delivered by websocket message
}

void handleFileList(AsyncWebServerRequest *request)
{
  Serial.println("handle fle list");
  if (!request->hasParam("dir"))
  {
    request->send(500, "text/plain", "BAD ARGS");
    return;
  }
}

void handleFileDelete(AsyncWebServerRequest *request)
{
  Serial.println("in file delete");
  if (request->params() == 0)
  {
    return request->send(500, "text/plain", "BAD ARGS");
  }
  AsyncWebParameter *p = request->getParam(0);
  String path = p->value();
  Serial.println("handleFileDelete: " + path);
  if (path == "/")
  {
    return request->send(500, "text/plain", "BAD PATH");
  }

  if (!SPIFFS.exists(path))
  {
    return request->send(404, "text/plain", "FileNotFound");
  }

  SPIFFS.remove(path);
  request->send(200, "text/plain", "");
  path = String();
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
  delay(500);

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

  m_wifitools_server->on("/list", HTTP_ANY, [](AsyncWebServerRequest *request)
                         { handleFileList(request); });

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

// Called when receiving any WebSocket message
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

void sendValueOverWebsocketJSON(int value_to_send_over_websocket) // sends integer as JSON object to websocket
{
  String json = "{\"value\":";
  json += String(value_to_send_over_websocket);
  json += "}";
  m_websocketserver.broadcastTXT(json.c_str(), json.length());
}

void handleFirmwareUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
  // handle upload and update
  if (!index)
  {
    Serial.printf("Update: %s\n", filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN))
    { // start with max available size
      Update.printError(Serial);
    }
  }

  /* flashing firmware to ESP*/
  if (len)
  {
    Update.write(data, len);
  }

  if (final)
  {
    if (Update.end(true))
    { // true to set the size to the current progress
      Serial.printf("Update Success: %ub written\nRebooting...\n", index + len);
    }
    else
    {
      Update.printError(Serial);
    }
  }
  // alternative approach
  // https://github.com/me-no-dev/ESPAsyncWebServer/issues/542#issuecomment-508489206
}

void handleFilesystemUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
  // handle upload and update
  if (!index)
  {
    Serial.printf("Update: %s\n", filename.c_str());
    // if (!Update.begin(UPDATE_SIZE_UNKNOWN))
    if (!Update.begin(SPIFFS.totalBytes(), U_SPIFFS))
    { // start with max available size
      Update.printError(Serial);
    }
  }

  /* flashing firmware to ESP*/
  if (len)
  {
    Update.write(data, len);
  }

  if (final)
  {
    if (Update.end(true))
    { // true to set the size to the current progress
      Serial.printf("Update Success: %ub written\nRebooting...\n", index + len);
    }
    else
    {
      Update.printError(Serial);
    }
  }
  // alternative approach
  // https://github.com/me-no-dev/ESPAsyncWebServer/issues/542#issuecomment-508489206
}

void handleUpload(AsyncWebServerRequest *request, String filename, String redirect, size_t index, uint8_t *data, size_t len, bool final)
{
  Serial.println("handleUpload called");
  Serial.println(filename);
  Serial.println(redirect);
  File fsUploadFile;
  if (!index)
  {
    if (!filename.startsWith("/"))
      filename = "/" + filename;
    Serial.println((String) "UploadStart: " + filename);
    fsUploadFile = SPIFFS.open(filename, "w"); // Open the file for writing in SPIFFS (create if it doesn't exist)
  }
  for (size_t i = 0; i < len; i++)
  {
    fsUploadFile.write(data[i]);
    // Serial.write(data[i]);
  }
  if (final)
  {
    Serial.println((String) "UploadEnd: " + filename);
    fsUploadFile.close();

    request->send(200, "text/HTML", "  <head> <meta http-equiv=\"refresh\" content=\"2; URL=files.html\" /> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> </head> <body> <h1> File uploaded! </h1> <p> Returning to file page. </p> </body>");

    // request->redirect(redirect);
  }
}

void configureserver()
// configures server
{
  // Need to tell server to accept packets from any source with any header via http methods GET, PUT:
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, PUT");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "*");

  // // Button #xyz
  // server.addHandler(new AsyncCallbackJsonWebHandler("/buttonxyzpressed", [](AsyncWebServerRequest *requestxyz, JsonVariant &jsonxyz) {
  //   const JsonObject &jsonObjxyz = jsonxyz.as<JsonObject>();
  //   if (jsonObjxyz["on"])
  //   {
  //     Serial.println("Button xyz pressed.");
  //     // digitalWrite(LEDPIN, HIGH);
  //     Sweep_Mode = CV;
  //   }
  //   requestxyz->send(200, "OK");
  // }));

  // Button #1
  server.addHandler(new AsyncCallbackJsonWebHandler("/button1pressed", [](AsyncWebServerRequest *request1, JsonVariant &json1)
                                                    {
                                                      const JsonObject &jsonObj1 = json1.as<JsonObject>();
                                                      if (jsonObj1["on"])
                                                      {
                                                        Serial.println("Button 1 pressed. Running CV sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);

                                                      }
                                                      request1->send(200, "OK"); }));
  // Button #2
  server.addHandler(new AsyncCallbackJsonWebHandler("/button2pressed", [](AsyncWebServerRequest *request2, JsonVariant &json2)
                                                    {
                                                      const JsonObject &jsonObj2 = json2.as<JsonObject>();
                                                      if (jsonObj2["on"])
                                                      {
                                                        Serial.println("Button 2 pressed. Running NPV sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);
 
                                                      }
                                                      request2->send(200, "OK"); }));

  // Button #11
  server.addHandler(new AsyncCallbackJsonWebHandler("/button11pressed", [](AsyncWebServerRequest *request2, JsonVariant &json2)
                                                    {
    const JsonObject &jsonObj2 = json2.as<JsonObject>();
    if (jsonObj2["on"])
    {
      Serial.println("Button 11 pressed. Running DPV sweep.");
      // digitalWrite(LEDPIN, HIGH);

    }
    request2->send(200, "OK"); }));
  // Button #3
  server.addHandler(new AsyncCallbackJsonWebHandler("/button3pressed", [](AsyncWebServerRequest *request3, JsonVariant &json3)
                                                    {
                                                      const JsonObject &jsonObj3 = json3.as<JsonObject>();
                                                      if (jsonObj3["on"])
                                                      {
                                                        Serial.println("Button 3 pressed. Running SQV sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);

                                                      }
                                                      request3->send(200, "OK"); }));
  // Button #4
  server.addHandler(new AsyncCallbackJsonWebHandler("/button4pressed", [](AsyncWebServerRequest *request4, JsonVariant &json4)
                                                    {
                                                      const JsonObject &jsonObj4 = json4.as<JsonObject>();
                                                      if (jsonObj4["on"])
                                                      {
                                                        Serial.println("Button 4 pressed. Running CA sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);
  
                                                      }
                                                      request4->send(200, "OK"); }));
  // Button #5
  server.addHandler(new AsyncCallbackJsonWebHandler("/button5pressed", [](AsyncWebServerRequest *request5, JsonVariant &json5)
                                                    {
                                                      const JsonObject &jsonObj5 = json5.as<JsonObject>();
                                                      if (jsonObj5["on"])
                                                      {
                                                        Serial.println("Button 5 pressed. Running DC sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);

                                                      }
                                                      request5->send(200, "OK"); }));
  // Button #6
  server.addHandler(new AsyncCallbackJsonWebHandler("/button6pressed", [](AsyncWebServerRequest *request6, JsonVariant &json6)
                                                    {
                                                      const JsonObject &jsonObj6 = json6.as<JsonObject>();
                                                      if (jsonObj6["on"])
                                                      {
                                                        Serial.println("Button 6 pressed. Running IV sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);

                                                      }
                                                      request6->send(200, "OK"); }));

  // Button #7
  server.addHandler(new AsyncCallbackJsonWebHandler("/button7pressed", [](AsyncWebServerRequest *request7, JsonVariant &json7)
                                                    {
                                                      const JsonObject &jsonObj7 = json7.as<JsonObject>();
                                                      if (jsonObj7["on"])
                                                      {
                                                        Serial.println("Button 7 pressed. Running CAL sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);
 
                                                      }
                                                      request7->send(200, "OK"); }));
  // Button #8
  server.addHandler(new AsyncCallbackJsonWebHandler("/button8pressed", [](AsyncWebServerRequest *request8, JsonVariant &json8)
                                                    {
                                                      const JsonObject &jsonObj8 = json8.as<JsonObject>();
                                                      if (jsonObj8["on"])
                                                      {
                                                        Serial.println("Button 8 pressed. Running MISC_MODE sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);

                                                      }
                                                      request8->send(200, "OK"); }));
  // Button #9
  server.addHandler(new AsyncCallbackJsonWebHandler("/button9pressed", [](AsyncWebServerRequest *request9, JsonVariant &json9)
                                                    {
                                                      const JsonObject &jsonObj9 = json9.as<JsonObject>();
                                                      if (jsonObj9["on"])
                                                      {
                                                        Serial.println("Button 9 pressed.");
                                                        // digitalWrite(LEDPIN, HIGH);

                                                      }
                                                      request9->send(200, "OK"); }));
  // Button #10
  server.addHandler(new AsyncCallbackJsonWebHandler("/button10pressed", [](AsyncWebServerRequest *request10, JsonVariant &json10)
                                                    {
                                                      const JsonObject &jsonObj10 = json10.as<JsonObject>();
                                                      if (jsonObj10["on"])
                                                      {
                                                        Serial.println("Button 10 pressed.");
                                                        // digitalWrite(LEDPIN, HIGH);

                                                      }
                                                      request10->send(200, "OK"); }));

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  server.on("/downloadfile", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/data.txt", "text/plain", true); });

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

              //   <head>
              //   <meta http-equiv="refresh" content="5; URL=https://www.bitdegree.org/" />
              // </head>
              // <body>
              //   <p>If you are not redirected in five seconds, <a href="https://www.bitdegree.org/">click here</a>.</p>
              // </body>

              // request->send(200, "text/URL", "www.google.com");
              // request->send(200, "text/URL", "<meta http-equiv=\"Refresh\" content=\"0; URL=https://google.com/\">");
              // <meta http-equiv="Refresh" content="0; URL=https://example.com/">
            });

  // Wifitools stuff:
  // Save credentials:
  server.on("/saveSecret", HTTP_POST, [](AsyncWebServerRequest *request)
            { handleGetSavSecreteJsonNoReboot(request); });

  // Wifi scan:
  server.on("/wifiScan.json", HTTP_GET, [](AsyncWebServerRequest *request)
            { getWifiScanJson(request); });

  // List directory:
  server.on("/list", HTTP_ANY, [](AsyncWebServerRequest *request)
            { handleFileList(request); });

  // Delete file
  server.on(
      "/edit", HTTP_DELETE, [](AsyncWebServerRequest *request)
      { handleFileDelete(request); });

  // Peter Burke custom code:
  server.on(
      "/m_fupload", HTTP_POST, [](AsyncWebServerRequest *request) {},
      [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data,
         size_t len, bool final)
      { handleUpload(request, filename, "files.html", index, data, len, final); });

  // From https://github.com/me-no-dev/ESPAsyncWebServer/issues/542#issuecomment-573445113
  // handling uploading firmware file
  server.on(
      "/m_firmware_update", HTTP_POST, [](AsyncWebServerRequest *request)
      {
        if (!Update.hasError())
        {
          AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
          response->addHeader("Connection", "close");
          request->send(response);
          ESP.restart();
        }
        else
        {
          AsyncWebServerResponse *response = request->beginResponse(500, "text/plain", "ERROR");
          response->addHeader("Connection", "close");
          request->send(response);
        } },
      handleFirmwareUpload);

  // handling uploading filesystem file
  // see https://github.com/espressif/arduino-esp32/blob/371f382db7dd36c470bb2669b222adf0a497600d/libraries/HTTPUpdateServer/src/HTTPUpdateServer.h
  server.on(
      "/m_filesystem_update", HTTP_POST, [](AsyncWebServerRequest *request)
      {
        if (!Update.hasError())
        {
          AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
          response->addHeader("Connection", "close");
          request->send(response);
          ESP.restart();
        }
        else
        {
          AsyncWebServerResponse *response = request->beginResponse(500, "text/plain", "ERROR");
          response->addHeader("Connection", "close");
          request->send(response);
        } },
      handleFilesystemUpload);

  // Done with configuration, begin server:
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

bool fileExists(fs::FS &fs, const char *path)
{
  return fs.exists(path);
}

String readFileAsString(fs::FS &fs, const char *path)
{
  File file = fs.open(path, FILE_READ);
  if (!file || file.isDirectory())
  {
    Serial.printf("Failed to open file %s for reading\n", path);
    return "";
  }

  String content = "";
  while (file.available())
  {
    content += char(file.read());
  }
  file.close();
  return content;
}

int readFileAsInt(fs::FS &fs, const char *path)
{
  String str = readFileAsString(fs, path);
  str.trim();
  return str.toInt();
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("- file written");
  }
  else
  {
    Serial.println("- write failed");
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, int value)
{
  writeFile(fs, path, String(value).c_str());
}

void vStatusLogger(void *arg)
{
  int lastPOS = -1;
  for (;;)
  {
    if (POS != lastPOS)
    {
      writeFile(SPIFFS, "/current_pos.txt", POS);
      lastPOS = POS;
    }
    vTaskDelay(1000);
  }
}

void listDir(const char *dirname, uint8_t levels)
{
  // from https://github.com/espressif/arduino-esp32/blob/master/libraries/SPIFFS/examples/SPIFFS_Test/SPIFFS_Test.ino#L9
  // see also https://techtutorialsx.com/2019/02/24/esp32-arduino-listing-files-in-a-spiffs-file-system-specific-path/
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = SPIFFS.open(dirname);
  if (!root)
  {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory())
  {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file)
  {

    Serial.print("  FILE: ");
    Serial.print(file.name());
    Serial.print("\tSIZE: ");
    Serial.println(file.size());

    file = root.openNextFile();
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
  // Serial.print("m_SSID1_name = ");
  // Serial.print(m_SSID1_name);
  // Serial.print(F("\t")); // tab
  // Serial.print("m_PWD1_name = ");
  // Serial.print(F("\t")); // tab
  // Serial.print("m_SSID2_name = ");
  // Serial.print(m_SSID2_name);
  // Serial.print(F("\t")); // tab
  // Serial.print("m_PWD2_name = ");
  // Serial.print(m_PWD2_name);
  // Serial.print(F("\t")); // tab
  // Serial.print("m_SSID3_name = ");
  // Serial.print(m_SSID3_name);
  // Serial.print(F("\t")); // tab
  // Serial.print("m_PWD3_name = ");
  // Serial.println(m_PWD3_name);

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

        String status_payload = statusToJson(publish_status);
        publishMqtt("kit/UT_1000", status_payload.c_str());
        hasChanged = false;
      }
      xSemaphoreGive(hasChangedMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
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
      BRK_OFF();
      Serial.println("start transit");
      moving_state = MOVING;
      ROTATE(transit.dir);

      publish_status.isBrake = false;
      publish_status.state = MOVING;
      hasChanged = true;

      xTimerStart(xStopTransit, 0);
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

      if (POS != target)
      {
        dir = (target > POS) ? UP : DOWN;

        transit.floor = target;
        transit.dir = dir;

        xTimerStart(xWaitTimer, 0);
      }
      else
      {
        if (btwFloor == true)
        {
          if (lastDir == UP)
            transit.dir = DOWN;
          if (lastDir == DOWN)
            transit.dir = UP;
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

// void vStopper(void *arg)
// {
//   for (;;)
//   {
//     if (xSemaphoreTake(xSemDoneTransit, portMAX_DELAY) == pdTRUE)
//     {
//       M_STP();
//       BRK_ON();
//       TARGET = 0;
//       moving_state = IDLE;

//       publish_status.targetFloor = 0;
//       publish_status.isBrake = true;
//       publish_status.state = IDLE;
//       hasChanged = true;
//     }
//   }
// }

void vLanding(void *arg)
{
  for (;;)
  {
    if (xSemaphoreTake(xSemLanding, portMAX_DELAY) == pdTRUE)
    {

      if (POS == 1 && emergency == 1)
      {
        Serial.println("finish Safety landing");
        // M_STP();
        BRK_OFF();
        emergency = false;
        publish_status.mode = NORMAL;
        publish_status.isBrake = false;
        hasChanged = true;
        xTimerStop(xDisbrakeTimer, 0);
        vTaskSuspend(NULL);
      }

      BRK_ON();
      publish_status.targetFloor = 1;
      publish_status.isBrake = true;
      hasChanged = true;
      xTimerStart(xDisbrakeTimer, 0);
    }
  }
}

void vDisbrake(TimerHandle_t xTimer)
{
  BRK_OFF();
  delay(500);
  publish_status.isBrake = false;
  hasChanged = true;
  xSemaphoreGive(xSemLanding);
}

void vWaitToTransit(TimerHandle_t xTimer)
{
  xSemaphoreGive(xSemTransit);
}

void vStopTransit(TimerHandle_t xTimer){
      M_STP();
      BRK_ON();
      TARGET = 0;
      moving_state = IDLE;

      publish_status.targetFloor = 0;
      publish_status.isBrake = true;
      publish_status.state = IDLE;
      hasChanged = true;
}

void vReceive(void *arg)
{
  int cmd_buf;
  for (;;)
  {
    if (RF.available())
    {
      int cmd = RF.getReceivedValue();
      RF.resetAvailable();
      switch (cmd)
      {
      case toFloor1: // from A
        cmd_buf = 1;
        if (POS == 0)
        {
          POS = 1;
        }
        Serial.println("received toFloor1 cmd");
        publish_status.cmd = "toFloor1";
        if (moving_state == IDLE)
          xQueueSend(xQueueGetDirection, &cmd_buf, (TickType_t)0);
        break;
      case toFloor2:
        cmd_buf = 2;
        Serial.println("received toFloor2 cmd");
        publish_status.cmd = "toFloor2";
        if (moving_state == IDLE)
          xQueueSend(xQueueGetDirection, &cmd_buf, (TickType_t)0);
        break;
      // case toFloor3:
      //   cmd_buf = 3;
      //   Serial.println("received toFloor3 cmd");
      //   if(moving_state == IDLE) xQueueSend(xQueueGetDirection, &cmd_buf, (TickType_t)0);
      //   break;
      case STOP:
        M_STP();
        BRK_ON();
        xQueueReset(xQueueGetDirection);
        TARGET = 0;
        moving_state = IDLE;
        lastDir = transit.dir;
        btwFloor = true;

        publish_status.btwFloor = true;
        publish_status.targetFloor = 0;
        publish_status.state = IDLE;
        publish_status.isBrake = true;
        break;
      }
      hasChanged = true;
    }
    vTaskDelay(10);
  }
}

void ARDUINO_ISR_ATTR ISR_LowerLim()
{
  unsigned long now = millis();
  if (now - lastLowerLim < DEBOUNCE_MS)
    return; // debounce 50ms
  lastLowerLim = now;
  Serial.println("reach lowest");

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = MIN_FLOOR;
  btwFloor = false;

  publish_status.pos = POS;
  publish_status.btwFloor = false;
  hasChanged = true;

  if (TARGET == POS && emergency == false)
  {
    Serial.println("finish command toLowest");
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ARDUINO_ISR_ATTR ISR_UpperLim()
{
  unsigned long now = millis();
  if (now - lastUpperLim < DEBOUNCE_MS)
    return;
  lastUpperLim = now;
  Serial.println("reach highest");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = MAX_FLOOR;
  btwFloor = false;

  publish_status.pos = POS;
  publish_status.btwFloor = false;
  hasChanged = true;
  if (TARGET == POS && emergency == false)
  {
    Serial.println("finish command toHighest");
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ARDUINO_ISR_ATTR ISR_Landing()
{
  unsigned long now = millis();
  if (now - lastNoPowerISR < DEBOUNCE_MS)
    return;
  lastNoPowerISR = now;
  Serial.println("NO POWER is detected!");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  emergency = true;
  publish_status.mode = EMERGENCY;
  hasChanged = true;

  if (POS != MIN_FLOOR)
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
  Serial.println("Reset System, Back to Floor1");
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

  // Start serial interface:
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("Welcome to Ximplex_KIT");

  delay(50);
  delay(50);

  // ############################### SPIFFS STARTUP #######################################
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  if (SPIFFS.exists("/current_pos.txt"))
  {
    POS = readFileAsInt(SPIFFS, "/current_pos.txt");
  }
  else
  {
    POS = defaultPOS;
    writeFile(SPIFFS, "/current_pos.txt", POS);
  }

  RF.enableReceive(RFReceiver); // attach interrupt to 22

  bool m_autoconnected_attempt_succeeded = false;
  m_autoconnected_attempt_succeeded = connectAttempt("", ""); // uses SSID/PWD stored in ESP32 secret memory.....
  // Serial.print("m_autoconnected_attempt_succeeded = ");
  // Serial.println(m_autoconnected_attempt_succeeded);
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

  MDNS.begin("keepintouch");

  server.reset(); // try putting this in setup
  configureserver();

  m_websocketserver.begin();
  m_websocketserver.onEvent(onWebSocketEvent); // Start WebSocket server and assign callback

  // wifiClient.setInsecure();
  Serial.println(WiFi.localIP());
  setupMQTT();

  // for (int i = 0; i < subTopicNum; i++) {
  //   sub_buff[i].topic = "";
  //   sub_buff[i].payload = "";
  // }

  pinMode(WIFI_READY, OUTPUT);

  pinMode(R_UP, OUTPUT);
  pinMode(R_DW, OUTPUT);
  pinMode(MOVING_DW, OUTPUT);
  pinMode(floorSensor1, INPUT); // INPUT_PULLUP
  pinMode(floorSensor2, INPUT);
  // pinMode(floorSensor3, INPUT_PULLUP);
  attachInterrupt(floorSensor1, ISR_LowerLim, FALLING);
  attachInterrupt(floorSensor2, ISR_UpperLim, FALLING);
  // attachInterrupt(floorSensor3, ISR_atFloor3, FALLING);
  pinMode(BRK, OUTPUT);
  pinMode(NP, INPUT_PULLUP);
  attachInterrupt(NP, ISR_Landing, FALLING);
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH); // wake receiver up
  pinMode(RST_SYS, INPUT_PULLUP);
  attachInterrupt(RST_SYS, ISR_ResetSystem, FALLING);

  BRK_ON;
  M_STP;

  xSemTransit = xSemaphoreCreateBinary();
  xSemDoneTransit = xSemaphoreCreateBinary();
  xSemLanding = xSemaphoreCreateBinary();
  xTransitMutex = xSemaphoreCreateMutex();
  xQueueGetDirection = xQueueCreate(1, sizeof(uint8_t));
  xWaitTimer = xTimerCreate("WaitTimer", WAIT_MS, pdFALSE, NULL, vWaitToTransit);
  xStopTransitTimer = xTimerCreate("StopTransitTimer", FloorToFloor_MS, pdFALSE, NULL, vStopTransit);
  xDisbrakeTimer = xTimerCreate("DisbrakeTimer", BRAKE_MS, pdFALSE, NULL, vDisbrake);

  mqttMutex = xSemaphoreCreateMutex();
  hasChangedMutex = xSemaphoreCreateMutex();
  // pubQueue = xQueueCreate(20, sizeof(msg));
  xTaskCreate(vReconnectTask, "ReconnectTask", 4096, NULL, 3, NULL);
  xTaskCreate(vPublishTask, "PublishTask", 4096, NULL, 3, &xPublishHandle);

  xTaskCreate(vReceive, "Receive", 1024, NULL, 2, NULL);
  xTaskCreate(vGetDirection, "GetDirection", 1024, NULL, 3, NULL);
  xTaskCreate(vTransit, "Transit", 1024, NULL, 3, NULL);
  // xTaskCreate(vStopper, "Stopper", 1024, NULL, 4, NULL);
  xTaskCreate(vLanding, "Landing", 2048, NULL, 4, &xLandingHandle);
  xTaskCreate(vStatusLogger, "StatusLogger", 2048, NULL, 2, NULL);

  Serial.print("Heap free memory (in bytes)= ");
  Serial.println(ESP.getFreeHeap());
  Serial.println(F("Setup complete."));
}

void loop()
{
  if (RF.available())
  {
    Serial.println(RF.getReceivedValue());
    Serial.println(RF.getReceivedBitlength());
    Serial.println(RF.getReceivedDelay());
    Serial.println(RF.getReceivedProtocol());

    RF.resetAvailable();
  }

  m_websocketserver.loop();

  if (m_send_websocket_test_data_in_loop == true) // do things here in loop at full speed
  {
    // Pseudocode: xxx_period_in_ms_xxx=period_in_s * 1e3 = (1/freqHz)*1e3
    if (millis() - last_time_sent_websocket_server > (1000 / m_websocket_send_rate)) // every half second, print
    {
      //sendTimeOverWebsocketJSON();
      sendValueOverWebsocketJSON(100 * 0.5 * sin(millis() / 1e3)); // value is sine wave of time , frequency 0.5 Hz, amplitude 100.
      last_time_sent_websocket_server = millis();
    }
    // m_microsbefore_websocketsendcalled=micros();
    // sendTimeOverWebsocketJSON(); // takes 2.5 ms on average, when client is connected, else 45 microseconds...
    // Serial.println(micros()-m_microsbefore_websocketsendcalled);
  }

  int curr_pos = readFileAsInt(SPIFFS, "/current_pos.txt");
  Serial.print("curr_pos in fs: ");
  Serial.print(curr_pos);

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
  last_time_loop_called = millis();
}
