#include "WebServerManager.h"
#

AsyncWebServer server(80);
WebSocketsServer m_websocketserver = WebSocketsServer(81);
// const char *PARAM_MESSAGE = "message"; // message server receives from client

void websocket_init()
{
  configure_server();
  m_websocketserver.begin();
  m_websocketserver.onEvent(on_websocket_event); // Start WebSocket server and assign callback
}

void send_websocket_alert(const char *alertType, const char *message)
{
  char alertBuf[128];
  snprintf(alertBuf, sizeof(alertBuf), "{\"alert\":\"%s\", \"msg\":\"%s\"}", alertType, message);

  m_websocketserver.broadcastTXT(alertBuf, strlen(alertBuf));
  Serial.printf(">> Sent Alert to UI: %s\n", message);
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

  if (m_JSONdoc_from_payload.containsKey("reset_mcu"))
  {
    if (m_JSONdoc_from_payload["reset_mcu"] == true)
    {
      Serial.println(">> WebCommand: Reset MCU Request Received.");

      send_websocket_alert("WARNING", "MCU is restarting...");

      delay(500);
      ESP.restart();
    }
  }

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

void on_websocket_event(uint8_t num,
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
    Serial.printf("[%u] Received text: %s\n", num, payload);
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

void configure_server()
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
                                                      }
                                                      request2->send(200, "OK"); }));

  // Button #3
  server.addHandler(new AsyncCallbackJsonWebHandler("/on_Button_STOP_pressed", [](AsyncWebServerRequest *request3, JsonVariant &json3)
                                                    {
                                                      const JsonObject &jsonObj3 = json3.as<JsonObject>();
                                                      if (jsonObj3["on"])
                                                      {
                                                        Serial.println("stop button pressed. Stopping all movement!");
                                                        Serial.println("------------------");
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
                                                      }
                                                      request4->send(200, "OK"); }));

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

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
              // preferences.begin("my-config", false);
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

                  // if (paramName == "force_state")
                  // {
                  //   int stateVal = paramValue.toInt();

                  //   state_t newState = (state_t)stateVal;
                  //   elevator.state = newState;

                  //   Serial.printf(">> Manual Force State to: %d \n", stateVal);
                  // }
                }
              }

              // if (request->hasParam(PARAM_MESSAGE, true))
              // {
              //   message = request->getParam(PARAM_MESSAGE, true)->value();
              //   Serial.println(message);
              // }
              // else
              // {
              //   message = "No message sent";
              // }
              request->send(200, "text/HTML", "  <head> <meta http-equiv=\"refresh\" content=\"2; URL=index.html\" /> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> </head> <body> <h1> Settings saved! </h1> <p> Returning to main page. </p> </body>"); });

  server.begin();
}

void update_ui_data(elevator_snapshot data)
{
    static char jsonBuf[512];
    snprintf(
        jsonBuf,
        sizeof(jsonBuf),
        "{\"floorValue\":%d,"
        "\"state\":%d,"
        "\"up\":%s,"
        "\"down\":%s,"
        "\"targetFloor\":%d,"
        "\"btwFloor\":%s,"
        "\"emo\":%s,"
        "\"inv_raw\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u]}", 
        data.current_floor,                                             
        (int)data.current_state,                                        
        (data.dir == elevator_direction_t::UP) ? "true" : "false",     
        (data.dir == elevator_direction_t::DOWN) ? "true" : "false",     
        data.target,                                                    
        data.btw_floor ? "true" : "false",                              
        (data.safety_flags & EMO_IS_PRESSED) ? "true" : "false"        
    );

    m_websocketserver.broadcastTXT(jsonBuf, strlen(jsonBuf));
}
