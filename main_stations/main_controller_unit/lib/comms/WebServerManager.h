#include "config.h"
#include <WiFi.h>
#include "Arduino.h"
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <AsyncJson.h>
#include "WebSocketsServer.h"
#include "DNSServer.h"
#include "Preferences.h"
#include <SPIFFS.h>
#include "ElevatorTypes.h"

void websocket_init();

void send_websocket_alert(const char *alertType, const char *message);

void configure_server();

void on_websocket_event(uint8_t num,
                      WStype_t type,
                      uint8_t *payload,
                      size_t length);

void handle_websocket_text(uint8_t *payload);

void update_ui_data(elevator_snapshot data);