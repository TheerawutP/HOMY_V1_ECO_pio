// WifiPortalManager.h
#pragma once
#include "CommTypes.h"
#include "WebSocketsServer.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include "DNSServer.h"
#include "FS.h"
#include "SPIFFS.h"
#include <AsyncJson.h>

boolean wifi_connect_attempt(String ssid, String password);

bool read_wifi_credentials_file(String m_pwd_filename_to_read);

void setup_ap_service();

void process();

void run_wifi_portal();

void get_wifi_scan_json(AsyncWebServerRequest *request);

void handle_get_save_secret_json(AsyncWebServerRequest *request);

void run_wifi_portal();
