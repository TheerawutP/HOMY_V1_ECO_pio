// EspNowManager.cpp
#include "EspNowManager.h"
#include <WiFi.h>

EspNow *EspNow::instance = nullptr;

// --- Constructor & Destructor ---

EspNow::EspNow()
{
    rxQueue = xQueueCreate(20, sizeof(espnow_msg_t));
    mailboxMutex = xSemaphoreCreateMutex();
    txMutex = xSemaphoreCreateMutex();
    isSendComplete = false;
    lastSendStatus = ESP_NOW_SEND_FAIL;

    for (int i = 0; i < 10; i++)
    {
        peers[i].is_registered = false;
        peers[i].last_seen = 0;
        memset(peers[i].mac, 0, 6);
        memset(&latest_data[i], 0, sizeof(espnow_msg_t));
    }
}

EspNow::~EspNow()
{
    if (rxQueue != NULL)
    {
        vQueueDelete(rxQueue);
    }

    if (mailboxMutex != NULL)
    {
        vSemaphoreDelete(mailboxMutex);
    }
    if (txMutex != NULL)
    {
        vSemaphoreDelete(txMutex);
    }
}

// --- Initialization ---

bool EspNow::init()
{
    instance = this;

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ESP-NOW] Init Failed!");
        return false;
    }

    esp_now_register_recv_cb(on_data_recv_static);
    esp_now_register_send_cb(on_data_sent_static);

    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcast_mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("[ESP-NOW] Failed to add Broadcast Peer");
    }
    
    Serial.println("[ESP-NOW] Initialized Successfully");
    return true;
}

// --- Static Callbacks ---

void EspNow::on_data_recv_static(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    if (instance)
    {
        instance->handle_data_recv(mac, incomingData, len);
    }
}

void EspNow::on_data_sent_static(const uint8_t *mac, esp_now_send_status_t status)
{
    if (instance)
    {
        instance->handle_data_sent(mac, status);
    }
}

void EspNow::handle_data_recv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    espnow_msg_t tempMsg;

    if (len == sizeof(tempMsg))
    {
        memcpy(&tempMsg, incomingData, sizeof(tempMsg));
        uint8_t role_idx = tempMsg.id;

        if (role_idx > 0 && role_idx < 10)
        {

            // AUTO-PAIRING LOGIC:
            if (!peers[role_idx].is_registered || memcmp(peers[role_idx].mac, mac, 6) != 0)
            {
                Serial.printf(">> Auto-Pairing Detected for Role ID: %d\n", role_idx);
                station_info_t new_station;
                new_station.role = (station_role_t)role_idx;
                memcpy(new_station.mac, mac, 6);

                regist_station(new_station);
            }

            peers[role_idx].last_seen = millis();

            if (xSemaphoreTake(mailboxMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                latest_data[role_idx] = tempMsg;
                xSemaphoreGive(mailboxMutex);
            }
        }

        if (xQueueSend(rxQueue, &tempMsg, 0) != pdPASS)
        {
            Serial.println("[ESP-NOW] RX Queue Full!");
        }
    }
}

void EspNow::handle_data_sent(const uint8_t *mac, esp_now_send_status_t status)
{
    lastSendStatus = status;
    isSendComplete = true;
}

bool EspNow::receive_message(espnow_msg_t *out_msg)
{
    if (rxQueue != NULL)
    {
        if (xQueueReceive(rxQueue, out_msg, 0) == pdPASS)
        {
            return true;
        }
    }
    return false;
}

bool EspNow::send_command(station_role_t target_role, uint16_t frame)
{
    uint8_t role_idx = (uint8_t)target_role;

    if (role_idx >= 10 || !peers[role_idx].is_registered)
    {
        Serial.printf("[ESP-NOW] Cannot send: Role ID %d is not registered!\n", role_idx);
        return false;
    }

    espnow_msg_t sendData;
    sendData.id = (uint8_t)station_role_t::MASTER;
    sendData.cmd = frame;

    bool sendSuccess = false;

    if (xSemaphoreTake(txMutex, portMAX_DELAY) == pdTRUE)
    {
        uint8_t retryCount = 0;
        const uint8_t MAX_RETRIES = 3;

        while (retryCount < MAX_RETRIES && !sendSuccess)
        {
            isSendComplete = false;

            esp_err_t result = esp_now_send(peers[role_idx].mac, (uint8_t *)&sendData, sizeof(sendData));

            if (result == ESP_OK)
            {
                uint32_t waitTime = 0;
                while (!isSendComplete && waitTime < 100)
                {
                    vTaskDelay(pdMS_TO_TICKS(5));
                    waitTime += 5;
                }

                if (isSendComplete && lastSendStatus == ESP_NOW_SEND_SUCCESS)
                {
                    sendSuccess = true;
                    // Serial.println(">> ESP-NOW: Delivery Success");
                }
                else
                {
                    Serial.printf(">> ESP-NOW: Delivery Fail, Retrying (%d/%d)...\n", retryCount + 1, MAX_RETRIES);
                }
            }
            else
            {
                Serial.printf(">> ESP-NOW: Send Queue Error (0x%X), Retrying...\n", result);
            }

            if (!sendSuccess)
            {
                retryCount++;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }

        xSemaphoreGive(txMutex);
    }

    return sendSuccess;
}

bool EspNow::regist_station(const station_info_t &station)
{
    uint8_t role_idx = (uint8_t)station.role;

    if (role_idx >= 10)
        return false;

    if (peers[role_idx].is_registered)
    {
        esp_now_del_peer(peers[role_idx].mac);
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, station.mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.printf("[ESP-NOW] Failed to register Role ID: %d\n", role_idx);
        return false;
    }

    memcpy(peers[role_idx].mac, station.mac, 6);
    peers[role_idx].is_registered = true;
    peers[role_idx].last_seen = millis();

    Serial.printf("[ESP-NOW] Successfully registered Role ID: %d\n", role_idx);
    return true;
}

void EspNow::del_station(station_role_t role)
{
    uint8_t role_idx = (uint8_t)role;
    if (role_idx < 10 && peers[role_idx].is_registered)
    {
        esp_now_del_peer(peers[role_idx].mac);
        peers[role_idx].is_registered = false;
        Serial.printf("[ESP-NOW] Deleted Role ID: %d\n", role_idx);
    }
}

bool EspNow::get_latest_data(station_role_t role, espnow_msg_t *out_data)
{
    uint8_t role_idx = (uint8_t)role;
    if (role_idx >= 10 || !peers[role_idx].is_registered)
        return false;

    if (xSemaphoreTake(mailboxMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        *out_data = latest_data[role_idx]; // Copy ข้อมูลออกไป
        xSemaphoreGive(mailboxMutex);
        return true;
    }
    return false;
}

void EspNow::update()
{
    static unsigned long last_heartbeat = 0;

    if (millis() - last_heartbeat >= 200)
    {
        espnow_msg_t hb_msg;
        hb_msg.id = (uint8_t)station_role_t::MASTER;
        hb_msg.cmd = 0;

        uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        esp_now_send(broadcast_mac, (uint8_t *)&hb_msg, sizeof(hb_msg));

        last_heartbeat = millis();
    }
}