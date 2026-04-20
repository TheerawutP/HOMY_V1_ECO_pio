//EspNowManager.h
#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include "ElevatorTypes.h"


// 1. Interface class
class EspNowManager
{
public:
    virtual ~EspNowManager() {} // virtual destructor

    virtual bool init() = 0;
    virtual void update() = 0;
    virtual bool send_command(station_role_t target_role, uint16_t frame) = 0;
    virtual bool regist_station(const station_info_t &station) = 0;
    virtual void del_station(station_role_t role) = 0;
    virtual bool receive_message(espnow_msg_t *out_msg) = 0;
    virtual bool get_latest_data(station_role_t role, espnow_msg_t *out_data) = 0;
};

// 2. Implementation class

class EspNow : public EspNowManager
{

private:
    static EspNow *instance;
    QueueHandle_t rxQueue;
    espnow_msg_t latest_data[10];
    SemaphoreHandle_t mailboxMutex;

    SemaphoreHandle_t txMutex;
    volatile bool isSendComplete;
    volatile esp_now_send_status_t lastSendStatus;

    struct peer_node_t
    {
        uint8_t mac[6];
        bool is_registered;
        uint32_t last_seen;
    };

    peer_node_t peers[10];

    // static void OnDataRecvStatic(const uint8_t *mac, const uint8_t *incomingData, int len);
    // static void OnDataSentStatic(const uint8_t *mac, esp_now_send_status_t status);

    void handle_data_recv(const uint8_t *mac, const uint8_t *incomingData, int len);
    void handle_data_sent(const uint8_t *mac, esp_now_send_status_t status);

public:
    EspNow();
    ~EspNow();

    static void on_data_recv_static(const uint8_t *mac, const uint8_t *incomingData, int len);
    static void on_data_sent_static(const uint8_t *mac, esp_now_send_status_t status);

    bool init() override;
    void update() override;
    bool send_command(station_role_t target_role, uint16_t frame) override;
    bool regist_station(const station_info_t &station) override;
    void del_station(station_role_t role) override;
    bool receive_message(espnow_msg_t *out_msg) override;
    bool get_latest_data(station_role_t role, espnow_msg_t *out_data) override;
};