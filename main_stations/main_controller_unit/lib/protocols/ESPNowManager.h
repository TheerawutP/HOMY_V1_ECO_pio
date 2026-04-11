#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include "ElevatorTypes.h"

typedef struct
{
    uint8_t fromID;
    uint16_t commandFrame;
    uint16_t responseFrame;
    bool shouldResponse;
} espnow_msg_t;

enum class station_role_t : uint8_t
{
    UNKNOWN = 0,
    INVERTER = 1,
    CABIN = 2,
    HALL_1 = 3,
    HALL_2 = 4,
    HALL_3 = 5,
    VSG = 6,
    VTG = 7,
    MASTER = 100
};

struct station_info_t
{
    station_role_t role;
    uint8_t mac[6];
};

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

    void handleDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
    void handleDataSent(const uint8_t *mac, esp_now_send_status_t status);

public:
    EspNow();
    ~EspNow();

    static void OnDataRecvStatic(const uint8_t *mac, const uint8_t *incomingData, int len);
    static void OnDataSentStatic(const uint8_t *mac, esp_now_send_status_t status);

    bool init() override;
    void update() override;
    bool send_command(station_role_t target_role, uint16_t frame) override;
    bool regist_station(const station_info_t &station) override;
    void del_station(station_role_t role) override;
    bool receive_message(espnow_msg_t *out_msg) override;
    bool get_latest_data(station_role_t role, espnow_msg_t *out_data) override;
};