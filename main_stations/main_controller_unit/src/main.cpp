// main.cpp
#include <Arduino.h>
#include "Config.h"
#include "ElevatorTypes.h"

#include "WebServerManager.h"
#include "WifiPortalManager.h"
#include "ElevatorLogic.h"
#include "ElevatorHal.h"
#include "ESPNowManager.h"
#include "ModbusManager.h"
#include "RFManager.h"
#include "ElevatorStorage.h"
#include "SystemObserver.h"

EspNow espnow;
RFManager rf;
IOManager io(PIN_UP, PIN_DOWN, PIN_BRAKE, PIN_SS_FLOOR_1, PIN_SS_FLOOR_2, PIN_EMO, PIN_NO_POWER, PIN_SPEED, PIN_SLING); // create obj hal
Orchestrator logic(&io);

TaskHandle_t xElevatorHandle;
QueueHandle_t xQueueCommand = NULL;
QueueHandle_t xQueueSending = NULL;
SemaphoreHandle_t dataMutex = NULL;

CabinObserver *cabin_observer;

void elevator_manager_task(void *pvParams)
{
    user_command cmd;
    uint32_t evt;
    for (;;)
    {

        if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &evt, 0) == pdPASS)
        {
            logic.event_handle(evt);
        }

        if (xQueueReceive(xQueueCommand, &cmd, 0) == pdPASS)
        {
            logic.user_command_handle(cmd);
        }

        logic.update_position();
        logic.execute_state_machine();

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void sensor_monitor_task(void *pvParams)
{

    bool is_at_floor_1 = false;
    bool is_at_floor_2 = false;
    bool is_emo = false;
    bool is_speed_trig = false;
    bool is_sling_cut = false;
    bool is_brake_engage = false;

    for (;;)
    {
        io.update_sensor();
        is_at_floor_1 = io.status_floor_1;
        is_at_floor_2 = io.status_floor_2;
        is_emo = io.status_emo;
        is_speed_trig = io.status_governor;
        is_sling_cut = io.status_sling;
        is_brake_engage = io.status_brake;

        // if (is_at_floor_1) xTaskNotify(xElevatorHandle, REACH_FLOOR_1, eSetValueWithOverwrite);
        // if (is_at_floor_2) xTaskNotify(xElevatorHandle, REACH_FLOOR_2, eSetValueWithOverwrite);
        // if (is_emo) xTaskNotify(xElevatorHandle, EMO_IS_PRESSED, eSetValueWithOverwrite);

        // if (is_sling_cut)
        // {
        //     xTaskNotify(xElevatorHandle, SAFETY_BRAKE_ENGAGE, eSetBits);
        // }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void esp_now_manager_task(void *pvParams)
{
    espnow.init();
    espnow_msg_t incoming_msg;
    espnow_msg_t outgoing_msg;
    for (;;)
    {
        espnow.update();
        while (espnow.receive_message(&incoming_msg))
        {
            logic.process_remote_message(incoming_msg);
        }

        while (xQueueReceive(xQueueSending, &outgoing_msg, 0) == pdPASS)
        {
            espnow.send_command((station_role_t)outgoing_msg.id, outgoing_msg.cmd);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void rf_receiver_task(void *pvParams)
{
    rf.init(pin_rf_receiver);

    for (;;)
    {
        rf.process_rf_cmd(xQueueCommand);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// void vModbusPolling(void *pvParams)
// void vTelemetry(void *pvParams)
// void vDataLog(void *pvParams)

void setup()
{
    Serial.begin(115200);

    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    wifi_portal_init();
    websocket_init();
    
    dataMutex = xSemaphoreCreateMutex();
    xQueueCommand = xQueueCreate(10, sizeof(user_command));
    xQueueSending = xQueueCreate(10, sizeof(espnow_msg_t));

    io.init_pins();

    cabin_observer = new CabinObserver(xQueueSending);
    logic.attach_observer(cabin_observer);

    xTaskCreate(elevator_manager_task, "ElevatorManager", 4096, NULL, 3, &xElevatorHandle);
    xTaskCreate(sensor_monitor_task, "SensorMonitor", 4096, NULL, 3, NULL);
    xTaskCreate(esp_now_manager_task, "EspNowManager", 4096, NULL, 3, NULL);
    xTaskCreate(rf_receiver_task, "RFReceiver", 4096, NULL, 3, NULL);
    // xTaskCreate(vTelemetry, "Telemetry", 8192, NULL, 2, NULL);
    // xTaskCreate(vDataLog, "DataLog", 2048, NULL, 2, NULL);
}

void loop()
{
    if (Serial.available() > 0)
    {
        char incoming = Serial.read();
        user_command cmd;

        switch (incoming)
        {
        case '1':
            Serial.println(">> [TEST] go to floor 1");
            cmd = {1, command_type_t::TRANSIT};
            xQueueSend(xQueueCommand, &cmd, 0);
            break;
        case '2':
            Serial.println(">> [TEST] go to floor 2");
            cmd = {2, command_type_t::TRANSIT};
            xQueueSend(xQueueCommand, &cmd, 0);
            break;
        case 's':
        case 'S':
            Serial.println(">> [TEST] user STOP");
            cmd = {0, command_type_t::STOP};
            xQueueSendToFront(xQueueCommand, &cmd, 0);
            break;

            // --- (Event) ---

        case 'e':
        case 'E':
            Serial.println(">> [TEST] user EMO!");
            xTaskNotify(xElevatorHandle, EMO_IS_PRESSED, eSetBits);
            break;
        case 'x':
        case 'X':
            Serial.println(">> [TEST] sling cut!");
            xTaskNotify(xElevatorHandle, SAFETY_BRAKE_ENGAGE, eSetBits);
            break;
        case 'o':
        case 'O':
            Serial.println(">> [TEST] door open");
            xTaskNotify(xElevatorHandle, DOOR_IS_OPEN, eSetBits);
            break;
        case 'c':
        case 'C':
            Serial.println(">> [TEST] door closed");
            xTaskNotify(xElevatorHandle, DOOR_IS_CLOSED, eSetBits);
            break;
        }
    }
}
