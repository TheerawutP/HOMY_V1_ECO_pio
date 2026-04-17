// main.cpp
#include <Arduino.h>
#include "Config.h"
#include "ElevatorTypes.h"

#include "MqttManager.h"
#include "WebManager.h"
#include "ElevatorLogic.h"
#include "ElevatorHal.h"
#include "ESPNowManager.h"
#include "ModbusManager.h"
#include "RFManager.h"
#include "ElevatorStorage.h"

EspNow espNowBroker;
IOManager io(PIN_UP, PIN_DOWN, PIN_BRAKE, PIN_SS_FLOOR_1, PIN_SS_FLOOR_2, PIN_EMO, PIN_NO_POWER, PIN_SPEED, PIN_SLING); // create obj hal
Orchestrator logic(&io);

TaskHandle_t xElevatorHandle;
QueueHandle_t xQueueCommand = NULL;
SemaphoreHandle_t dataMutex = NULL;

void vElevatorManager(void *pvParams)
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

void vSensorMonitor(void *pvParams)
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

void vEspNowManager(void *pvParams)
{
    espNowBroker.init();
    espnow_msg_t incoming_msg;

    for (;;)
    {
        while (espNowBroker.receive_message(&incoming_msg))
        {
            logic.process_remote_message(incoming_msg);
        }

        // espNowBroker.update();

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// void vRFReceiver(void *pvParams)
// {
//     for(;;){
//         rf.receive();
//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
// }

// void vModbusPolling(void *pvParams)
// {
//     for(;;){
//     switch (currentStation) {
//         case INVERTER:
//         mb.read(currentStation);
//         break;
//     default:
//     break;
//     }
//     if(xQueueReceive(xQueueWrite, &sta, 0) == pdPASS){
//         mb.write(sta);
//     }
//     vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }

// void vTelemetry(void *pvParams)
// {
//     for (;;)
//     {
//         tl.initWebsocket();
//         tl.reconnect();
//         tl.publish();
//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }

// void vDataLog(void *pvParams)
// {
//     for (;;)
//     {
//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }

void setup()
{
    Serial.begin(115200);
    dataMutex = xSemaphoreCreateMutex();
    xQueueCommand = xQueueCreate(10, sizeof(user_command));

    io.init_pins();

    xTaskCreate(vElevatorManager, "ElevatorManager", 4096, NULL, 3, &xElevatorHandle);
    xTaskCreate(vSensorMonitor, "SensorMonitor", 4096, NULL, 3, NULL);
    xTaskCreate(vEspNowManager, "EspNowManager", 4096, NULL, 3, NULL);
    // xTaskCreate(vSensorMonitor, "SensorMonitor", 4096, NULL, 2, NULL);
    // xTaskCreate(vTelemetry, "Telemetry", 8192, NULL, 2, NULL);
    // xTaskCreate(vDataLog, "DataLog", 2048, NULL, 2, NULL);
}

void loop()
{
    if (Serial.available() > 0)
    {
        char incoming = Serial.read();
        if (incoming == '1' || incoming == '2')
        {
            user_command cmd;
            cmd.target = (uint8_t)(incoming - '0');
            cmd.type = command_type_t::TRANSIT;

            if (xQueueSend(xQueueCommand, &cmd, 0) == pdPASS)
            {
                Serial.printf("[TEST] Sending command to Floor %d\n", cmd.target);
            }
        }

        else if (incoming == 's' || incoming == 'S')
        {
            user_command cmd;
            cmd.type = command_type_t::STOP;

            if (xQueueSend(xQueueCommand, &cmd, 0) == pdPASS)
            {
                Serial.println("[TEST] Sending STOP command!");
            }
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}
