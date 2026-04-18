// RFManager.cpp
#include "RFManager.h"
#include "Config.h"
#include "ElevatorTypes.h"  

RFManager::RFManager() {
}

void RFManager::init(int pin_rx) {
  rf_receiver.enableReceive(pin_rx);
  Serial.println("[RF] Initialized Successfully");
}

rf_button_t RFManager::decode_signal(unsigned long code) { 
    for (int i = 0; i < num_rf_keys; i++) {
        if (rf_keys[i].rf_code == code) {
            return rf_keys[i].type;
        }
    }
    return BTN_UNKNOWN;
}

void RFManager::process_rf_cmd(QueueHandle_t cmd_queue) {
    if (rf_receiver.available()) {
        unsigned long cmd = rf_receiver.getReceivedValue();
        rf_receiver.resetAvailable();

        if (cmd == 0) return;

        Serial.printf(">> [RF] Received Code: %lu\n", cmd);

        rf_button_t pressed_btn = decode_signal(cmd);
        unsigned long now = millis();
        user_command user_cmd;

        if (pressed_btn >= BTN_TO_FLOOR_1 && pressed_btn <= BTN_TO_FLOOR_6) {
            int floorTarget = (pressed_btn - BTN_TO_FLOOR_1) + 1; 
            int floorIndex = floorTarget - 1; 

            if (now - last_time_floor_cmd[floorIndex] > DEBOUNCE_DELAY) {
                last_time_floor_cmd[floorIndex] = now;
                Serial.printf(">> [RF] Command: TO FLOOR %d\n", floorTarget);
                
                user_cmd.target = floorTarget;
                user_cmd.type = command_type_t::TRANSIT;
                xQueueSend(cmd_queue, &user_cmd, 0); 
            }
        } 
        else if (pressed_btn == BTN_STOP) {
            Serial.println(">> [RF] Command: STOP");
            
            user_cmd.target = 0;
            user_cmd.type = command_type_t::STOP;
            
            xQueueSendToFront(cmd_queue, &user_cmd, 0); 

            // clear all debounce last time
            for(int i=0; i<6; i++) last_time_floor_cmd[i] = 0;
        }
    }
}