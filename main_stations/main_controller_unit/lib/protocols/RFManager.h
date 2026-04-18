// RFManager.h
#pragma once
#include <Arduino.h>
#include "ElevatorTypes.h" 
#include "Config.h"
#include <RCSwitch.h> 

class RFManager {
private:
    RCSwitch rf_receiver; 
    rf_button_t decode_signal(unsigned long code);

    unsigned long last_time_floor_cmd[6] = {0}; 
    unsigned long last_time_stop_cmd = 0;
    const unsigned long DEBOUNCE_DELAY = 3000;

public:
    RFManager();
    void init(int pin_rx);
    void process_rf_cmd(QueueHandle_t cmd_queue);
};

 