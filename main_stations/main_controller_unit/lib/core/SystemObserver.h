#pragma once
#include "IElevatorObserver.h"
#include "arduino.h"
#include <WebServerManager.h>

class CabinObserver : public IElevatorObserver
{
private:
    QueueHandle_t txQueue;

public:
    CabinObserver(QueueHandle_t queue) : txQueue(queue) {}

    void on_floor_changed(uint8_t new_floor) override
    {
        espnow_msg_t msg;
        msg.id = (uint8_t)station_role_t::CABIN;

        uint16_t track_num = (new_floor == 1) ? SF_1014 : SF_1015;
        // OUT_EN_SOUND (Bit 6) + Track Number (Shift to Bit 8-11)
        msg.cmd = OUT_EN_SOUND | (track_num << 8);
        xQueueSend(txQueue, &msg, 0);
    }

    void on_state_changed(elevator_snapshot new_data) override
    {
    //void on_state_changed(elevator_snapshot data) override

        espnow_msg_t msg;
        msg.id = (uint8_t)station_role_t::CABIN;
        uint16_t frame = 0;

        if (new_data.current_state == elevator_state_t::RUNNING)
        {
            if (new_data.dir == elevator_direction_t::UP)
                frame |= OUT_EN_L_UP | OUT_EN_SOUND | (SF_1001 << 8); // going up
            else if (new_data.dir == elevator_direction_t::DOWN)
                frame |= OUT_EN_L_DW | OUT_EN_SOUND | (SF_1002 << 8); // going down
        }
        else if (new_data.current_state == elevator_state_t::IDLE)
        {
            frame |= OUT_EN_L_STOP;
        }

        if (frame != 0)
        {
            msg.cmd = frame;
            xQueueSend(txQueue, &msg, 0);
        }
    }

    void on_event_triggered(uint32_t event_mask) override
    {
        espnow_msg_t msg;
        msg.id = (uint8_t)station_role_t::CABIN;
        uint16_t frame = 0;

        if (event_mask & SAFETY_BRAKE_ENGAGE)
        {
            frame = OUT_EN_SOUND | (SF_1008 << 8) | OUT_EN_L_EM; // safety brake
        }
        else if (event_mask & EMO_IS_PRESSED)
        {
            frame = OUT_EN_SOUND | (SF_1016 << 8) | OUT_EN_L_EM; // emerg stop
        }
        else if (event_mask & VSG_ALARM_TRIGGER)
        {
            frame = OUT_EN_SOUND | (SF_1004 << 8) | OUT_EN_L_STOP; // obstacle under cabin
        }
        else if (event_mask & DOOR_IS_OPEN)
        {
            frame = OUT_EN_SOUND | (SF_1009 << 8) | OUT_EN_L_STOP; // please close the door
        }
        else if (event_mask & VTG_ALARM_TRIGGER)
        {
            frame = OUT_EN_SOUND | (SF_1005 << 8) | OUT_EN_L_STOP; // beware pinch 
        }

        if (frame != 0)
        {
            msg.cmd = frame;
            xQueueSend(txQueue, &msg, 0);
        }
    }
};

class WebServerObserver : public IElevatorObserver
{
private:
    QueueHandle_t txQueue;

public:
    WebServerObserver(QueueHandle_t queue) : txQueue(queue) {}

    void on_floor_changed(uint8_t new_floor) override
    {   
        elevator_snapshot msg;
        msg.current_floor = new_floor;
        xQueueSend(txQueue, &msg, 0);
    }

    void on_state_changed(elevator_snapshot new_data) override
    {
        elevator_snapshot msg;
        msg = new_data;
        xQueueSend(txQueue, &msg, 0);
        
    }

    void on_event_triggered(uint32_t event_mask) override
    {
        
        elevator_snapshot msg;

        if (event_mask & SAFETY_BRAKE_ENGAGE)
        {
            msg.safety_flags = OUT_EN_SOUND | (SF_1008 << 8) | OUT_EN_L_EM; // safety brake
        }
        else if (event_mask & EMO_IS_PRESSED)
        {
            msg.safety_flags = OUT_EN_SOUND | (SF_1016 << 8) | OUT_EN_L_EM; // emerg stop
        }
        else if (event_mask & VSG_ALARM_TRIGGER)
        {
            msg.safety_flags = OUT_EN_SOUND | (SF_1004 << 8) | OUT_EN_L_STOP; // obstacle under cabin
        }
        else if (event_mask & DOOR_IS_OPEN)
        {
            msg.safety_flags = OUT_EN_SOUND | (SF_1009 << 8) | OUT_EN_L_STOP; // please close the door
        }
        else if (event_mask & VTG_ALARM_TRIGGER)
        {
            msg.safety_flags = OUT_EN_SOUND | (SF_1005 << 8) | OUT_EN_L_STOP; // beware pinch 
        }

        if (msg.safety_flags != 0)
        {
            xQueueSend(txQueue, &msg, 0);
        }

    }
};

class MqttObserver : public IElevatorObserver
{
private:
    QueueHandle_t txQueue;
public:
    MqttObserver(QueueHandle_t queue) : txQueue(queue) {}

    void on_floor_changed(uint8_t new_floor) override
    {
        // xQueueSend(txQueue, &msg, 0);
    }

    void on_state_changed(elevator_snapshot new_data) override
    {

        // xQueueSend(txQueue, &msg, 0);
        
    }

    void on_event_triggered(uint32_t event_mask) override
    {
        // xQueueSend(txQueue, &msg, 0);
    }
};

