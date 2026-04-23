#pragma once
#include "IElevatorObserver.h"
#include "arduino.h"

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

    void on_state_changed(elevator_state_t new_state, elevator_direction_t dir) override
    {
        espnow_msg_t msg;
        msg.id = (uint8_t)station_role_t::CABIN;
        uint16_t frame = 0;

        if (new_state == elevator_state_t::RUNNING)
        {
            if (dir == elevator_direction_t::UP)
                frame |= OUT_EN_L_UP | OUT_EN_SOUND | (SF_1001 << 8); // going up
            else if (dir == elevator_direction_t::DOWN)
                frame |= OUT_EN_L_DW | OUT_EN_SOUND | (SF_1002 << 8); // going down
        }
        else if (new_state == elevator_state_t::IDLE)
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

class WebSocketObserver : public IElevatorObserver
{
private:
    QueueHandle_t txQueue;

public:
    WebSocketObserver(QueueHandle_t queue) : txQueue(queue) {}

    void on_floor_changed(uint8_t new_floor) override
    {   
        // xQueueSend(txQueue, &msg, 0);
    }

    void on_state_changed(elevator_state_t new_state, elevator_direction_t dir) override
    {

        // xQueueSend(txQueue, &msg, 0);
        
    }

    void on_event_triggered(uint32_t event_mask) override
    {
        // xQueueSend(txQueue, &msg, 0);
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

    void on_state_changed(elevator_state_t new_state, elevator_direction_t dir) override
    {

        // xQueueSend(txQueue, &msg, 0);
        
    }

    void on_event_triggered(uint32_t event_mask) override
    {
        // xQueueSend(txQueue, &msg, 0);
    }
};

