#pragma once
#include "IElevatorObserver.h"
#include "arduino.h"


class SystemUIObserver : public IElevatorObserver {
private:
    QueueHandle_t txQueue;

public:
    SystemUIObserver(QueueHandle_t queue) : txQueue(queue) {}

    void on_floor_changed(uint8_t new_floor) override {
        espnow_msg_t msg;
        msg.id = (uint8_t)station_role_t::CABIN;
        // OUT_EN_SOUND (Bit 6) + Track Number (Shift ไปที่ Bit 8-11)
        msg.cmd = OUT_EN_SOUND | ((new_floor & 0x0F) << 8);
        xQueueSend(txQueue, &msg, 0);
    }

    void on_state_changed(elevator_state_t new_state, elevator_direction_t dir) override {
        espnow_msg_t msg;
        msg.id = (uint8_t)station_role_t::CABIN;
        uint16_t frame = 0;

        if (new_state == elevator_state_t::RUNNING) {
            if (dir == elevator_direction_t::UP) frame |= OUT_EN_L_UP;
            else if (dir == elevator_direction_t::DOWN) frame |= OUT_EN_L_DW;
        } 
        else if (new_state == elevator_state_t::IDLE) {
            frame |= OUT_EN_L_STOP;
        }

        if (frame != 0) {
            msg.cmd = frame;
            xQueueSend(txQueue, &msg, 0);
        }
    }

    void on_emergency_triggered() override {
        espnow_msg_t msg;
        msg.id = (uint8_t)station_role_t::CABIN;
        msg.cmd = OUT_EN_L_EM | OUT_EN_BRAKE;
        xQueueSend(txQueue, &msg, 0);
    }
};