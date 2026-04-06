#pragma once
#include "ElevatorTypes.h"

class ElevatorHal {
public:
    // Hardware initialization
    virtual void init_pins() = 0;

    virtual void motor_rotate(elevator_direction_t dir) = 0;
    virtual void motor_stop() = 0;
    virtual void engage_brake(bool engage) = 0;
    virtual void update_sensor() = 0;
    virtual bool is_at_floor(uint8_t floor) = 0;
    virtual uint8_t get_active_floor() = 0;
};

class IOManager : public ElevatorHal {
private:
    uint8_t pin_motor_up;
    uint8_t pin_motor_down;
    uint8_t pin_brake;
    uint8_t pin_floor1;
    uint8_t pin_floor2;
    uint8_t pin_emo;
    uint8_t pin_no_power;
    uint8_t pin_speed;
    uint8_t pin_sling;

    bool status_floor1;
    bool status_floor2;
    bool status_emo;
    bool status_governor;
    bool status_sling;
    bool status_brake;

public:
    IOManager(
        uint8_t pin_motor_up,
        uint8_t pin_motor_down,
        uint8_t pin_brake,
        uint8_t pin_floor1,
        uint8_t pin_floor2,
        uint8_t pin_emo,
        uint8_t pin_no_power,
        uint8_t pin_speed,
        uint8_t pin_sling
    );

    void init_pins() override;
    void motor_rotate(elevator_direction_t dir) override;
    void motor_stop() override;
    void engage_brake(bool engage) override;
    void update_sensor() override;
    bool is_at_floor(uint8_t floor) override;
    uint8_t get_active_floor() override;
};
