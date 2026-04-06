#include "ElevatorHal.h"
#include "Config.h"
#include <Arduino.h>

IOManager::IOManager(
    uint8_t pin_motor_up,
    uint8_t pin_motor_down,
    uint8_t pin_brake,
    uint8_t pin_floor1,
    uint8_t pin_floor2,
    uint8_t pin_emo,
    uint8_t pin_no_power,
    uint8_t pin_speed,
    uint8_t pin_sling
) {
    this->pin_motor_up = pin_motor_up;
    this->pin_motor_down = pin_motor_down;
    this->pin_brake = pin_brake;
    this->pin_floor1 = pin_floor1;
    this->pin_floor2 = pin_floor2;
    this->pin_emo = pin_emo;
    this->pin_no_power = pin_no_power;
    this->pin_speed = pin_speed;
    this->pin_sling = pin_sling;

    this->status_floor1 = false;
    this->status_floor2 = false;
    this->status_emo = false;
    this->status_governor = false; 
    this->status_sling = false;
    this->status_brake = false;
}

void IOManager::init_pins() {
    pinMode(pin_motor_up, OUTPUT);
    pinMode(pin_motor_down, OUTPUT);
    pinMode(pin_brake, OUTPUT);
    pinMode(pin_emo, OUTPUT);

    digitalWrite(pin_motor_up, LOW);
    digitalWrite(pin_motor_down, LOW);
    digitalWrite(pin_brake, LOW);
    digitalWrite(pin_emo, LOW);

    pinMode(pin_floor1, INPUT_PULLUP);
    pinMode(pin_floor2, INPUT_PULLUP);
    pinMode(pin_no_power, INPUT_PULLUP);
    pinMode(pin_speed, INPUT_PULLUP);
    pinMode(pin_sling, INPUT_PULLUP);

    motor_stop();
}

void IOManager::motor_rotate(elevator_direction_t dir) {
    if (dir == elevator_direction_t::UP) {
        digitalWrite(pin_motor_down, LOW);
        digitalWrite(pin_motor_up, HIGH);
    } else if (dir == elevator_direction_t::DOWN) {
        digitalWrite(pin_motor_up, LOW);
        digitalWrite(pin_motor_down, HIGH);
    } else {
        motor_stop();
    }
}

void IOManager::motor_stop() {
    digitalWrite(pin_motor_up, LOW);
    digitalWrite(pin_motor_down, LOW);
}

void IOManager::engage_brake(bool engage) {
    digitalWrite(pin_brake, engage ? HIGH : LOW);
}

void IOManager::update_sensor() {
    status_floor1 = (digitalRead(pin_floor1) == LOW);
    status_floor2 = (digitalRead(pin_floor2) == LOW);
    status_emo = (digitalRead(pin_emo) == LOW);
    status_governor = (digitalRead(pin_speed) == LOW);
    status_sling = (digitalRead(pin_sling) == HIGH);
}

bool IOManager::is_at_floor(uint8_t floor_num) {
    if (floor_num == 1)
        return status_floor1;
    if (floor_num == 2)
        return status_floor2;
    return false;
}

uint8_t IOManager::get_active_floor() {
    if (status_floor1) return 1;
    if (status_floor2) return 2;
    return 0;
}
