//ElevatorHal.cpp
#include "ElevatorHal.h"
#include "Config.h"
#include <Arduino.h>

IOManager::IOManager(
    uint8_t pin_motor_up,
    uint8_t pin_motor_down,
    uint8_t pin_brake,
    uint8_t pin_floor_1,
    uint8_t pin_floor_2,
    uint8_t pin_emo,
    uint8_t pin_no_power,
    uint8_t pin_speed, 
    uint8_t pin_sling
) {
    this->pin_motor_up = pin_motor_up;
    this->pin_motor_down = pin_motor_down;
    this->pin_brake = pin_brake;
    this->pin_floor_1 = pin_floor_1;
    this->pin_floor_2 = pin_floor_2;
    this->pin_emo = pin_emo;
    this->pin_no_power = pin_no_power;
    this->pin_speed = pin_speed;
    this->pin_sling = pin_sling;

    this->status_floor_1 = false;
    this->status_floor_2 = false;
    this->status_emo = false;
    this->status_no_power = false;
    this->status_governor = false; 
    this->status_sling = false;
    this->status_brake = false;

    
    floor_1_counter = 0;
    floor_2_counter = 0;
    no_power_counter = 0;
    sling_counter = 0;
    emo_counter = 0;

}

void IOManager::init_pins() {
    pinMode(pin_motor_up, OUTPUT);
    pinMode(pin_motor_down, OUTPUT);
    pinMode(pin_brake, OUTPUT);
    pinMode(pin_emo, OUTPUT);

    digitalWrite(pin_motor_up, LOW);
    digitalWrite(pin_motor_down, LOW);
    digitalWrite(pin_brake, LOW);

    pinMode(pin_floor_1, INPUT_PULLUP);
    pinMode(pin_floor_2, INPUT_PULLUP);
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

void IOManager::emergency_stop() {
    motor_stop();
    digitalWrite(pin_emo, HIGH);
}

void IOManager::update_sensor() {

    if (digitalRead(pin_floor_1) == LOW) {
        if (floor_1_counter < STABLE_THRESHOLD) floor_1_counter++;
    } else {
        floor_1_counter = 0; 
    }
    status_floor_1 = (floor_1_counter >= STABLE_THRESHOLD);



    if (digitalRead(pin_floor_2) == LOW) {
        if (floor_2_counter < STABLE_THRESHOLD) floor_2_counter++;
    } else {
        floor_2_counter = 0;
    }
    status_floor_2 = (floor_2_counter >= STABLE_THRESHOLD);




    if (digitalRead(pin_emo) == LOW) {
        if (emo_counter < STABLE_THRESHOLD) emo_counter++;
    } else {
        emo_counter = 0;
    }
    status_emo = (emo_counter >= STABLE_THRESHOLD);




    if (digitalRead(pin_sling) == HIGH) {
        if (sling_counter < STABLE_THRESHOLD) sling_counter++;
    } else {
        sling_counter = 0;
    }
    status_sling = (sling_counter >= STABLE_THRESHOLD);



    if (digitalRead(pin_no_power) == LOW) {
        if (no_power_counter < STABLE_THRESHOLD) no_power_counter++;
    } else {
        no_power_counter = 0;
    }
    status_no_power = (no_power_counter >= STABLE_THRESHOLD);


    status_governor = (digitalRead(pin_speed) == LOW);


}

bool IOManager::is_at_floor(uint8_t floor_num) {
    if (floor_num == 1)
        return status_floor_1;
    if (floor_num == 2)
        return status_floor_2;
    return false;
}

uint8_t IOManager::get_active_floor() {
    if (status_floor_1) return 1;
    if (status_floor_2) return 2;
    return 0;
}
