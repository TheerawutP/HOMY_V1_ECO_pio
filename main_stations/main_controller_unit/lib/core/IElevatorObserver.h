#pragma once
#include "ElevatorTypes.h"
#include <stdint.h>

class IElevatorObserver {
public:
    virtual ~IElevatorObserver() = default;
    
    virtual void on_floor_changed(uint8_t new_floor) = 0;
    virtual void on_state_changed(elevator_state_t new_state, elevator_direction_t dir) = 0;
    virtual void on_emergency_triggered() = 0;
};