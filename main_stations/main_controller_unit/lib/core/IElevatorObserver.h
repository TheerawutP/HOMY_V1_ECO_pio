#pragma once
#include "ElevatorTypes.h"
#include <stdint.h>

class IElevatorObserver {
public:
    virtual ~IElevatorObserver() = default;
    
    virtual void on_floor_changed(elevator_snapshot new_data) = 0;
    virtual void on_state_changed(elevator_snapshot new_data) = 0;
    virtual void on_event_triggered(uint32_t event_mask) = 0;
    
};