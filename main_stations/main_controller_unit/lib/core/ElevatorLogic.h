// ElevatorLogic.h
#pragma once
#include "ElevatorTypes.h"
#include "ElevatorHal.h"
#include "IElevatorObserver.h"
#include <Arduino.h>

#define MAX_OBSERVERS 8
class ElevatorLogic
{
public:
    virtual ~ElevatorLogic() = default;

    virtual void update_position() = 0;
    virtual void execute_state_machine() = 0;
    virtual void stop_running() = 0;
    virtual void user_command_handle(user_command cmd) = 0;
    virtual void event_handle(uint32_t evt) = 0;
    virtual void process_remote_message(espnow_msg_t msg) = 0;
    virtual bool is_safe_to_run(elevator_direction_t dir) = 0;
    // virtual void is_reach_floor(uint8_t floorNum) = 0;
    // virtual void clearCommand() = 0;
    // virtual void isSafeToRun(ElevatorDirection dir) = 0;
};

class Orchestrator : public ElevatorLogic
{
public:
    elevator_snapshot data;

private:
    // elevator_snapshot data;
    elevator_direction_t calculate_direction();
    elevator_direction_t last_direction = elevator_direction_t::NONE;

    elevator_state_t last_notified_state = elevator_state_t::IDLE;
    elevator_direction_t last_notified_dir = elevator_direction_t::NONE;

    uint16_t last_cabin_frame = 0;
    uint16_t last_vsg_frame = 0;

    EventGroupHandle_t xSafetyEventGroup;
    ElevatorHal *hal;

    // Observer pattern for notifying changes
    IElevatorObserver *observers[MAX_OBSERVERS];
    uint8_t observer_count = 0;

    void notify_floor_changed();
    void notify_state_changed();
    void notify_event_triggered(uint32_t event_mask);

public:
    Orchestrator(ElevatorHal *hardwarePtr);

    void attach_observer(IElevatorObserver *obs);

    void update_position() override;
    void execute_state_machine() override;
    void stop_running() override;
    void user_command_handle(user_command cmd) override;
    void event_handle(uint32_t evt) override;
    void process_remote_message(espnow_msg_t msg) override;
    bool is_safe_to_run(elevator_direction_t dir) override;
};
