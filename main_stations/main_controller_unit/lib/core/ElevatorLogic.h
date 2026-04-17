// ElevatorLogic.h
#pragma once
#include "ElevatorTypes.h"
#include "ElevatorHal.h"


class ElevatorLogic
{
public:
    virtual void update_position() = 0;
    virtual void execute_state_machine() = 0;
    virtual void stop_running() = 0;
    virtual void user_command_handle(user_command cmd) = 0;
    virtual void event_handle(uint32_t evt) = 0;
    virtual void process_remote_message(espnow_msg_t msg) = 0;
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
    uint16_t last_cabin_frame = 0;
    uint16_t last_vsg_frame = 0;

    ElevatorHal *hal;

public:
    Orchestrator(ElevatorHal *hardwarePtr);

    void update_position() override;
    void execute_state_machine() override;
    void stop_running() override;
    void user_command_handle(user_command cmd) override;
    void event_handle(uint32_t evt) override;
    void process_remote_message(espnow_msg_t msg);
    // void on_reach_floor(uint8_t floorNum) override;
    // void clearCommand() override;
    // void isSafeToRun(ElevatorDirection dir) override;
};
