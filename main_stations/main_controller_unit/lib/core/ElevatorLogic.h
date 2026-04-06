// ElevatorLogic.h
#pragma once
#include "ElevatorTypes.h"
#include "ElevatorHal.h"

class ElevatorLogic
{
public:
    virtual void update() = 0;
    virtual void stop_running() = 0;
    virtual void start_running() = 0;
    virtual void user_command_handle(user_command cmd) = 0;

    // virtual void is_reach_floor(uint8_t floorNum) = 0;
    // virtual void clearCommand() = 0;
    // virtual void isSafeToRun(ElevatorDirection dir) = 0;
    // virtual void eventHandle() = 0;
};

class Orchestrator : public ElevatorLogic
{
public:
    elevator_snapshot data;

private:
    // elevator_snapshot data;
    elevator_direction_t calculate_direction();
    elevator_direction_t last_direction = elevator_direction_t::NONE;

    ElevatorHal *hal;

public:
    Orchestrator(ElevatorHal *hardwarePtr);

    void update() override;
    void stop_running() override;
    void start_running() override;
    void user_command_handle(user_command cmd) override;
    // void on_reach_floor(uint8_t floorNum) override;
    // void clearCommand() override;
    // void isSafeToRun(ElevatorDirection dir) override;
    // void eventHandle() override;
};