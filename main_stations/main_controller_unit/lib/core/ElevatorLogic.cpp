//ElevatorLogic.cpp
#include "ElevatorLogic.h"
#include "Config.h"
#include "ElevatorHal.h"
#include "ElevatorTypes.h"
#include <Arduino.h>

Orchestrator::Orchestrator(ElevatorHal *hardwarePtr)

{
    this->hal = hardwarePtr;
    data.current_floor = 1;
    data.btw_floor = false;
    data.current_state = elevator_state_t::IDLE;
    last_direction = elevator_direction_t::NONE;
};

void Orchestrator::update()
{
    // 1. Sync data with hardware
    hal->update_sensor();
    uint8_t current_sensor_floor = hal->get_active_floor();

    // Update internal state if a floor is hit
    if (current_sensor_floor != 0)
    {
        data.current_floor = current_sensor_floor;
        data.btw_floor = false;
    }
    else
    {
        data.btw_floor = true;
    }

    // 2. Control flow based on state
    if (data.current_state == elevator_state_t::RUNNING)
    {
        // Check if reached target
        if (hal->is_at_floor(data.target))
        {
            hal->motor_stop();
            data.current_state = elevator_state_t::IDLE;
            last_direction = elevator_direction_t::NONE;
            Serial.printf("[INFO] Reached target floor %d\n", data.target);
            return;
        }

        // Calculate and apply movement
        elevator_direction_t dir = calculate_direction();
        if (dir != elevator_direction_t::NONE)
        {
            hal->motor_rotate(dir);
            last_direction = dir;
        }
        else
        {
            // If calculate_direction returns NONE but we are RUNNING, 
            // something is wrong or we are already there.
            hal->motor_stop();
            data.current_state = elevator_state_t::IDLE;
        }
    }
    else if (data.current_state == elevator_state_t::IDLE)
    {
        // Ensure motor is stopped and brake is engaged in IDLE
        hal->motor_stop();
    }
};

void Orchestrator::stop_running()
{
    this->hal->motor_stop();
    data.target = 0;
    data.current_state = elevator_state_t::IDLE;
};

void Orchestrator::start_running() {
    // this->hal->motor_rotate();
    // data.current_state = elevator_state_t::RUNNING;

};

elevator_direction_t Orchestrator::calculate_direction()
{
    // 1. reach the floor at exact position
    if (data.target == data.current_floor && data.btw_floor == false)
    {
        return elevator_direction_t::NONE;
    }

    // 2. in case of (btwFloor == true)
    if (data.btw_floor == true)
    {

        // last direction = UP THEN CABIN is higher than currentFloor
        if (last_direction == elevator_direction_t::UP)
        {

            // if wanna go back, GO DOWN
            if (data.target <= data.current_floor)
            {
                return elevator_direction_t::DOWN;
            }
            else
            {
                return elevator_direction_t::UP;
            }
        }

        // last direction = DOWN THEN CABIN is lower than currentFloor
        else if (last_direction == elevator_direction_t::DOWN)
        {

            // if wanna go back, GO UP
            if (data.target >= data.current_floor)
            {
                return elevator_direction_t::UP;
            }
            else
            {
                return elevator_direction_t::DOWN;
            }
        }

        // edge case if ele cant remember last_dir
        //  else {
        //      Serial.println("[WARN] Lost position! Homing down to find a floor.");
        //      return ElevatorDirection::DOWN;
        //  }
    }

    // 3. normal case: (btwFloor == false)
    if (data.target > data.current_floor)
    {
        return elevator_direction_t::UP;
    }

    if (data.target < data.current_floor)
    {
        return elevator_direction_t::DOWN;
    }

    // prevent false (Fallback)
    return elevator_direction_t::NONE;
};

void Orchestrator::user_command_handle(user_command cmd)
{
    if (data.current_state == elevator_state_t::EMERGENCY)
    {
        Serial.println("[WARN] Command Ignored: Elevator is in EMERGENCY state!");
        return;
    }

    data.target = cmd.target;

    switch (cmd.type)
    {
    case command_type_t::TRANSIT:
        if (data.current_state == elevator_state_t::IDLE)
        {
            data.current_state = elevator_state_t::RUNNING;
        }
        break;

    case command_type_t::STOP:
        if (data.current_state == elevator_state_t::RUNNING)
        {
            stop_running();
        }
        break;

    case command_type_t::E_STOP:
        /* code */
        break;

    default:
        break;
    }
};

// void Ochestrator::eventHandle() {};
// void Ochestrator::isReachFloor(uint8_t floorNum) {};
// void Ochestrator::clearCommand() {};
// void Ochestrator::isSafeToRun(ElevatorDirection dir) {};