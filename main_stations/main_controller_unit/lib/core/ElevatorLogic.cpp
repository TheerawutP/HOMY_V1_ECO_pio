// ElevatorLogic.cpp
#include "ElevatorLogic.h"
#include "Config.h"
#include "ElevatorHal.h"
#include "ElevatorTypes.h"
#include <Arduino.h>

extern QueueHandle_t xQueueCommand;
extern TaskHandle_t xElevatorHandle;
extern QueueHandle_t xQueueSending;

Orchestrator::Orchestrator(ElevatorHal *hardwarePtr)

{
    this->hal = hardwarePtr;
    data.current_floor = 1;
    data.btw_floor = false;
    data.current_state = elevator_state_t::IDLE;
    last_direction = elevator_direction_t::NONE;

    xSafetyEventGroup = xEventGroupCreate();
    xEventGroupClearBits(xSafetyEventGroup, 0xFFFFFF);
};

void Orchestrator::update_position()
{
    // 1. Sync data with hardware
    // hal->update_sensor();  //move to vSensor
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
};

void Orchestrator::execute_state_machine()
{

    // Control flow based on state
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
            if (is_safe_to_run(dir))
            {
                hal->motor_rotate(dir);
                last_direction = dir;
            }
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

    case command_type_t::EMG_STOP:
        /* code */
        break;

    default:
        break;
    }
};

void Orchestrator::event_handle(uint32_t evt_mask)
{
    // !!! high priority events that can cause safety issues should be handled first, regardless of current state
    espnow_msg_t outgoing_msg;

    if (evt_mask & SAFETY_BRAKE_ENGAGE)
    {
        Serial.println("[CRITICAL] Sling cut detected! EMERGENCY state!");
        stop_running();
        data.current_state = elevator_state_t::EMERGENCY;

        outgoing_msg = {(uint8_t)station_role_t::CABIN, OUT_EN_BRAKE};
        xQueueSend(xQueueSending, &outgoing_msg, 0);
    }

    // =========================================================
    // 🟠 Priority 2:  (System Protection)
    // if it's not in EMERGENCY, then these events can cause safety issues and should be handled immediately.
    // =========================================================
    if (data.current_state != elevator_state_t::EMERGENCY)
    {
        if (evt_mask & VSG_ALARM_TRIGGER)
        {
            xEventGroupSetBits(xSafetyEventGroup, VSG_BIT);

            if (data.current_state == elevator_state_t::RUNNING)
            {
                Serial.println("[WARN] VSG IS ALARM (VSG) -> STOP!");
                stop_running();
                data.current_state = elevator_state_t::IDLE;

                user_command cmd = {2, command_type_t::TRANSIT};
                xQueueSend(xQueueCommand, &cmd, 0);
            }
        }

        if (evt_mask & VSG_ALARM_CLEAR)
        {
            xEventGroupClearBits(xSafetyEventGroup, VSG_BIT);
            Serial.println("[INFO] VSG ALARM IS CLEARED (VSG) -> CAN RUN!");
        }

        if (evt_mask & VTG_ALARM_TRIGGER)
        {
            xEventGroupSetBits(xSafetyEventGroup, VTG_BIT);
            if (data.current_state == elevator_state_t::RUNNING)
            {
                Serial.println("[WARN] VTG IS ALARM (VTG) -> STOP!");
                stop_running();
                this->hal->emergency_stop();
                data.current_state = elevator_state_t::IDLE;

                user_command cmd = {1, command_type_t::TRANSIT};
                xQueueSend(xQueueCommand, &cmd, 0);
            }
        }

        if (evt_mask & VTG_ALARM_CLEAR)
        {
            xEventGroupClearBits(xSafetyEventGroup, VTG_BIT);
            Serial.println("[INFO] VTG ALARM IS CLEARED (VTG) -> CAN RUN!");
        }

        if (evt_mask & DOOR_IS_OPEN)
        {
            xEventGroupSetBits(xSafetyEventGroup, DOOR_OPEN_BIT);
            Serial.println("[INFO] DOOR IS OPEN -> STOP!");
            stop_running();
            this->hal->emergency_stop();
            data.current_state = elevator_state_t::IDLE;
        }
    }

    // =========================================================
    // 🟢 Priority 3: (Normal Events)
    // =========================================================
    if (evt_mask & DOOR_IS_CLOSED)
    {
        xEventGroupClearBits(xSafetyEventGroup, DOOR_OPEN_BIT);
        Serial.println("[INFO] DOOR IS CLOSED -> CAN RUN!");
    }
};

void Orchestrator::process_remote_message(espnow_msg_t msg)
{

    if (msg.id == (uint8_t)station_role_t::CABIN)
    {
        uint16_t current_cabin = msg.cmd;

        if (current_cabin != last_cabin_frame)
        {
            bool is_door_closed = current_cabin & (1 << 0);
            bool aim_2 = current_cabin & (1 << 1);
            bool aim_1 = current_cabin & (1 << 2);
            bool user_stop = current_cabin & (1 << 3);
            bool emerg_stop = current_cabin & (1 << 4);

            bool last_is_door_closed = last_cabin_frame & (1 << 0);
            bool last_aim_2 = last_cabin_frame & (1 << 1);
            bool last_aim_1 = last_cabin_frame & (1 << 2);
            bool last_user_stop = last_cabin_frame & (1 << 3);
            bool last_emerg_stop = last_cabin_frame & (1 << 4);

            // ----------------------------------------------------
            // user command -> get in queue
            // ----------------------------------------------------
            if (aim_1 && !last_aim_1)
            {
                user_command cmd = {1, command_type_t::TRANSIT};
                xQueueSend(xQueueCommand, &cmd, 0);
            }
            if (aim_2 && !last_aim_2)
            {
                user_command cmd = {2, command_type_t::TRANSIT};
                xQueueSend(xQueueCommand, &cmd, 0);
            }
            if (user_stop && !last_user_stop)
            {
                user_command cmd = {0, command_type_t::STOP};
                xQueueSend(xQueueCommand, &cmd, 0);
            }
            if (emerg_stop && !last_emerg_stop)
            {
                user_command cmd = {0, command_type_t::EMG_STOP};
                xQueueSendToFront(xQueueCommand, &cmd, 0);
            }

            // ----------------------------------------------------
            // event notify -> send event to task
            // ----------------------------------------------------
            // eSetBits

            if (is_door_closed != last_is_door_closed)
            {
                if (is_door_closed)
                {
                    xTaskNotify(xElevatorHandle, DOOR_IS_CLOSED, eSetBits);
                }
                else
                {
                    xTaskNotify(xElevatorHandle, DOOR_IS_OPEN, eSetBits);
                }
            }

            last_cabin_frame = current_cabin;
        }
    }

    // ==========================================
    // data from VSG
    // ==========================================
    else if (msg.id == (uint8_t)station_role_t::VSG)
    {
        uint16_t current_vsg = msg.cmd;

        if (current_vsg != last_vsg_frame)
        {
            bool vsgPause = current_vsg & (1 << 0);
            bool last_vsgPause = last_vsg_frame & (1 << 0);

            if (vsgPause && !last_vsgPause)
            {
                xTaskNotify(xElevatorHandle, VSG_ALARM_TRIGGER, eSetBits);
            }
            else if (!vsgPause && last_vsgPause)
            {
                xTaskNotify(xElevatorHandle, VSG_ALARM_CLEAR, eSetBits);
            }

            last_vsg_frame = current_vsg;
        }
    }
};

bool Orchestrator::is_safe_to_run(elevator_direction_t dir)
{
    EventBits_t safety_status = xEventGroupGetBits(xSafetyEventGroup);

    // Check door status
    if (safety_status & DOOR_OPEN_BIT)
    {
        Serial.println("[SAFETY CHECK] Door is open! Not safe to run.");
        return false;
    }

    // Check VSG and VTG status
    if (safety_status & (VSG_BIT | VTG_BIT))
    {
        Serial.println("[SAFETY CHECK] VSG or VTG is in alarm state! Not safe to run.");
        return false;
    }

    return true; // Safe to run
};

// void Ochestrator::isReachFloor(uint8_t floorNum) {};
// void Ochestrator::clearCommand() {};
// void Ochestrator::isSafeToRun(ElevatorDirection dir) {};

// elevator_snapshot get_current_state() {
//     elevator_snapshot temp;
//     if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
//         temp = this->state;
//         xSemaphoreGive(dataMutex);
//     }
//     return temp;
// }
