//ElevatorTypes.h
#pragma once
#include "Config.h"
#include <stdint.h>

enum class elevator_direction_t
{
  NONE,
  UP,
  DOWN
};

enum class elevator_state_t
{
  IDLE,
  RUNNING,
  PAUSED,
  EMERGENCY
};

enum class command_type_t
{
  TRANSIT,
  STOP,
  EMG_STOP
};

struct elevator_snapshot {
    uint8_t target;
    elevator_state_t current_state;
    uint8_t current_floor;      
    bool btw_floor;           
    uint32_t safety_flags;       // event bitmask 
};


struct user_command
{
  uint8_t target;
  command_type_t type;
  // commandSource_t from;
};

typedef struct
{
    uint8_t id;
    uint16_t cmd;
    uint16_t response;
} espnow_msg_t;

enum class station_role_t : uint8_t
{
    UNKNOWN = 0,
    INVERTER = 1,
    CABIN = 2,
    HALL_1 = 3,
    HALL_2 = 4,
    HALL_3 = 5,
    VSG = 6,
    VTG = 7,
    MASTER = 100
};

struct station_info_t
{
    station_role_t role;
    uint8_t mac[6];
};

typedef enum
{
  BTN_TO_FLOOR_1,
  BTN_TO_FLOOR_2,
  BTN_TO_FLOOR_3,
  BTN_TO_FLOOR_4,
  BTN_TO_FLOOR_5,
  BTN_TO_FLOOR_6,
  BTN_STOP,
  BTN_EMERGENCY,
  BTN_UNKNOWN
} rf_button_t;

typedef struct
{
  unsigned long rf_code;
  rf_button_t type;
} rf_keymap_t;

const rf_keymap_t rf_keys[] = {
    // --- set 2 ---
    {cmd_floor_1_2, BTN_TO_FLOOR_1},
    {cmd_floor_2_2, BTN_TO_FLOOR_2},
    {cmd_stop_2, BTN_STOP},

    // --- set 3 ---
    {cmd_floor_1_3, BTN_TO_FLOOR_1},
    {cmd_floor_2_3, BTN_TO_FLOOR_2},
    {cmd_stop_3, BTN_STOP},

    // --- set 4 ---
    {cmd_floor_1_4, BTN_TO_FLOOR_1},
    {cmd_floor_2_4, BTN_TO_FLOOR_2},
    {cmd_stop_4, BTN_STOP}
};

const int num_rf_keys = sizeof(rf_keys) / sizeof(rf_keymap_t);

#define SAFETY_BRAKE_ENGAGE   (1 << 0)
#define DOOR_IS_OPEN          (1 << 1)
#define DOOR_IS_CLOSED        (1 << 2)
#define EMO_IS_PRESSED        (1 << 3)
#define EMO_IS_RELEASED       (1 << 4)
#define VSG_ALARM_TRIGGER     (1 << 5)
#define VSG_ALARM_CLEAR       (1 << 6)
