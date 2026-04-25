// ElevatorTypes.h
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

struct elevator_snapshot
{
  uint8_t target;
  elevator_state_t current_state;
  uint8_t current_floor;
  bool btw_floor;
  elevator_direction_t dir;
  elevator_direction_t last_dir;
  uint32_t safety_flags; // event bitmask
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
    {cmd_stop_4, BTN_STOP}};

const int num_rf_keys = sizeof(rf_keys) / sizeof(rf_keymap_t);

// event bitmask

#define SAFETY_BRAKE_ENGAGE (1 << 0)
#define DOOR_IS_OPEN (1 << 1)
#define DOOR_IS_CLOSED (1 << 2)
#define EMO_IS_PRESSED (1 << 3)
#define EMO_IS_RELEASED (1 << 4)
#define VSG_ALARM_TRIGGER (1 << 5)
#define VSG_ALARM_CLEAR (1 << 6)
#define VTG_ALARM_TRIGGER (1 << 7)
#define VTG_ALARM_CLEAR (1 << 8)
#define REACH_FLOOR_1 (1 << 9)
#define REACH_FLOOR_2 (1 << 10)
#define REACH_FLOOR_3 (1 << 11)
#define BETWEEN_FLOOR (1 << 12)


#define OUT_EN_BRAKE (1 << 0)    // Bit 0: Enable safety sling
#define OUT_EN_L_UP (1 << 1)     // Bit 1: Light UP
#define OUT_EN_L_DW (1 << 2)     // Bit 2: Light DOWN
#define OUT_EN_L_STOP (1 << 3)   // Bit 3: Light STOP
#define OUT_EN_L_EM (1 << 4)     // Bit 4: Light EMERGENCY
#define OUT_EN_LIGHT (1 << 5)    // Bit 5: Enable Light
#define OUT_EN_SOUND (1 << 6)    // Bit 6: Enable Special Function
#define OUT_CANCEL_BUSY (1 << 7) // Bit 7: Cancel BUSY state

// Special Function Data (4-bit Payload)
#define OUT_TRACK_BIT0 (1 << 8)  // Bit 8: SFunc BIT0
#define OUT_TRACK_BIT1 (1 << 9)  // Bit 9: SFunc BIT1
#define OUT_TRACK_BIT2 (1 << 10) // Bit 10: SFunc BIT2
#define OUT_TRACK_BIT3 (1 << 11) // Bit 11: SFunc BIT3
#define OUT_TRACK_BIT4 (1 << 12)

// block moving
#define DOOR_OPEN_BIT (1 << 0)
#define EMG_STOP_BIT (1 << 1)

// block go up
#define VTG_BIT (1 << 2)

// block go down
#define SAFETY_BRAKE_BIT (1 << 3)
#define VSG_BIT (1 << 4)

// block dir group (true = dont allowed)
#define BLOCK_UP_MASK (VTG_BIT | EMG_STOP_BIT | DOOR_OPEN_BIT)
#define BLOCK_DOWN_MASK (SAFETY_BRAKE_BIT | VSG_BIT | EMG_STOP_BIT | DOOR_OPEN_BIT)

#define SF_0000 0
#define SF_1001 1  // going up
#define SF_1002 2  // going dw
#define SF_1003 3  // reaching
#define SF_1004 4  // obstable under cabin
#define SF_1005 5  // beware pinch
#define SF_1006 6  // no power go to floor1
#define SF_1007 7  // overweight
#define SF_1008 8  // safety brake
#define SF_1009 9  // please close the door
#define SF_1010 10 // elevator ready to use
#define SF_1011 11 // cant connect to wifi
#define SF_1012 12 // system going to reset
#define SF_1013 13 // wait for modbus
#define SF_1014 14 // reach1
#define SF_1015 15 // reach2
#define SF_1016 16 // emerg stop