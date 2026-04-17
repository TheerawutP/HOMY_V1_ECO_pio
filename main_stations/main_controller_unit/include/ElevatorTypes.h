//ElevatorTypes.h
#pragma once
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
  E_STOP
  // cutPower
};

struct elevator_snapshot {
    uint8_t target;
    elevator_state_t current_state;
    uint8_t current_floor;      
    bool btw_floor;           
    uint32_t safetyFlags;       // event bitmask 
};


struct user_command
{
  uint8_t target;
  command_type_t type;
  // commandSource_t from;

};

typedef struct
{
    uint8_t fromID;
    uint16_t commandFrame;
    uint16_t responseFrame;
    bool shouldResponse;
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

// constexpr uint32_t REACH_FLOOR_1       = (1 << 0); 
// constexpr uint32_t REACH_FLOOR_2       = (1 << 1);  
// constexpr uint32_t EMO_IS_PRESSED      = (1 << 2);  
// constexpr uint32_t SAFETY_BRAKE_ENGAGE = (1 << 3);

// enum {
//   REACH_FLOOR_1,
//   REACH_FLOOR_2,
//   EMO_IS_PRESSED,
//   SAFETY_BRAKE_ENGAGE
// }event_type_t;

// enum modbusStation_t
// {
//   INVERTER_STA,
//   CABIN_STA,
//   HALL_2_STA,
//   HALL_1_STA,
//   VSG_STA
// };

// typedef struct
// {
//   uint8_t target;
//   direction_t dir;
// } transitCommand_t;

// enum elevatorEvent_t
// {
//   SAFETY_BRAKE,
//   OVERSPEED,
//   DOOR_IS_OPEN,
//   DOOR_IS_CLOSED,
//   OVERTORQUE,
//   VTG_ALARM,
//   VSG_ALARM,
//   VTG_CLEAR,
//   VSG_CLEAR,
//   MODBUS_TIMEOUT,
//   NO_POWER,
//   COMMAND_CLEAR,
//   FLOOR1_REACHED,
//   FLOOR2_REACHED,
//   POWER_RESTORED,
//   PAUSED_CLEARED,
//   EMERG_PRESSED,
//   EMERG_RELEASED
// };

// typedef struct
// {
//   uint8_t pos;
//   state_t state;
//   direction_t dir;
//   direction_t lastDir;
//   uint8_t target;
//   uint8_t lastTarget;
//   bool isBrake;
//   bool btwFloor;
//   elevatorEvent_t evt;
//   bool hasChanged;
// } status_t;


// typedef struct {
//     struct {
//         bool pos : 1;
//         bool state : 1;
//         bool dir : 1;
//         bool lastDir : 1;
//         bool target : 1;
//         bool lastTarget : 1;
//         bool isBrake : 1;
//         bool btwFloor : 1;
//         bool hasChanged : 1;
//     } set; 

//     uint8_t pos;
//     state_t state;
//     direction_t dir;
//     direction_t lastDir;
//     uint8_t target;
//     uint8_t lastTarget;
//     bool isBrake;
//     bool btwFloor;
//     bool hasChanged;
// } update_status_t;


// enum commandSource_t
// {
//   FROM_RF,
//   FROM_WS,
//   FROM_CABIN,
//   FROM_HALL2
// };


// ///////////////////////////////obj in system.///////////////////////////
// typedef struct{
//   uint16_t writtenFrame[16];
//   bool shouldWrite;
  
//   uint32_t running_hz;
//   uint32_t torque;
//   uint16_t digitalInput;
// } inverter_t; 

// typedef struct{
//   uint16_t writtenFrame[16];
//   bool shouldWrite;

//   bool isDoorClosed;
//   bool isAim2;
//   bool isAim1;
//   bool isUserStop;
//   bool isEmergStop;
//   bool isBusy;
//   bool vtgAlarm;
// } cabin_t;

// typedef struct{ 
//  uint16_t writtenFrame[16];
//  bool shouldWrite;

//  bool isAlarm[5];
//  bool shouldPause;
// } vsg_t;

// typedef struct{
//   uint16_t writtenFrame[16];
//   bool shouldWrite;
  
//   bool isDoorClosed;
//   bool vtgAlarm;
// } hall_t;

// extern transitCommand_t command;
// extern status_t elevator;
// extern cabin_t cabinState;
// extern inverter_t inverterState;
// extern vsg_t vsgState;
// extern hall_t hallState;
// extern volatile uint16_t writeFrame[5][16];
// extern uint8_t MAX_FLOOR;




