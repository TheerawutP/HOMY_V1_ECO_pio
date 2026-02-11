#ifndef ELEVATOR_TYPES_H
#define ELEVATOR_TYPES_H
// #include "elevatorTypes.h"
#include <Arduino.h>

enum direction_t
{
  DIR_NONE,
  DIR_UP,
  DIR_DOWN
};

enum state_t
{
  STATE_IDLE,
  STATE_PENDING,
  STATE_RUNNING,
  STATE_PAUSED,
  STATE_EMERGENCY
};

enum modbusStation_t
{
  INVERTER_STA,
  CABIN_STA,
  HALL_2_STA,
  HALL_1_STA,
  VSG_STA
};

typedef struct
{
  uint8_t target;
  direction_t dir;
} transitCommand_t;

typedef struct
{
  uint8_t pos;
  state_t state;
  direction_t dir;
  direction_t lastDir;
  uint8_t target;
  uint8_t lastTarget;
  bool isBrake;
  bool btwFloor;
  bool hasChanged;
} status_t;


typedef struct {
    struct {
        bool pos : 1;
        bool state : 1;
        bool dir : 1;
        bool lastDir : 1;
        bool target : 1;
        bool lastTarget : 1;
        bool isBrake : 1;
        bool btwFloor : 1;
        bool hasChanged : 1;
    } set; 

    uint8_t pos;
    state_t state;
    direction_t dir;
    direction_t lastDir;
    uint8_t target;
    uint8_t lastTarget;
    bool isBrake;
    bool btwFloor;
    bool hasChanged;
} update_status_t;

enum elevatorEvent_t
{
  safetySling,
  emergStop,
  pauseClear,
  noPowerLanding,
  modbusTimeout,
  clearCommand,
  reachFloor1,
  reachFloor2,
  powerRestored
};

enum commandType_t
{
  moveToFloor,
  userAbort,
  userEmergStop,
  cutPower
};

enum commandSource_t
{
  FROM_RF,
  FROM_WS,
  FROM_CABIN,
  FROM_HALL
};

typedef struct
{
  uint8_t target;
  commandType_t type;
  commandSource_t from;
} userCommand_t;

#endif  