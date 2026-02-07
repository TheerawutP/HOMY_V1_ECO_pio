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
  HALL_STA,
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
} status_t;

typedef struct
{
  uint8_t *pos;
  state_t *state;
  direction_t *dir;
  direction_t *lastDir;
  uint8_t *target;
  uint8_t *lastTarget;
  bool *isBrake;
  bool *btwFloor;
} update_status_t;

enum elevatorEvent_t
{
  safetySling,
  emergStop,
  noPowerLanding,
  pollingTimeout,
  clearCommand,
  reachFloor1,
  reachFloor2
};

enum commandType_t
{
  moveToFloor,
  stop,
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