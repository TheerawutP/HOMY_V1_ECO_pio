#ifndef ELEVATOR_TYPES_H
#define ELEVATOR_TYPES_H

#include <Arduino.h>

enum direction_t
{
  STAY,
  UP,
  DOWN
};


enum state_t
{
  IDLE,
  MOVING,
};


enum elevatorMode_t
{
  NORMAL,
  EMERGENCY
};

enum read_state
{
  PLC,
  INV,
};


typedef struct
{
  char cmd[16];
  bool isBrake;
  direction_t dir;
  state_t state;
  uint8_t pos;
  bool btwFloor;
  elevatorMode_t mode;
  uint8_t targetFloor;
} status_t;

typedef struct
{
  uint8_t floor;
  direction_t dir;
} TRANSIT;

#endif