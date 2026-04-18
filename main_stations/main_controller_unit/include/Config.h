#pragma once
#include <stdint.h>

// Hardware pins
#define pin_mb_rx 16
#define pin_mb_tx 17
#define pin_sys_ready 13
#define pin_rf_receiver 23

const uint8_t PIN_UP = 19;
const uint8_t PIN_DOWN = 18;
const uint8_t PIN_BRAKE = 5;
const uint8_t PIN_SS_FLOOR_1 =32;
const uint8_t PIN_SS_FLOOR_2 = 33;
const uint8_t PIN_EMO = 21;
const uint8_t PIN_NO_POWER = 27;
const uint8_t PIN_SPEED = 14;
const uint8_t PIN_SLING = 26;

// remote set2
#define cmd_floor_1_2 12418580
#define cmd_floor_2_2 12418584
#define cmd_stop_2 12418578
#define cmd_emg_remote_stop_2 12418577

// remote set3
#define cmd_floor_1_3 16003636
#define cmd_floor_2_3 16003640
#define cmd_stop_3 16003634
#define cmd_emg_stop_3 16003633

// remote set4
#define cmd_floor_1_4 16751236
#define cmd_floor_2_4 16751240
#define cmd_stop_4 16751234
#define cmd_emg_stop_4 16751233
