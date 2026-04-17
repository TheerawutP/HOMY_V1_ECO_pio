// #ifndef CONFIG_H
// #define CONFIG_H
#pragma once
#include <stdint.h>

// Hardware pins
#define PIN_RX 16
#define PIN_TX 17
#define WIFI_READY 13
#define RFReceiver 23
// #define floorSensor1 32
// #define floorSensor2 33
// #define R_UP 19        // Relay UP
// #define R_DW 18        // Relay DOWN
#define R_POWER_CUT 15 // Relay 4
// #define BRK 5          // brake
// #define NoPower 27
// #define safetySling 26
// #define speedGovernor 14
// #define EMO 21 // emergency stop

const uint8_t PIN_UP = 19;
const uint8_t PIN_DOWN = 18;
const uint8_t PIN_BRAKE = 5;
const uint8_t PIN_SS_FLOOR_1 =32;
const uint8_t PIN_SS_FLOOR_2 = 33;
const uint8_t PIN_EMO = 21;
const uint8_t PIN_NO_POWER = 27;
const uint8_t PIN_SPEED = 14;
const uint8_t PIN_SLING = 26;

#define SAFETY_BRAKE_ENGAGE   (1 << 0)
#define DOOR_IS_OPEN          (1 << 1)
#define DOOR_IS_CLOSED        (1 << 2)
#define EMO_IS_PRESSED        (1 << 3)
#define EMO_IS_RELEASED       (1 << 4)
#define VSG_ALARM_TRIGGER     (1 << 5)
#define VSG_ALARM_CLEAR       (1 << 6)



// #define MASTER_ID 100

// // RF remote codes
// #define toFloor1 174744
// #define toFloor2 174740
// #define STOP 174737
// #define INVERTER_DI_STOP 992
// #define INVERTER_DI_EMO 5092

// #define toFloor1_2 12418580
// #define toFloor2_2 12418584
// #define STOP_2 12418578
// #define EM_remote_2 12418577

// #define toFloor1_3 16003636
// #define toFloor2_3 16003640
// #define STOP_3 16003634
// #define EM_remote_3 16003633

// #define toFloor1_4 16751236
// #define toFloor2_4 16751240
// #define STOP_4 16751234
// #define EM_remote_4 16751233

// // Cabin sound tracks
// #define SF_0000 0
// #define SF_1001 1  // going up
// #define SF_1002 2  // going dw
// #define SF_1003 3  // reaching
// #define SF_1004 4  // obstable under cabin
// #define SF_1005 5  // beware pinch
// #define SF_1006 6  // no power go to floor1
// #define SF_1007 7  // overweight
// #define SF_1008 8  // safety brake
// #define SF_1009 9  // please close the door
// #define SF_1010 10 // elevator ready to use
// #define SF_1011 11 // cant connect to wifi
// #define SF_1012 12 // system going to reset
// #define SF_1013 13 // wait for modbus
// #define SF_1014 14 // reach1
// #define SF_1015 15 // reach2
// #define SF_1016 16 // emerg stop

// // Safety event bits
// #define MODBUS_DIS_BIT (1 << 0)
// #define DOOR_OPEN_BIT (1 << 1)
// #define EMERG_BIT (1 << 2)
// #define VTG_BIT (1 << 3)
// #define SAFETY_BRAKE_BIT (1 << 4)
// #define VSG_BIT (1 << 5)

// #define BLOCK_UP_MASK (VTG_BIT | EMERG_BIT | DOOR_OPEN_BIT | MODBUS_DIS_BIT)
// #define BLOCK_DOWN_MASK (SAFETY_BRAKE_BIT | VSG_BIT | EMERG_BIT | DOOR_OPEN_BIT | MODBUS_DIS_BIT)

// // Timing config
// #define WAIT_TO_RUNNING_MS 600
// #define IDLE_LIGHT_TIMEOUT_MS 60000

// static const uint32_t upper_bound_speed_interval = 200;
// static const uint32_t lower_bound_speed_interval = 250;
// static const uint32_t overSpeed_threshold = 2;
// static const uint8_t MIN_FLOOR = 1;

// // MQTT / network config
// static const char *mqtt_broker = "kit.flinkone.com";
// static const int mqtt_port = 1883;
// static const char *KIT_topic = "kit";
// static const char *UT_case = "/UT_55555";
// static const char *system_status = "/sys_v2";
// static const char *elevator_status = "/ele_status";
// static const char *inverter_status = "/inv_status";

// static const int WAIT_FOR_WIFI_TIME_OUT = 6000;
// static const char *PARAM_MESSAGE = "message";
// static const uint8_t DNS_PORT = 53;

// static const char *PREF_NS_ESP_NOW_MAC = "espnow-mac";
// static const char *PREF_KEY_CABIN_MAC = "cabin_mac";
// static const char *PREF_KEY_VSG_MAC = "vsg_mac";
// static const char *PREF_KEY_VTG_MAC = "vtg_mac";

// #endif // CONFIG_H
