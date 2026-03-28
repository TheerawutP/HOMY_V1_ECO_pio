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

enum elevatorEvent_t
{
  SAFETY_BRAKE,
  OVERSPEED,
  DOOR_IS_OPEN,
  DOOR_IS_CLOSED,
  OVERTORQUE,
  VTG_ALARM,
  VSG_ALARM,
  VTG_CLEAR,
  VSG_CLEAR,
  MODBUS_TIMEOUT,
  NO_POWER,
  COMMAND_CLEAR,
  FLOOR1_REACHED,
  FLOOR2_REACHED,
  POWER_RESTORED,
  PAUSED_CLEARED,
  EMERG_PRESSED,
  EMERG_RELEASED
};

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
  elevatorEvent_t evt;
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
  FROM_HALL2
};

typedef struct
{
  uint8_t target;
  commandType_t type;
  commandSource_t from;
} userCommand_t;

///////////////////////////////obj in system.///////////////////////////
typedef struct{
  uint16_t writtenFrame[16];
  bool shouldWrite;
  
  uint32_t running_hz;
  uint32_t torque;
  uint16_t digitalInput;
} inverter_t; 

typedef struct{
  uint16_t writtenFrame[16];
  bool shouldWrite;

  bool isDoorClosed;
  bool isAim2;
  bool isAim1;
  bool isUserStop;
  bool isEmergStop;
  bool isBusy;
  bool vtgAlarm;
} cabin_t;

typedef struct{ 
 uint16_t writtenFrame[16];
 bool shouldWrite;

 bool isAlarm[5];
 bool shouldPause;
} vsg_t;

typedef struct{
  uint16_t writtenFrame[16];
  bool shouldWrite;
  
  bool isDoorClosed;
  bool vtgAlarm;
} hall_t;

#endif  

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void vSafetySling(void *pvParams)
// {
//   transitCommand_t command;
//   for (;;)
//   {
//     if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0)
//     {

//       if (elevator.dir == DIR_UP)
//       {
//         command.dir = DIR_UP;
//         command.target = elevator.pos + 1;
//       }
//       else
//       {
//         command.dir = DIR_UP;
//         command.target = elevator.pos;
//       }

//       transit(command);
//     }
//   }
// }

// void vPollingModbus(void *pvParams)
// {
//   uint8_t INVERTER_ID = 1;
//   uint8_t CABIN_ID = 2;
//   uint8_t HALL_2_ID = 3;
//   uint8_t VSG_ID = 4;

//   uint16_t FIRST_REG_INVERTER = 28672;
//   uint16_t FIRST_REG_CABIN = 0;
//   uint16_t FIRST_REG_HALL = 0;
//   uint16_t FIRST_REG_VSG = 0;

//   uint16_t NUM_READ_INVERTER = 1;
//   uint16_t NUM_READ_CABIN = 2;
//   uint16_t NUM_READ_HALL = 2;
//   uint16_t NUM_READ_VSG = 2;

//   uint16_t pollingData[5][32];
//   memset(pollingData, 0, sizeof(pollingData));

//   const uint8_t MAX_RETRIES = 3;

//   for (;;)
//   {

//     switch (read_current_sta)
//     {

//     case CABIN_STA:
//       readDataFrom(CABIN_ID, FIRST_REG_CABIN, NUM_READ_CABIN, pollingData[CABIN_ID]);
//       if (pollingData[CABIN_ID][0] == 2)
//       {
//           Serial.println("up");
//       }
//       else if (pollingData[CABIN_ID][0] == 4)
//       {
//           Serial.println("down");
//       }

//       read_current_sta = CABIN_STA;
//       break;

//     }
//     vTaskDelay(pdMS_TO_TICKS(20));
//   }
// }

// void vPollingModbus(void *pvParams)
// {
//   uint8_t INVERTER_ID = 1;
//   uint8_t CABIN_ID = 2;
//   uint8_t HALL_2_ID = 3;
//   uint8_t VSG_ID = 4;

//   uint16_t FIRST_REG_INVERTER = 28672;
//   uint16_t FIRST_REG_CABIN = 0;
//   uint16_t FIRST_REG_HALL = 0;
//   uint16_t FIRST_REG_VSG = 0;

//   uint16_t NUM_READ_INVERTER = 10;
//   uint16_t NUM_READ_CABIN = 1;
//   uint16_t NUM_READ_HALL = 1;
//   uint16_t NUM_READ_VSG = 1;

//   uint16_t pollingData[5][32];
//   // memset(pollingData, 0, sizeof(pollingData));

//   const uint8_t MAX_RETRIES = 3;

//   bool is_inverter_safe = true;
//   bool is_cabin_safe = true;
//   bool is_hall2_safe = true;
//   bool is_vsg_safe = true;

//   for (;;)
//   {
//     bool read_success = false;
//     uint8_t retry_i = 0;

//     if (xSemaphoreTake(modbusMutex, pdMS_TO_TICKS(10)) == pdTRUE)
//     {
//       switch (read_current_sta)
//       {
//       // -----------------------------------------------------------
//       // CASE 1: INVERTER
//       // -----------------------------------------------------------
//       case INVERTER_STA:
//         read_success = false;
//         for (retry_i = 0; retry_i < MAX_RETRIES; retry_i++)
//         {
//           if (readDataFrom(INVERTER_ID, FIRST_REG_INVERTER, NUM_READ_INVERTER, pollingData[INVERTER_ID]))
//           {
//             read_success = true;
//             break;
//           }
//           vTaskDelay(modbusRetryTime);
//         }

//         if (read_success)
//         {
//           if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
//           {
//             inverterState.running_hz = pollingData[INVERTER_ID][0] & (1 << 0);
//             inverterState.torque = pollingData[INVERTER_ID][0] & (1 << 6);
//             inverterState.digitalInput = pollingData[INVERTER_ID][0] & (1 << 7);
//             xSemaphoreGive(dataMutex);
//           }
//         }
//         else
//         {
//           xTaskNotify(xOchestratorHandle, modbusTimeout, eSetValueWithOverwrite);
//           is_inverter_safe = false;
//         }
//         read_current_sta = CABIN_STA;
//         break;

//       // -----------------------------------------------------------
//       // CASE 2: CABIN
//       // -----------------------------------------------------------
//       case CABIN_STA:
//         read_success = false;
//         for (retry_i = 0; retry_i < MAX_RETRIES; retry_i++)
//         {
//           if (readDataFrom(CABIN_ID, FIRST_REG_CABIN, NUM_READ_CABIN, pollingData[CABIN_ID]))
//           {
//             read_success = true;
//             break;
//           }
//           vTaskDelay(modbusRetryTime);
//         }

//         if (read_success)
//         {
//           if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
//           {
//             cabinState.isDoorClosed = pollingData[CABIN_ID][0] & (1 << 0);
//             cabinState.isAim2 = pollingData[CABIN_ID][0] & (1 << 1);
//             cabinState.isAim1 = pollingData[CABIN_ID][0] & (1 << 2);
//             cabinState.isUserStop = pollingData[CABIN_ID][0] & (1 << 3);
//             cabinState.isEmergStop = pollingData[CABIN_ID][0] & (1 << 4);
//             cabinState.isBusy = pollingData[CABIN_ID][0] & (1 << 5);
//             xSemaphoreGive(dataMutex);
//           }
//         }
//         else
//         {
//           xTaskNotify(xOchestratorHandle, modbusTimeout, eSetValueWithOverwrite);
//           is_cabin_safe = false;
//         }

//         read_current_sta = HALL_2_STA;
//         break;

//       // -----------------------------------------------------------
//       // CASE 3: HALL 2
//       // -----------------------------------------------------------
//       case HALL_2_STA:
//         // read_success = false;
//         // for (retry_i = 0; retry_i < MAX_RETRIES; retry_i++)
//         // {
//         //   if (readDataFrom(HALL_2_ID, FIRST_REG_HALL, NUM_READ_HALL, pollingData[HALL_2_ID]))
//         //   {
//         //     read_success = true;
//         //     break;
//         //   }
//         //   vTaskDelay(pdMS_TO_TICKS(10));
//         // }

//         // if (read_success)
//         // {
//         //   if (pollingData[HALL_2_ID][0] == 1)
//         //   {
//         //     is_hall2_safe = false;
//         //     xTaskNotify(xOchestratorHandle, emergStop, eSetValueWithOverwrite);
//         //   }
//         //   else
//         //   {
//         //     is_hall2_safe = true;
//         //   }
//         // }
//         // else
//         // {
//         //   // xTaskNotify(xOchestratorHandle, modbusTimeout, eSetValueWithOverwrite);
//         //   is_hall2_safe = false;
//         // }

//         read_current_sta = VSG_STA;
//         break;

//       // -----------------------------------------------------------
//       // CASE 4: VSG
//       // -----------------------------------------------------------
//       case VSG_STA:
//         read_success = false;
//         for (retry_i = 0; retry_i < MAX_RETRIES; retry_i++)
//         {
//           if (readDataFrom(VSG_ID, FIRST_REG_VSG, NUM_READ_VSG, pollingData[VSG_ID]))
//           {
//             read_success = true;
//             break;
//           }
//           vTaskDelay(modbusRetryTime);
//         }

//         if (read_success)
//         {
//           if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
//           {
//             vsgState.shouldPause = pollingData[VSG_ID][0] & (1 << 0);
//             vsgState.isAlarm[0] = pollingData[VSG_ID][0] & (1 << 1);
//             vsgState.isAlarm[1] = pollingData[VSG_ID][0] & (1 << 2);
//             vsgState.isAlarm[2] = pollingData[VSG_ID][0] & (1 << 3);
//             vsgState.isAlarm[3] = pollingData[VSG_ID][0] & (1 << 4);
//             vsgState.isAlarm[4] = pollingData[VSG_ID][0] & (1 << 5);
//             xSemaphoreGive(dataMutex);
//           }
//         }
//         else
//         {
//           xTaskNotify(xOchestratorHandle, modbusTimeout, eSetValueWithOverwrite);
//           is_vsg_safe = false;
//         }

//         if (elevator.state == STATE_PAUSED)
//         {
//           if (is_inverter_safe && is_cabin_safe && is_vsg_safe)
//           {
//             xTaskNotify(xOchestratorHandle, pauseClear, eSetValueWithOverwrite);
//           }
//         }

//         read_current_sta = INVERTER_STA;
//         break;
//       }
//       xSemaphoreGive(modbusMutex);
//     }
//     vTaskDelay(modbusDelayTime);
//   }
// }

// void vWriteStation(void *pvParams)
// {
// uint8_t INVERTER_ID = 1;
// uint8_t CABIN_ID = 2;
// uint8_t HALL_2_ID = 3;
// uint8_t VSG_ID = 4;

// uint16_t FIRST_REG_INVERTER = 28672;
// uint16_t FIRST_REG_CABIN = 1;
// uint16_t FIRST_REG_HALL = 1;
// uint16_t FIRST_REG_VSG = 1;

// uint16_t NUM_READ_INVERTER = 10;
// uint16_t NUM_READ_CABIN = 1;
// uint16_t NUM_READ_HALL = 1;
// uint16_t NUM_READ_VSG = 1;

//   uint16_t localValToSend = 0;
//   bool needToSend = false;

//   for (;;)
//   {

//     if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
//     {
//       if (cabinState.shouldWrite == true)
//       {
//         localValToSend = cabinState.writtenFrame[1];
//         needToSend = true;
//         cabinState.shouldWrite = false;
//       }
//       else
//       {
//         needToSend = false;
//       }
//       xSemaphoreGive(dataMutex);
//     }

//     if (needToSend)
//     {
//       if (xSemaphoreTake(modbusMutex, pdMS_TO_TICKS(10)) == pdTRUE)
//       {

//         node.begin(CABIN_ID, Serial1);
//         node.writeSingleRegister(0x0001, localValToSend);
//         cabinState.shouldWrite = false;

//         xSemaphoreGive(modbusMutex);
//       }
//     }

//     vTaskDelay(50);
//   }
// }

// void clearBitFrom(elevatorEvent_t casue){
//   switch (casue)
//   {
//     case SAFETY_BRAKE:
//         xEventGroupSetBits(xRunningEventGroup, SAFETY_BRAKE_BIT);
//     break;

//     case DOOR_IS_OPEN:
//     break;

//     case MODBUS_TIMEOUT:
//     break;

//   }
// }

// main threads
// void processCabinCall(uint8_t targetFloor, uint8_t soundTrack, uint8_t ledBit) {
//     userCommand_t userCommand;
//     userCommand.target = targetFloor;
//     userCommand.type = moveToFloor;
//     userCommand.from = FROM_CABIN;

//     // ส่ง Queue (เช็คเต็มด้วย)
//     if(xQueueSend(xQueueCommand, &userCommand, 0) == pdTRUE) {
//         // สั่ง Modbus
//         writeFrameDFPlayer(soundTrack, cabinState.writtenFrame[1], cabinState.isBusy, 6);
//         writeBit(cabinState.writtenFrame[1], ledBit, true);
//         enableTransmit(cabinState.shouldWrite);
//     }
// }

// void vProcessData(void *pvParams)
// {
//   for (;;)
//   {
//     if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
//     {

//       // inverter sta
//       if (inverterState.digitalInput == INVERTER_DI_STOP)
//       {
//         xTaskNotify(xOchestratorHandle, COMMAND_CLEAR, eSetValueWithOverwrite);
//       }

//       if (inverterState.digitalInput == INVERTER_DI_EMO)
//       {
//         xTaskNotify(xOchestratorHandle, EMERG_PRESSED, eSetValueWithOverwrite);
//       }

//       // cabin sta
//       if (cabinState.isDoorClosed == false)
//       {
//         xTaskNotify(xOchestratorHandle, DOOR_IS_OPEN, eSetValueWithOverwrite);
//       }
//       else
//       {
//         xTaskNotify(xOchestratorHandle, DOOR_IS_CLOSED, eSetValueWithOverwrite);
//       }

//       if (cabinState.isAim2)
//       {
//         userCommand_t userCommand; // target, type, from
//         userCommand.target = 2;
//         userCommand.type = moveToFloor;
//         userCommand.from = FROM_CABIN;
//         xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);

//         writeFrameDFPlayer(SF_1001, cabinState.writtenFrame[1], cabinState.isBusy, 6);
//         writeBit(cabinState.writtenFrame[1], 1, true);
//         enableTransmit(cabinState.shouldWrite);
//       }

//       if (cabinState.isAim1)
//       {
//         userCommand_t userCommand; // target, type, from
//         userCommand.target = 1;
//         userCommand.type = moveToFloor;
//         userCommand.from = FROM_CABIN;
//         xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);

//         writeFrameDFPlayer(SF_1002, cabinState.writtenFrame[1], cabinState.isBusy, 6);
//         writeBit(cabinState.writtenFrame[1], 2, true); // wriiten reg, enable L_UP, true
//         enableTransmit(cabinState.shouldWrite);
//       }

//       if (cabinState.isUserStop)
//       {
//         userCommand_t userCommand; // target, type, from
//         userCommand.target = 0;
//         userCommand.type = userAbort;
//         userCommand.from = FROM_CABIN;
//         xQueueSend(xQueueCommand, &userCommand, (TickType_t)0);
//       }

//       if (cabinState.isEmergStop)
//       {
//         xTaskNotify(xOchestratorHandle, EMERG_PRESSED, eSetValueWithOverwrite);
//       }

//       if (cabinState.isBusy)
//       {
//       }

//       if (cabinState.vtgAlarm == true)
//       {
//         xTaskNotify(xOchestratorHandle, VTG_ALARM, eSetValueWithOverwrite);
//       }
//       else
//       {
//         xTaskNotify(xOchestratorHandle, VTG_CLEAR, eSetValueWithOverwrite);
//       }

//       // vsg sta
//       if (vsgState.shouldPause == true)
//       {
//         xTaskNotify(xOchestratorHandle, VSG_ALARM, eSetValueWithOverwrite);
//       }
//       else
//       {
//         xTaskNotify(xOchestratorHandle, VSG_CLEAR, eSetValueWithOverwrite);
//       }
//     }

//     vTaskDelay(pdTICKS_TO_MS(20));
//   }
// }