#include <ModbusRTU.h>
#include <math.h>
#include <SoftwareSerial.h>
// #include "DFRobotDFPlayerMini.h"

#define SLAVE 3

#define modbus_RX 16
#define modbus_TX 17

//24v output
// #define R_safetySling 23

// #define R_OP 22
// #define R_CL 21

#define L_CALL 19
#define R_MAG 18


//24v input
#define CALL_SIG 32
#define VTG_SIG_1 33
#define VTG_SIG_2 25
#define LOCK_HALL_SIG 14

#define read_reg 0
#define write_reg 1
// uint16_t lastVal;
// uint16_t val;


ModbusRTU RTU_SLAVE;


volatile uint16_t parsing_data[16];
QueueHandle_t xProcessingQueue = NULL;

//----------------------------------------------------------------------------------------------------



uint16_t cbWrite(TRegister *reg, uint16_t val) {
  // if (lastVal != val) {
  //   lastVal = val;
  //   Serial.println(String("HregSet val:") + String(val));
  // }
  return val;
}

uint16_t cbRead(TRegister *reg, uint16_t val) {
  reg->value = val;
  return val;
}

void writeBit(uint16_t &value, uint8_t bit, bool state) {
  if (state) {
    value |= (1 << bit);  // set
  } else {
    value &= ~(1 << bit);  // clear
  }
}

// inline void safetySling(bool en) {
//   if (en) {
//     digitalWrite(R_safetySling, HIGH);
//   } else {
//     digitalWrite(R_safetySling, LOW);
//   }
// }

inline void L_upButton(bool en) {
  if (en) {
    digitalWrite(L_up, HIGH);
  } else {
    digitalWrite(L_up, LOW);
  }
}

inline void L_downButton(bool en) {
  if (en) {
    digitalWrite(L_down, HIGH);
  } else {
    digitalWrite(L_down, LOW);
  }
}

inline void L_stopButton(bool en) {
  if (en) {
    digitalWrite(L_stop, HIGH);
  } else {
    digitalWrite(L_stop, LOW);
  }
}

inline void L_emergButton(bool en) {
  if (en) {
    digitalWrite(L_emerg, HIGH);
  } else {
    digitalWrite(L_emerg, LOW);
  }
}

inline void light(bool en) {
  if (en) {
    digitalWrite(R_light, HIGH);
  } else {
    digitalWrite(R_light, LOW);
  }
}

void vModbusCom(void *pvParam) {

  // uint16_t parsing_data[16];
  uint16_t lastVal = 0;

  for (;;) {
    // memset(parsing_data, 0, sizeof(parsing_data));
    RTU_SLAVE.task();

    uint16_t val = RTU_SLAVE.Hreg(write_reg);


    for (int i = 0; i < 16; i++) {
      parsing_data[i] = (val >> i) & 0x01;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void vProcessing(void *pvParam) {
  uint16_t data[16];

  for (;;) {

      L_upButton(parsing_data[1]);
      L_downButton(parsing_data[2]);
      L_stopButton(parsing_data[3]);
      L_emergButton(parsing_data[4]);
      light(parsing_data[5]);


    vTaskDelay(20);
  }
}

void vWritePackage(void *pvParam) {
  uint16_t package = 0;
  bool doorState = false;
  bool aimUp = false;
  bool aimDw = false;
  bool userStop = false;
  bool emergStop = false;
  bool dfBusy = false;
  uint16_t lastPackage = 0;

  for (;;) {
    doorState = digitalRead(door_SS);
    // doorState = false;
    aimUp = digitalRead(upButton);
    aimDw = digitalRead(downButton);
    userStop = digitalRead(stopButton);
    emergStop = digitalRead(emergButton);
    // dfBusy = digitalRead(DF_BUSY);
    dfBusy = false;

    if (!doorState) {
      writeBit(package, 0, true);
    } else {
      writeBit(package, 0, false);
    }

    if (!aimUp) {
      writeBit(package, 1, true);
    } else {
      writeBit(package, 1, false);
    }

    if (!aimDw) {
      writeBit(package, 2, true);
    } else {
      writeBit(package, 2, false);
    }

    if (!userStop) {
      writeBit(package, 3, true);
    } else {
      writeBit(package, 3, false);
    }

    if (!emergStop) {
      writeBit(package, 4, true);
    } else {
      writeBit(package, 4, false);
    }

    if (!dfBusy) {
      writeBit(package, 5, true);
    } else {
      writeBit(package, 5, false);
    }


    if (package != lastPackage) {
      printDebugBits("TX (Slave->Master)", package, outgoingDesc);
      lastPackage = package;
    }

    RTU_SLAVE.Hreg(read_reg, package);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}



void setup() {
  // mySoftwareSerial.begin(9600);
  Serial.begin(115200);
  Serial2.begin(38400, SERIAL_8E1, modbus_RX, modbus_TX);  // RX=16, TX=17

  RTU_SLAVE.begin(&Serial2);
  RTU_SLAVE.slave(SLAVE);

  RTU_SLAVE.addHreg(0x0000, 0);
  // RTU_SLAVE.onSetHreg(0x0000, cbWrite);
  // RTU_SLAVE.onGetHreg(0x0000, cbRead);

  RTU_SLAVE.addHreg(0x0001, 0);
  // RTU_SLAVE.onSetHreg(0x0001, cbWrite);
  // RTU_SLAVE.onGetHreg(0x0001, cbRead);


  //set output
  //
  //
  //

  //set input
  //
  //
  //

  xProcessingQueue = xQueueCreate(32, sizeof(parsing_data));

  xTaskCreate(vWritePackage, "WritePackage", 4096, NULL, 3, NULL);
  xTaskCreate(vProcessing, "Processing", 4096, NULL, 3, NULL);
  xTaskCreate(vModbusCom, "ModbusCom", 4096, NULL, 3, NULL);
  // xTaskCreate(vClearBusy, "ClearBusy", 2048, NULL, 3, NULL);
}

void loop() {

  Serial.print("door_SS: ");
  Serial.println(digitalRead(door_SS));
  Serial.print("upButton: ");
  Serial.println(digitalRead(upButton));
  Serial.print("downButton: ");
  Serial.println(digitalRead(downButton));
  Serial.print("stopButton: ");
  Serial.println(digitalRead(stopButton));
  Serial.print("emergButton: ");
  Serial.println(digitalRead(emergButton));


  Serial.print("Hreg 0: ");
  Serial.println(RTU_SLAVE.Hreg(0), BIN);
  Serial.print("Hreg 1: ");
  Serial.println(RTU_SLAVE.Hreg(1), BIN);
  
  Serial.println("-----------------------------------");

  vTaskDelay(1000);
}
