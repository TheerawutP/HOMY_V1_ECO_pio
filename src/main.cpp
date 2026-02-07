// ตัวแปรหลัก
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
  elevatorMode_t mode;
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
  elevatorMode_t *mode;
} update_status_t;

typedef struct
{
  uint8_t target;
  direction_t dir;
} transitCommand_t;

status_t elevator = {
    1,
    STATE_IDLE,
    DIR_NONE,
    DIR_NONE,
    true false,
    MODE_NORMAL};

// main helpers
void eventListener(uint32_t ulNotificationValue, event_t *emg)
{
  emg = (event_t)ulNotifiedValue;

  switch (emg)
  {
  case safetySling:
    // handle command received
    break;

  case emergStop:
    // handle limit switch
    break;

  case noPowerLanding:
    // handle no power
    break;

  case pollingTimeout:
    break;

  case clearCommand:
    break;

  case reachFloor1:
    break;

  case reachFloor2:
    break;

  default:
    break;
  }
}

void getDir(uint8_t target, transitCommand_t *cmd)
{
  if (elevator.pos != target)
  {
    direction_t newDir = (target > elevator.pos) ? UP : DOWN;

    cmd->dir = newDir;
    cmd->target = target;

    updateElevator(&elevator, (update_status_t){
                                  .state = STATE_PENDING,
                                  .dir = &newDir,
                                  .lastTarget = &target});
  }
  else
  {
    if (elevator.btwFloor == true)
    {
      direction_t newDir = DIR_NONE;

      if (elevator.lastTarget > elevator.pos)
        newDir = DOWN;
      if (elevator.lastTarget < elevator.pos)
        newDir = UP;

      cmd->dir = newDir;
      cmd->target = target;

      updateElevator(&elevator, (update_status_t){
                                    .state = STATE_PENDING,
                                    .dir = &newDir,
                                    .target = &target});
    }
    else
    {
      Serial.println("It's here");
    }
  }
}

void transit(transitCommand_t cmd)
{
  ROTATE(cmd.dir);
  updateElevator(&elevator, (update_status_t){
                                .state = STATE_RUNNING,
                                .btwFloor = true,
                            });
}

void stopMotion()
{
  M_STOP();
}

void abortAll()
{
}

bool readDataFrom(uint8_t slaveID, uint16_t startAddress, uint8_t numRead, uint16_t *hreg_row)
{
  node.begin(slaveID, Serial1);
  uint8_t result = node.readHoldingRegisters(startAddress, numRead);

  if (result == node.ku8MBSuccess)
  {
    for (int i = 0; i < numRead; i++)
    {
      hreg_row[slaveID][i] = node.getResponseBuffer(i);
    }
    return true;
  }
  else
  {
    Serial.print("Error at ID ");
    Serial.print(slaveID);
    Serial.print(": ");
    Serial.println(result, HEX);
    return false;
  }
}

void emergencyHandler(event_t emergeType)
{
  switch (emergeType)
  {
  case safetySling:
    xTaskNotify(xSafetySlingHandle, 1, eSetValueWithOverwrite) break;

  case emergStop:
    xTaskNotify(xEmergeStopHandle, 1, eSetValueWithOverwrite)

        break;

  case noPowerLanding:
    xTaskNotify(xNoPowerLandingHandle, 1, eSetValueWithOverwrite)

        break;

  case pollingTimeout:
    xTaskNotify(xPollingTimeoutHandle, 1, eSetValueWithOverwrite)

        break;

  case clearCommand:
    xTaskNotify(xClearCommandHandle, 1, eSetValueWithOverwrite) break;

  default:
    break;
  }
}

// other helpers

// central state manager
void vOchestrator(void *pvParameters)
{

  uint32_t ulNotificationValue;
  userCommand_t userCommand; // cmdType, source of command
  transitCommand_t command;  // dir + target
  event_t evtType;

  for (;;)
  {
    if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &ulNotificationValue, (Tick_t)10) == pdPASS)
    {
      eventListener(ulNotificationValue, &evtType);
    }

    if (xQueueReceive(commandQueue, &userCommand, 0) == pdPASS)
    {
      command_t cmd = userCommand.type;
      uint8_t targetFloor = userCommand.target;

      switch (cmd)
      {
      case moveToFloor:
        getDir(targetFloor, command);
        break;

      case stop:
        if (elevator.state == STATE_RUNNING)
          updateElevator(&elevator, (update_status_t){
                                        .state = STATE_PAUSED});
      }
    }

    switch (elevator.state)
    {

    case STATE_IDLE:
      //...
      break;

    case STATE_PENDING:

      vTaskDelay(WAIT_TO_RUNNING_MS);
      updateElevator(&elevator, (update_status_t){
                                    .state = STATE_RUNNING,
                                });

      break;

    case STATE_RUNNING:
      transit(command);
      break;

    case STATE_PAUSED:
      stopMotion();
      break;

    case STATE_EMERGENCY:
      emergencyHandler(evtType);
      break;

    default:
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// main threads
void vRFReceiver(void *pvParams)
{
  for (;;)
  {
  }
}

// timer callbacks

// polling threads
void vPollingModbus(void *pvParams)
{
  uint16_t pollingData[5][32];

  for (;;)
  {
    uint32_t result;
    switch (currentStation)
    {
    case INVERTER_STA:
      readDataFrom(INVERTER_ID, FIRST_REG_INVERTER, NUM_READ_INVERTER, pollingData[INVERTER_ID]);

      if (pollingData[INVERTER_ID][7] == 992)
      {
        xTaskNotify(xEventListenerHandle, clearCommand, eSetValueWithOverwrite);
      }
      else if (pollingData(INVERTER_ID)[6] > TORQUE_RATED)
      {
        xTaskNotify(xEventListenerHandle, emergStop, eSetValueWithOverwrite);
      }
      currentStation = CABIN_STA;
      break;

    case CABIN_STA:
      readDataFrom(CABIN_ID, FIRST_REG_CABIN, NUM_READ_CABIN, pollingData[CABIN_ID]);
      if (pollingData[CABIN_ID][0] == 1)
      {
        xTaskNotify(xEventListenerHandle, emergStop, eSetValueWithOverwrite);
      }
       else if (pollingData[CABIN_ID][1] == 1)
      {
        xTaskNotify(xEventListenerHandle, safetySling, eSetValueWithOverwrite);
      }

      currentStation = HALL_STA;
      break;

    case HALL_STA:
      readDataFrom(HALL_ID, FIRST_REG_HALL, NUM_READ_HALL, pollingData[HALL_ID]);
      if (pollingData[HALL_ID][0] == 1 || pollingData[HALL_ID][1] == 1)
      {
        xTaskNotify(xEventListenerHandle, emergStop, eSetValueWithOverwrite);
      }
      currentStation = VSG_STA;
      break;

    case VSG_STA:
      readDataFrom(VSG_ID, FIRST_REG_VSG, NUM_READ_VSGL, pollingData[VSG_ID]);

      currentStation = INVERTER_STA;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vPollingLowerLim(void *pvParams) // first floor sensor
{
  uint8_t floorSensor1_counter = 0;
  const uint8_t STABLE_THRESHOLD = 20;

  for (;;)
  {
    bool raw_floorSensor1 = (digitalRead(floorSensor1) == LOW);
    if (raw_floorSensor1)
    {
      if (floorSensor1_counter < STABLE_THRESHOLD)
        floorSensor1_counter++;
    }
    else
    {
      floorSensor1_counter = 0;
    }

    bool isAtFloor1 = (floorSensor1_counter >= STABLE_THRESHOLD);

    if (isAtFloor1 == true)
    {
      xTaskNotify(xEventListenerHandle, reachFloor1, eSetValueWithOverwrite)
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void vPollingNoPower(void *pvParams)
{
  for (;;)
  {
    uint8_t noPower_counter = 0;
    const uint8_t STABLE_THRESHOLD = 20;

    for (;;)
    {
      bool raw_noPower = (digitalRead(NoPower) == LOW);
      if (raw_noPower)
      {
        if (noPower_counter < STABLE_THRESHOLD)
          noPower_counter++;
      }
      else
      {
        noPower_counter = 0;
      }

      bool isNoPower = (noPower_counter >= STABLE_THRESHOLD);

      if (isNoPower == true)
      {
        xTaskNotify(xEventListenerHandle, NoPowerLanding, eSetValueWithOverwrite)
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

// safety threads
void vSafetySling()
{
  for (;;)
  {
  }
}

void vEmergeStop()
{
  for (;;)
  {
  }
}

void vNoPowerLanding()
{
  for (;;)
  {
  }
}

void vPollingTimeout()
{
  for (;;)
  {
  }
}

void vClearCommand()
{
  for (;;)
  {
  }
}

void setup()
{
}

void loop()
{
}
