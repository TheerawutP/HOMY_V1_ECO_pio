

// main helpers
void eventListener(uint32_t ulNotificationValue)
{
  emergType_t receivedType = (emergType_t)ulNotifiedValue;

  switch (receivedType)
  {
  case safetySling:
    // handle command received
    break;

  case emergStop:
    // handle limit switch
    break;

  case NoPowerLanding:
    // handle no power
    break;

  case lostConnection:
    break;

  case ClearCommand:
    break;

  case reachFloor1:
    break;

  case reachFloor2:
    break;

  default:
    break;
  }
}

void getDir() void abortAll() void stopMotion() 

bool readDataFrom(uint8_t slaveID, uint16_t startAddress, uint8_t numRead) {
  node.begin(slaveID, Serial1);
  uint8_t result = node.readHoldingRegisters(startAddress, numRead);
  
  if (result == node.ku8MBSuccess) {
    for (int i = 0; i < numRead; i++) {
      hreg[slaveID][i] = node.getResponseBuffer(i);
    }
    return true; 

  } else {
    Serial.print("Error at ID ");
    Serial.print(slaveID);
    Serial.print(": ");
    Serial.println(result, HEX); 
    return false; 
  }
}



    // other helpers

    // central state manager
    void vOchestrator(void *pvParameters)
{
  uint32_t ulNotificationValue;
  userCommand_t userCommand; // cmdType, source of command
  transitCommand_t command;
  emergency_t emergType;

  for (;;)
  {
    if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &ulNotificationValue, (Tick_t)10) == pdPASS)
    {
      eventListener(ulNotificationValue);
    }

    switch (currentState)
    {

    case STATE_IDLE:
      if (xQueueReceive(commandQueue, &userCommand, 0) == pdPASS)
      {
        command_t cmd = userCommand.type;
        uint8_t targetFloor = userCommand.target;

        switch (cmd)
        {
        case moveToFloor:
          getDir(targetFloor);
          break;

        case stop:
          if (currentState == STATE_RUNNING)
            stopMotion();
        }
      }
      break;

    case STATE_PENDING:
      xTimerStart(startRunningTimerHandle, 0);
      break;

    case STATE_RUNNING:
      transit(command);
      break;

    case STATE_PAUSED:
      stopMotion();
      break;

    case STATE_EMERGENCY:
      emergencyHandler(emergType);
      break;

    default:
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// main threads
void vRFReceiver()

    // timer callbacks
    void vStartRunningCallback(TimerHandle_t xTimer)

    // polling threads
    void vPollingModbus()
{
  for (;;)
  {
    uint32_t result;
    switch (currentStation)
    {
    case INVERTER_STA:
      readDataFrom(INVERTER_ID, FIRST_REG_INVERTER, NUM_READ_INVERTER);
      currentStation = CABIN_STA;
      break;

    case CABIN_STA:
      readDataFrom(CABIN_ID, FIRST_REG_CABIN, NUM_READ_CABIN);
      currentStation = HALL_STA;
      break;

    case HALL_STA:
      readDataFrom(HALL_ID, FIRST_REG_HALL, NUM_READ_HALL);
      currentStation = VSG_STA;
      break;

    case VSG_STA:
      readDataFrom(VSG_ID, FIRST_REG_VSG, NUM_READ_VSGL);
      currentStation = INVERTER_STA;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vPollingLowerLim() void vPollingNoPower()
    // safety threads
    void vAbortAll() void vStopMotion() void vNoPowerLanding() void vPollingTimeout() void vClearCommand()

        void setup()
{
}

void loop()
{
}
