/************************************************************************
 * NASA Docket No. GSC-18,719-1, and identified as “core Flight System: Bootes”
 *
 * Copyright (c) 2020 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License. You may obtain
 * a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ************************************************************************/

/**
 * \file
 *   This file contains the source code for the Temperature App.
 */

/*
** Include Files:
*/
#include "temperature_app_events.h"
#include "temperature_app_version.h"
#include "temperature_app.h"

/*
** global data
*/
TEMPERATURE_APP_Data_t TEMPERATURE_APP_Data;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/*                                                                            */
/* Application entry point and main process loop                              */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void TEMPERATURE_APP_Main(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
    ** Create the first Performance Log entry
    */
    CFE_ES_PerfLogEntry(TEMPERATURE_APP_PERF_ID);

    /*
    ** Perform application specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = TEMPERATURE_APP_Init();
    if (status != CFE_SUCCESS)
    {
        TEMPERATURE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    status = aht10_init();
    if (status != CFE_SUCCESS)
    {
        TEMPERATURE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        CFE_EVS_SendEvent(TEMPERATURE_APP_DEV_INF_EID, CFE_EVS_EventType_ERROR,
                          "TEMPERATURE APP: Error configurin MPU6050\n");
    }

    /*
    ** TEMPERATURE Runloop
    */
    while (CFE_ES_RunLoop(&TEMPERATURE_APP_Data.RunStatus) == true)
    {
        /*
        ** Performance Log Exit Stamp
        */
        CFE_ES_PerfLogExit(TEMPERATURE_APP_PERF_ID);

        /* Pend on receipt of command packet */
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, TEMPERATURE_APP_Data.CommandPipe, CFE_SB_PEND_FOREVER);

        /*
        ** Performance Log Entry Stamp
        */
        CFE_ES_PerfLogEntry(TEMPERATURE_APP_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            TEMPERATURE_APP_ProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_EVS_SendEvent(TEMPERATURE_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "TEMPERATURE APP: SB Pipe Read Error, App Will Exit");

            TEMPERATURE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    /*
    ** Performance Log Exit Stamp
    */
    CFE_ES_PerfLogExit(TEMPERATURE_APP_PERF_ID);

    CFE_ES_ExitApp(TEMPERATURE_APP_Data.RunStatus);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* Initialization                                                             */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 TEMPERATURE_APP_Init(void)
{
    int32 status;

    TEMPERATURE_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Initialize app command execution counters
    */
    TEMPERATURE_APP_Data.CmdCounter = 0;
    TEMPERATURE_APP_Data.ErrCounter = 0;
    TEMPERATURE_APP_Data.TimeCounter = 0;

    /*
    ** Initialize app configuration data
    */
    TEMPERATURE_APP_Data.PipeDepth = TEMPERATURE_APP_PIPE_DEPTH;

    strncpy(TEMPERATURE_APP_Data.PipeName, "TEMPERATURE_APP_CMD_PIPE", sizeof(TEMPERATURE_APP_Data.PipeName));
    TEMPERATURE_APP_Data.PipeName[sizeof(TEMPERATURE_APP_Data.PipeName) - 1] = 0;

    /*
    ** Initialize event filter table...
    */
    TEMPERATURE_APP_Data.EventFilters[0].EventID = TEMPERATURE_APP_STARTUP_INF_EID;
    TEMPERATURE_APP_Data.EventFilters[0].Mask    = 0x0000;
    TEMPERATURE_APP_Data.EventFilters[1].EventID = TEMPERATURE_APP_COMMAND_ERR_EID;
    TEMPERATURE_APP_Data.EventFilters[1].Mask    = 0x0000;
    TEMPERATURE_APP_Data.EventFilters[2].EventID = TEMPERATURE_APP_COMMANDNOP_INF_EID;
    TEMPERATURE_APP_Data.EventFilters[2].Mask    = 0x0000;
    TEMPERATURE_APP_Data.EventFilters[3].EventID = TEMPERATURE_APP_COMMANDRST_INF_EID;
    TEMPERATURE_APP_Data.EventFilters[3].Mask    = 0x0000;
    TEMPERATURE_APP_Data.EventFilters[4].EventID = TEMPERATURE_APP_INVALID_MSGID_ERR_EID;
    TEMPERATURE_APP_Data.EventFilters[4].Mask    = 0x0000;
    TEMPERATURE_APP_Data.EventFilters[5].EventID = TEMPERATURE_APP_LEN_ERR_EID;
    TEMPERATURE_APP_Data.EventFilters[5].Mask    = 0x0000;
    TEMPERATURE_APP_Data.EventFilters[6].EventID = TEMPERATURE_APP_PIPE_ERR_EID;
    TEMPERATURE_APP_Data.EventFilters[6].Mask    = 0x0000;
    TEMPERATURE_APP_Data.EventFilters[7].EventID = TEMPERATURE_APP_DEV_INF_EID;
    TEMPERATURE_APP_Data.EventFilters[7].Mask    = 0x0000;

    /*
    ** Register the events
    */
    status = CFE_EVS_Register(TEMPERATURE_APP_Data.EventFilters, TEMPERATURE_APP_EVENT_COUNTS, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Temperature App: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /*
    ** Initialize housekeeping packet (clear user data area).
    */
    CFE_MSG_Init(CFE_MSG_PTR(TEMPERATURE_APP_Data.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(TEMPERATURE_APP_HK_TLM_MID),
                 sizeof(TEMPERATURE_APP_Data.HkTlm));

    /*
    ** Initialize output RF packet.
    */
    CFE_MSG_Init(CFE_MSG_PTR(TEMPERATURE_APP_Data.OutData.TelemetryHeader), CFE_SB_ValueToMsgId(TEMPERATURE_APP_RF_DATA_MID),
                 sizeof(TEMPERATURE_APP_Data.OutData));

    /*
    ** Create Software Bus message pipe.
    */
    status = CFE_SB_CreatePipe(&TEMPERATURE_APP_Data.CommandPipe, TEMPERATURE_APP_Data.PipeDepth, TEMPERATURE_APP_Data.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Temperature App: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /*
    ** Subscribe to Housekeeping request commands
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(TEMPERATURE_APP_SEND_HK_MID), TEMPERATURE_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Temperature App: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /*
    ** Subscribe to RF command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(TEMPERATURE_APP_SEND_RF_MID), TEMPERATURE_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Temperature App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return status;
    }

    /*
    ** Subscribe to ground command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(TEMPERATURE_APP_CMD_MID), TEMPERATURE_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Temperature App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return status;
    }


    CFE_EVS_SendEvent(TEMPERATURE_APP_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMPERATURE App Initialized.%s",
                      TEMPERATURE_APP_VERSION_STRING);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the TEMPERATURE    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void TEMPERATURE_APP_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case TEMPERATURE_APP_CMD_MID:
            TEMPERATURE_APP_ProcessGroundCommand(SBBufPtr);
            break;

        case TEMPERATURE_APP_SEND_HK_MID:
            TEMPERATURE_APP_ReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        case TEMPERATURE_APP_SEND_RF_MID:
            TEMPERATURE_APP_ReportRFTelemetry((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        default:
            CFE_EVS_SendEvent(TEMPERATURE_APP_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "TEMPERATURE: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* TEMPERATURE ground commands                                                     */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void TEMPERATURE_APP_ProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    /*
    ** Process "known" TEMPERATURE app ground commands
    */
    switch (CommandCode)
    {
        case TEMPERATURE_APP_NOOP_CC:
            if (TEMPERATURE_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(TEMPERATURE_APP_NoopCmd_t)))
            {
                TEMPERATURE_APP_Noop((TEMPERATURE_APP_NoopCmd_t *)SBBufPtr);
            }

            break;

        case TEMPERATURE_APP_RESET_COUNTERS_CC:
            if (TEMPERATURE_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(TEMPERATURE_APP_ResetCountersCmd_t)))
            {
                TEMPERATURE_APP_ResetCounters((TEMPERATURE_APP_ResetCountersCmd_t *)SBBufPtr);
            }

            break;

	// TODO: Add the commands for the temperature control...

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(TEMPERATURE_APP_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }
}

int32 TEMPERATURE_APP_ReportRFTelemetry(const CFE_MSG_CommandHeader_t *Msg){

  /*
  ** Get command execution counters...
  */
  TEMPERATURE_APP_Data.OutData.CommandErrorCounter = TEMPERATURE_APP_Data.ErrCounter;
  TEMPERATURE_APP_Data.OutData.CommandCounter      = TEMPERATURE_APP_Data.CmdCounter;

  TEMPERATURE_APP_Data.OutData.AppID_H = (uint8_t) ((TEMPERATURE_APP_HK_TLM_MID >> 8) & 0xff);
  TEMPERATURE_APP_Data.OutData.AppID_L = (uint8_t) (TEMPERATURE_APP_HK_TLM_MID & 0xff);

  /* Copy the Temperature data */
  uint8_t *aux_array1;
  uint8_t *aux_array2;
  uint8_t *aux_array3;
  uint8_t *aux_array4;

  aux_array1 = NULL;
  aux_array1 = malloc(4 * sizeof(uint8_t));
  aux_array1 = (uint8_t*)(&TEMPERATURE_APP_Data.TemperatureRead);

  aux_array2 = NULL;
  aux_array2 = malloc(4 * sizeof(uint8_t));
  aux_array2 = (uint8_t*)(&TEMPERATURE_APP_Data.HumidityRead);

  aux_array3 = NULL;
  aux_array3 = malloc(4 * sizeof(uint8_t));
  aux_array3 = (uint8_t*)(&TEMPERATURE_APP_Data.MPU6050Temp);

  aux_array4 = NULL;
  aux_array4 = malloc(4 * sizeof(uint8_t));
  aux_array4 = (uint8_t*)(&TEMPERATURE_APP_Data.MPL3115A2Temp);

  for(int i=0;i<3;i++){

      TEMPERATURE_APP_Data.OutData.byte_group_1[i] = aux_array1[i];
      TEMPERATURE_APP_Data.OutData.byte_group_2[i] = aux_array2[i];
      TEMPERATURE_APP_Data.OutData.byte_group_3[i] = aux_array3[i];
      TEMPERATURE_APP_Data.OutData.byte_group_4[i] = aux_array4[i];

      TEMPERATURE_APP_Data.OutData.byte_group_5[i] = 0;
      TEMPERATURE_APP_Data.OutData.byte_group_6[i] = 0;
      TEMPERATURE_APP_Data.OutData.byte_group_7[i] = 0;
      TEMPERATURE_APP_Data.OutData.byte_group_8[i] = 0;
      TEMPERATURE_APP_Data.OutData.byte_group_9[i] = 0;

  }

  /*
  ** Send housekeeping telemetry packet...
  */
  CFE_SB_TimeStampMsg(CFE_MSG_PTR(TEMPERATURE_APP_Data.OutData.TelemetryHeader));
  CFE_SB_TransmitMsg(CFE_MSG_PTR(TEMPERATURE_APP_Data.OutData.TelemetryHeader), true);

  return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 TEMPERATURE_APP_ReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{

    /*
    ** Get command execution counters...
    */
    TEMPERATURE_APP_Data.HkTlm.Payload.CommandErrorCounter = TEMPERATURE_APP_Data.ErrCounter;
    TEMPERATURE_APP_Data.HkTlm.Payload.CommandCounter      = TEMPERATURE_APP_Data.CmdCounter;

    /* Copy the AHT10 data */
    aht10_read();
    TEMPERATURE_APP_Data.HkTlm.Payload.TemperatureRead = TEMPERATURE_APP_Data.TemperatureRead;
    TEMPERATURE_APP_Data.HkTlm.Payload.HumidityRead = TEMPERATURE_APP_Data.HumidityRead;

    /* Copy the other sensors data */
    TEMPERATURE_APP_Data.HkTlm.Payload.MPU6050Temp = TEMPERATURE_APP_Data.MPU6050Temp;
    TEMPERATURE_APP_Data.HkTlm.Payload.MPL3115A2Temp = TEMPERATURE_APP_Data.MPL3115A2Temp;

    TEMPERATURE_APP_Data.HkTlm.Payload.TimeCounter = TEMPERATURE_APP_Data.TimeCounter;

    /*
    ** Send housekeeping telemetry packet...
    */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(TEMPERATURE_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(TEMPERATURE_APP_Data.HkTlm.TelemetryHeader), true);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* TEMPERATURE NOOP commands                                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 TEMPERATURE_APP_Noop(const TEMPERATURE_APP_NoopCmd_t *Msg)
{
    TEMPERATURE_APP_Data.CmdCounter++;

    CFE_EVS_SendEvent(TEMPERATURE_APP_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMPERATURE: NOOP command %s",
                      TEMPERATURE_APP_VERSION);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 TEMPERATURE_APP_ResetCounters(const TEMPERATURE_APP_ResetCountersCmd_t *Msg)
{
    TEMPERATURE_APP_Data.CmdCounter = 0;
    TEMPERATURE_APP_Data.ErrCounter = 0;

    CFE_EVS_SendEvent(TEMPERATURE_APP_COMMANDRST_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMPERATURE: RESET command");

    return CFE_SUCCESS;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* Verify command packet length                                               */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool TEMPERATURE_APP_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    bool              result       = true;
    size_t            ActualLength = 0;
    CFE_SB_MsgId_t    MsgId        = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_FcnCode_t FcnCode      = 0;

    CFE_MSG_GetSize(MsgPtr, &ActualLength);

    /*
    ** Verify the command packet length.
    */
    if (ExpectedLength != ActualLength)
    {
        CFE_MSG_GetMsgId(MsgPtr, &MsgId);
        CFE_MSG_GetFcnCode(MsgPtr, &FcnCode);

        CFE_EVS_SendEvent(TEMPERATURE_APP_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), (unsigned int)FcnCode, (unsigned int)ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        TEMPERATURE_APP_Data.ErrCounter++;
    }

    return result;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* Functions to interact with the AHT10                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 aht10_init(void){
  int rv;
  int fd;

  // Device registration
  rv = i2c_dev_register_sensor_aht10(
    &bus_path[0],
    &aht10_path[0]
  );
  if(rv == 0)
    CFE_EVS_SendEvent(TEMPERATURE_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMPERATURE: Device registered correctly at %s",
                      aht10_path);

  fd = open(&aht10_path[0], O_RDWR);
  if(fd >= 0)
    CFE_EVS_SendEvent(TEMPERATURE_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMPERATURE: Device opened correctly at %s",
                      aht10_path);

  // Device configuration
  rv = sensor_aht10_begin(fd);
  CFE_EVS_SendEvent(TEMPERATURE_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMPERATURE: Device AHT10 initialized");

  close(fd);

  return CFE_SUCCESS;

}

void aht10_read(void){
  int fd;

  //Data reading every 120 seg approx
  if(TEMPERATURE_APP_Data.TimeCounter == 0){
    CFE_EVS_SendEvent(TEMPERATURE_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMPERATURE: Reading Device");
    fd = open(&aht10_path[0], O_RDWR);
    if(fd >= 0)
    sensor_aht10_read(fd);
    close(fd);
  }

  if(TEMPERATURE_APP_Data.TimeCounter <= 120){
    TEMPERATURE_APP_Data.TimeCounter++;
  }else{
    TEMPERATURE_APP_Data.TimeCounter = 0; // After 120 seg reset the counter
  }

  TEMPERATURE_APP_Data.TemperatureRead = sensor_aht10_get_temp();
  TEMPERATURE_APP_Data.HumidityRead = sensor_aht10_get_humid();

}

// Prototypes
static int sensor_aht10_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg);

static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff);
static int set_bytes(uint16_t chip_address, uint8_t **val, int numBytes);
static int sensor_aht10_get_reg_8(uint8_t register_add, uint8_t **buff);

static void updateHumidity();
static void updateTemperature();
static int readMeasurement(uint8_t **buff);

static int readStatusRegister(uint8_t **buff);
static uint8_t get_calibration_bit();
static void get_busy_bit();

// Functions
static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff){
  int rv;
  uint8_t value[nr_bytes];
  i2c_msg msgs[] = {{
    .addr = i2c_address,
    .flags = 0,
    .buf = &data_address,
    .len = 1,
  }, {
    .addr = i2c_address,
    .flags = I2C_M_RD,
    .buf = value,
    .len = nr_bytes,
  }};
  struct i2c_rdwr_ioctl_data payload = {
    .msgs = msgs,
    .nmsgs = sizeof(msgs)/sizeof(msgs[0]),
  };
  uint16_t i;

  rv = ioctl(fd, I2C_RDWR, &payload);
  if (rv < 0) {
    printf("ioctl failed...\n");
  } else {

    free(*buff);
    *buff = malloc(nr_bytes * sizeof(uint8_t));

    for (i = 0; i < nr_bytes; ++i) {
      (*buff)[i] = value[i];
    }
  }

  return rv;
}

static int set_bytes(uint16_t chip_address, uint8_t **val, int numBytes){

  int fd;
  int rv;

  if(chip_address == 0){
    chip_address = (uint16_t) AHT10_ADDRESS_X38;
  }

  uint8_t writebuff[numBytes];

  for(int i = 0; i<numBytes; i++){
    writebuff[i] = (*val)[i];
  }

  i2c_msg msgs[] = {{
    .addr = chip_address,
    .flags = 0,
    .buf = writebuff,
    .len = numBytes,
  }};
  struct i2c_rdwr_ioctl_data payload = {
    .msgs = msgs,
    .nmsgs = sizeof(msgs)/sizeof(msgs[0]),
  };

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  rv = ioctl(fd, I2C_RDWR, &payload);
  if (rv < 0) {
    perror("ioctl failed");
  }
  close(fd);

  return rv;
}

static int sensor_aht10_get_reg_8(uint8_t register_add, uint8_t **buff){

  int fd;
  int rv;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(1 * sizeof(uint8_t));

  uint16_t nr_bytes = (uint16_t) 1;
  uint16_t chip_address = (uint16_t) AHT10_ADDRESS_X38;
  uint8_t data_address = (uint8_t) register_add;

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  rv = read_bytes(fd, chip_address, data_address, nr_bytes, &tmp);

  close(fd);

  (*buff)[0] = *tmp;
  free(tmp);

  return rv;
}

static int sensor_aht10_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg){
  int err;
  uint8_t *val;
  int rv;

  switch (command) {
    case SENSOR_AHT10_SOFT_RST:

      val = NULL;
      val = malloc(1 * sizeof(uint8_t));

      val[0] = AHTXX_SOFT_RESET_REG;

      err = set_bytes(AHT10_ADDRESS_X38, &val, 1);

      OS_TaskDelay(AHTXX_SOFT_RESET_DELAY);
      break;

    case SENSOR_AHT10_NORMAL_MODE:
      OS_TaskDelay(AHTXX_CMD_DELAY);

      val = NULL;
      val = malloc(3 * sizeof(uint8_t));

      val[0] = AHT1X_INIT_REG;
      val[1] = AHTXX_INIT_CTRL_CAL_ON | AHT1X_INIT_CTRL_NORMAL_MODE;
      val[2] = AHTXX_INIT_CTRL_NOP;

      err = set_bytes(AHT10_ADDRESS_X38, &val, 3);

      break;

    case SENSOR_AHT10_READ:
      val = NULL;
      val = malloc(6 * sizeof(uint8_t));
      rv = readMeasurement(&val);

      if (rv >= 0){
        SENSOR_AHT10_Data.status = (val)[0];
        for (int i = 0; i < 5; i++) {
          SENSOR_AHT10_Data.rawData[i] = (val)[i+1];
        }
        updateHumidity();
        updateTemperature();
        err = 0;
      }else{
        printf("Error reading data...\n");
        err = -1;
      }
      break;

    default:
      err = -ENOTTY;
      break;
  }

  return err;
}

static void updateHumidity(){
  uint32_t  humidity   = SENSOR_AHT10_Data.rawData[0];                          //20-bit raw humidity data
            humidity <<= 8;
            humidity  |= SENSOR_AHT10_Data.rawData[1];
            humidity <<= 4;
            humidity  |= SENSOR_AHT10_Data.rawData[2] >> 4;

  if (humidity > 0x100000) {humidity = 0x100000;}             //check if RH>100

  SENSOR_AHT10_Data.sensor_humidity = ((float)humidity / 0x100000) * 100;
}

static void updateTemperature(){
  uint32_t temperature   = SENSOR_AHT10_Data.rawData[2] & 0x0F;                //20-bit raw temperature data
           temperature <<= 8;
           temperature  |= SENSOR_AHT10_Data.rawData[3];
           temperature <<= 8;
           temperature  |= SENSOR_AHT10_Data.rawData[4];

  SENSOR_AHT10_Data.sensor_temperature = ((float)temperature / 0x100000) * 200 - 50;
}

static int readMeasurement(uint8_t **buff){
  int fd;
  int rv;

  /* send measurement command */
  uint8_t *val;
  val = NULL;
  val = malloc(4 * sizeof(uint8_t));

  val[0] = AHTXX_START_MEASUREMENT_CTRL_NOP;
  val[1] = AHTXX_START_MEASUREMENT_REG;
  val[2] = AHTXX_START_MEASUREMENT_CTRL;
  val[3] = AHTXX_START_MEASUREMENT_CTRL_NOP;

  set_bytes(AHT10_ADDRESS_X38, &val, 4);

  /* check busy bit */
  get_busy_bit();                                                //update status byte, read status byte & check busy bit

  if      (SENSOR_AHT10_Data.status == AHTXX_BUSY_ERROR) {OS_TaskDelay(AHTXX_MEASUREMENT_DELAY - AHTXX_CMD_DELAY);}
  else if (SENSOR_AHT10_Data.status != AHTXX_NO_ERROR)   {return 1;}                                           //no reason to continue, received data smaller than expected

  /* read data from sensor */
  uint16_t nr_bytes = (uint16_t) 6;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(nr_bytes * sizeof(uint8_t));

  uint16_t chip_address = (uint16_t) AHT10_ADDRESS_X38;
  uint8_t data_address = (uint8_t) 0x00;  // No register address to read

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  rv = read_bytes(fd, chip_address, data_address, nr_bytes, &tmp);

  close(fd);

  for (int i = 0; i < nr_bytes; ++i) {
    (*buff)[i] = tmp[i];
  }
  free(tmp);

  /* check busy bit after measurement dalay */
  get_busy_bit(); //update status byte, read status byte & check busy bit

  if (SENSOR_AHT10_Data.status != AHTXX_NO_ERROR){
    return 1;
  } //no reason to continue, sensor is busy

  return rv;

}

static int readStatusRegister(uint8_t **buff){
  int err;

  OS_TaskDelay(AHTXX_CMD_DELAY);
  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(sizeof(uint8_t));

  err = sensor_aht10_get_reg_8(AHTXX_STATUS_REG, &tmp);
  (*buff)[0] = *tmp;

  return err;
}

static uint8_t get_calibration_bit(){
  uint8_t *value;
  value = NULL;

  readStatusRegister(&value);

  return ((*value) & AHTXX_STATUS_CTRL_CAL_ON); //0x08=loaded, 0x00=not loaded
}

static void get_busy_bit(){

  OS_TaskDelay(AHTXX_CMD_DELAY);

  uint8_t *value;
  value = NULL;

  readStatusRegister(&value);

  if(((*value) & AHTXX_STATUS_CTRL_BUSY) == AHTXX_STATUS_CTRL_BUSY){
    SENSOR_AHT10_Data.status = AHTXX_BUSY_ERROR; //0x80=busy, 0x00=measurement completed
  }else{
    SENSOR_AHT10_Data.status = AHTXX_NO_ERROR;
  }
}

int i2c_dev_register_sensor_aht10(const char *bus_path, const char *dev_path){
  i2c_dev *dev;

  dev = i2c_dev_alloc_and_init(sizeof(*dev), bus_path, AHT10_ADDRESS_X38);
  if (dev == NULL) {
    return -1;
  }

  dev->ioctl = sensor_aht10_ioctl;

  return i2c_dev_register(dev, dev_path);
}

int sensor_aht10_begin(int fd){
  int err;

  // Begin variables
  SENSOR_AHT10_Data.sensor_humidity = 0;
  SENSOR_AHT10_Data.sensor_temperature = 0;
  for (int i = 0; i < 5; i++) {
    SENSOR_AHT10_Data.rawData[i] = 0;
  }

  OS_TaskDelay(100);                  //wait for sensor to initialize

  // Do a soft reset before setting Normal Mode
  ioctl(fd, SENSOR_AHT10_SOFT_RST, NULL);
  ioctl(fd, SENSOR_AHT10_NORMAL_MODE, NULL);
  if(get_calibration_bit() == AHTXX_STATUS_CTRL_CAL_ON){
    err = 0;
  }else{
    err = 1;
  }
  return err;
}

int sensor_aht10_read(int fd){
  return ioctl(fd, SENSOR_AHT10_READ, NULL);
}

float sensor_aht10_get_temp(){
  return SENSOR_AHT10_Data.sensor_temperature;
}

float sensor_aht10_get_humid(){
  return SENSOR_AHT10_Data.sensor_humidity;
}
