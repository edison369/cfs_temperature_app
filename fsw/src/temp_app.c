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
#include "temp_app_events.h"
#include "temp_app_version.h"
#include "temp_app.h"


#include "sensor-aht10.h"

/*
** global data
*/
TEMP_APP_Data_t TEMP_APP_Data;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/*                                                                            */
/* Application entry point and main process loop                              */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void TEMP_APP_Main(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
    ** Create the first Performance Log entry
    */
    CFE_ES_PerfLogEntry(TEMP_APP_PERF_ID);

    /*
    ** Perform application specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = TEMP_APP_Init();
    if (status != CFE_SUCCESS)
    {
        TEMP_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    status = aht10_init();
    if (status != CFE_SUCCESS)
    {
        TEMP_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        CFE_EVS_SendEvent(TEMP_APP_DEV_INF_EID, CFE_EVS_EventType_ERROR,
                          "TEMP APP: Error initializing AHT10\n");
    }

    /*
    ** TEMP Runloop
    */
    while (CFE_ES_RunLoop(&TEMP_APP_Data.RunStatus) == true)
    {
        /*
        ** Performance Log Exit Stamp
        */
        CFE_ES_PerfLogExit(TEMP_APP_PERF_ID);

        get_sensors_temp();

        /* Pend on receipt of command packet */
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, TEMP_APP_Data.CommandPipe, CFE_SB_PEND_FOREVER);

        /*
        ** Performance Log Entry Stamp
        */
        CFE_ES_PerfLogEntry(TEMP_APP_PERF_ID);

        if (status == CFE_SUCCESS)
        {
            TEMP_APP_ProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_EVS_SendEvent(TEMP_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "TEMP APP: SB Pipe Read Error, App Will Exit");

            TEMP_APP_Data.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    /*
    ** Performance Log Exit Stamp
    */
    CFE_ES_PerfLogExit(TEMP_APP_PERF_ID);

    CFE_ES_ExitApp(TEMP_APP_Data.RunStatus);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* Initialization                                                             */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 TEMP_APP_Init(void)
{
    int32 status;

    TEMP_APP_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Initialize app command execution counters
    */
    TEMP_APP_Data.CmdCounter = 0;
    TEMP_APP_Data.ErrCounter = 0;
    TEMP_APP_Data.TimeCounter = 0;

    /*
    ** Initialize app configuration data
    */
    TEMP_APP_Data.PipeDepth = TEMP_APP_PIPE_DEPTH;

    strncpy(TEMP_APP_Data.PipeName, "TEMP_APP_CMD_PIPE", sizeof(TEMP_APP_Data.PipeName));
    TEMP_APP_Data.PipeName[sizeof(TEMP_APP_Data.PipeName) - 1] = 0;

    /*
    ** Initialize event filter table...
    */
    TEMP_APP_Data.EventFilters[0].EventID = TEMP_APP_STARTUP_INF_EID;
    TEMP_APP_Data.EventFilters[0].Mask    = 0x0000;
    TEMP_APP_Data.EventFilters[1].EventID = TEMP_APP_COMMAND_ERR_EID;
    TEMP_APP_Data.EventFilters[1].Mask    = 0x0000;
    TEMP_APP_Data.EventFilters[2].EventID = TEMP_APP_COMMANDNOP_INF_EID;
    TEMP_APP_Data.EventFilters[2].Mask    = 0x0000;
    TEMP_APP_Data.EventFilters[3].EventID = TEMP_APP_COMMANDRST_INF_EID;
    TEMP_APP_Data.EventFilters[3].Mask    = 0x0000;
    TEMP_APP_Data.EventFilters[4].EventID = TEMP_APP_INVALID_MSGID_ERR_EID;
    TEMP_APP_Data.EventFilters[4].Mask    = 0x0000;
    TEMP_APP_Data.EventFilters[5].EventID = TEMP_APP_LEN_ERR_EID;
    TEMP_APP_Data.EventFilters[5].Mask    = 0x0000;
    TEMP_APP_Data.EventFilters[6].EventID = TEMP_APP_PIPE_ERR_EID;
    TEMP_APP_Data.EventFilters[6].Mask    = 0x0000;
    TEMP_APP_Data.EventFilters[7].EventID = TEMP_APP_DEV_INF_EID;
    TEMP_APP_Data.EventFilters[7].Mask    = 0x0000;

    /*
    ** Register the events
    */
    status = CFE_EVS_Register(TEMP_APP_Data.EventFilters, TEMP_APP_EVENT_COUNTS, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Temperature App: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Initialize housekeeping packet (clear user data area).
    */
    CFE_MSG_Init(CFE_MSG_PTR(TEMP_APP_Data.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(TEMP_APP_HK_TLM_MID),
                 sizeof(TEMP_APP_Data.HkTlm));

     /*
     ** Initialize output RF packet.
     */
     CFE_MSG_Init(CFE_MSG_PTR(TEMP_APP_Data.OutData.TelemetryHeader), CFE_SB_ValueToMsgId(TEMP_APP_RF_DATA_MID),
                  sizeof(TEMP_APP_Data.OutData));

    /*
    ** Create Software Bus message pipe.
    */
    status = CFE_SB_CreatePipe(&TEMP_APP_Data.CommandPipe, TEMP_APP_Data.PipeDepth, TEMP_APP_Data.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Temperature App: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /*
    ** Software Bus temperature pipe.
    */
    char   TempPipeName[16];
    uint16 TempPipeDepth;

    TempPipeDepth = TEMP_SENS_PIPE_DEPTH;
    strcpy(TempPipeName, "TEMP_SENS_PIPE");

    status = CFE_SB_CreatePipe(&TEMP_APP_Data.TempPipe, TempPipeDepth, TempPipeName);
    if (status != CFE_SUCCESS){
        CFE_EVS_SendEvent(TEMP_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "TEMP Can't create TEMP_SENS pipe status %i\n",(int)status);
        return status;
    }

    /*
    ** Subscribe to Housekeeping request commands
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(TEMP_APP_SEND_HK_MID), TEMP_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Temperature App: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return status;
    }

    /*
    ** Subscribe to RF command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(TEMP_APP_SEND_RF_MID), TEMP_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Temperature App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return status;
    }

    /*
    ** Subscribe to ground command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(TEMP_APP_CMD_MID), TEMP_APP_Data.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Temperature App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return status;
    }

    /*
    ** Subscribe to Altitude App Temperature packets
    */
    // status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(ALTITUDE_APP_TEMP_MID), RF_TLM_Data.TlmPipe);
    status = CFE_SB_SubscribeEx(CFE_SB_ValueToMsgId(ALTITUDE_APP_TEMP_MID),  /* Msg Id to Receive */
                                TEMP_APP_Data.TempPipe,                            /* Pipe Msg is to be Rcvd on */
                                CFE_SB_DEFAULT_QOS,                             /* Quality of Service */
                                10);                                            /* Max Number to Queue */
    if (status != CFE_SUCCESS){
       CFE_EVS_SendEvent(TEMP_APP_SUBSCRIBE_ERR_EID, CFE_EVS_EventType_ERROR,
         "Temperature App: Error Subscribing to Altitude App, RC = 0x%08lX\n",
         (unsigned long)status);
       return status;
    }

    /*
    ** Subscribe to IMU App Temperature packets
    */
    // status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(IMU_APP_TEMP_MID), RF_TLM_Data.TlmPipe);
    status = CFE_SB_SubscribeEx(CFE_SB_ValueToMsgId(IMU_APP_TEMP_MID),  /* Msg Id to Receive */
                                TEMP_APP_Data.TempPipe,                            /* Pipe Msg is to be Rcvd on */
                                CFE_SB_DEFAULT_QOS,                             /* Quality of Service */
                                10);                                            /* Max Number to Queue */
    if (status != CFE_SUCCESS){
       CFE_EVS_SendEvent(TEMP_APP_SUBSCRIBE_ERR_EID, CFE_EVS_EventType_ERROR,
         "Temperature App: Error Subscribing to IMU App, RC = 0x%08lX\n",
         (unsigned long)status);
       return status;
    }


    CFE_EVS_SendEvent(TEMP_APP_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMP App Initialized.%s",
                      TEMP_APP_VERSION_STRING);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the TEMP    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void TEMP_APP_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case TEMP_APP_CMD_MID:
            TEMP_APP_ProcessGroundCommand(SBBufPtr);
            break;

        case TEMP_APP_SEND_HK_MID:
            TEMP_APP_ReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        case TEMP_APP_SEND_RF_MID:
            TEMP_APP_ReportRFTelemetry((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        default:
            CFE_EVS_SendEvent(TEMP_APP_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "TEMP: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* TEMP ground commands                                                     */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void TEMP_APP_ProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    /*
    ** Process "known" TEMP app ground commands
    */
    switch (CommandCode)
    {
        case TEMP_APP_NOOP_CC:
            if (TEMP_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(TEMP_APP_NoopCmd_t)))
            {
                TEMP_APP_Noop((TEMP_APP_NoopCmd_t *)SBBufPtr);
            }

            break;

        case TEMP_APP_RESET_COUNTERS_CC:
            if (TEMP_APP_VerifyCmdLength(&SBBufPtr->Msg, sizeof(TEMP_APP_ResetCountersCmd_t)))
            {
                TEMP_APP_ResetCounters((TEMP_APP_ResetCountersCmd_t *)SBBufPtr);
            }

            break;

	// TODO: Add the commands for the temperature control...

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(TEMP_APP_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }
}

int32 TEMP_APP_ReportRFTelemetry(const CFE_MSG_CommandHeader_t *Msg)
{

    /*
    ** Get command execution counters...
    */
    TEMP_APP_Data.OutData.CommandErrorCounter = TEMP_APP_Data.ErrCounter;
    TEMP_APP_Data.OutData.CommandCounter      = TEMP_APP_Data.CmdCounter;

    TEMP_APP_Data.OutData.AppID_H = (uint8_t) ((TEMP_APP_HK_TLM_MID >> 8) & 0xff);
    TEMP_APP_Data.OutData.AppID_L = (uint8_t) (TEMP_APP_HK_TLM_MID & 0xff);

    uint8_t *aux_array1;
    aux_array1 = NULL;
    aux_array1 = malloc(4 * sizeof(uint8_t));
    aux_array1 = (uint8_t*)(&TEMP_APP_Data.TemperatureRead);

    uint8_t *aux_array2;
    aux_array2 = NULL;
    aux_array2 = malloc(4 * sizeof(uint8_t));
    aux_array2 = (uint8_t*)(&TEMP_APP_Data.HumidityRead);

    uint8_t *aux_array3;
    aux_array3 = NULL;
    aux_array3 = malloc(4 * sizeof(uint8_t));
    aux_array3 = (uint8_t*)(&TEMP_APP_Data.MPU6050Temp);

    uint8_t *aux_array4;
    aux_array4 = NULL;
    aux_array4 = malloc(4 * sizeof(uint8_t));
    aux_array4 = (uint8_t*)(&TEMP_APP_Data.MPL3115A2Temp);

    for(int i=0;i<4;i++){
      TEMP_APP_Data.OutData.byte_group_1[i] = aux_array1[i];
      TEMP_APP_Data.OutData.byte_group_2[i] = aux_array2[i];
      TEMP_APP_Data.OutData.byte_group_3[i] = aux_array3[i];
      TEMP_APP_Data.OutData.byte_group_4[i] = aux_array4[i];
      TEMP_APP_Data.OutData.byte_group_5[i] = 0;
      TEMP_APP_Data.OutData.byte_group_6[i] = 0;
    }

    /*
    ** Send housekeeping telemetry packet...
    */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(TEMP_APP_Data.OutData.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(TEMP_APP_Data.OutData.TelemetryHeader), true);

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
int32 TEMP_APP_ReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{

    /*
    ** Get command execution counters...
    */
    TEMP_APP_Data.HkTlm.Payload.CommandErrorCounter = TEMP_APP_Data.ErrCounter;
    TEMP_APP_Data.HkTlm.Payload.CommandCounter      = TEMP_APP_Data.CmdCounter;

    /* Copy the AHT10 data */
    temperature_read();
    TEMP_APP_Data.HkTlm.Payload.TemperatureRead = TEMP_APP_Data.TemperatureRead;
    TEMP_APP_Data.HkTlm.Payload.HumidityRead = TEMP_APP_Data.HumidityRead;

    /* Copy the other sensors data */
    TEMP_APP_Data.HkTlm.Payload.MPU6050Temp = TEMP_APP_Data.MPU6050Temp;
    TEMP_APP_Data.HkTlm.Payload.MPL3115A2Temp = TEMP_APP_Data.MPL3115A2Temp;

    TEMP_APP_Data.HkTlm.Payload.TimeCounter = TEMP_APP_Data.TimeCounter;

    /*
    ** Send housekeeping telemetry packet...
    */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(TEMP_APP_Data.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(TEMP_APP_Data.HkTlm.TelemetryHeader), true);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* TEMP NOOP commands                                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 TEMP_APP_Noop(const TEMP_APP_NoopCmd_t *Msg)
{
    TEMP_APP_Data.CmdCounter++;

    CFE_EVS_SendEvent(TEMP_APP_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMP: NOOP command %s",
                      TEMP_APP_VERSION);

    return CFE_SUCCESS;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function resets all the global counter variables that are     */
/*         part of the task telemetry.                                        */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 TEMP_APP_ResetCounters(const TEMP_APP_ResetCountersCmd_t *Msg)
{
    TEMP_APP_Data.CmdCounter = 0;
    TEMP_APP_Data.ErrCounter = 0;

    CFE_EVS_SendEvent(TEMP_APP_COMMANDRST_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMP: RESET command");

    return CFE_SUCCESS;
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* Verify command packet length                                               */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool TEMP_APP_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
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

        CFE_EVS_SendEvent(TEMP_APP_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), (unsigned int)FcnCode, (unsigned int)ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        TEMP_APP_Data.ErrCounter++;
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

  static const char bus_path[] = "/dev/i2c-1";
  static const char aht10_path[] = "/dev/i2c-1.aht10-0";

  // Device registration
  rv = i2c_dev_register_sensor_aht10(
    &bus_path[0],
    &aht10_path[0]
  );
  if(rv == 0)
    CFE_EVS_SendEvent(TEMP_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMP: Device registered correctly at %s",
                      aht10_path);

  fd = open(&aht10_path[0], O_RDWR);
  if(fd >= 0)
    CFE_EVS_SendEvent(TEMP_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMP: Device opened correctly at %s",
                      aht10_path);

  // Device configuration
  TEMP_APP_Data.TemperatureRead = 0;
  TEMP_APP_Data.HumidityRead = 0;
  rv = sensor_aht10_begin(fd);
  if(rv == 0)
    CFE_EVS_SendEvent(TEMP_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMP: Device AHT10 initialized");

  close(fd);

  return CFE_SUCCESS;

}

void temperature_read(void){

  // Data reading every 32 segs approx
  // Each read is every 4 segs approx
  if(TEMP_APP_Data.TimeCounter == 7){
    // Alternatively the function sensor_aht10_get_temp() can be used
    // instead of aht10_get_data(). sensor_aht10_get_temp() is a function from
    // the sensor public API, and it only returns the temperature value.
    // aht10_get_data() gets both temperature and humidity values from the
    // sensor.
    // Both functions CAN NOT be used at the same time, this will heat the sensor.
    if(aht10_get_data() != 0){
      CFE_EVS_SendEvent(TEMP_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMP: Error reading AHT10 temperature");
    }
  }

  if(TEMP_APP_Data.TimeCounter < 7){
    TEMP_APP_Data.TimeCounter++;
  }else{
    TEMP_APP_Data.TimeCounter = 0;
  }

}

int aht10_get_data(void){
  int fd;

  //Data reading
  static const char aht10_path[] = "/dev/i2c-1.aht10-0";
  fd = open(&aht10_path[0], O_RDWR);
  if(fd < 0){
    CFE_EVS_SendEvent(TEMP_APP_DEV_INF_EID, CFE_EVS_EventType_INFORMATION, "TEMP: Could not open device");
    return 1;
  }

  sensor_aht10_read(fd);
  close(fd);

  TEMP_APP_Data.TemperatureRead = sensor_aht10_get_temp();
  TEMP_APP_Data.HumidityRead = sensor_aht10_get_humid();

  return 0;
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* Function to get the temperature sent by other apps to the TempPipe         */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void get_sensors_temp(void){
  int32            CFE_SB_status = CFE_SUCCESS;
  CFE_SB_Buffer_t* TempMsgPtr = NULL;
  CFE_SB_MsgId_t   TempMsgId;


  SUBS_APP_TempData_t* dataPtr = NULL;

  do{
    CFE_SB_status = CFE_SB_ReceiveBuffer(&TempMsgPtr, TEMP_APP_Data.TempPipe, CFE_SB_POLL);
    dataPtr = NULL;

    if (CFE_SB_status == CFE_SUCCESS){
      CFE_MSG_GetMsgId(&TempMsgPtr->Msg, &TempMsgId);

      /* Update the private data */
      dataPtr = (SUBS_APP_TempData_t *)TempMsgPtr;
      TEMP_APP_Data.sensor_temp = dataPtr->temperature;

      switch (CFE_SB_MsgIdToValue(TempMsgId)){

        case ALTITUDE_APP_TEMP_MID:
        TEMP_APP_Data.MPL3115A2Temp = TEMP_APP_Data.sensor_temp;
        break;

        case IMU_APP_TEMP_MID:
        TEMP_APP_Data.MPU6050Temp = TEMP_APP_Data.sensor_temp;
        break;

        default:
        CFE_EVS_SendEvent(TEMP_APP_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
          "TEMP - Recvd invalid TEMP msgId (0x%08X)", CFE_SB_MsgIdToValue(TempMsgId));
          break;
        }
      }else if(CFE_SB_status == CFE_SB_NO_MESSAGE){
        // The pipe is empty
        break;
      }else if(CFE_SB_status == CFE_SB_TIME_OUT){
        CFE_EVS_SendEvent(TEMP_APP_PIPE_ERR_EID, CFE_EVS_EventType_INFORMATION,
          "TEMP: Message fails to arrive within the specified timeout period\n");
          break;
        }
        // OS_TaskDelay(500); /*2 Hz*/
      }while(CFE_SB_status == CFE_SUCCESS);
    }
