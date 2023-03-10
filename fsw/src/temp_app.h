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
 * @file
 *
 * Main header file for the TEMP application
 */

#ifndef TEMP_APP_H
#define TEMP_APP_H

/*
** Required header files.
*/
#include "cfe.h"
#include "cfe_error.h"
#include "cfe_evs.h"
#include "cfe_sb.h"
#include "cfe_es.h"

#include "temp_app_perfids.h"
#include "temp_app_msgids.h"
#include "temp_app_msg.h"

#include "altitude_app_msgids.h"
#include "altitude_app_msg.h"

#include "imu_app_msgids.h"
#include "imu_app_msg.h"

/***********************************************************************/
#define TEMP_APP_PIPE_DEPTH 32 /* Depth of the Command Pipe for Application */

#define PIN_HIGH		1
#define PIN_LOW			0

/**
 * Depth of pipe for temperature from other sensors
 */
#define TEMP_SENS_PIPE_DEPTH OS_QUEUE_MAX_DEPTH
/************************************************************************
** Type Definitions
*************************************************************************/

/*
** Global Data
*/
typedef struct
{
    /*
    ** Command interface counters...
    */
    uint8 CmdCounter;
    uint8 ErrCounter;

    int TimeCounter;

    /*
    ** AHT10 data...
    */
    float TemperatureRead;
    float HumidityRead;
    int RegisterPtr;
    uint8 DataVal;

    /*
    ** Other sensors data...
    */
    float MPU6050Temp;
    float MPL3115A2Temp;
    float sensor_temp;

    /*
    ** Housekeeping telemetry packet...
    */
    TEMP_APP_HkTlm_t HkTlm;
    TEMP_APP_OutData_t OutData;

    /*
    ** Run Status variable used in the main processing loop
    */
    uint32 RunStatus;

    /*
    ** Operational data (not reported in housekeeping)...
    */
    CFE_SB_PipeId_t CommandPipe;
    CFE_SB_PipeId_t TempPipe;

    /*
    ** Initialization data (not reported in housekeeping)...
    */
    char   PipeName[CFE_MISSION_MAX_API_LEN];
    uint16 PipeDepth;

    CFE_EVS_BinFilter_t EventFilters[TEMP_APP_EVENT_COUNTS];

} TEMP_APP_Data_t;

/****************************************************************************/
/*
** Local function prototypes.
**
** Note: Except for the entry point (TEMP_APP_Main), these
**       functions are not called from any other source module.
*/
void  TEMP_APP_Main(void);
int32 TEMP_APP_Init(void);
void  TEMP_APP_ProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr);
void  TEMP_APP_ProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr);
int32 TEMP_APP_ReadSensor(const CFE_MSG_CommandHeader_t *Msg);
int32 TEMP_APP_ReportRFTelemetry(const CFE_MSG_CommandHeader_t *Msg);
int32 TEMP_APP_ReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg);
int32 TEMP_APP_ResetCounters(const TEMP_APP_ResetCountersCmd_t *Msg);
int32 TEMP_APP_Noop(const TEMP_APP_NoopCmd_t *Msg);

bool TEMP_APP_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);

int32 aht10_init(void);
int aht10_get_data(void);

void get_sensors_temp(void);

#endif /* TEMP_APP_H */
