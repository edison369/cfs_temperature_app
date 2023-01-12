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
 * Define Temperature App  Messages and info
 */

#ifndef TEMP_APP_MSG_H
#define TEMP_APP_MSG_H

/*
** Temperature App command codes
*/
#define TEMP_APP_NOOP_CC           0
#define TEMP_APP_RESET_COUNTERS_CC 1

/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct
{
    CFE_MSG_CommandHeader_t CmdHeader; /**< \brief Command header */
} TEMP_APP_NoArgsCmd_t;

/*
** The following commands all share the "NoArgs" format
**
** They are each given their own type name matching the command name, which
** allows them to change independently in the future without changing the prototype
** of the handler function
*/
typedef TEMP_APP_NoArgsCmd_t TEMP_APP_NoopCmd_t;
typedef TEMP_APP_NoArgsCmd_t TEMP_APP_ResetCountersCmd_t;

/*************************************************************************/
/*
** Type definition (Temperature App housekeeping)
*/

typedef struct
{
    uint8 CommandCounter;
    uint8 CommandErrorCounter;
    uint8 spare[2];
    float TemperatureRead;
    float HumidityRead;
    float MPU6050Temp;
    float MPL3115A2Temp;
    uint8 TimeCounter;
} TEMP_APP_HkTlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TelemetryHeader; /**< \brief Telemetry header */
    uint8_t AppID_H;
    uint8_t AppID_L;
    uint8 CommandCounter;
    uint8 CommandErrorCounter;
    uint8 spare[2];
    uint8 byte_group_1[4];    // AHT10 Temperature
    uint8 byte_group_2[4];    // AHT10 Humidity
    uint8 byte_group_3[4];    // MPU6050 Temperature
    uint8 byte_group_4[4];    // MPL3115A2 Temperature
    uint8 byte_group_5[4];    // empty
    uint8 byte_group_6[4];    // empty
} TEMP_APP_OutData_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TelemetryHeader; /**< \brief Telemetry header */
    TEMP_APP_HkTlm_Payload_t Payload;         /**< \brief Telemetry payload */
} TEMP_APP_HkTlm_t;

#endif /* TEMP_APP_MSG_H */
