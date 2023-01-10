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
 * Define TEMP App Events IDs
 */

#ifndef TEMP_APP_EVENTS_H
#define TEMP_APP_EVENTS_H

#define TEMP_APP_RESERVED_EID          0
#define TEMP_APP_STARTUP_INF_EID       1
#define TEMP_APP_COMMAND_ERR_EID       2
#define TEMP_APP_COMMANDNOP_INF_EID    3
#define TEMP_APP_COMMANDRST_INF_EID    4
#define TEMP_APP_INVALID_MSGID_ERR_EID 5
#define TEMP_APP_LEN_ERR_EID           6
#define TEMP_APP_PIPE_ERR_EID          7

// Events IDs related to the MPU6050
#define TEMP_APP_DEV_INF_EID           11

//TODO: Here you add the new commands events IDs

#define TEMP_APP_EVENT_COUNTS          8

#endif /* TEMP_APP_EVENTS_H */
