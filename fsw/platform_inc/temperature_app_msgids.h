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
 * Define Temperature App Message IDs
 *
 * \note The Temperature App assumes default configuration which uses V1 of message id implementation
 */

#ifndef TEMPERATURE_APP_MSGIDS_H
#define TEMPERATURE_APP_MSGIDS_H

/* V1 Command Message IDs must be 0x18xx */
#define TEMPERATURE_APP_CMD_MID     0x18E2
#define TEMPERATURE_APP_SEND_HK_MID 0x18E3
#define TEMPERATURE_APP_SEND_RF_MID 0x18F2
#define TEMPERATURE_APP_RF_DATA_MID 0x18F3
/* V1 Telemetry Message IDs must be 0x08xx */
#define TEMPERATURE_APP_HK_TLM_MID 0x08E1



#endif /* TEMPERATURE_APP_MSGIDS_H */
