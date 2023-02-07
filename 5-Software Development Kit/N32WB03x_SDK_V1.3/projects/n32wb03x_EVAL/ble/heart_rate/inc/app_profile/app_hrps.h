/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file app_hrps.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#ifndef APP_HRPS_H_
#define APP_HRPS_H_

/**
 * @addtogroup APP
 *
 * @brief Heart Rate Application Module Entry point.
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"     // SW Configuration

#if (BLE_APP_HRPS)

#include <stdint.h>

/* Define ------------------------------------------------------------*/

#define APP_HRPS_FEATURES       (0x04)      //HRPS ALL FEATURES 0x07

#define HEART_RATE_SEND_DELAY   1000        

enum hrps_cfg
{
    HRPS_CFG_NTF_EN = 1,
};

/// Heart Rate Application Module Environment Structure
struct app_hrps_env_tag
{
    /// Connection handle
    uint8_t conidx;
    /// Heart Rate Value
    uint8_t hrps_value;
};

/// Heart Rate Application environment
extern struct app_hrps_env_tag app_hrps_env;

extern uint16_t heart_rate_timer_id;

/* Public variables ---------------------------------------------------------*/

/// Table of message handlers
extern const struct app_subtask_handlers app_hrps_handlers;

/* Public function prototypes -----------------------------------------------*/

/**
 * @brief Initialize Heart Rate Service Application
 **/
void app_hrps_init(void);

/**
 * @brief Add a Heart Rate Service in the DB
 **/
void app_hrps_add_hrps(void);

/**
 * @brief Enable the Heart Rate profile
 **/
void app_hrps_enable_prf(uint8_t conidx, uint8_t cfg);

/**
 * @brief Send heart rate value
 */
void app_heart_rate_send(uint16_t heart_rate);

/**
 * @brief Send heart rate value periodically
 */
void app_heart_rate_timeout_handler(void);

#endif //BLE_APP_HRPS

/// @} APP

#endif //APP_HRPS_H_

