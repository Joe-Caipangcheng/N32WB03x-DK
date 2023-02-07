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
 * @file app_batt.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef APP_BATT_H_
#define APP_BATT_H_

/**
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief Battery Application Module entry point
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_BATT)

#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition

/* Public typedef -----------------------------------------------------------*/

/// Battery Application Module Environment Structure
struct app_batt_env_tag
{
    /// Connection handle
    uint8_t conidx;
    /// Current Battery Level
    uint8_t batt_lvl;
};

/* Public variables ---------------------------------------------------------*/

/// Battery Application environment
extern struct app_batt_env_tag app_batt_env;

/// Table of message handlers
extern const struct app_subtask_handlers app_batt_handlers;

/* Public function prototypes -----------------------------------------------*/

/**
 *
 * Health Thermometer Application Functions
 *
 **/

/**
 * @brief Initialize Battery Application Module
 **/
void app_batt_init(void);

/**
 * @brief Add a Battery Service instance in the DB
 **/
void app_batt_add_bas(void);

/**
 * @brief Enable the Battery Service
 **/
void app_batt_enable_prf(uint8_t conidx);

/**
 * @brief Send a Battery level value
 **/
void app_batt_send_lvl(uint8_t batt_lvl);

#endif //(BLE_APP_BATT)

/// @} APP

#endif // APP_BATT_H_
