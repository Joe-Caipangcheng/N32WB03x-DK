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
 * @file app_hid.h
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#ifndef APP_HID_H_
#define APP_HID_H_

/**
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief HID Application Module entry point
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_HID)

#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition

#if (PS2_SUPPORT)
#include "ps2.h"             // PS2 Mouse Driver
#endif //(PS2_SUPPORT)

/* Public typedef -----------------------------------------------------------*/


/// HID Application Module Environment Structure
struct app_hid_env_tag
{
    /// Connection handle
    uint8_t conidx;
    /// Mouse timeout value
    uint16_t timeout;
    /// Internal state of the module
    uint8_t state;
    /// Timer enabled
    bool timer_enabled;
    /// Number of report that can be sent
    uint8_t nb_report;
};

/// Mouse report (data packet)
struct ps2_mouse_msg
{
    /// Buttons
    uint8_t b;
    /// X Axis Relative Movement
    uint8_t x;
    /// Y Axis Relative Movement
    uint8_t y;
    /// Wheel Relative Movement
    uint8_t w;
};


/* Public variables ---------------------------------------------------------*/

/// Table of message handlers
extern const struct app_subtask_handlers app_hid_handlers;

/* Public function prototypes -----------------------------------------------*/

/**
 *
 * Health Thermometer Application Functions
 *
 **/

/**
 * @brief Initialize HID Application Module
 **/
void app_hid_init(void);

/**
 * @brief Add a HID Service instance in the DB
 **/
void app_hid_add_hids(void);

/**
 * @brief Enable the HID Over GATT Profile device role
 *
 * @param[in]:  conhdl - Connection handle for the connection
 **/
void app_hid_enable_prf(uint8_t conidx);

/**
 * @brief Send a mouse report to the peer device
 *
 * @param[in]:  report - Mouse report sent by the PS2 driver
 **/
void app_hid_send_mouse_report(struct ps2_mouse_msg report);

int app_hid_mouse_timeout_timer_handler(ke_msg_id_t const msgid,void const *param);
                                                   
void app_hid_send_consumer_report(uint8_t* report);
void app_hid_send_voice_report(uint8_t* report, uint8_t len);
bool is_app_hid_ready(void);
#endif //(BLE_APP_HID)

/// @} APP

#endif // APP_HID_H_
