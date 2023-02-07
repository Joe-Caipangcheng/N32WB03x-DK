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
 * @file app_blps.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#ifndef APP_BLPS_H_
#define APP_BLPS_H_

/**
 * @addtogroup APP
 *
 * @brief Blood Pressure Application Module Entry point.
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"     // SW Configuration

#if (BLE_APP_BLPS)

#include <stdint.h>
#include "blp_common.h"
/* Define ------------------------------------------------------------*/

#define APP_BLPS_ALL_FEATURES               0x1FF

#define APP_BLPS_SUP_FEATUEES               APP_BLPS_ALL_FEATURES

enum blps_cfg
{
    BLPS_CFG_NTFIND_STP,
    BLPS_CFG_NTFIND_NTF_EN,
    BLPS_CFG_NTFIND_IND_EN,
};

enum app_blps_prfl_cfg
{
    APP_BLPS_INTM_CUFF_PRESS_SUP = 1,
    APP_BLPS_RACP_SUP = 2,
};



/* Public variables ---------------------------------------------------------*/

/// Table of message handlers
extern const struct app_subtask_handlers app_blps_handlers;

/* Public function prototypes -----------------------------------------------*/

/**
 * @brief Initialize Blood Pressure Sensor Application
 **/
void app_blps_init(void);

/**
 * @brief Add a Blood Pressure Sensor instance in the DB
 **/
void app_blps_add_blps(void);

/**
 * @brief Enable Blood Pressure Sensor 
 */
void app_blps_enable_prf(uint8_t conidx);

/**
 * @brief  Send Blood Pressure Measurement value  
 */
void app_blps_measurement_send(uint8_t conidx, struct bps_bp_meas *data);

/**
 * @brief  Send Intermediate Cuff Pressure value 
 */
void app_blps_intm_cuff_press_send(uint8_t conidx, struct bps_bp_meas *data);

/**
 * @brief  send record access control point response value 
 */
void app_blps_racp_resp_send(uint8_t conidx, uint8_t *data, uint8_t data_len);

#endif //BLE_APP_BLPSS

/// @} APP

#endif //APP_BLPS_H_
