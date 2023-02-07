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
 * @file blps_task.h
 * @author Nations Firmware Team
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */



#ifndef _BLPS_TASK_H_
#define _BLPS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup BLPSTASK Task
 * @ingroup BLPS
 * @brief Blood Pressure Profile Task.
 *
 * The BLPSTASK is responsible for handling the messages coming in and out of the
 * @ref BLPS collector block of the BLE Host.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include "rwip_task.h" // Task definitions
#include "blp_common.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

enum
{
    /// measurement sent by profile
    BLPS_BP_MEAS_SEND,
    /// peer device confirm reception
    BLPS_CENTRAL_IND_CFM,
};

/// Messages for Blood Pressure Profile Sensor
/*@TRACE*/
enum blps_msg_id
{
    /// Start the Blood Pressure Profile Sensor - at connection
    BLPS_ENABLE_REQ = TASK_FIRST_MSG(TASK_ID_BLPS),
    /// Start the Blood Pressure Profile Sensor - at connection
    BLPS_ENABLE_RSP,

    /// Send blood pressure measurement value from APP
    BLPS_MEAS_SEND_CMD,

    /// Inform APP of new configuration value
    BLPS_CFG_INDNTF_IND,

    /// Inform APP of new RACP value
    BLPS_RACP_WRITE_IND,
    
    /// Send record access control point response value from APP
    BLPS_RACP_RESP_SEND_CMD,
    
    /// Complete Event Information
    BLPS_CMP_EVT,
};

/// Operation codes
enum blps_op_codes
{
    /// Database Creation Procedure
    BLPS_RESERVED_OP_CODE = 0,

    /// Indicate Measurement Operation Code
    BLPS_MEAS_SEND_CMD_OP_CODE = 1,
    
    /// Indicate Record Access Control Point Code
    BLPS_RACP_RSP_SEND_CMD_OP_CODE = 2,
};

/// Parameters of the @ref BLPS_ENABLE_REQ message
struct blps_enable_req
{
    ///Connection index
    uint8_t conidx;

    /// Blood Pressure indication configuration
    uint16_t bp_meas_ind_en;
    /// Intermediate Cuff Pressure Notification configuration
    uint16_t interm_cp_ntf_en;
    /// Record Access Control Point configuration
    uint16_t racp_ind_en;
};

/// Parameters of the @ref BLPS_ENABLE_RSP message
struct blps_enable_rsp
{
    ///Connection index
    uint8_t conidx;
    ///Status
    uint8_t status;
};

///Parameters of the @ref BLPS_CFG_INDNTF_IND message
struct blps_cfg_indntf_ind
{
    ///Connection index
    uint8_t conidx;
    ///Own code for differentiating between Blood Pressure Measurement, and Intermediate
    /// Cuff Pressure Measurement characteristics
    uint8_t char_code;
    ///Stop/notify/indicate value to configure into the peer characteristic
    uint16_t cfg_val;
};

/////Parameters of the @ref BLPS_MEAS_SEND_CMD message
struct blps_meas_send_cmd
{
    ///Connection index
    uint8_t conidx;
    /// Flag indicating if it is a intermediary cuff pressure measurement (0) or
    /// stable blood pressure measurement (1).
    uint8_t flag_interm_cp;
    ///Blood Pressure measurement
    struct bps_bp_meas meas_val;
};

/////Parameters of the @ref BLPS_RACP_WRITE_IND message
struct blps_racp_write_ind
{
    ///Connection index
    uint8_t conidx;
    /// RACP write value
    struct bps_racp write_val;
    /// RACP write value length
    uint16_t write_val_len;
};

/////Parameters of the @ref BLPS_RACP_RESP_SEND_CMD message
struct blps_racp_resp_send_cmd
{
    ///Connection index
    uint8_t conidx;
    /// RACP response value
    struct bps_racp_rsp write_val;
    /// RACP response value length
    uint16_t write_val_len;
};

///Parameters of the @ref BLPS_CMP_EVT message
struct blps_cmp_evt
{
    /// Operation
    uint8_t operation;
    /// Operation code      see enum blps_op_codes
    uint8_t operation_code;
    /// Status
    uint8_t status;
};

///Parameters of the @ref BLPS_CREATE_DB_REQ message
struct blps_db_cfg
{
    /// Supported features
    uint16_t features;
    /// Profile Configuration
    uint8_t prfl_cfg;
};

/// @} BLPSTASK

#endif /* _BLPS_TASK_H_ */
