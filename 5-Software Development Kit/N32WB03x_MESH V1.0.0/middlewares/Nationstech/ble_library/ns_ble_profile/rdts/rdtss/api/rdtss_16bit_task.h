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
 * @file rdtss_16bit_task.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#ifndef __RDTSS_16BIT_TASK_PRF_H
#define __RDTSS_16BIT_TASK_PRF_H

 /* Includes ------------------------------------------------------------------*/
#if (BLE_RDTSS_16BIT_SERVER)

#include <stdint.h>
#include "ke_task.h"
#include "prf_types.h"
#include "compiler.h"        // compiler definition
#include "att.h"
#include "attm.h"

#include "rwip_task.h"
/* Public typedef -----------------------------------------------------------*/

/// Possible states of the rdtss_16bit task
enum rdtss_16bit_state
{
    /// Idle state
    RDTSS_16BIT_IDLE,
    /// Busy state
    RDTSS_16BIT_BUSY,
    /// Number of defined states.
    RDTSS_16BIT_STATE_MAX,
};

/// Messages for RDTSS_16BIT
enum
{
    /// Add a rdtss_16bit instance into the database
    RDTSS_16BIT_CREATE_DB_REQ = TASK_FIRST_MSG(TASK_ID_RDTSS_16BIT),
    /// Inform APP of database creation status
    RDTSS_16BIT_CREATE_DB_CFM,

    /// Start the Custom Service Task - at connection
    RDTSS_16BIT_ENABLE_REQ,
    /// Set/update characteristic value
    RDTSS_16BIT_VAL_SET_REQ,
    /// Peer device request to get a non-database value (RI enabled)
    RDTSS_16BIT_VALUE_REQ_IND,
    /// Response to non-database value request
    RDTSS_16BIT_VALUE_REQ_RSP,
    /// Set/update characteristic value and trigger a notification
    RDTSS_16BIT_VAL_NTF_REQ,
    /// Response after receiving a RDTSS_16BIT_VAL_NTF_REQ message and a notification is triggered
    RDTSS_16BIT_VAL_NTF_CFM,
    /// Set/update characteristic value and trigger an indication
    RDTSS_16BIT_VAL_IND_REQ,
    ///Response after receiving a RDTSS_16BIT_VAL_IND_REQ message and an indication is triggered
    RDTSS_16BIT_VAL_IND_CFM,
    /// Indicate that the characteristic value has been written
    RDTSS_16BIT_VAL_WRITE_IND,
    /// Inform the application that the profile service role task has been disabled after a disconnection
    RDTSS_16BIT_DISABLE_IND,
    /// Profile error report
    RDTSS_16BIT_ERROR_IND,
    /// Inform the application that there is an attribute info request that shall be processed
    RDTSS_16BIT_ATT_INFO_REQ,
    /// Inform back that the attribute info request has been processed
    RDTSS_16BIT_ATT_INFO_RSP,
};



/// Parameters for the database creation
struct rdtss_16bit_db_cfg
{
    ///max number of casts1 service characteristics
    uint8_t max_nb_att;
    uint8_t *att_tbl;
    uint8_t *cfg_flag;

    ///Database configuration
    uint16_t features;
};

/// Parameters of the @ref RDTSS_16BIT_CREATE_DB_CFM message
struct rdtss_16bit_create_db_cfm
{
    ///Status
    uint8_t status;
};

/// Parameters of the @ref RDTSS_16BIT_ENABLE_REQ message
struct rdtss_16bit_enable_req
{
    /// Connection index
    uint8_t conidx;
    /// security level: b0= nothing, b1=unauthenticated, b2=authenticated, b3=authorized; b1 or b2 and b3 can go together
    uint8_t sec_lvl;
    /// Type of connection
    uint8_t con_type;
};

/// Parameters of the @ref RDTSS_16BIT_DISABLE_IND message
struct rdtss_16bit_disable_ind
{
    /// Connection index
    uint8_t conidx;
};

/// Parameters of the @ref RDTSS_16BIT_VAL_WRITE_IND massage
struct rdtss_16bit_val_write_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Handle of the attribute that has to be written
    uint16_t handle;
    /// Data length to be written
    uint16_t length;
    /// Data to be written in attribute database
    uint8_t  value[__ARRAY_EMPTY];
};

/// Parameters of the @ref RDTSS_16BIT_VAL_NTF_CFM massage
struct rdtss_16bit_val_ntf_cfm
{
    /// Connection index
//    uint8_t  conidx;
    /// Handle of the attribute that has been updated
    uint16_t handle;
    /// Confirmation status
    uint8_t status;
};

/// Parameters of the @ref RDTSS_16BIT_VAL_IND_CFM massage
struct rdtss_16bit_val_ind_cfm
{
    /// Connection index
    uint8_t  conidx;
    /// Handle of the attribute that has been updated
    uint16_t handle;
    /// Confirmation status
    uint8_t status;
};

/// Parameters of the @ref RDTSS_16BIT_VAL_SET_REQ massage
struct rdtss_16bit_val_set_req
{
    /// Connection index
    uint8_t  conidx;
    /// Handle of the attribute that has to be written
    uint16_t handle;
    /// Data length to be written
    uint16_t length;
    /// Data to be written in attribute database
    uint8_t  value[__ARRAY_EMPTY];
};

/// Parameters of the @ref RDTSS_16BIT_VAL_REQ_IND message
struct rdtss_16bit_value_req_ind
{
    /// Connection index
    uint8_t  conidx;
    /// Index of the attribute for which value has been requested
    uint16_t att_idx;
};

/// Parameters of the @ref RDTSS_16BIT_VAL_REQ_RSP message
struct rdtss_16bit_value_req_rsp
{
    /// Connection index
    uint8_t  conidx;
    /// Index of the attribute for which value has been requested
    uint16_t att_idx;
    /// Current length of that attribute
    uint16_t length;
    /// ATT error code
    uint8_t  status;
    /// Data value
    uint8_t  value[__ARRAY_EMPTY];
};

/// Parameters of the @ref RDTSS_16BIT_VAL_NTF_REQ massage
struct rdtss_16bit_val_ntf_ind_req
{
    /// Connection index
    uint8_t  conidx;
    /// Notificatin/indication
    bool     notification;
    /// Handle of the attribute that has to be written
    uint16_t handle;
    /// Data length to be written
    uint16_t length;
    /// Data to be written in attribute database
    uint8_t  value[__ARRAY_EMPTY];
};

/// Parameters of the @ref RDTSS_16BIT_VAL_IND_REQ massage
struct rdtss_16bit_val_ind_req
{
    /// Connection index
    uint8_t  conidx;
    /// Handle of the attribute that has to be written
    uint16_t handle;
    /// Data length to be written
    uint16_t length;
    /// Data to be written in attribute database
    uint8_t  value[__ARRAY_EMPTY];
};

/// Parameters of the @ref RDTSS_16BIT_ATT_INFO_REQ message
struct rdtss_16bit_att_info_req
{
    /// Connection index
    uint8_t  conidx;
    /// Index of the attribute for which info has been requested
    uint16_t att_idx;
};

/// Parameters of the @ref RDTSS_16BIT_ATT_INFO_RSP message
struct rdtss_16bit_att_info_rsp
{
    /// Connection index
    uint8_t  conidx;
    /// Index of the attribute for which info has been requested
    uint16_t att_idx;
    /// Current length of that attribute
    uint16_t length;
    /// ATT error code
    uint8_t  status;
};

/** 
 * @brief Initialize Client Characteristic Configuration fields.
 * @details Function initializes all CCC fields to default value.
 * @param[in] att_db         Id of the message received.
 * @param[in] max_nb_att     Pointer to the parameters of the message. 
 */
void rdtss_16bit_init_ccc_values(const struct attm_desc *att_db, int max_nb_att);

/** 
 * @brief Set per connection CCC value for attribute
 * @details Function sets CCC for specified connection.
 * @param[in] conidx         Connection index.
 * @param[in] att_idx        Attribute index.
 * @param[in] ccc            Value of ccc. 
 */
void rdtss_16bit_set_ccc_value(uint8_t conidx, uint8_t att_idx, uint16_t ccc);


#endif // BLE_RDTSS_16BIT_SERVER
#endif // __RDTSS_16BIT_TASK_PRF_H
