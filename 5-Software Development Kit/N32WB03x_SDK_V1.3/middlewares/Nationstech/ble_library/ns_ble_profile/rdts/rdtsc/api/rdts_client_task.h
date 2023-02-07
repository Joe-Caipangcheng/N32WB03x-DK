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
 * @file rdts_client_task.h
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef RDTS_CLIENT_TASK_H_
#define RDTS_CLIENT_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup RDTSCTASK Raw Data Transfer Service client Task
 * @ingroup RDTSCTASK
 * @brief Raw Data Transfer Service client Task
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "ke_task.h"
#include "rdts_client.h"

#if (BLE_RDTS_CLIENT)
/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Possible states of the RTDS_CLIENT task
enum
{
    /// IDLE state
    RDTSC_IDLE,
    
    /// Connected state
    RDTSC_CONNECTED,
    
    /// Discovering
    RDTSC_DISCOVERING,
    
    /// Number of defined states.
    RDTSC_STATE_MAX
};

///RDTSC Host API messages
enum
{
    /// RDTSC client role enable request from application.
    RDTSC_ENABLE_REQ = TASK_FIRST_MSG(TASK_ID_RDTSC),
    
    /// RDTSC Host role enable confirmation to application.
    RDTSC_ENABLE_CFM,
 
    /// Request to transmit data
    RDTSC_DATA_TX_REQ,
    
    /// Confirm that data has been sent
    RDTSC_DATA_TX_CFM,
    
    /// Send data to app
    RDTSC_DATA_RX_IND,

    /// Indicate flow control state
    RDTSC_TX_FLOW_CTRL_IND,
};

/*
 * API Messages Structures
 ****************************************************************************************
 */

///Parameters of the @ref RDTSC_ENABLE_REQ message
struct rdtsc_enable_req
{
    /// Connection handle
    uint8_t conidx;
    
    /// Connection type
    uint8_t con_type;
    
};

///Parameters of the @ref RDTSC_ENABLE_CFM message
struct rdtsc_enable_cfm
{
   
    /// Status
    uint8_t status;
    
    /// RDTS Device details to keep in APP
    struct rdtsc_rdts_content rdts;
};

///Parameters of the @ref RDTSC_DATA_TX_REQ message
///WARNING, DO NOT ALTER THIS STRUCT, IT SHOULD BE COMPATIBLE WITH gattc_write_cmd struct
struct rdtsc_data_tx_req
{
    uint8_t operation;
    /// Perform automatic execution
    /// (if false, an ATT Prepare Write will be used this shall be use for reliable write)
    bool auto_execute;
    /// operation sequence number
    uint16_t seq_num;
    /// Attribute handle
    uint16_t handle;
    /// Write offset
    uint16_t offset;
    /// Write length
    uint16_t length;
    /// Internal write cursor shall be initialized to 0
    uint16_t cursor;
    /// Value to write
    uint8_t data[__ARRAY_EMPTY];
};
///Parameters of the @ref RDTSC_DATA_TX_CFM message
struct rdtsc_data_tx_cfm
{
    ///Status
    uint8_t status;
};

///Parameters of the @ref RDTSC_DATA_RX_IND message
///WARNING, DO NOT ALTER THIS STRUCT, IT SHOULD BE COMPATIBLE WITH gattc_event_ind struct
struct rdtsc_data_rx_ind
{
    /// Event Type
    uint8_t type;
    /// Data length
    uint16_t length;
    /// Attribute handle
    uint16_t handle;
    /// Event Value
    uint8_t data[__ARRAY_EMPTY];
};

///Parameters of the @ref RDTSC_RX_FLOW_CTRL_REQ message
struct rdtsc_rx_flow_ctrl_req
{
    // flow control state
    uint8_t flow_control_state;
};

///Parameters of the @ref RDTSC_TX_FLOW_CTRL_IND message
struct rdtsc_tx_flow_ctrl_ind
{
    // flow control state
    uint8_t flow_control_state;
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern const struct ke_state_handler rdtsc_default_handler;

/*
 * Functions
 ****************************************************************************************
 */

#endif //BLE_RDTS_CLIENT

/// @} RDTSCTASK

#endif // RDTS_CLIENT_TASK_H_
