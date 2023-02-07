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
 * @file rdts_client.h
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef RDTS_CLIENT_H_
#define RDTS_CLIENT_H_

/**
 ****************************************************************************************
 * @addtogroup RDTS Raw Data Transfer Service
 * @ingroup PROFILE
 * @brief Raw Data Transfer Service
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup RDTS Raw Data Transfer Service Client
 * @ingroup RDTSC
 * @brief RDTS Profile role: Client
 * @{
 ****************************************************************************************
 */
 
#include "rwprf_config.h"

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_RDTS_CLIENT)

#include "ke_task.h"
#include "prf.h"
#include "prf_types.h"

/*
 * Defines
 ****************************************************************************************
 */
#define RDTSC_IDX_MAX 1



/*
 * ENUMERATIONS
 ****************************************************************************************
 */

#define RDTSC_CLEANUP_FNCT        (NULL)

/// RDTS Service Characteristics
enum
{
    RDTSC_SRV_TX_DATA_CHAR = 0,
    RDTSC_SRV_RX_DATA_CHAR,

    RDTSC_CHAR_MAX
};

/// RDTS Service Characteristic Descriptors
enum
{
    /// RDTS client config
    RDTSC_SRV_TX_DATA_CLI_CFG,
    
    RDTSC_DESC_MAX,
};

typedef enum flow_ctrl_states
{
    FLOW_ON     = (uint8_t)0x01,
    FLOW_OFF    = (uint8_t)0x02,
}flow_ctrl_state_t;

/*
 * STRUCTURES
 ****************************************************************************************
 */

struct rdtsc_db_cfg
{
    const struct prf_char_def_128* p_char;
    const struct prf_char_desc_def* p_desc;
    const uint8_t* p_uuid128;               

};

/// Service information
struct rdtsc_rdts_content
{
    /// Service info
    struct prf_svc svc;

    /// Characteristic info:
    struct prf_char_inf chars[RDTSC_CHAR_MAX];

    /// Characteristic cfg
    struct prf_char_desc_inf descs[RDTSC_DESC_MAX];
};

/// RDTS Host environment variable
struct rdtsc_env_tag
{
   
     /// profile environment
    prf_env_t prf_env;
    /// Service Start Handle
    uint16_t shdl;
    /// To store the DB max number of attributes
    uint8_t max_nb_att;
    /// On-going operation
    struct ke_msg *operation;

    /// Cursor on connection used to notify peer devices
    uint8_t cursor;
    /// CCC handle index, used during notification/indication busy state
    uint8_t ccc_idx;

    /// List of values set by application
    struct co_list values;
    /// RDTSC task state
    ke_state_t state[RDTSC_IDX_MAX];
    
    /// Profile Connection Info

    /// Last requested UUID(to keep track of the two services and char)
    uint16_t last_uuid_req;
    /// Last service for which something was discovered
    uint8_t last_svc_req[ATT_UUID_128_LEN];

    /// Last char. code requested to read.
    uint8_t last_char_code;

    /// counter used to check service uniqueness
    uint8_t nb_svc;

    /// RDTS Device Service information recovered from discovery
    struct rdtsc_rdts_content rdts;
    
    /// Flow control states
    bool tx_flow_en;
    bool rx_flow_en;
    bool tx_busy_flag;
    
    /// Updated mtu
    uint16_t mtu;
    uint16_t tx_size;
    uint16_t tx_wait_level;
        
    /// Pending TX
    struct ke_msg *pending_tx_msg;
    /// Write no rsp completion remaining
    uint8_t pending_wr_no_rsp_cmp;
    /// Pending notification of data tx
    bool pending_tx_ntf_cmp;
    
    uint8_t conidx;
    
    struct rdtsc_db_cfg  rdtsc_cfg;
};



/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief     Profile Initialization
 * @param[in] env enviroment 
 * @param[in] start_hdl start handle 
 * @param[in] app_task application task
 * @param[in] sec_lvl security level
 * @param[in] params configuration parameters
 * @return    void
 ****************************************************************************************
 */
uint8_t rdtsc_init(struct prf_task_env *env, uint16_t *start_hdl, uint16_t app_task, uint8_t sec_lvl, struct rdtsc_db_cfg *params);
/**
 ****************************************************************************************
 * @brief     Profile enable confirmation
 * @param[in] env_tag enviroment 
 * @param[in] conidx Connection index
 * @param[in] status
 * @return    void
 ****************************************************************************************
 */
void rdtsc_enable_cfm_send(struct rdtsc_env_tag *rdtsc_env, uint8_t conidx, uint8_t status);

/**
 ****************************************************************************************
 * @brief Confirm that data has been sent
 * @param[in] rdtsc_env enviroment 
 * @param[in] status
 * @return    void
 ****************************************************************************************
 */
void rdtsc_confirm_data_tx(struct rdtsc_env_tag *rdtsc_env, uint8_t status);

/**
 ****************************************************************************************
 * @brief  Find next empty characteristic description
 * @param[in] rdtsc_env   enviroment 
 * @param[in] desc_def  service characteristic description information table
 * @return    position of next characteristic description
 ****************************************************************************************
 */
uint8_t rdtsc_get_next_desc_char_code(struct rdtsc_env_tag *rdtsc_env,
                                     const struct prf_char_desc_def *desc_def);

/**
 ****************************************************************************************
 * @brief Retrieve RDTS profile interface
 *
 * @return RDTS service profile interface
 ****************************************************************************************
 */

const struct prf_task_cbs* rdtsc_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Receives data and forwards it to application
 * @param[in] rdtsc_env enviroment 
 * @param[in] message received
 * @param[in] length
 * @return    void
 ****************************************************************************************
 */
void rdtsc_data_receive(struct rdtsc_env_tag *rdtsc_env, void *msg );


/**
 ****************************************************************************************
 * @brief Receives data and forwards it to application
 * @param[in] msg voi pointer to RDTS_CLIENT_DATA_TX_REQ message
 * @param[in] prf_env rdtsc profile enviroment 
 * @param[in] conidx connection index
 * @param[in] handle RDTSC_SRV_RX_DATA_CHAR handle
 * @param[in] value data pointer to be written
 * @param[in] operation GATTC_WRITE_NO_RESPONSE
 * @return    void
 ****************************************************************************************
 */
 
void rdtsc_write_data_tx(void *msg, prf_env_t *prf_env, uint8_t conidx,
                                                         uint16_t handle, uint8_t* value, uint16_t length, uint8_t operation);

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * Initialize task handler
 *
 * @param task_desc Task descriptor to fill
 ****************************************************************************************
 */
void rdtsc_task_init(struct ke_task_desc *p_task_desc);

#endif //BLE_RDTS_CLIENT

/// @} RDTSC

#endif // RDTS_CLIENT_H_
