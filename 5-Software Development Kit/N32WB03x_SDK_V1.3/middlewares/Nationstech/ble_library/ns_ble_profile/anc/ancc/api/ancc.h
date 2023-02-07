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
 * @file ancc.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#ifndef ANCC_H_
#define ANCC_H_

/**
 ****************************************************************************************
 * @addtogroup ANCC Apple Notification Center Client
 * @ingroup ANCC
 * @brief Apple Notification Center Client
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_ANC_CLIENT)

#include "ke_task.h"
#include "prf_types.h"
#include "prf_utils.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of ANCC task instances
#define ANCC_IDX_MAX                    (BLE_CONNECTION_MAX)
/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// ANCS Characteristics
enum ancc_anc_chars
{
    /// Notification Source
    ANCC_CHAR_NTF_SRC,
    /// Control Point
    ANCC_CHAR_CTRL_PT,
    /// Data Source
    ANCC_CHAR_DATA_SRC,

    ANCC_CHAR_MAX
};

/// ANCS Characteristic Descriptors
enum ancc_anc_descs
{
    /// Notification Source - Client Characteristic Configuration
    ANCC_DESC_NTF_SRC_CL_CFG,
    /// Data Source - Client Characteristic Configuration
    ANCC_DESC_DATA_SRC_CL_CFG,

    ANCC_DESC_MAX,

    ANCC_DESC_MASK = 0x10
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
///Structure containing the characteristics handles, value handles and descriptors

struct ancc_content
{
    /// Service info
    struct prf_svc svc;
    /// Characteristics
    struct prf_char_inf chars[ANCC_CHAR_MAX];
    /// Descriptors
    struct prf_char_desc_inf descs[ANCC_DESC_MAX];
};

/// Structure for ANCS Notification reception variables
struct ancc_data_src_ntf_tag
{
    /// Notification Reception state
    uint8_t state;
    /// Number of requested attributes
    uint8_t nb_req_atts;
    /// Current attribute id
    uint8_t curr_att_id;
    /// Current attribute id
    uint8_t curr_app_id_length;
    /// Current attribute id
    uint8_t curr_att_length;
    /// Remaining bytes for current value
    uint16_t curr_value_idx;
    /// Last allocated notification
    void *alloc_rsp_ind;
};

struct ancc_cnx_env
{
    /// Last requested operation code
    uint8_t last_op_code;
    /// Provide an indication about the last operation
    uint16_t last_req;
    /// Last requested UUID(to keep track of the two services and char)
    uint16_t last_uuid_req;
    /// Last characteristic code used in a read or a write request
    uint16_t last_char_code;
    /// Counter used to check service uniqueness
    uint8_t nb_svc;
    /// Data Source Notification reception variables structure
    struct ancc_data_src_ntf_tag data_src;
    /// ANC characteristics
    struct ancc_content anc;
    /// Current operation code
    void *operation;
};

/// ANCC environment variable
struct ancc_env_tag
{
    /// Profile environment
    prf_env_t prf_env;
    /// Environment variable pointer for each connections
    struct ancc_cnx_env* env[ANCC_IDX_MAX];
    /// State of different task instances
    ke_state_t state[ANCC_IDX_MAX];
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve ANC client profile interface
 * @return ANC client profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *ancc_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Send ANC ATT DB discovery results to ANCC host.
 ****************************************************************************************
 */
void ancc_enable_rsp_send(struct ancc_env_tag *ancc_env, uint8_t conidx, uint8_t status);

/**
 ****************************************************************************************
 * @brief Send Notification Configuration Response
 ****************************************************************************************
 */
void ancc_wr_cfg_ntf_rsp_send(struct ancc_env_tag *ancc_env, uint8_t conidx,
                              uint8_t op_code, uint8_t status);

/**
 ****************************************************************************************
 * @brief Send Read Notification Configuration Response
 ****************************************************************************************
 */
void ancc_rd_cfg_ntf_rsp_send(struct ancc_env_tag *ancc_env, uint8_t conidx,
                              uint8_t op_code, uint16_t value, uint8_t status);

/**
 ****************************************************************************************
 * @brief Send Operation Completion Event
 ****************************************************************************************
 */
void ancc_cmp_evt_send(struct ancc_env_tag *ancc_env, uint8_t conidx, uint8_t op_code,
                       uint8_t status);

/**
 ****************************************************************************************
 * @brief Handles reception of response to a Get Notification Attributes command.
 * @param[in] ancc_env Pointer to the profile environment.
 * @param[in] conidx Connection index
 * @param[in] value Pointer to the response value.
 * @param[in] length Length of the response value.
 ****************************************************************************************
 */
void ancc_get_ntf_atts_rsp(struct ancc_env_tag *ancc_env, uint8_t conidx,
                           const uint8_t *value, uint16_t length);

/**
 ****************************************************************************************
 * @brief Handles reception of response to a Get App Attributes command.
 * @param[in] ancc_env Pointer to the profile environment.
 * @param[in] value Pointer to the response value.
 * @param[in] length Length of the response value.
 ****************************************************************************************
 */
void ancc_get_app_atts_rsp(struct ancc_env_tag *ancc_env, uint8_t conidx,
                           const uint8_t *value, uint16_t length);

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
void ancc_task_init(struct ke_task_desc *p_task_desc);

#endif // (BLE_ANC_CLIENT)

/// @} ANCC

#endif // ANCC_H_
