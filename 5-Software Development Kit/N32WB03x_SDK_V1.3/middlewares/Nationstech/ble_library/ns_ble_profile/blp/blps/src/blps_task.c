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
 * @file blps_task.c
 * @author Nations Firmware Team
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */



/**
 ****************************************************************************************
 * @addtogroup BLPSTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_BP_SENSOR)
#include "co_utils.h"

#include "gap.h"
#include "gattc_task.h"
#include "blps.h"
#include "blps_task.h"
#include "prf_utils.h"



/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref BLPS_ENABLE_REQ message.
 * The handler enables the Blood Pressure Sensor Profile and initialize readable values.
 * @param[in] msgid Id of the message received (probably unused).off
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int blps_enable_req_handler(ke_msg_id_t const msgid,
                                   struct blps_enable_req const *p_param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct blps_env_tag *p_blps_env = PRF_ENV_GET(BLPS, blps);
    struct blps_enable_rsp *p_cmp_evt;

    // Status
    uint8_t status = PRF_ERR_REQ_DISALLOWED;

    if (gapc_get_conhdl(p_param->conidx) == GAP_INVALID_CONHDL)
    {
        status = PRF_ERR_DISCONNECTED;
    }
    else if (ke_state_get(dest_id) == BLPS_IDLE)
    {
        if (p_param->interm_cp_ntf_en == PRF_CLI_START_NTF)
        {
            // Enable Bonded Data
            SETB(p_blps_env->prfl_ntf_ind_cfg[p_param->conidx], BLPS_INTM_CUFF_PRESS_NTF_CFG, 1);
        }

        if (p_param->bp_meas_ind_en == PRF_CLI_START_IND)
        {
            // Enable Bonded Data
            SETB(p_blps_env->prfl_ntf_ind_cfg[p_param->conidx], BLPS_BP_MEAS_IND_CFG, 1);
        }
        
        if (p_param->racp_ind_en == PRF_CLI_START_IND)
        {
            // Enable Bonded Data
            SETB(p_blps_env->prfl_ntf_ind_cfg[p_param->conidx], BLPS_RACP_IND_CFG, 1);
        }

        status = GAP_ERR_NO_ERROR;
    }

    // send completed information to APP task that contains error status
    p_cmp_evt = KE_MSG_ALLOC(BLPS_ENABLE_RSP, src_id, dest_id, blps_enable_rsp);
    p_cmp_evt->status = status;
    p_cmp_evt->conidx = p_param->conidx;
    ke_msg_send(p_cmp_evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the read request from peer device
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_read_req_ind_handler(ke_msg_id_t const msgid,
                                      struct gattc_read_req_ind const *p_param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    if (ke_state_get(dest_id) == BLPS_IDLE)
    {
        // Get the address of the environment
        struct blps_env_tag *p_blps_env = PRF_ENV_GET(BLPS, blps);
        uint8_t conidx = KE_IDX_GET(src_id);

        uint16_t value = 0;
        uint8_t status = ATT_ERR_NO_ERROR;
        struct gattc_read_cfm *p_cfm;

        switch (p_param->handle - p_blps_env->shdl)
        {
            case BPS_IDX_BP_FEATURE_VAL:
            {
                value = p_blps_env->features;
            } break;

            case BPS_IDX_BP_MEAS_IND_CFG:
            {
                value = GETB(p_blps_env->prfl_ntf_ind_cfg[conidx], BLPS_BP_MEAS_IND_CFG)
                        ? PRF_CLI_START_IND : PRF_CLI_STOP_NTFIND;
            } break;

            case BPS_IDX_INTM_CUFF_PRESS_NTF_CFG:
            {
                // Characteristic is profile specific
                if (GETB(p_blps_env->prfl_cfg, BLPS_INTM_CUFF_PRESS_SUP))
                {
                    // Fill data
                    value = GETB(p_blps_env->prfl_ntf_ind_cfg[conidx], BLPS_INTM_CUFF_PRESS_NTF_CFG)
                            ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;
                }
                else
                {
                    status = ATT_ERR_ATTRIBUTE_NOT_FOUND;
                }
            } break;

            default:
            {
                status = ATT_ERR_REQUEST_NOT_SUPPORTED;
            } break;
        }

        // Send data to peer device
        p_cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, sizeof(uint16_t));
        p_cfm->length = sizeof(uint16_t);
        memcpy(p_cfm->value, &value, sizeof(uint16_t));
        p_cfm->handle = p_param->handle;
        p_cfm->status = status;

        // Send value to peer device.
        ke_msg_send(p_cfm);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref BLPS_MEAS_SEND_CMD message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int blps_meas_send_cmd_handler(ke_msg_id_t const msgid,
                                      struct blps_meas_send_cmd const *p_param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Message status
    int msg_status = KE_MSG_CONSUMED;
    // Status
    uint8_t status = GAP_ERR_NO_ERROR;
    // Get the address of the environment
    struct blps_env_tag *p_blps_env = PRF_ENV_GET(BLPS, blps);

    if (gapc_get_conhdl(p_param->conidx) == GAP_INVALID_CONHDL)
    {
        status = PRF_ERR_DISCONNECTED;
    }
    else if (ke_state_get(dest_id) == BLPS_IDLE)
    {

        // Intermediary blood pressure, must be notified if enabled
        if (p_param->flag_interm_cp)
        {
            // Check if supported
            if (GETB(p_blps_env->prfl_cfg, BLPS_INTM_CUFF_PRESS_SUP))
            {
                if (!GETB(p_blps_env->prfl_ntf_ind_cfg[p_param->conidx], BLPS_INTM_CUFF_PRESS_NTF_CFG))
                {
                    status = PRF_ERR_NTF_DISABLED;
                }
            }
            else
            {
                status = PRF_ERR_FEATURE_NOT_SUPPORTED;
            }
        }

        // Stable Blood Pressure Measurement, must be indicated if enabled
        else if (!GETB(p_blps_env->prfl_ntf_ind_cfg[p_param->conidx], BLPS_BP_MEAS_IND_CFG))
        {
            status = PRF_ERR_IND_DISABLED;
        }

        // Check if message can be sent
        if (status == GAP_ERR_NO_ERROR)
        {
            // Allocate the GATT notification message
            struct gattc_send_evt_cmd *p_meas_val = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                    KE_BUILD_ID(TASK_GATTC, p_param->conidx), dest_id,
                    gattc_send_evt_cmd, BLPS_BP_MEAS_MAX_LEN);

            // Fill event type and handle which trigger event
            if (p_param->flag_interm_cp)
            {
                p_meas_val->operation = GATTC_NOTIFY;
                p_meas_val->handle = p_blps_env->shdl + BPS_IDX_INTM_CUFF_PRESS_VAL;
            }
            else
            {
                // Fill in the p_parameter structure
                p_meas_val->operation = GATTC_INDICATE;
                p_meas_val->handle = p_blps_env->shdl + BPS_IDX_BP_MEAS_VAL;
            }
            
            p_meas_val->seq_num = BLPS_MEAS_SEND_CMD_OP_CODE;
            #if 1
            // Pack the BP Measurement value
            p_meas_val->length = blps_pack_meas_value(p_meas_val->value, &p_param->meas_val);
            #else
            //copy raw data if already pack
            p_meas_val->length = BLPS_BP_MEAS_MAX_LEN;
            memcpy(p_meas_val->value,(const uint8_t*)&p_param->meas_val,p_meas_val->length);
            #endif
            // Send the event
            ke_msg_send(p_meas_val);
            // Go to busy state
            ke_state_set(dest_id, BLPS_BUSY);
        }
    }
    else
    {
        // Save it for later
        msg_status = KE_MSG_SAVED;
    }

    if (status != GAP_ERR_NO_ERROR)
    {
        // Send error message to application
        blps_send_cmp_evt(p_blps_env, p_param->conidx, GATTC_INDICATE, BLPS_MEAS_SEND_CMD_OP_CODE, status);
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref BLPS_RACP_RESP_SEND_CMD message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int blps_racp_resp_send_cmd_handler(ke_msg_id_t const msgid,
                                      struct blps_racp_resp_send_cmd const *p_param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Message status
    int msg_status = KE_MSG_CONSUMED;
    // Status
    uint8_t status = GAP_ERR_NO_ERROR;
    // Get the address of the environment
    struct blps_env_tag *p_blps_env = PRF_ENV_GET(BLPS, blps);

    if (gapc_get_conhdl(p_param->conidx) == GAP_INVALID_CONHDL)
    {
        status = PRF_ERR_DISCONNECTED;
    }
    else if (ke_state_get(dest_id) == BLPS_IDLE)
    {
        // Send Record Access Control Point Value, must be indicated if enabled
        if (!GETB(p_blps_env->prfl_ntf_ind_cfg[p_param->conidx], BLPS_RACP_IND_CFG))
        {
            status = PRF_ERR_IND_DISABLED;
        }

        // Check if message can be sent
        if (status == GAP_ERR_NO_ERROR)
        {
            // Allocate the GATT notification message
            struct gattc_send_evt_cmd *p_racp_val = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                    KE_BUILD_ID(TASK_GATTC, p_param->conidx), dest_id,
                    gattc_send_evt_cmd, BLPS_RACP_MAX_LEN);

            // Fill in the p_parameter structure
            p_racp_val->operation = GATTC_INDICATE;
            p_racp_val->handle = p_blps_env->shdl + BPS_IDX_RACP_VAL;

            p_racp_val->seq_num = BLPS_RACP_RSP_SEND_CMD_OP_CODE;
            
            if(p_param->write_val_len <= BLPS_RACP_MAX_LEN)
            {
                p_racp_val->length = p_param->write_val_len;
            }
            else
            {
                p_racp_val->length = BLPS_RACP_MAX_LEN;
            }
            
            memcpy(p_racp_val->value, &(p_param->write_val), p_racp_val->length);
            
            // Send the event
            ke_msg_send(p_racp_val);
            // Go to busy state
            ke_state_set(dest_id, BLPS_BUSY);
        }
    }
    else
    {
        // Save it for later
        msg_status = KE_MSG_SAVED;
    }

    if (status != GAP_ERR_NO_ERROR)
    {
        // Send error message to application
        blps_send_cmp_evt(p_blps_env, p_param->conidx, GATTC_INDICATE, BLPS_RACP_RSP_SEND_CMD_OP_CODE, status);
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GL2C_CODE_ATT_WR_CMD_IND message.
 * The handler compares the new values with current ones and notifies them if they changed.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_write_req_ind_handler(ke_msg_id_t const msgid,
                                      struct gattc_write_req_ind const *p_param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct blps_env_tag *p_blps_env = PRF_ENV_GET(BLPS, blps);
    struct gattc_write_cfm *p_cfm;
    uint8_t conidx = KE_IDX_GET(src_id);

    uint16_t value = 0x0000;
    uint8_t status = GAP_ERR_NO_ERROR;
    uint8_t char_code = 0;

    //Extract value before check, for CCC attribute
    memcpy(&value, &(p_param->value), sizeof(uint16_t));

    //BP Measurement Char. - Client Char. Configuration
    if (p_param->handle == (p_blps_env->shdl + BPS_IDX_BP_MEAS_IND_CFG))
    {
        if ((value == PRF_CLI_STOP_NTFIND) || (value == PRF_CLI_START_IND))
        {
            char_code = BPS_BP_MEAS_CHAR;

            if (value == PRF_CLI_STOP_NTFIND)
            {
                SETB(p_blps_env->prfl_ntf_ind_cfg[conidx], BLPS_BP_MEAS_IND_CFG, 0);
            }
            else //PRF_CLI_START_IND
            {
                SETB(p_blps_env->prfl_ntf_ind_cfg[conidx], BLPS_BP_MEAS_IND_CFG, 1);
            }
        }
        else
        {
            status = PRF_APP_ERROR;
        }
    }
    else if (p_param->handle == (p_blps_env->shdl + BPS_IDX_INTM_CUFF_PRESS_NTF_CFG))
    {
        if ((value == PRF_CLI_STOP_NTFIND) || (value == PRF_CLI_START_NTF))
        {
            char_code = BPS_INTM_CUFF_MEAS_CHAR;

            if (value == PRF_CLI_STOP_NTFIND)
            {
                SETB(p_blps_env->prfl_ntf_ind_cfg[conidx], BLPS_INTM_CUFF_PRESS_NTF_CFG, 0);
            }
            else //PRF_CLI_START_NTF
            {
                SETB(p_blps_env->prfl_ntf_ind_cfg[conidx], BLPS_INTM_CUFF_PRESS_NTF_CFG, 1);
            }
        }
        else
        {
            status = PRF_APP_ERROR;
        }
    }
    else if (p_param->handle == (p_blps_env->shdl + BPS_IDX_RACP_IND_CFG))
    {
        if ((value == PRF_CLI_STOP_NTFIND) || (value == PRF_CLI_START_IND))
        {
            char_code = BPS_RACP_CHAR;

            if (value == PRF_CLI_STOP_NTFIND)
            {
                SETB(p_blps_env->prfl_ntf_ind_cfg[conidx], BLPS_RACP_IND_CFG, 0);
            }
            else //PRF_CLI_START_IND
            {
                SETB(p_blps_env->prfl_ntf_ind_cfg[conidx], BLPS_RACP_IND_CFG, 1);
            }
        }
        else
        {
            status = PRF_APP_ERROR;
        }
    }

    if (status == GAP_ERR_NO_ERROR)
    {
        if(     (p_param->handle == (p_blps_env->shdl + BPS_IDX_BP_MEAS_IND_CFG))
            ||  (p_param->handle == (p_blps_env->shdl + BPS_IDX_INTM_CUFF_PRESS_NTF_CFG))
            ||  (p_param->handle == (p_blps_env->shdl + BPS_IDX_RACP_IND_CFG))
            )
        {
            // Inform APP of configuration change
            struct blps_cfg_indntf_ind *p_ind = KE_MSG_ALLOC(
                    BLPS_CFG_INDNTF_IND,
                    prf_dst_task_get(&p_blps_env->prf_env, conidx),
                    prf_src_task_get(&p_blps_env->prf_env, conidx),
                    blps_cfg_indntf_ind);

            p_ind->conidx = conidx;
            p_ind->char_code = char_code;
            memcpy(&p_ind->cfg_val, &value, sizeof(uint16_t));

            ke_msg_send(p_ind);
        }
        else if(p_param->handle == (p_blps_env->shdl + BPS_IDX_RACP_VAL))
        {
            // Inform APP of RACP write REQ
            struct blps_racp_write_ind *p_ind = KE_MSG_ALLOC(
                    BLPS_RACP_WRITE_IND,
                    prf_dst_task_get(&p_blps_env->prf_env, conidx),
                    prf_src_task_get(&p_blps_env->prf_env, conidx),
                    blps_racp_write_ind);

            p_ind->conidx = conidx;
            
            if(p_param->length <= BLPS_RACP_MAX_LEN)
            {
                memcpy(&p_ind->write_val, &(p_param->value), p_param->length);
            }
            else
            {
                memcpy(&p_ind->write_val, &(p_param->value), BLPS_RACP_MAX_LEN);
            }
            
            p_ind->write_val_len = p_param->length;
            
            ke_msg_send(p_ind);
        }
    }

    // Send write response
    p_cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
    p_cfm->handle = p_param->handle;
    p_cfm->status = status;
    ke_msg_send(p_cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATTC_CMP_EVT for GATTC_NOTIFY and GATT_INDICATE message meaning
 * that Measurement notification/indication has been correctly sent to peer device
 *
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_cmp_evt_handler(ke_msg_id_t const msgid, struct gattc_cmp_evt const *p_param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct blps_env_tag *p_blps_env = PRF_ENV_GET(BLPS, blps);
    uint8_t conidx = KE_IDX_GET(src_id);

    switch (p_param->operation)
    {
        case GATTC_NOTIFY:
        case GATTC_INDICATE:
        {
            // Send a complete event status to the application
            blps_send_cmp_evt(p_blps_env, conidx, p_param->operation, p_param->seq_num, p_param->status);
        } break;

        default:
        {

        } break;
    }

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
KE_MSG_HANDLER_TAB(blps)
{
    {BLPS_ENABLE_REQ,           (ke_msg_func_t) blps_enable_req_handler},
    {GATTC_READ_REQ_IND,        (ke_msg_func_t) gattc_read_req_ind_handler},
    {BLPS_MEAS_SEND_CMD,        (ke_msg_func_t) blps_meas_send_cmd_handler},
    {BLPS_RACP_RESP_SEND_CMD,   (ke_msg_func_t) blps_racp_resp_send_cmd_handler},
    {GATTC_CMP_EVT,             (ke_msg_func_t) gattc_cmp_evt_handler},
    {GATTC_WRITE_REQ_IND,       (ke_msg_func_t) gattc_write_req_ind_handler},
};

void blps_task_init(struct ke_task_desc *p_task_desc)
{
    // Get the address of the environment
    struct blps_env_tag *p_blps_env = PRF_ENV_GET(BLPS, blps);

    p_task_desc->msg_handler_tab = blps_msg_handler_tab;
    p_task_desc->msg_cnt         = ARRAY_LEN(blps_msg_handler_tab);
    p_task_desc->state           = p_blps_env->state;
    p_task_desc->idx_max         = BLPS_IDX_MAX;
}

#endif /* #if (BLE_BP_SENSOR) */

/// @} BLPSTASK
