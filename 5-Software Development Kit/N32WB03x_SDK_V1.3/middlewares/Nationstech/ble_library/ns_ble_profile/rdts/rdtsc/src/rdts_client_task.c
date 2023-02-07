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
 * @file rdts_client_task.c
 * @author Nations Firmware Team
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


/**
 ****************************************************************************************
 * @addtogroup RDTSC_TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_RDTS_CLIENT)
#include "gap.h"
#include "gattc_task.h"
#include "prf.h"
#include "rdts_client.h"
#include "rdts_client_task.h"
#include "prf_utils.h"
#include "prf_utils_128.h"
#include "co_utils.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * EXTERNAL DEFINITIONS
 ****************************************************************************************
 */     

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable the RDTS client role, used after connection.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int rdtsc_enable_req_handler(ke_msg_id_t const msgid,
                                    struct rdtsc_enable_req const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
   
    // RDTS Client Role Task Environment
    struct rdtsc_env_tag *rdtsc_env = PRF_ENV_GET(RDTSC, rdtsc);

    if (param->con_type == PRF_CON_DISCOVERY)
    { 
        // Start discovering RDTS on peer
        rdtsc_env->last_uuid_req = ATT_DECL_PRIMARY_SERVICE;
        memcpy(rdtsc_env->last_svc_req, rdtsc_env->rdtsc_cfg.p_uuid128, ATT_UUID_128_LEN);
        prf_disc_svc_send_128(&rdtsc_env->prf_env, rdtsc_env->last_svc_req, param->conidx);
        // Set state to discovering
        ke_state_set(dest_id, RDTSC_DISCOVERING );
            
    }
    
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_DISC_SVC_IND message.
 * Get the start and ending handle of rdts in the enviroment.
 * @param[in] msgid Id of the message received .
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance 
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */ 
static int gattc_disc_svc_ind_handler(ke_msg_id_t const msgid,
                                             struct gattc_disc_svc_ind const *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    // Get the address of the environment
     struct rdtsc_env_tag *rdtsc_env = PRF_ENV_GET(RDTSC, rdtsc);

    // Even if we get multiple responses we only store 1 range
    rdtsc_env->rdts.svc.shdl = param->start_hdl;
    rdtsc_env->rdts.svc.ehdl = param->end_hdl;
    rdtsc_env->nb_svc++;

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_DISC_CHAR_IND message.
 * Characteristics for the currently desired service handle range are obtained and kept.
 * @param[in] msgid Id of the message received .
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance 
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_disc_char_ind_handler(ke_msg_id_t const msgid,
                                          struct gattc_disc_char_ind const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct rdtsc_env_tag *rdtsc_env = PRF_ENV_GET(RDTSC, rdtsc);
    
    prf_search_chars_128(rdtsc_env->rdts.svc.ehdl, RDTSC_CHAR_MAX, &rdtsc_env->rdts.chars[0], &rdtsc_env->rdtsc_cfg.p_char[0], param, &rdtsc_env->last_char_code);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_DISC_CHAR_DESC_IND message.
 * This event can be received 2-4 times, depending if measurement interval has seevral properties.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_disc_char_desc_ind_handler(ke_msg_id_t const msgid,
                                           struct gattc_disc_char_desc_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    // Get the address of the environment
   struct rdtsc_env_tag *rdtsc_env = PRF_ENV_GET(RDTSC, rdtsc);

   // Retrieve RDTS descriptors
   prf_search_descs(RDTSC_DESC_MAX, &rdtsc_env->rdts.descs[0], &rdtsc_env->rdtsc_cfg.p_desc[0], param, rdtsc_env->last_char_code);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATTC_CMP_EVT for GATTC_NOTIFY message meaning that
 * notification has been correctly sent to peer device (but not confirmed by peer device).
 * *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

static int gattc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gattc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(src_id);
    
    // Get the address of the environment
    struct rdtsc_env_tag *rdtsc_env = PRF_ENV_GET(RDTSC, rdtsc);
    uint8_t status;

    if(state == RDTSC_DISCOVERING)
    {
        if ((param->status == ATT_ERR_ATTRIBUTE_NOT_FOUND)||(param->status == ATT_ERR_NO_ERROR))
        {
            //Currently discovering rdts_server Service
            if (!memcmp(rdtsc_env->last_svc_req, rdtsc_env->rdtsc_cfg.p_uuid128, ATT_UUID_128_LEN))
            {
                if (rdtsc_env->last_uuid_req == ATT_DECL_PRIMARY_SERVICE)
                {
                    if (rdtsc_env->rdts.svc.shdl == ATT_INVALID_HANDLE)
                    {
                        // stop discovery procedure.
                        rdtsc_enable_cfm_send(rdtsc_env, conidx, PRF_ERR_STOP_DISC_CHAR_MISSING);
                    }
                    // Too many services found only one such service should exist
                    else if(rdtsc_env->nb_svc > 1)
                    {
                        // stop discovery procedure.
                        rdtsc_enable_cfm_send(rdtsc_env, conidx, PRF_ERR_MULTIPLE_SVC);
                    }
                    else
                    {
                        // Discover RDTS Device characteristics
                        prf_disc_char_all_send(&(rdtsc_env->prf_env),  &(rdtsc_env->rdts.svc), conidx); 

                        // Keep last UUID requested and for which service in env
                        rdtsc_env->last_uuid_req = ATT_DECL_CHARACTERISTIC;
                    }
                }
                else if (rdtsc_env->last_uuid_req == ATT_DECL_CHARACTERISTIC)
                {
                    status = prf_check_svc_char_validity_128(RDTSC_CHAR_MAX, rdtsc_env->rdts.chars, rdtsc_env->rdtsc_cfg.p_char);
                    
                    // Check for characteristic properties.
                    if (status == ATT_ERR_NO_ERROR)
                    {
                        rdtsc_env->last_uuid_req = ATT_INVALID_HANDLE;
                        rdtsc_env->last_char_code = rdtsc_env->rdtsc_cfg.p_desc[RDTSC_SRV_TX_DATA_CLI_CFG].char_code;

                        rdtsc_env->conidx = conidx;
                        
                        //Discover characteristic descriptor 
                        prf_disc_char_desc_send(&(rdtsc_env->prf_env), &(rdtsc_env->rdts.chars[rdtsc_env->last_char_code]), conidx); 
                    }
                    else
                    {
                        // Stop discovery procedure.
                        rdtsc_enable_cfm_send(rdtsc_env, conidx, status);
                    }
                }
                else //Descriptors
                {
                    //Get code of next char. having descriptors
                    rdtsc_env->last_char_code = rdtsc_get_next_desc_char_code(rdtsc_env, &rdtsc_env->rdtsc_cfg.p_desc[0]);
                    if (rdtsc_env->last_char_code != RDTSC_CHAR_MAX)
                    {
                        prf_disc_char_desc_send(&(rdtsc_env->prf_env), &(rdtsc_env->rdts.chars[rdtsc_env->last_char_code]), conidx); 
                    }
                    else
                    {
                         status = prf_check_svc_char_desc_validity(RDTSC_DESC_MAX,
                                                                    rdtsc_env->rdts.descs,
                                                                    rdtsc_env->rdtsc_cfg.p_desc,
                                                                    rdtsc_env->rdts.chars);

                        rdtsc_env->nb_svc = 0;
                        rdtsc_enable_cfm_send(rdtsc_env,conidx, status);
                    }
                }
            }
        }
        else
        {
            rdtsc_enable_cfm_send(rdtsc_env, conidx, param->status);
        }
    }
    else if (state == RDTSC_CONNECTED)
    {
        if(param->operation == GATTC_WRITE_NO_RESPONSE)
        {
            rdtsc_env->pending_wr_no_rsp_cmp--;
            if (!rdtsc_env->pending_wr_no_rsp_cmp && rdtsc_env->pending_tx_ntf_cmp)
            {
                rdtsc_env->pending_tx_ntf_cmp = false;
                rdtsc_confirm_data_tx(rdtsc_env, param->status);
            }
        }
        else if(param->operation == GATTC_WRITE)
        {
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_EVENT_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

static int gattc_event_ind_handler(ke_msg_id_t const msgid,
                                        struct gattc_event_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    
    struct rdtsc_env_tag *rdtsc_env = PRF_ENV_GET(RDTSC, rdtsc);
     
    if (rdtsc_env->rdts.chars[RDTSC_SRV_TX_DATA_CHAR].val_hdl == param->handle)
    {

        rdtsc_data_receive( rdtsc_env, (void *)param);
        return (KE_MSG_NO_FREE);
    }

    return (KE_MSG_CONSUMED);
        
}

/**
 ****************************************************************************************
 * @brief Data transmitt request handler
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
 
static int rdtsc_data_tx_req_handler(ke_msg_id_t const msgid,
                                          struct rdtsc_data_tx_req const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    struct rdtsc_env_tag *rdtsc_env = PRF_ENV_GET(RDTSC, rdtsc);

	uint8_t conidx = param->cursor;
  
    if (ke_state_get(rdtsc_env->prf_env.prf_task)==RDTSC_CONNECTED)
    {
        rdtsc_write_data_tx((void *)param, &rdtsc_env->prf_env, conidx, rdtsc_env->rdts.chars[RDTSC_SRV_RX_DATA_CHAR].val_hdl, 
                            (uint8_t *)param->data, sizeof(uint8_t) * param->length, GATTC_WRITE_NO_RESPONSE);
        rdtsc_env->pending_tx_ntf_cmp = true;
        rdtsc_env->pending_wr_no_rsp_cmp++;

        return (KE_MSG_NO_FREE);
    }
    
    return (KE_MSG_CONSUMED);
}    

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

//const struct ke_msg_handler rdtsc_default_state[] =
KE_MSG_HANDLER_TAB(rdtsc)
{
    {GATTC_EVENT_IND,               (ke_msg_func_t)gattc_event_ind_handler},
    {RDTSC_DATA_TX_REQ,             (ke_msg_func_t)rdtsc_data_tx_req_handler},
    {GATTC_DISC_CHAR_IND,           (ke_msg_func_t)gattc_disc_char_ind_handler},
    {GATTC_DISC_SVC_IND,            (ke_msg_func_t)gattc_disc_svc_ind_handler},
    {GATTC_DISC_CHAR_DESC_IND,      (ke_msg_func_t)gattc_disc_char_desc_ind_handler},
    {RDTSC_ENABLE_REQ,              (ke_msg_func_t)rdtsc_enable_req_handler},
    {GATTC_CMP_EVT,                 (ke_msg_func_t)gattc_cmp_evt_handler},
};

void rdtsc_task_init(struct ke_task_desc *p_task_desc)
{
    // Get the address of the environment
    struct rdtsc_env_tag *p_rdtsc_env = PRF_ENV_GET(RDTSC, rdtsc);

    p_task_desc->msg_handler_tab = rdtsc_msg_handler_tab;
    p_task_desc->msg_cnt         = ARRAY_LEN(rdtsc_msg_handler_tab);
    p_task_desc->state           = p_rdtsc_env->state;
    p_task_desc->idx_max         = RDTSC_IDX_MAX;
}

#endif //BLE_RDTS_CLIENT

/// @} RDTSC_TASK

