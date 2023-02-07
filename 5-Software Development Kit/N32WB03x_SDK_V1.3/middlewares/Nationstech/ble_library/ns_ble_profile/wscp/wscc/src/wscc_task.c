/**
 ****************************************************************************************
 *
 * @file wscc_task.c
 *
 * @brief Weight SCale Profile Collector Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 * $ Rev $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup WSCCTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_WSC_CLIENT)
#include "wsc_common.h"
#include "wscc_task.h"
#include "wscc/src/wscc.h"

#include "gap.h"
#include "attm.h"
#include "gattc_task.h"
#include "ke_mem.h"
#include "co_utils.h"


/*
 * STRUCTURES
 ****************************************************************************************
 */

/// State machine used to retrieve Weight Scale Service characteristics information
const struct prf_char_def wscc_wss_char[WSCC_CHAR_WSS_MAX] =
{
    [WSCC_CHAR_WSS_FEATURE] = {ATT_CHAR_WEIGHT_SCALE_FEATURE,
                                         ATT_MANDATORY,
                                        (ATT_CHAR_PROP_RD ) },

    [WSCC_CHAR_WSS_MEAS]    = {ATT_CHAR_WEIGHT_MEASUREMENT,
                                         ATT_MANDATORY,
                                        (ATT_CHAR_PROP_IND) },
};

/// State machine used to retrieve Weight Scale Service characteristic description information
const struct prf_char_desc_def wscc_wss_char_desc[WSCC_DESC_WSS_MAX] =
{
    /// Client config
    [WSCC_DESC_WSS_MEAS_CCC] = {ATT_DESC_CLIENT_CHAR_CFG,
                                         ATT_MANDATORY,
                                         WSCC_CHAR_WSS_MEAS},
};


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_SDP_SVC_IND_HANDLER message.
 * The handler stores the found service details for service discovery.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_ind Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_sdp_svc_ind_handler(ke_msg_id_t const msgid,
                                             struct gattc_sdp_svc_ind const *p_ind,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    do
    {
        // Weight Scale Collector Role Task Environment
        struct wscc_env_tag *p_wscc_env = PRF_ENV_GET(WSCC, wscc);
        uint8_t conidx = KE_IDX_GET(dest_id);

        if ((ke_state_get(dest_id) != WSCC_DISCOVERING_SVC) || p_wscc_env == NULL)
        {
            break;
        }

        ASSERT_INFO(p_wscc_env != NULL, dest_id, src_id);
        ASSERT_INFO(p_wscc_env->env[conidx] != NULL, dest_id, src_id);

        if(attm_uuid16_comp((unsigned char *)p_ind->uuid, p_ind->uuid_len, ATT_SVC_WEIGHT_SCALE))
        {
            uint8_t fnd_att;

            // Retrieve WS characteristics and descriptors
            prf_extract_svc_info(p_ind, WSCC_CHAR_WSS_MAX, &wscc_wss_char[0],  &p_wscc_env->env[conidx]->wss.chars[0],
                    WSCC_DESC_WSS_MAX, &wscc_wss_char_desc[0], &p_wscc_env->env[conidx]->wss.descs[0]);

            //Even if we get multiple responses we only store 1 range
            p_wscc_env->env[conidx]->wss.svc.shdl = p_ind->start_hdl;
            p_wscc_env->env[conidx]->wss.svc.ehdl = p_ind->end_hdl;

            for (fnd_att = 0; fnd_att < (p_ind->end_hdl - p_ind->start_hdl); fnd_att++)
            {
                if((p_ind->info[fnd_att].att_type == GATTC_SDP_INC_SVC) &&
                    (attm_uuid16_comp((uint8_t *)&p_ind->info[fnd_att].inc_svc.uuid[0],
                            p_ind->info[fnd_att].inc_svc.uuid_len,ATT_SVC_BODY_COMPOSITION)))
                {
                    // Retrieve Info on included BCS service
                    p_wscc_env->env[conidx]->wss.incl_svc.handle = p_ind->start_hdl+ 1 + fnd_att;
                    p_wscc_env->env[conidx]->wss.incl_svc.start_hdl = p_ind->info[fnd_att].inc_svc.start_hdl;
                    p_wscc_env->env[conidx]->wss.incl_svc.end_hdl = p_ind->info[fnd_att].inc_svc.end_hdl;
                    p_wscc_env->env[conidx]->wss.incl_svc.uuid_len = p_ind->info[fnd_att].inc_svc.uuid_len;
                    memcpy(&p_wscc_env->env[conidx]->wss.incl_svc.uuid[0],
                            &p_ind->info[fnd_att].inc_svc.uuid[0],p_ind->info[fnd_att].inc_svc.uuid_len);

                    break;
                }
            }

            p_wscc_env->env[conidx]->nb_svc++;
        }

    } while(0);

    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles reception of the @ref WSCC_ENABLE_REQ message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

__STATIC int wscc_enable_req_handler(ke_msg_id_t const msgid,
        struct wscc_enable_req *p_param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Status
    uint8_t status     = GAP_ERR_NO_ERROR;
    // Get connection index
    uint8_t conidx = KE_IDX_GET(dest_id);
    uint8_t state = ke_state_get(dest_id);
    // Weight Scale Collector Role Task Environment
    struct wscc_env_tag *p_wscc_env = PRF_ENV_GET(WSCC, wscc);

    ASSERT_INFO(p_wscc_env != NULL, dest_id, src_id);
    if ((state == WSCC_IDLE) && (p_wscc_env->env[conidx] == NULL))
    {
        // allocate environment variable for task instance
        p_wscc_env->env[conidx] = (struct wscc_cnx_env *) ke_malloc(sizeof(struct wscc_cnx_env), KE_MEM_ATT_DB);
        memset(p_wscc_env->env[conidx], 0, sizeof(struct wscc_cnx_env));

        // Start discovering
        if (p_param->con_type == PRF_CON_DISCOVERY)
        {
            // Start discovery with Weight Scale
            prf_disc_svc_send(&(p_wscc_env->prf_env), conidx, ATT_SVC_WEIGHT_SCALE);

            // Go to DISCOVERING SERVICE state
            ke_state_set(dest_id, WSCC_DISCOVERING_SVC);
        }
        //normal connection, get saved att details
        else
        {
            p_wscc_env->env[conidx]->wss = p_param->wss;

            //send APP confirmation that can start normal connection to TH
            wscc_enable_rsp_send(p_wscc_env, conidx, GAP_ERR_NO_ERROR);
        }
    }
    else if (state != WSCC_FREE)
    {
        // The message will be forwarded towards the good task instance
       status = PRF_ERR_REQ_DISALLOWED;
    }

    if(status != GAP_ERR_NO_ERROR)
    {
        // The request is disallowed (profile already enabled for this connection, or not enough memory, ...)
        wscc_enable_rsp_send(p_wscc_env, conidx, status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref WSCC_RD_FEAT_CMD message from the application.
 * @brief To read the Feature Characteristic in the peer server.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

__STATIC int wscc_rd_feat_cmd_handler(ke_msg_id_t const msgid,
        struct wscc_rd_feat_cmd *p_param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t status = PRF_ERR_REQ_DISALLOWED;
    // Message status
    int msg_status = KE_MSG_CONSUMED;
    // Get the address of the environment
    struct wscc_env_tag *p_wscc_env = PRF_ENV_GET(WSCC, wscc);
    // Get connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state == WSCC_IDLE)
    {
        ASSERT_INFO(p_wscc_env != NULL, dest_id, src_id);
        // environment variable not ready
        if(p_wscc_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else
        {
            // Attribute Handle
            uint16_t search_hdl = p_wscc_env->env[conidx]->wss.chars[WSCC_CHAR_WSS_FEATURE].val_hdl;
            // Service
            struct prf_svc *p_svc = &p_wscc_env->env[conidx]->wss.svc;

            // Check if handle is viable
            if ((search_hdl != ATT_INVALID_SEARCH_HANDLE) && (p_svc != NULL))
            {
                // Force the operation value
                p_wscc_env->env[conidx]->op_pending = WSCC_READ_FEAT_OP_CODE;
                // Send read request
                prf_read_char_send(&(p_wscc_env->prf_env), conidx, p_svc->shdl, p_svc->ehdl, search_hdl);
                // Go to the Busy state
                ke_state_set(dest_id, WSCC_BUSY);

                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status =  PRF_ERR_INEXISTENT_HDL;
            }
        }
    }
    else if (state == WSCC_FREE)
    {
        status = GAP_ERR_DISCONNECTED;
    }
    else
    {
        // Another procedure is pending, keep the command for later
        msg_status = KE_MSG_SAVED;
        status = GAP_ERR_NO_ERROR;
    }

    if (status != GAP_ERR_NO_ERROR)
    {
        wscc_send_cmp_evt(p_wscc_env, conidx, WSCC_READ_FEAT_OP_CODE, status);
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref WSCC_RD_MEAS_CCC_CMD  message from the application.
 * @brief To read the CCC value of the Measurement characteristic in the peer server.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

__STATIC int wscc_rd_meas_ccc_cmd_handler(ke_msg_id_t const msgid,
        struct wscc_rd_meas_ccc_cmd *p_param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t status = PRF_ERR_REQ_DISALLOWED;
    // Message status
    int msg_status = KE_MSG_CONSUMED;
    // Get the address of the environment
    struct wscc_env_tag *p_wscc_env = PRF_ENV_GET(WSCC, wscc);
    // Get connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state == WSCC_IDLE)
    {
        ASSERT_INFO(p_wscc_env != NULL, dest_id, src_id);
        // environment variable not ready
        if(p_wscc_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else
        {
            // Attribute Handle
            uint16_t search_hdl = p_wscc_env->env[conidx]->wss.descs[WSCC_DESC_WSS_MEAS_CCC].desc_hdl;
            // Service
            struct prf_svc *p_svc = &p_wscc_env->env[conidx]->wss.svc;

            // Check if handle is viable
            if ((search_hdl != ATT_INVALID_SEARCH_HANDLE) && (p_svc != NULL))
            {
                // Force the operation value
                p_wscc_env->env[conidx]->op_pending = WSCC_READ_CCC_OP_CODE;
                // Send read request
                prf_read_char_send(&(p_wscc_env->prf_env), conidx, p_svc->shdl, p_svc->ehdl, search_hdl);
                // Go to the Busy state
                ke_state_set(dest_id, WSCC_BUSY);

                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status =  PRF_ERR_INEXISTENT_HDL;
            }
        }
    }
    else if (state == WSCC_FREE)
    {
        status = GAP_ERR_DISCONNECTED;
    }
    else
    {
        // Another procedure is pending, keep the command for later
        msg_status = KE_MSG_SAVED;
        status = GAP_ERR_NO_ERROR;
    }

    if (status != GAP_ERR_NO_ERROR)
    {
        wscc_send_cmp_evt(p_wscc_env, conidx, WSCC_READ_CCC_OP_CODE, status);
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref WSCC_WR_MEAS_CCC_CMD message.
 * Allows the application to write new CCC values to a Characteristic in the peer server
 *
 * @param[in] msgid Id of the message received.
 * @param[in] p_cmd Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int wscc_wr_meas_ccc_cmd_handler(ke_msg_id_t const msgid,
                                   struct wscc_wr_meas_ccc_cmd *p_cmd,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    // Status
    uint8_t status = GAP_ERR_NO_ERROR;
    // Message status
    int msg_status = KE_MSG_CONSUMED;

    // Get the address of the environment
    struct wscc_env_tag *p_wscc_env = PRF_ENV_GET(WSCC, wscc);
    // Get connection index

    if (state == WSCC_IDLE)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);

        ASSERT_INFO(p_wscc_env != NULL, dest_id, src_id);
        // environment variable not ready
        if (p_wscc_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else
        {
             // Attribute Handle
            uint16_t handle = p_wscc_env->env[conidx]->wss.descs[WSCC_DESC_WSS_MEAS_CCC].desc_hdl;
            // Service
            struct prf_svc *p_svc = &p_wscc_env->env[conidx]->wss.svc;

            // Check if handle is viable
            if ((handle != ATT_INVALID_SEARCH_HANDLE) && (p_svc != NULL))
            {
                // Force the operation value
                p_wscc_env->env[conidx]->op_pending = WSCC_WRITE_CCC_OP_CODE;

                // Send the write request
                prf_gatt_write_ntf_ind(&(p_wscc_env->prf_env), conidx, handle, p_cmd->ccc);

                // Go to the Busy state
                ke_state_set(dest_id, WSCC_BUSY);

                status = GAP_ERR_NO_ERROR;
            }
            else
            {
                status = PRF_ERR_INEXISTENT_HDL;
            }
        }

        if (status != GAP_ERR_NO_ERROR)
        {
            wscc_send_cmp_evt(p_wscc_env, conidx, WSCC_WRITE_CCC_OP_CODE, status);
        }
    }
    else if (state == WSCC_FREE)
    {
        status = GAP_ERR_DISCONNECTED;
    }
    else
    {
        // Another procedure is pending, keep the command for later
        msg_status = KE_MSG_SAVED;
        status = GAP_ERR_NO_ERROR;
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_CMP_EVT message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_cmp_evt_handler(ke_msg_id_t const msgid,
                                 struct gattc_cmp_evt const *p_param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct wscc_env_tag *p_wscc_env = PRF_ENV_GET(WSCC, wscc);

    if (p_wscc_env != NULL)
    {
        // Status
        uint8_t status;
        uint8_t conidx = KE_IDX_GET(dest_id);
        uint8_t state = ke_state_get(dest_id);

        if (state == WSCC_DISCOVERING_SVC)
        {
            if ((p_param->status == GAP_ERR_NO_ERROR) || (p_param->status == ATT_ERR_ATTRIBUTE_NOT_FOUND))
            {
                // Check service (if found)
                if (p_wscc_env->env[conidx]->nb_svc)
                {
                    // Check service (mandatory)
                    status = prf_check_svc_char_validity(WSCC_CHAR_WSS_MAX,
                            p_wscc_env->env[conidx]->wss.chars,
                            wscc_wss_char);

                    // Check Descriptors (mandatory)
                    if(status == GAP_ERR_NO_ERROR)
                    {
                        status = prf_check_svc_char_desc_validity(WSCC_DESC_WSS_MAX,
                                p_wscc_env->env[conidx]->wss.descs,
                                wscc_wss_char_desc,
                                p_wscc_env->env[conidx]->wss.chars);
                    }
                }
            }

            // Raise an WSCC_ENABLE_REQ complete event.
            ke_state_set(dest_id, WSCC_IDLE);

            wscc_enable_rsp_send(p_wscc_env, conidx, p_param->status);
        }
        else if (state == WSCC_BUSY)
        {
            uint8_t op_pending = p_wscc_env->env[conidx]->op_pending;

            status = p_param->status;

            switch (p_param->operation)
            {
                case GATTC_READ:
                {
                    switch(op_pending)
                    {
                        case WSCC_READ_FEAT_OP_CODE:
                        case WSCC_READ_CCC_OP_CODE:
                        {
                            wscc_send_cmp_evt(p_wscc_env, conidx, op_pending, status);
                        }
                        break;

                        default:
                        {
                            break;
                        }
                    }
                    ke_state_set(dest_id, WSCC_IDLE);
                }
                break;

                case GATTC_WRITE:
                case GATTC_WRITE_NO_RESPONSE:
                {
                    // Send the complete event status
                    wscc_send_cmp_evt(p_wscc_env, conidx, op_pending, status);
                    ke_state_set(dest_id, WSCC_IDLE);

                } break;

                case GATTC_REGISTER:
                {
                    if (status != GAP_ERR_NO_ERROR)
                    {
                        // Send the complete event error status
                        wscc_send_cmp_evt(p_wscc_env, conidx, GATTC_REGISTER, status);
                    }
                    // Go to connected state
                    ke_state_set(dest_id, WSCC_IDLE);
                } break;

                case GATTC_UNREGISTER:
                {
                    // Do nothing
                } break;

                default:
                {
                    ASSERT_ERR(0);
                } break;
            }
        }
    }
    // else ignore the message

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_READ_IND message.
 * Generic event received after every simple read command sent to peer server.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_read_ind_handler(ke_msg_id_t const msgid,
                                    struct gattc_read_ind const *p_param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    if (ke_state_get(dest_id) == WSCC_BUSY)
    {
        // Get the address of the environment
        struct wscc_env_tag *p_wscc_env = PRF_ENV_GET(WSCC, wscc);
        uint8_t conidx = KE_IDX_GET(dest_id);
        uint8_t op_pending;

        ASSERT_INFO(p_wscc_env != NULL, dest_id, src_id);
        ASSERT_INFO(p_wscc_env->env[conidx] != NULL, dest_id, src_id);

        // Find the op_pending -  and check it is a valid read code.
        op_pending = p_wscc_env->env[conidx]->op_pending;
        if ((op_pending == WSCC_READ_FEAT_OP_CODE) || (op_pending == WSCC_READ_CCC_OP_CODE))
        {
            switch (op_pending)
            {
                case WSCC_READ_FEAT_OP_CODE:
                {
                    struct wscc_feat_ind *p_ind = KE_MSG_ALLOC(WSCC_FEAT_IND,
                                                prf_dst_task_get(&(p_wscc_env->prf_env), conidx),
                                                dest_id,
                                                wscc_feat_ind);

                    p_ind->feature = co_read32p(&p_param->value[0]);

                    ke_msg_send(p_ind);
                }
                break;

                case WSCC_READ_CCC_OP_CODE :
                {
                    struct wscc_meas_ccc_ind *p_ind = KE_MSG_ALLOC(WSCC_MEAS_CCC_IND,
                                                prf_dst_task_get(&(p_wscc_env->prf_env), conidx),
                                                dest_id,
                                                wscc_meas_ccc_ind);

                    p_ind->ccc = co_read16p(&p_param->value[0]);

                    ke_msg_send(p_ind);
                }
                break;

                default:
                {
                    ASSERT_ERR(0);
                } break;
            }
        }
    }
    // else drop the message

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_EVENT_IND message.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_event_ind_handler(ke_msg_id_t const msgid,
                                   struct gattc_event_ind const *p_param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(dest_id);
    // Get the address of the environment
    struct wscc_env_tag *p_wscc_env = PRF_ENV_GET(WSCC, wscc);

    if (p_wscc_env != NULL)
    {
        switch (p_param->type)
        {
            case (GATTC_INDICATE):
            {
                struct gattc_event_cfm *p_cfm = NULL;

                if (p_param->handle == p_wscc_env->env[conidx]->wss.chars[WSCC_CHAR_WSS_MEAS].val_hdl)
                {
                    //indication Build a WSCC_WSS_IND message
                    uint8_t *p_val;

                    struct wscc_meas_ind *p_ind = KE_MSG_ALLOC(WSCC_MEAS_IND,
                                               prf_dst_task_get(&p_wscc_env->prf_env, conidx),
                                               prf_src_task_get(&p_wscc_env->prf_env, conidx),
                                               wscc_meas_ind);

                    // Read the Flags and restrict to valid values only
                    p_ind->flags = p_param->value[0] & WSC_MEAS_FLAGS_VALID;

                    // decode message
                    p_val = (uint8_t *)&p_param->value[1];
                    // Mandatory weight
                    p_ind->weight = co_read16p(p_val);
                    p_val += sizeof(uint16_t);

                    // Weight resolution
                    p_ind->wght_resol = *p_val++;

                    // Measurement Units
                    p_ind->meas_u = *p_val++;

                    // Timestamp if present
                    if (GETB(p_ind->flags, WSC_MEAS_FLAGS_TIMESTAMP_PRESENT))
                    {
                        p_val += prf_unpack_date_time(p_val, &p_ind->time_stamp);
                    }

                    // User Id if present
                    if (GETB(p_ind->flags, WSC_MEAS_FLAGS_USER_ID_PRESENT))
                    {
                        p_ind->user_id = *p_val++;
                    }

                    // BMI & Height if present
                    if (GETB(p_ind->flags, WSC_MEAS_FLAGS_BMI_PRESENT))
                    {
                        // BMI
                        p_ind->bmi = co_read16p(p_val);
                        p_val += sizeof(uint16_t);
                        // Height
                        p_ind->height = co_read16p(p_val);
                        p_val += sizeof(uint16_t);
                        // Height Resolution
                        p_ind->hght_resol = *p_val++;
                    }

                    ke_msg_send(p_ind);
                }

                // confirm that indication has been correctly received
                p_cfm = KE_MSG_ALLOC(GATTC_EVENT_CFM, src_id, dest_id, gattc_event_cfm);
                p_cfm->handle = p_param->handle;
                ke_msg_send(p_cfm);

                // else drop the message
            } break;

            default:
            {
                ASSERT_ERR(0);
            } break;
        }
    }

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the default message handlers
KE_MSG_HANDLER_TAB(wscc)
{
    {WSCC_ENABLE_REQ,                   (ke_msg_func_t)wscc_enable_req_handler},

    {GATTC_SDP_SVC_IND,                 (ke_msg_func_t)gattc_sdp_svc_ind_handler},
    {GATTC_CMP_EVT,                     (ke_msg_func_t)gattc_cmp_evt_handler},

    {GATTC_READ_IND,                    (ke_msg_func_t)gattc_read_ind_handler},
    {GATTC_EVENT_IND,                   (ke_msg_func_t)gattc_event_ind_handler},
    {GATTC_EVENT_REQ_IND,               (ke_msg_func_t)gattc_event_ind_handler},
    {WSCC_RD_FEAT_CMD,                  (ke_msg_func_t)wscc_rd_feat_cmd_handler},
    {WSCC_RD_MEAS_CCC_CMD,              (ke_msg_func_t)wscc_rd_meas_ccc_cmd_handler},
    {WSCC_WR_MEAS_CCC_CMD,              (ke_msg_func_t)wscc_wr_meas_ccc_cmd_handler},

};

/*
 * GLOBAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void wscc_task_init(struct ke_task_desc *p_task_desc)
{
    // Get the address of the environment
    struct wscc_env_tag *p_wscc_env = PRF_ENV_GET(WSCC, wscc);

    p_task_desc->msg_handler_tab = wscc_msg_handler_tab;
    p_task_desc->msg_cnt         = ARRAY_LEN(wscc_msg_handler_tab);
    p_task_desc->state           = p_wscc_env->state;
    p_task_desc->idx_max         = WSCC_IDX_MAX;
}

#endif //(BLE_WSC_CLIENT)

/// @} WSCCTASK
