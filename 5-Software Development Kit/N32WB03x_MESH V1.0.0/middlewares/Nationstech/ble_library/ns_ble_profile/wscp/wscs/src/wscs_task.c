/**
 ****************************************************************************************
 *
 * @file wscs_task.c
 *
 * @brief Weight SCale Profile Sensor Task Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup WSCSTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_WSC_SERVER)
#include "wsc_common.h"

#include "gapc.h"
#include "gattc_task.h"
#include "attm.h"
#include "wscs/src/wscs.h"
#include "wscs_task.h"
#include "prf_utils.h"

#include "ke_mem.h"
#include "co_utils.h"

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref WSCS_ENABLE_REQ message.
 *
 * @param[in] msgid Id of the message received
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int wscs_enable_req_handler(ke_msg_id_t const msgid,
                                    struct wscs_enable_req *p_param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Status
    uint8_t status = PRF_ERR_REQ_DISALLOWED;
    struct wscs_enable_rsp *p_cmp_evt = NULL;

    if (ke_state_get(dest_id) == WSCS_IDLE)
    {
        // Get the address of the environment
        struct wscs_env_tag *p_wscs_env = PRF_ENV_GET(WSCS, wscs);

        // Save indication config
        p_wscs_env->prfl_ind_cfg[KE_IDX_GET(src_id)] = p_param->ind_cfg;
        status = GAP_ERR_NO_ERROR;
    }

    // send completed information to APP task that contains error status
    p_cmp_evt = KE_MSG_ALLOC(WSCS_ENABLE_RSP, src_id, dest_id, wscs_enable_rsp);
    p_cmp_evt->status     = status;

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
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_read_req_ind_handler(ke_msg_id_t const msgid,
                                      struct gattc_read_req_ind const *p_param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    int msg_status = KE_MSG_CONSUMED;

    // check that task is in idle state
    if(ke_state_get(dest_id) == WSCS_IDLE)
    {
        // Get the address of the environment
        struct wscs_env_tag *p_wscs_env = PRF_ENV_GET(WSCS, wscs);
        uint8_t att_idx = p_param->handle - p_wscs_env->shdl;
        uint8_t value[WCS_FEAT_VAL_LEN];
        uint8_t value_size = 0;
        uint8_t status = ATT_ERR_NO_ERROR;
        // Send data to peer device
        struct gattc_read_cfm *p_cfm = NULL;

        switch(att_idx)
        {
            case WSCS_IDX_FEAT_VAL:
            {
                value_size = WCS_FEAT_VAL_LEN;
                co_write32p(&(value[0]), p_wscs_env->feature);
            } break;

            case WSCS_IDX_MEAS_CCC:
            {
                value_size = WCS_MEAS_CCC_LEN;
                co_write16p(&(value[0]), p_wscs_env->prfl_ind_cfg[KE_IDX_GET(src_id)]);
            } break;

            default:
            {
                if (p_cfm == NULL)
                {
                    status = ATT_ERR_REQUEST_NOT_SUPPORTED;
                }
            } break;
        }

        p_cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, value_size);
        p_cfm->length = value_size;
        memcpy(p_cfm->value, value, value_size);
        p_cfm->handle = p_param->handle;
        p_cfm->status = status;

        // Send value to peer device.
        ke_msg_send(p_cfm);
    }
    // else process it later
    else
    {
        msg_status = KE_MSG_SAVED;
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_att_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gattc_att_info_req_ind *p_param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct wscs_env_tag *p_wscs_env = PRF_ENV_GET(WSCS, wscs);
    uint8_t att_idx = p_param->handle - p_wscs_env->shdl;
    struct gattc_att_info_cfm *p_cfm;

    //Send write response
    p_cfm = KE_MSG_ALLOC(GATTC_ATT_INFO_CFM, src_id, dest_id, gattc_att_info_cfm);
    p_cfm->handle = p_param->handle;

    // check if it's a client configuration char
    if(att_idx == WSCS_IDX_MEAS_CCC)
    {
        // CCC attribute length = 2
        p_cfm->length = WCS_MEAS_CCC_LEN;
        p_cfm->status = GAP_ERR_NO_ERROR;
    }
    else // not expected request
    {
        p_cfm->length = 0;
        p_cfm->status = ATT_ERR_WRITE_NOT_PERMITTED;
    }

    ke_msg_send(p_cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref WSCS_MEAS_INDICATE_CMD message.
 * Send MEASUREMENT INDICATION to the connected peer case of CCC enabled
 *
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int wscs_meas_indicate_cmd_handler(ke_msg_id_t const msgid,
                                          struct wscs_meas_indicate_cmd *p_param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    // Message status
    int msg_status = KE_MSG_CONSUMED;
    // Get the address of the environment
    struct wscs_env_tag *p_wscs_env = PRF_ENV_GET(WSCS, wscs);

     // check that task is in idle state
    if(ke_state_get(dest_id) == WSCS_IDLE)
    {
        // Connection index
        uint8_t conidx = KE_IDX_GET(dest_id);

        if (p_wscs_env->prfl_ind_cfg[conidx] & PRF_CLI_START_IND)
        {
            // Allocate the GATT notification message
            struct gattc_send_evt_cmd *p_ind;
            // allocate the maximum block for indication
            uint8_t value[WSCS_MEAS_IND_SIZE];
            uint8_t status = ATT_ERR_NO_ERROR;

            // Mask off any illegal bits in the flags field
            p_param->flags &= WSC_MEAS_FLAGS_VALID;
            // If the Weight indicates Measurement Unsuccessful - 0xFFFF
            // The the flags field should not include BMI and Height

            if (p_param->weight == WSC_MEASUREMENT_UNSUCCESSFUL)
            {
                // Disable all other optional fields other than Timestamp and User ID
                SETB(p_param->flags, WSC_MEAS_FLAGS_BMI_PRESENT, 0);
            }
            else
            {
                // If Body Composition Service is included then Mandatory to have
                // both the Height and BMI fields present (in flags) otherwise
                // reject command.
                if ((p_wscs_env->bcs_included) && (!GETB(p_param->flags, WSC_MEAS_FLAGS_BMI_PRESENT)))
                {
                    status = PRF_ERR_INVALID_PARAM;
                }
            }

            //**************************************************************
            // Encode the Fields of the Weight Measurement
            // if the Application provide flags and fields which do not correspond
            // to the features declared by the server, we adjust the flags field to
            // ensure we only Indicate with fields compatible with our features.
            // Thus the flags fields is encoded last as it will be modifed by checks on
            // features.
            //********************************************************************
            if (status == ATT_ERR_NO_ERROR)
            {
                uint16_t length = 0;

                // Flags is 8 bits
                length++;

                // Mandatory weight
                // pack measured value in database
                co_write16p(&value[length], p_param->weight);
                length += sizeof(uint16_t);

                // Get Weight Resolution
                value[length++] = wscs_get_wght_resol(p_wscs_env->feature);

                // Measurement Units
                // 0 for SI and 1 for Imperial
                value[length++] = GETB(p_param->flags, WSC_MEAS_FLAGS_UNITS_IMPERIAL);

                // Time Stamp shall be included in flags field if the Server supports Time Stamp feature
                if (GETB(p_wscs_env->feature, WSC_FEAT_TIME_STAMP_SUPP))
                {
                    SETB(p_param->flags, WSC_MEAS_FLAGS_TIMESTAMP_PRESENT, 1);
                    length += prf_pack_date_time(&value[length], &p_param->time_stamp);
                }
                else
                {
                    // If Time-Stamp is not supported in the features - it should not be transmitted
                    SETB(p_param->flags, WSC_MEAS_FLAGS_TIMESTAMP_PRESENT, 0);
                }

                // User ID shall be included in flags field if the Server supports Multiple Users feature
                if (GETB(p_wscs_env->feature, WSC_FEAT_MULTIPLE_USERS_SUPP))
                {
                    // If the Multiple users fields is enabled in the features and not present in the flags field
                    // then an "UNKNOWN/GUEST" user is indicated.
                    if (!GETB(p_param->flags, WSC_MEAS_FLAGS_USER_ID_PRESENT))
                    {
                        value[length++] = WSC_MEAS_USER_ID_UNKNOWN_USER;
                    }
                    else
                    {
                        value[length++] = p_param->user_id;
                    }

                    SETB(p_param->flags, WSC_MEAS_FLAGS_USER_ID_PRESENT, 1);
                }
                else
                {
                    // Shall not be included if the Multiple Users feature is not supported
                    SETB(p_param->flags, WSC_MEAS_FLAGS_USER_ID_PRESENT, 0);
                }

                // BMI & Height if present and enabled in the features.
                if (GETB(p_param->flags, WSC_MEAS_FLAGS_BMI_PRESENT))
                {
                    if (GETB(p_wscs_env->feature, WSC_FEAT_BMI_SUPP))
                    {
                        co_write16p(&value[length], p_param->bmi);
                        length += sizeof(uint16_t);
                        co_write16p(&value[length], p_param->height);
                        length += sizeof(uint16_t);
                        value[length++] = wscs_get_hght_resol(p_wscs_env->feature);
                    }
                    else
                    {
                        SETB(p_param->flags, WSC_MEAS_FLAGS_BMI_PRESENT, 0);
                    }
                }
                // Finally store the flags in Byte 0 - the flags may have been changed in pre-ceeding steps.
                value[0] = p_param->flags;

                // Allocate the GATT notification message
                p_ind = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                        KE_BUILD_ID(TASK_GATTC, conidx),
                        dest_id,
                        gattc_send_evt_cmd, length);

                // Fill in the parameter structure
                p_ind->operation = GATTC_INDICATE;
                p_ind->handle = p_wscs_env->shdl + WSCS_IDX_MEAS_IND;
                // Pack Measurement record
                p_ind->length = length;
                memcpy (p_ind->value, &value[0], length);

                // Send the event
                ke_msg_send(p_ind);

                // go to busy state
                ke_state_set(dest_id, WSCS_OP_INDICATE);
            }
            else
            {
                // Send a command complete to the App, indicate msg has wrong parameters.
                wscs_send_cmp_evt(p_wscs_env, conidx, WSCS_MEAS_INDICATE_CMD_OP_CODE, status);
            }
        }
        else
        {
            // Send a command complete to the App indicate msg could not be sent.
            wscs_send_cmp_evt(p_wscs_env, conidx, WSCS_MEAS_INDICATE_CMD_OP_CODE, PRF_ERR_IND_DISABLED);
        }
    }
    // else process it later
    else
    {
        msg_status = KE_MSG_SAVED;
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_WRITE_REQ_IND message.
 *
 * @param[in] msgid Id of the message received.
 * @param[in] p_param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_write_req_ind_handler(ke_msg_id_t const msgid,
                                       struct gattc_write_req_ind const *p_param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct wscs_env_tag *p_wscs_env = PRF_ENV_GET(WSCS, wscs);
    // Message status
    int msg_status = KE_MSG_CONSUMED;

    // Check the connection handle
    if (p_wscs_env != NULL)
    {
        // check that task is in idle state
        if(ke_state_get(dest_id) == WSCS_IDLE)
        {
            // Status
            uint8_t status = GAP_ERR_NO_ERROR;
            uint8_t att_idx = p_param->handle - p_wscs_env->shdl;
            struct gattc_write_cfm *p_cfm = NULL;

            // CSC Measurement Characteristic, Client Characteristic Configuration Descriptor
            if (att_idx == WSCS_IDX_MEAS_CCC)
            {
                uint16_t ntf_cfg;
                uint8_t conidx = KE_IDX_GET(src_id);
                struct wscs_wr_ccc_ind *p_ind = NULL;

                // Get the value
                co_write16p(&ntf_cfg, p_param->value[0]);
                p_wscs_env->prfl_ind_cfg[conidx] = ntf_cfg;

                // Inform the HL about the new configuration
                p_ind = KE_MSG_ALLOC(WSCS_MEAS_CCC_IND,
                        prf_dst_task_get(&p_wscs_env->prf_env, conidx),
                        prf_src_task_get(&p_wscs_env->prf_env, conidx),
                        wscs_wr_ccc_ind);

                p_ind->ind_cfg = ntf_cfg;

                ke_msg_send(p_ind);

            }
            else
            {
                status = PRF_ERR_INVALID_PARAM;
            }
            
            // Send the write response to the peer device
            p_cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
            p_cfm->handle = p_param->handle;
            p_cfm->status = status;
            ke_msg_send(p_cfm);
        }
        else
        {
            msg_status = KE_MSG_SAVED;
        }
    }
    // else drop the message

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATT_NOTIFY_CMP_EVT message meaning that an indication
 * has been correctly sent to peer device (but not confirmed by peer device).
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_cmp_evt_handler(ke_msg_id_t const msgid,  struct gattc_cmp_evt const *p_param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Get the address of the environment
    struct wscs_env_tag *p_wscs_env = PRF_ENV_GET(WSCS, wscs);

    do
    {
        if ((p_wscs_env == NULL) || p_param->operation != GATTC_INDICATE ||
                (ke_state_get(dest_id) != WSCS_OP_INDICATE))
        {
            break;
        }
        else
        {
            // Inform the application that a procedure has been completed
            wscs_send_cmp_evt(p_wscs_env,
                    KE_IDX_GET(src_id),
                    WSCS_MEAS_INDICATE_CMD_OP_CODE,
                    p_param->status);

            ke_state_set(dest_id, WSCS_IDLE);
        }

    } while (0);

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the default message handlers
KE_MSG_HANDLER_TAB(wscs)
{
    {WSCS_ENABLE_REQ,          (ke_msg_func_t) wscs_enable_req_handler},
    /// Send a WSC Measurement to the peer device (Indication)
    {WSCS_MEAS_INDICATE_CMD,   (ke_msg_func_t) wscs_meas_indicate_cmd_handler},

    {GATTC_READ_REQ_IND,       (ke_msg_func_t) gattc_read_req_ind_handler},
    {GATTC_ATT_INFO_REQ_IND,   (ke_msg_func_t) gattc_att_info_req_ind_handler},
    {GATTC_WRITE_REQ_IND,      (ke_msg_func_t) gattc_write_req_ind_handler},
    {GATTC_CMP_EVT,            (ke_msg_func_t) gattc_cmp_evt_handler},
};

/*
 * GLOBAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void wscs_task_init(struct ke_task_desc *p_task_desc)
{
    // Get the address of the environment
    struct wscs_env_tag *p_wscs_env = PRF_ENV_GET(WSCS, wscs);

    p_task_desc->msg_handler_tab = wscs_msg_handler_tab;
    p_task_desc->msg_cnt         = ARRAY_LEN(wscs_msg_handler_tab);
    p_task_desc->state           = p_wscs_env->state;
    p_task_desc->idx_max         = WSCS_IDX_MAX;
}

#endif //(BLE_WSC_SERVER)

/// @} WSCSTASK
