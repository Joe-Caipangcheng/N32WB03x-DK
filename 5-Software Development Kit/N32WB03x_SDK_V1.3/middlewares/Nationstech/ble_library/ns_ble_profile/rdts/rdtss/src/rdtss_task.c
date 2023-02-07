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
 * @file rdtss_task.c
 * @author Nations Firmware Team
 * @version v1.0.3
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

 
/* Includes ------------------------------------------------------------------*/
#include "rwip_config.h"              // SW configuration

#if (BLE_RDTSS_SERVER)
#include "rdtss_task.h"
#include "rdtss.h"
#include "rdts_common.h"
#include "attm.h"
#include "ke_task.h"
#include "gapc.h"
#include "gapc_task.h"
#include "gattc_task.h"
#include "attm_db.h"
#include "prf_utils.h"
#include "ke_mem.h"
#include "co_utils.h"
/* Private functions ---------------------------------------------------------*/

/** 
 * @brief Stores characteristic value.
 * @param[in] att_idx  Custom attribut index.
 * @param[in] length   Value length.
 * @param[in] data     Pointer to value data.
 * @return 0 on success. 
 */
static int rdtss_att_set_value(uint8_t att_idx, uint16_t length, const uint8_t *data)
{
    struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);
    // Check value in already present in service
    struct rdtss_val_elmt *val = (struct rdtss_val_elmt *) co_list_pick(&(rdtss_env->values));
    // loop until value found
    while (val != NULL)
    {
        // value is present in service
        if (val->att_idx == att_idx)
        {
            // Value present but size changed, free old value
            if (length != val->length)
            {
                //co_list_extract(&rdtss_env->values, &val->hdr, 0);
                co_list_extract(&rdtss_env->values, &val->hdr);        //lxm

                ke_free(val);
                val = NULL;
            }
            break;
        }

        val = (struct rdtss_val_elmt *)val->hdr.next;
    }

    if (val == NULL)
    {
        // allocate value data
        val = (struct rdtss_val_elmt *) ke_malloc(sizeof(struct rdtss_val_elmt) + length, KE_MEM_ATT_DB);
        // insert value into the list
        co_list_push_back(&rdtss_env->values, &val->hdr);
    }
    val->att_idx = att_idx;
    val->length = length;
    memcpy(val->data, data, length);

    return 0;
}

/** 
 * @brief Read characteristic value from.
 * Function checks if attribute exists, and if so return its length and pointer to data.
 * @param[in]  att_idx  Custom attribute index.
 * @param[out] length   Pointer to variable that receive length of the attribute.
 * @param[out] data     Pointer to variable that receive pointer characteristic value.
 * @return 0 on success, ATT_ERR_ATTRIBUTE_NOT_FOUND if there is no value for such attribyte. 
 */
static int rdtss_att_get_value(uint8_t att_idx, uint16_t *length, const uint8_t **data)
{
    struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);
    // Check value in already present in service
    struct rdtss_val_elmt *val = (struct rdtss_val_elmt *) co_list_pick(&rdtss_env->values);
    ASSERT_ERR(data);
    ASSERT_ERR(length);

    // loop until value found
    while (val != NULL)
    {
        // value is present in service
        if (val->att_idx == att_idx)
        {
            *length = val->length;
            *data = val->data;
            break;
        }

        val = (struct rdtss_val_elmt *)val->hdr.next;
    }

    if (val == NULL)
    {
        *length = 0;
        *data = NULL;
    }
    return val ? 0 : ATT_ERR_ATTRIBUTE_NOT_FOUND;
}

/** 
 * @brief Sets initial values for all Clinet Characteristic Configurations.
 * @param[in]  att_db     Custom service attribute definition table.
 * @param[in]  max_nb_att Number of elements in att_db. 
 */
void rdtss_init_ccc_values(const struct attm_desc_128 *att_db, int max_nb_att)
{
    // Default values 0 means no notification
    uint8_t ccc_values[BLE_CONNECTION_MAX] = {0};
    int i;
    // Start form 1, skip service description
    for (i = 1; i < max_nb_att; i++)
    {
        if( PERM_GET(att_db[i].perm, UUID_LEN) == PERM_UUID_16 &&
            (att_db[i].uuid[0] + (att_db[i].uuid[1]<<8) ) == ATT_DESC_CLIENT_CHAR_CFG
        )
        {
            // Set default values for all possible connections
            rdtss_att_set_value(i, sizeof(ccc_values), ccc_values);
        }
    }
}

/** 
 * @brief Set value of CCC for given attribute and connection index.
 * @param[in] conidx   Connection index.
 * @param[in] att_idx  CCC attribute index.
 * @param[in] cc       Value to store. 
 */
void rdtss_set_ccc_value(uint8_t conidx, uint8_t att_idx, uint16_t ccc)
{
    uint16_t length;
    const uint8_t *value;
    uint8_t new_value[BLE_CONNECTION_MAX];
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    rdtss_att_get_value(att_idx, &length, &value);
    ASSERT_ERR(length);
    ASSERT_ERR(value);
    memcpy(new_value, value, length);
    // For now there are only two valid values for ccc, store just one byte other is 0 anyway
    new_value[conidx] = (uint8_t)ccc;
    rdtss_att_set_value(att_idx, length, new_value);
}

/** 
 * @brief Read value of CCC for given attribute and connection index.
 * @param[in]  conidx   Connection index.
 * @param[in]  att_idx  Custom attribute index.
 * @return Value of CCC. 
 */
static uint16_t rdtss_get_ccc_value(uint8_t conidx, uint8_t att_idx)
{
    uint16_t length;
    const uint8_t *value;
    uint16_t ccc_value;

    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    rdtss_att_get_value(att_idx, &length, &value);
    ASSERT_ERR(length);
    ASSERT_ERR(value);

    ccc_value = value[conidx];

    return ccc_value;
}

static void rdtss_exe_operation(void)
{
    struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);
    ASSERT_ERR(rdtss_env->operation != NULL);
    bool notification_sent = false;
    const uint8_t *ccc_values;
    uint16_t length;

    struct rdtss_val_ntf_ind_req *app_req = (struct rdtss_val_ntf_ind_req *)ke_msg2param(rdtss_env->operation);

    rdtss_att_get_value(rdtss_env->ccc_idx, &length, &ccc_values);
    ASSERT_ERR(length == BLE_CONNECTION_MAX);

    // loop on all connections
    while (!notification_sent && rdtss_env->cursor < BLE_CONNECTION_MAX)
    {
        struct gattc_send_evt_cmd *req;
        uint8_t cursor = rdtss_env->cursor++;

        // Check if notification or indication is set for connection
        if ((app_req->notification && ((ccc_values[cursor] & PRF_CLI_START_NTF) == 0)) ||
            (!app_req->notification && ((ccc_values[cursor] & PRF_CLI_START_IND) == 0)))
            continue;

        notification_sent = true;

        // Allocate the GATT notification message
        req = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
            KE_BUILD_ID(TASK_GATTC, cursor), rdtss_env->operation->dest_id, gattc_send_evt_cmd, app_req->length);

        // Fill in the parameter structure
        req->operation = app_req->notification ? GATTC_NOTIFY : GATTC_INDICATE;
        req->handle = rdtss_env->shdl + app_req->handle;
        req->length = app_req->length;
        memcpy(req->value, app_req->value, app_req->length);
        
        // Send the event
        ke_msg_send(req);
    }

    // check if operation finished
    if (!notification_sent)
    {
        if (app_req->notification)
        {
            // Inform the application that the notification PDU has been sent over the air.
            struct rdtss_val_ntf_cfm *cfm = KE_MSG_ALLOC(RDTSS_VAL_NTF_CFM,
                                                          rdtss_env->operation->src_id, rdtss_env->operation->dest_id,
                                                          rdtss_val_ntf_cfm);
            cfm->handle = app_req->handle;
            cfm->status = GAP_ERR_NO_ERROR;
            ke_msg_send(cfm);
        }
        else
        {
            // Inform the application that the indication has been confirmed by the peer device.
            struct rdtss_val_ind_cfm *cfm = KE_MSG_ALLOC(RDTSS_VAL_IND_CFM,
                                                          rdtss_env->operation->src_id, rdtss_env->operation->dest_id,
                                                          rdtss_val_ind_cfm);
            cfm->handle = app_req->handle;
            cfm->status = GAP_ERR_NO_ERROR;
            ke_msg_send(cfm);
        }
        ke_free(rdtss_env->operation);
        rdtss_env->operation = NULL;
        ke_state_set(prf_src_task_get(&(rdtss_env->prf_env), 0), RDTSS_IDLE);
    }
}

/** 
 * @brief Handles reception of the @ref GATTC_CMP_EVT message.
 * @details The GATTC_CMP_EVT message that signals the completion of a GATTC_NOTIFY
 *          operation is sent back as soon as the notification PDU has been sent over
 *          the air.
 *          The GATTC_CMP_EVT message that signals the completion of a GATTC_INDICATE
 *          operation is sent back as soon as the ATT_HANDLE_VALUE_CONFIRMATION PDU is
 *          received confirming that the indication has been correctly received by the
 *          peer device.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not. 
 */
static int gattc_cmp_evt_handler(ke_msg_id_t const msgid,
                                 struct gattc_cmp_evt const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);

    if (param->operation == GATTC_NOTIFY || param->operation == GATTC_INDICATE)
    {
        rdtss_exe_operation();
    }

    return (KE_MSG_CONSUMED);
}

/** 
 * @brief Handles reception of the @ref RDTSS_VAL_SET_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not. 
 */
static int rdtss_val_set_req_handler(ke_msg_id_t const msgid,
                                      struct rdtss_val_set_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);
    // Update value in DB
    attm_att_set_value(rdtss_env->shdl + param->handle, param->length, 0, (uint8_t *)&param->value);
    
    return (KE_MSG_CONSUMED);
}

/** 
 * @brief Handles reception of the @ref RDTSS_VAL_NTF_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not. 
 */
static int rdtss_val_ntf_req_handler(ke_msg_id_t const msgid,
                                      struct rdtss_val_ntf_ind_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);
    uint16_t ccc_hdl;
    uint16_t handle = rdtss_env->shdl + param->handle;
    uint8_t ccc_idx;
    uint8_t status;

    uint8_t state = ke_state_get(dest_id);
    
    if (state == RDTSS_BUSY)
    {
        return KE_MSG_SAVED;
    }
    
    // Find the handle of the Characteristic Client Configuration
    ccc_hdl = get_cfg_handle(handle);
    ASSERT_ERR(ccc_hdl);

    // Convert handle to index
    status = rdtss_get_att_idx(ccc_hdl, &ccc_idx);
    ASSERT_ERR(status == ATT_ERR_NO_ERROR);
    if (status != ATT_ERR_NO_ERROR)
    {
        return KE_MSG_SAVED;
    }

    ke_state_set(dest_id, RDTSS_BUSY);
    rdtss_env->operation = ke_param2msg(param);
    rdtss_env->cursor = param->conidx;
    rdtss_env->ccc_idx = ccc_idx;
    
    // Trigger notification
    if (param->conidx == RDTSS_NOTIFY_ALL_CONN)
    {
        rdtss_env->cursor = 0;
        rdtss_exe_operation();
    }
    else
    {
        rdtss_exe_operation();
        rdtss_env->cursor = BLE_CONNECTION_MAX;
    }

    return KE_MSG_NO_FREE;
}

/** 
 * @brief Handles reception of the @ref RDTSS_VAL_IND_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not. 
 */
static int rdtss_val_ind_req_handler(ke_msg_id_t const msgid,
                                      struct rdtss_val_ind_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);
    uint16_t ccc_hdl;
    uint16_t handle = rdtss_env->shdl + param->handle;
    uint8_t ccc_idx;
    uint8_t status;

    uint8_t state = ke_state_get(dest_id);
    if (state == RDTSS_BUSY)
    {
        return KE_MSG_SAVED;
    }

    // Find the handle of the Characteristic Client Configuration
    ccc_hdl = get_cfg_handle(handle);
    ASSERT_ERR(ccc_hdl);

    // Convert handle to index
    status = rdtss_get_att_idx(ccc_hdl, &ccc_idx);
    ASSERT_ERR(status == ATT_ERR_NO_ERROR);
    if (status != ATT_ERR_NO_ERROR)
    {
        return KE_MSG_SAVED;
    }

    ke_state_set(dest_id, RDTSS_BUSY);
    rdtss_env->operation = ke_param2msg(param);
    rdtss_env->cursor = 0;
    rdtss_env->ccc_idx = ccc_idx;

    // Trigger indication
    rdtss_exe_operation();

    return KE_MSG_NO_FREE;
}

/** 
 * @brief Handles reception of the @ref RDTSS_ATT_INFO_RSP message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance (probably unused).
 * @return If the message shall be consumed or not. 
 */
static int rdtss_att_info_rsp_handler(ke_msg_id_t const msgid,
                                       struct rdtss_att_info_rsp const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);

    if (state == RDTSS_IDLE)
    {
        // Extract the service start handle.
        struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);
        // Allocate the attribute information confirmation message.
        struct gattc_att_info_cfm *cfm;
        cfm = KE_MSG_ALLOC(GATTC_ATT_INFO_CFM, TASK_GATTC, dest_id, gattc_att_info_cfm);
        // Fill the message.
        cfm->handle = rdtss_env->shdl + param->att_idx;
        cfm->length = param->length;
        cfm->status = param->status;
        // Send the confirmation message to GATTC.
        ke_msg_send(cfm);
    }

    return (KE_MSG_CONSUMED);
}

/** 
 * @brief Handles reception of the @ref GATTC_READ_REQ_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not. 
 */
static int gattc_read_req_ind_handler(ke_msg_id_t const msgid, struct gattc_read_req_ind const *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    ke_state_t state = ke_state_get(dest_id);

    if(state == RDTSS_IDLE)
    {
        struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);
        struct gattc_read_cfm * cfm;
        uint8_t att_idx = 0;
        uint8_t conidx = KE_IDX_GET(src_id);
        // retrieve handle information
        
        uint8_t status = rdtss_get_att_idx(param->handle, &att_idx);
        
        uint16_t length = 0;
        uint16_t ccc_val = 0;

        // If the attribute has been found, status is GAP_ERR_NO_ERROR
        if (status == GAP_ERR_NO_ERROR)
        {
            if( PERM_GET(rdtss_env->db_cfg.att_tbl[att_idx].perm, UUID_LEN) == PERM_UUID_16 &&
                (rdtss_env->db_cfg.att_tbl[att_idx].uuid[0] + (rdtss_env->db_cfg.att_tbl[att_idx].uuid[1]<<8) ) == ATT_DESC_CLIENT_CHAR_CFG
            )
            {
                ccc_val = rdtss_get_ccc_value(conidx, att_idx);
                length = 2;
            }
            else
            {
                // Request value from application
                struct rdtss_value_req_ind* req_ind = KE_MSG_ALLOC(RDTSS_VALUE_REQ_IND,
                                                        prf_dst_task_get(&(rdtss_env->prf_env), KE_IDX_GET(src_id)),
                                                        dest_id,
                                                        rdtss_value_req_ind);

                req_ind->conidx  = KE_IDX_GET(src_id);
                req_ind->att_idx = att_idx;

                // Send message to application
                ke_msg_send(req_ind);

                // Put Service in a busy state
                ke_state_set(dest_id, RDTSS_BUSY);

                return (KE_MSG_CONSUMED);
            }
        }

        // Send read response
        cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM,
                               src_id,
                               dest_id,
                               gattc_read_cfm,
                               length);

        cfm->handle = param->handle;
        cfm->status = status;
        cfm->length = length;

        if (status == GAP_ERR_NO_ERROR)
        {
            memcpy(cfm->value, &ccc_val, length);
        }

        ke_msg_send(cfm);

        return (KE_MSG_CONSUMED);
    }
    // Postpone request if profile is in a busy state
    else
    {
        return (KE_MSG_SAVED);
    }
}

/** 
 * @brief Handles reception of the @ref GATTC_WRITE_REQ_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not. 
 */
static int gattc_write_req_ind_handler(ke_msg_id_t const msgid, const struct gattc_write_req_ind *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);
    struct gattc_write_cfm * cfm;
    uint8_t att_idx = 0;
    uint8_t conidx = KE_IDX_GET(src_id);
    // retrieve handle information
    uint8_t status = rdtss_get_att_idx(param->handle, &att_idx);

    uint16_t perm;

    ASSERT_ERR(param->offset == 0);
    
    // If the attribute has been found, status is ATT_ERR_NO_ERROR
    if (status == ATT_ERR_NO_ERROR)
    {
        if( PERM_GET(rdtss_env->db_cfg.att_tbl[att_idx].perm, UUID_LEN) == PERM_UUID_16 &&
            (rdtss_env->db_cfg.att_tbl[att_idx].uuid[0] + (rdtss_env->db_cfg.att_tbl[att_idx].uuid[1]<<8) ) == ATT_DESC_CLIENT_CHAR_CFG
        )
        {            
            struct attm_elmt elem = {0};

            // Find the handle of the Characteristic Value
            uint16_t value_hdl = get_value_handle(param->handle);
            ASSERT_ERR(value_hdl);

            // Get permissions to identify if it is NTF or IND.    
            attmdb_att_get_permission(value_hdl, &perm, PERM_MASK_ALL, 0, &elem);
            
            status = check_client_char_cfg(PERM_IS_SET(perm, NTF, ENABLE), param);

            if (status == ATT_ERR_NO_ERROR)
            {
                rdtss_set_ccc_value(conidx, att_idx, *(uint16_t *)param->value);
            }
        }


        if (status == ATT_ERR_NO_ERROR)
        {
            // Inform APP
            struct rdtss_val_write_ind *req_id = KE_MSG_ALLOC_DYN(RDTSS_VAL_WRITE_IND,
                                                    prf_dst_task_get(&(rdtss_env->prf_env), KE_IDX_GET(src_id)),
                                                    dest_id, rdtss_val_write_ind,
                                                    param->length);
            memcpy(req_id->value, param->value, param->length);
            req_id->conidx = conidx;
            req_id->handle = att_idx;
            req_id->length = param->length;

            ke_msg_send(req_id);
        }
    }

    //Send write response
    cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
    cfm->handle = param->handle;
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/** 
 * @brief Handles reception of the attribute info request message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance (probably unused).
 * @return If the message shall be consumed or not. 
 */
static int gattc_att_info_req_ind_handler(ke_msg_id_t const msgid,
                                          struct gattc_att_info_req_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);

    if (state == RDTSS_IDLE)
    {
        // Extract the service start handle.
        struct rdtss_env_tag *rdtss_env = PRF_ENV_GET(RDTSS, rdtss);
        // Allocate the attribute information request message to be processed by
        // the application.
        struct rdtss_att_info_req *req = KE_MSG_ALLOC(RDTSS_ATT_INFO_REQ,
                                                       TASK_APP,
                                                       dest_id,
                                                       rdtss_att_info_req);
        // Fill the message.
        req->conidx  = KE_IDX_GET(src_id);
        ASSERT_ERR((req->conidx) < BLE_CONNECTION_MAX);
        req->att_idx = param->handle - rdtss_env->shdl;
        // Send the message to the application.
        ke_msg_send(req);
    }

    return (KE_MSG_CONSUMED);
}

/** 
 * @brief Handles reception of the @ref RDTSS_VALUE_REQ_RSP message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance (probably unused).
 * @return If the message shall be consumed or not. 
 */
static int rdtss_value_req_rsp_handler(ke_msg_id_t const msgid,
                                        struct rdtss_value_req_rsp *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    ke_state_t state = ke_state_get(dest_id);

    if(state == RDTSS_BUSY)
    {
        if (param->status == ATT_ERR_NO_ERROR)
        {
            // Send value to peer device.
            struct gattc_read_cfm* cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM,
                                                          KE_BUILD_ID(TASK_GATTC, param->conidx),
                                                          dest_id,
                                                          gattc_read_cfm,
                                                          param->length);

            // Fill parameters
            cfm->handle = rdtss_get_att_handle(param->att_idx);
            
            cfm->status = ATT_ERR_NO_ERROR;
            cfm->length = param->length;
            memcpy(cfm->value, param->value, param->length);

            // Send message to GATTC
            ke_msg_send(cfm);
        }
        else
        {
            // Application error, value provided by application is not the expected one
            struct gattc_read_cfm* cfm = KE_MSG_ALLOC(GATTC_READ_CFM,
                                                      KE_BUILD_ID(TASK_GATTC, param->conidx),
                                                      dest_id,
                                                      gattc_read_cfm);

            // Fill parameters
            cfm->handle = rdtss_get_att_handle(param->att_idx);
            cfm->status = ATT_ERR_APP_ERROR;

            // Send message to GATTC
            ke_msg_send(cfm);
        }

        // Return to idle state
        ke_state_set(dest_id, RDTSS_IDLE);
    }
    return (KE_MSG_CONSUMED);
}

/* Public variables ---------------------------------------------------------*/

/// Default State handlers definition
KE_MSG_HANDLER_TAB(rdtss)
{
    {GATTC_READ_REQ_IND,            (ke_msg_func_t)gattc_read_req_ind_handler},
    {GATTC_WRITE_REQ_IND,           (ke_msg_func_t)gattc_write_req_ind_handler},
    {GATTC_ATT_INFO_REQ_IND,        (ke_msg_func_t)gattc_att_info_req_ind_handler},
    {GATTC_CMP_EVT,                 (ke_msg_func_t)gattc_cmp_evt_handler},
    {RDTSS_VAL_NTF_REQ,             (ke_msg_func_t)rdtss_val_ntf_req_handler},
    {RDTSS_VAL_SET_REQ,             (ke_msg_func_t)rdtss_val_set_req_handler},
    {RDTSS_VAL_IND_REQ,             (ke_msg_func_t)rdtss_val_ind_req_handler},
    {RDTSS_ATT_INFO_RSP,            (ke_msg_func_t)rdtss_att_info_rsp_handler},
    {RDTSS_VALUE_REQ_RSP,           (ke_msg_func_t)rdtss_value_req_rsp_handler},
};

void rdtss_task_init(struct ke_task_desc *p_task_desc)
{
    // Get the address of the environment
    struct rdtss_env_tag *p_rdtss_env = PRF_ENV_GET(RDTSS, rdtss);

    p_task_desc->msg_handler_tab = rdtss_msg_handler_tab;
    p_task_desc->msg_cnt         = ARRAY_LEN(rdtss_msg_handler_tab);
    p_task_desc->state           = p_rdtss_env->state;
    p_task_desc->idx_max         = RDTSS_IDX_MAX;
}

#endif // BLE_RDTSS_SERVER
