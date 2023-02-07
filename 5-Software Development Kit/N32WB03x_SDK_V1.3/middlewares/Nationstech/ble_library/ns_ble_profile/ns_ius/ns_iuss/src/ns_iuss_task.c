/**
 ****************************************************************************************
 *
 * @file ns_iuss_task.c
 *
 * @brief Custom Service profile source file.
 *
 *
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"     // SW configuration

#if (BLE_APP_NS_IUS)
#include "ns_iuss_task.h"
#include "ns_iuss.h"
#include "app_ns_ius.h"


#include "prf_utils.h"
#include "prf.h"
#include "attm_db.h"
#include "ke_mem.h"
#include "att.h"
#include "co_utils.h"
#include "attm.h"
#include "ke_task.h"
#include "gapc.h"
#include "gapc_task.h"
#include "gattc_task.h"
#include "attm_db.h"
#include "ns_log.h"


ke_task_id_t  ns_ble_ius_task;



/* Private functions ---------------------------------------------------------*/




/** 
 * @brief Stores characteristic value.
 * @param[in] att_idx  Custom attribut index.
 * @param[in] length   Value length.
 * @param[in] data     Pointer to value data.
 * @return 0 on success. 
 */
static int ns_ius_att_set_value(uint8_t att_idx, uint16_t length, const uint8_t *data)
{
    struct ns_ius_env_tag *ns_ius_env = PRF_ENV_GET(NS_IUS, ns_ius);
    struct ns_ius_val_elmt *val = (struct ns_ius_val_elmt *) co_list_pick(&(ns_ius_env->values));
    while (val != NULL)
    {
        if (val->att_idx == att_idx)
        {
            if (length != val->length)
            {
								co_list_extract(&ns_ius_env->values, &val->hdr);
                ke_free(val);
                val = NULL;
            }
            break;
        }
        val = (struct ns_ius_val_elmt *)val->hdr.next;
    }

    if (val == NULL)
    {
        val = (struct ns_ius_val_elmt *) ke_malloc(sizeof(struct ns_ius_val_elmt) + length, KE_MEM_ATT_DB);
        co_list_push_back(&ns_ius_env->values, &val->hdr);
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
static int ns_ius_att_get_value(uint8_t att_idx, uint16_t *length, const uint8_t **data)
{
    struct ns_ius_env_tag *ns_ius_env = PRF_ENV_GET(NS_IUS, ns_ius);
    struct ns_ius_val_elmt *val = (struct ns_ius_val_elmt *) co_list_pick(&ns_ius_env->values);
	
    while (val != NULL)
    {
        if (val->att_idx == att_idx)
        {
            *length = val->length;
            *data = val->data;
            break;
        }

        val = (struct ns_ius_val_elmt *)val->hdr.next;
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
void ns_ius_init_ccc_values(const struct attm_desc_128 *att_db, int max_nb_att)
{
    uint8_t ccc_values[BLE_CONNECTION_MAX] = {0};
    for (int i = 1; i < max_nb_att; i++)
    {
			if( PERM_GET(att_db[i].perm, UUID_LEN) == PERM_UUID_16 && (att_db[i].uuid[0] + (att_db[i].uuid[1]<<8) ) == ATT_DESC_CLIENT_CHAR_CFG)
			{
				ns_ius_att_set_value(i, sizeof(ccc_values), ccc_values);
			}
	}
}
/** 
 * @brief Set value of CCC for given attribute and connection index.
 * @param[in] conidx   Connection index.
 * @param[in] att_idx  CCC attribute index.
 * @param[in] cc       Value to store. 
 */
void ns_ius_set_ccc_value(uint8_t conidx, uint8_t att_idx, uint16_t ccc)
{
    uint16_t length;
    const uint8_t *value;
    uint8_t new_value[BLE_CONNECTION_MAX];
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    ns_ius_att_get_value(att_idx, &length, &value);
    ASSERT_ERR(length);
    ASSERT_ERR(value);
    memcpy(new_value, value, length);
    new_value[conidx] = (uint8_t)ccc;
    ns_ius_att_set_value(att_idx, length, new_value);
}
/** 
 * @brief Read value of CCC for given attribute and connection index.
 * @param[in]  conidx   Connection index.
 * @param[in]  att_idx  Custom attribute index.
 * @return Value of CCC. 
 */
static uint16_t ns_ius_get_ccc_value(uint8_t conidx, uint8_t att_idx)
{
    uint16_t length;
    const uint8_t *value;
    uint16_t ccc_value;
    ns_ius_att_get_value(att_idx, &length, &value);
    ccc_value = value[conidx];
    return ccc_value;
}

static uint8_t ns_ius_get_att_idx(uint16_t handle, uint8_t *att_idx)
{
    struct ns_ius_env_tag *ns_ius_env = PRF_ENV_GET(NS_IUS, ns_ius);
    uint8_t status = PRF_APP_ERROR;

    if ((handle >= ns_ius_env->shdl) && (handle < ns_ius_env->shdl + ns_ius_env->max_nb_att))
    {
        *att_idx = handle - ns_ius_env->shdl;
        status = ATT_ERR_NO_ERROR;
    }

    return status;
}
static uint16_t get_value_handle(uint16_t cfg_handle)
{
    uint8_t uuid[ATT_UUID_128_LEN];
    uint8_t uuid_len;
    uint16_t handle = cfg_handle;
    struct attm_svc *srv;

    srv = attmdb_get_service(handle);

    while ((handle >= srv->svc.start_hdl) && (handle <= srv->svc.end_hdl))
    {
        struct attm_elmt elmt;
        attmdb_get_attribute(handle, &elmt);
        attmdb_get_uuid(&elmt, &uuid_len, uuid, false, false);
        if (*(uint16_t *)&uuid[0] == ATT_DECL_CHARACTERISTIC)
            return handle + 1;

        handle--;
    }

    return 0; 
}
static int check_client_char_cfg(bool is_notification, const struct gattc_write_req_ind *param)
{
    uint8_t status = GAP_ERR_NO_ERROR;
    uint16_t ntf_cfg = 0;

    if (param->length != sizeof(uint16_t))
    {
        status =  ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN;
    }
    else
    {
        ntf_cfg = *((uint16_t*)param->value);

        if (is_notification)
        {
            if ( (ntf_cfg != PRF_CLI_STOP_NTFIND) && (ntf_cfg != PRF_CLI_START_NTF) )
            {
                status =  PRF_ERR_INVALID_PARAM;
            }
        }
        else
        {
            if ( (ntf_cfg != PRF_CLI_STOP_NTFIND) && (ntf_cfg != PRF_CLI_START_IND) )
            {
                status =  PRF_ERR_INVALID_PARAM;
            }
        }
    }

    return status;
}

/** 
 * @brief Handles reception of the @ref GATTC_READ_REQ_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not. 
 */
static int gattc_read_req_ind_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
	const struct gattc_read_req_ind *p_param = (const struct gattc_read_req_ind *)param;
	
	ke_state_t state = ke_state_get(dest_id);

	if(state == NS_IUS_IDLE)
	{
			struct ns_ius_env_tag *ns_ius_env = PRF_ENV_GET(NS_IUS, ns_ius);
			struct gattc_read_cfm * cfm;
			uint8_t att_idx = 0;
			uint8_t conidx = KE_IDX_GET(src_id);
	
			uint8_t status = ns_ius_get_att_idx(p_param->handle, &att_idx);
			uint16_t length = 0;
			uint16_t ccc_val = 0;

			if (status == GAP_ERR_NO_ERROR)
			{
					if( PERM_GET(ns_ius_att_db[att_idx].perm, UUID_LEN) == PERM_UUID_16 && (ns_ius_att_db[att_idx].uuid[0] + (ns_ius_att_db[att_idx].uuid[1]<<8) ) == ATT_DESC_CLIENT_CHAR_CFG)
					{
							ccc_val = ns_ius_get_ccc_value(conidx, att_idx);
							length = 2;
					}
					else
					{
							struct ns_ius_value_req_ind* req_ind = KE_MSG_ALLOC(NS_IUS_VALUE_REQ_IND,
																											prf_dst_task_get(&(ns_ius_env->prf_env), KE_IDX_GET(src_id)),
																											dest_id,
																											ns_ius_value_req_ind);

							req_ind->conidx  = KE_IDX_GET(src_id);
							req_ind->att_idx = att_idx;
							ke_msg_send(req_ind);
							ke_state_set(dest_id, NS_IUS_BUSY);

							return KE_MSG_CONSUMED;
					}
			}
			cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM,
														 src_id,
														 dest_id,
														 gattc_read_cfm,
														 length);

			cfm->handle = p_param->handle;
			cfm->status = status;
			cfm->length = length;

			if (status == GAP_ERR_NO_ERROR)
			{
					memcpy(cfm->value, &ccc_val, length);
			}
			ke_msg_send(cfm);

			return KE_MSG_CONSUMED;
	}
	else
	{
			return KE_MSG_SAVED;
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
static int gattc_write_req_ind_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
	const struct gattc_write_req_ind *p_param = (const struct gattc_write_req_ind *)param;
	
	struct ns_ius_env_tag *ns_ius_env = PRF_ENV_GET(NS_IUS, ns_ius);
	uint8_t att_idx = 0;
	uint8_t conidx = KE_IDX_GET(src_id);
	uint8_t status = ns_ius_get_att_idx(p_param->handle, &att_idx);

	uint16_t perm;
	if (status == ATT_ERR_NO_ERROR)
	{
		if( PERM_GET(ns_ius_att_db[att_idx].perm, UUID_LEN) == PERM_UUID_16 &&
			(ns_ius_att_db[att_idx].uuid[0] + (ns_ius_att_db[att_idx].uuid[1]<<8) ) == ATT_DESC_CLIENT_CHAR_CFG)
		{			
				struct attm_elmt elem = {0};
				uint16_t value_hdl = get_value_handle(p_param->handle);
				attmdb_att_get_permission(value_hdl, &perm, PERM_MASK_ALL, 0, &elem);
				status = check_client_char_cfg(PERM_IS_SET(perm, NTF, ENABLE), p_param);

				if (status == ATT_ERR_NO_ERROR)
				{
						ns_ius_set_ccc_value(conidx, att_idx, *(uint16_t *)p_param->value);
				}
		}
		else
		{
		}

		if (status == ATT_ERR_NO_ERROR)
		{		
			
				struct ns_ius_val_write_ind *req_id = KE_MSG_ALLOC_DYN(NS_IUS_VAL_WRITE_IND,
																								prf_dst_task_get(&(ns_ius_env->prf_env), KE_IDX_GET(src_id)),
																								dest_id, ns_ius_val_write_ind,
																								p_param->length);
				memcpy(req_id->value, p_param->value, p_param->length);
				req_id->conidx = conidx;
				req_id->handle = att_idx;
				req_id->length = p_param->length;
				ke_msg_send(req_id);
		}
	}

	struct gattc_write_cfm * cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
	cfm->handle = p_param->handle;
	cfm->status = status;
	ke_msg_send(cfm);
	
	return KE_MSG_CONSUMED;
}			
/** 
 * @brief Handles reception of the @ref RDTS_ATT_INFO_RSP message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance (probably unused).
 * @return If the message shall be consumed or not. 
 */
static int gattc_att_info_req_ind_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
	const struct gattc_att_info_req_ind *p_param = (const struct gattc_att_info_req_ind *)param;
	uint8_t state = ke_state_get(dest_id);
	if (state == NS_IUS_IDLE)
	{
		struct ns_ius_env_tag *ns_ius_env = PRF_ENV_GET(NS_IUS, ns_ius);
		struct ns_ius_att_info_req *req = KE_MSG_ALLOC(NS_IUS_ATT_INFO_REQ,
																									 TASK_APP,
																									 dest_id,
																									 ns_ius_att_info_req);
		req->conidx  = KE_IDX_GET(src_id);
		req->att_idx = p_param->handle - ns_ius_env->shdl;
		ke_msg_send(req);
	}	
	return KE_MSG_CONSUMED;
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
static int gattc_cmp_evt_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
	struct gattc_cmp_evt const *p_param = (struct gattc_cmp_evt const *)param;
	
	
	if (p_param->operation == GATTC_NOTIFY)
	{
		ke_state_set(ns_ble_ius_task, NS_IUS_IDLE);	
	}	
	return KE_MSG_CONSUMED;
}	
/*
 * GLOBAL VARIABLE DEFINITIONS 
 */

/// Default State handlers definition
KE_MSG_HANDLER_TAB(ns_ius)
{
    {GATTC_READ_REQ_IND,            gattc_read_req_ind_handler},
    {GATTC_WRITE_REQ_IND,           gattc_write_req_ind_handler},
    {GATTC_ATT_INFO_REQ_IND,        gattc_att_info_req_ind_handler},
    {GATTC_CMP_EVT,                 gattc_cmp_evt_handler},
};


void ns_ius_task_init(struct ke_task_desc *p_task_desc)
{
    // Get the address of the environment
    struct ns_ius_env_tag *p_ns_ius_env = PRF_ENV_GET(NS_IUS, ns_ius);

    p_task_desc->msg_handler_tab = ns_ius_msg_handler_tab;
    p_task_desc->msg_cnt         = ARRAY_LEN(ns_ius_msg_handler_tab);
    p_task_desc->state           = p_ns_ius_env->state;
    p_task_desc->idx_max         = NS_IUS_IDX_MAX;
}




#endif  //BLE_APP_NS_IUS



