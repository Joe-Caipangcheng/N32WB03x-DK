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
 * @file app_task.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** 
 * @addtogroup APPTASK
 * @{ 
 */

/* Includes ------------------------------------------------------------------*/
#include "app_task.h"
#include "co_utils.h"
#include "gapm_task.h" 
#include "gapc_task.h" 
#include "ahi_task.h"
#include "co_hci.h"
#include "hci.h"
#include "stdio.h"
#include "attm.h"
#include "atts.h"
#include "gapm_int.h"
#include "llm_int.h"
#include "prf.h"
#include "bsp_led.h"
#include "app.h"
#include "ke_timer.h"
#include "global_var.h"
#include "mesh_gatt.h"



uint8_t m_actv_idx;
uint8_t m_scan_actv_idx;
static uint8_t m_con_idx = 0;

/** 
 * @brief Handles GAP manager command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid, struct gapm_cmp_evt const *p_param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
		switch(p_param->operation)
		{
			case GAPM_RESET:{
			}break;
			case GAPM_SET_DEV_CONFIG:{
				prf_init(RWIP_1ST_RST); 
			}break;            
			case GAPM_PROFILE_TASK_ADD:{
			}break;
			case GAPM_CREATE_ADV_ACTIVITY:{
			}break;	  
			case GAPM_SET_ADV_DATA:{
			}break;
			case GAPM_SET_SCAN_RSP_DATA:{
			}break;	
			case GAPM_START_ACTIVITY:{
			}break;            
			case GAPM_STOP_ACTIVITY:{

			}break;   
			case GAPM_DELETE_ALL_ACTIVITIES:{
			}break;     

			case GAPM_CREATE_SCAN_ACTIVITY:{

			}break;
			case GAPM_DELETE_ACTIVITY:{

			}break;            
            
		}
	
	
    return KE_MSG_CONSUMED;
}
/** 
 * @brief GAPC message complete event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gapc_cmp_evt_handler(ke_msg_id_t const msgid,struct gapc_cmp_evt const *p_param, ke_task_id_t const dest_id,ke_task_id_t const src_id)
{
    switch(p_param->operation)
    {
        case GAPC_UPDATE_PARAMS:{
            }break;
        case GAPC_SET_LE_PKT_SIZE:{	
            }break;

        default:
        break;
    }
    return KE_MSG_CONSUMED;
}
/** 
 * @brief Get application profile message handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static uint8_t app_get_handler(const struct app_subtask_handlers *handler_list_desc, ke_msg_id_t msgid, void const *p_param, ke_task_id_t src_id)
{
    uint8_t counter;
    for (counter = handler_list_desc->msg_cnt; 0 < counter; counter--)
    {
        struct ke_msg_handler handler = (*(handler_list_desc->p_msg_handler_tab + counter - 1));
        
        if ((handler.id == msgid) || (handler.id == KE_MSG_DEFAULT_HANDLER))
        {
            return (uint8_t)(handler.func(msgid, p_param, TASK_APP, src_id));
        }
    }
    return KE_MSG_CONSUMED;
}
/** 
 * @brief Handles application message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int app_msg_default_handler(ke_msg_id_t const msgid,void *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id,enum ke_msg_status_tag *msg_ret)
{
	ke_task_id_t src_task_id = MSG_T(msgid);
	uint8_t msg_pol = KE_MSG_CONSUMED;
	switch (src_task_id)
	{	
		case TASK_ID_MESH_GATT:{
            app_get_handler(&mesh_gatt_app_handlers, msgid, p_param, src_id);
		} break;	
		
	}
  return msg_pol;
}

/** 
 * @brief Handles connection parameter update event from the GAP
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
int app_timer_conn_param_handler(ke_msg_id_t const msgid, void const *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id)
{
    struct app_conn_param conn_param;
    conn_param.intv_min = 150;
    conn_param.intv_max = 200;
    conn_param.latency = 0;
    conn_param.time_out = 500; 
    app_update_param(m_con_idx, &conn_param);
    return KE_MSG_CONSUMED;
}
/** 
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gapc_connection_req_ind_handler(ke_msg_id_t const msgid, struct gapc_connection_req_ind const *p_param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
	struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM,KE_BUILD_ID(TASK_GAPC, KE_IDX_GET(src_id)), TASK_APP, gapc_connection_cfm);    
    cfm->pairing_lvl = GAP_AUTH_REQ_NO_MITM_NO_BOND; 
    ke_msg_send(cfm);
	struct mesh_gatt_connection_req_ind *p_cmd = KE_MSG_ALLOC(MESH_GATT_CONNECT, TASK_APP, dest_id, mesh_gatt_connection_req_ind);	
	p_cmd->conhdl = p_param->conhdl;
    p_cmd->con_index = m_con_idx = KE_IDX_GET(src_id);
	ke_msg_send(p_cmd);	   

    return KE_MSG_CONSUMED;
}
/** 
 * @brief Handles disconnection complete event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid, struct gapc_disconnect_ind const *p_param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    prov_proxy_adv_delete();
	struct gapc_disconnect_ind *p_cmd = KE_MSG_ALLOC(MESH_GATT_DISCONNECT, TASK_APP, dest_id, gapc_disconnect_ind);	
	p_cmd->conhdl = p_param->conhdl;
	ke_msg_send(p_cmd);	 
    return KE_MSG_CONSUMED;
}
/** 
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gapc_param_update_req_ind_handler(ke_msg_id_t const msgid,struct gapc_param_update_req_ind const *p_param, ke_task_id_t const dest_id,ke_task_id_t const src_id)
{
    struct gapc_param_update_cfm *cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM,KE_BUILD_ID(TASK_GAPC, KE_IDX_GET(src_id)), TASK_APP,gapc_param_update_cfm);
    cfm->accept = true;
    cfm->ce_len_min = 0xFFFF;
    cfm->ce_len_max = 0xFFFF;
    ke_msg_send(cfm);  
    
    if (mesh_is_device_provisioned()){
        struct app_conn_param conn_param;
        conn_param.intv_min = 100;
        conn_param.intv_max = 150;
        conn_param.latency = 0;
        conn_param.time_out = 500; 
        app_update_param(m_con_idx, &conn_param);
    }
	return KE_MSG_CONSUMED;
}
/** 
 * @brief Handles 
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gapc_param_updated_ind_handler(ke_msg_id_t const msgid, struct gapc_param_updated_ind const *p_param, ke_task_id_t const dest_id,ke_task_id_t const src_id)
{
    return KE_MSG_CONSUMED;
}
/** 
 * @brief Handles GAPM_ACTIVITY_STOPPED_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gapm_activity_stopped_ind_handler(ke_msg_id_t const msgid, struct gapm_activity_stopped_ind const *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id)
{
    if(p_param->actv_type == GAPM_ACTV_TYPE_SCAN)
    {
    } else if(p_param->actv_type == GAPM_ACTV_TYPE_ADV)  {
        broadcast_stop();
    }  
    return KE_MSG_CONSUMED;
}
/** 
 * @brief Handles GATTC_MTU_CHANGED_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gattc_mtu_changed_ind_handler(ke_msg_id_t const msgid, struct  gattc_mtu_changed_ind const *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id)
{
    return KE_MSG_CONSUMED;
}
/** 
 * @brief Handles GAPC_LE_PKT_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gapc_le_pkt_size_ind_handler(ke_msg_id_t const msgid, struct  gapc_le_pkt_size_ind const *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id)
{
    return KE_MSG_CONSUMED;
}
/** 
 * @brief Handles GAPM_ACTIVITY_CREATED_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gapm_activity_created_ind_handler(ke_msg_id_t const msgid,struct gapm_activity_created_ind const *p_param, ke_task_id_t const dest_id,ke_task_id_t const src_id)
{
    if(p_param->actv_type == GAPM_ACTV_TYPE_ADV)
    {
        m_actv_idx = p_param->actv_idx;   
    }
    else if(p_param->actv_type == GAPM_ACTV_TYPE_SCAN)
    {
        m_scan_actv_idx = p_param->actv_idx;  
    }
    return KE_MSG_CONSUMED;
}

/** 
 * @brief Handles GAPM_ACTIVITY_STOPPED_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int appm_gapm_ext_adv_repoer_ind_handler(ke_msg_id_t const msgid, struct gapm_ext_adv_report_ind const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if(param->info == (GAPM_REPORT_TYPE_ADV_LEG | GAPM_REPORT_INFO_COMPLETE_BIT)){    
        mesh_rx_metadata_scanner_t metadata;
        metadata.adv_type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
        metadata.adv_addr.addr[0] = param->trans_addr.addr.addr[0];
        metadata.adv_addr.addr[1] = param->trans_addr.addr.addr[1];
        metadata.adv_addr.addr[2] = param->trans_addr.addr.addr[2];
        metadata.adv_addr.addr[3] = param->trans_addr.addr.addr[3];
        metadata.adv_addr.addr[4] = param->trans_addr.addr.addr[4];
        metadata.adv_addr.addr[5] = param->trans_addr.addr.addr[5];
        scanner_handle_end_event(&metadata, (uint8_t *)param->data,param->length);
    }
    return (KE_MSG_CONSUMED);
}



extern int app_bearer_event_handler(ke_msg_id_t const msgid, void const *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id);
extern int flash_evt_handler(ke_msg_id_t const msgid, void const *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id);
extern int flash_end_evt_handler(ke_msg_id_t const msgid, void const *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id);
extern int gatt_reset_handler(ke_msg_id_t const msgid, void const *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id);
extern int app_timer_wdt_handler(ke_msg_id_t const msgid, void const *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id);

/* Default State handlers definition. */
KE_MSG_HANDLER_TAB(app)
{
    {KE_MSG_DEFAULT_HANDLER,         (ke_msg_func_t)app_msg_default_handler},
    {GAPM_CMP_EVT,                   (ke_msg_func_t)gapm_cmp_evt_handler},
    {GAPC_CMP_EVT,                   (ke_msg_func_t)gapc_cmp_evt_handler},
    {GAPC_CONNECTION_REQ_IND,        (ke_msg_func_t)gapc_connection_req_ind_handler},
    {GAPC_DISCONNECT_IND,            (ke_msg_func_t)gapc_disconnect_ind_handler},
    {GAPC_PARAM_UPDATE_REQ_IND,      (ke_msg_func_t)gapc_param_update_req_ind_handler},
    {GAPM_ACTIVITY_CREATED_IND,      (ke_msg_func_t)gapm_activity_created_ind_handler},
    {GAPM_ACTIVITY_STOPPED_IND,      (ke_msg_func_t)gapm_activity_stopped_ind_handler},       
    {GATTC_MTU_CHANGED_IND,          (ke_msg_func_t)gattc_mtu_changed_ind_handler},
    {GAPC_LE_PKT_SIZE_IND,           (ke_msg_func_t)gapc_le_pkt_size_ind_handler},
    {GAPC_PARAM_UPDATED_IND,         (ke_msg_func_t)gapc_param_updated_ind_handler},
    {GAPM_EXT_ADV_REPORT_IND,        (ke_msg_func_t)appm_gapm_ext_adv_repoer_ind_handler},
    {TIMER_CONN_PARAM_MSG,               (ke_msg_func_t)app_timer_conn_param_handler},
    

    
};

/* Defines the place holder for the states of all the task instances. */
ke_state_t app_state[1];

// Application task descriptor
const struct ke_task_desc TASK_DESC_APP_M = {app_msg_handler_tab, app_state, 1, ARRAY_LEN(app_msg_handler_tab)};








