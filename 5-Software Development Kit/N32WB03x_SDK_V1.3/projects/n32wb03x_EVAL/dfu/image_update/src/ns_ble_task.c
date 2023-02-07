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
 * @file ns_ble_task.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"          // SW configuration

#if (BLE_APP_PRESENT)

#include "ns_ble_task.h"             // Application Manager Task API
#include "ns_ble.h"                  // Application Manager Definition
#include "gapm_task.h"            // GAP Manager Task API
#include <string.h>
#include "co_utils.h"
#include "ke_timer.h"             // Kernel timer

#if (BLE_APP_SEC)
#include "ns_sec.h"              // Security Module Definition
#endif //(BLE_APP_SEC)

#if (BLE_APP_HT)
#include "app_ht.h"               // Health Thermometer Module Definition
#include "htpt_task.h"
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
#include "app_dis.h"              // Device Information Module Definition
#include "diss_task.h"
#endif //(BLE_APP_DIS)

#if (BLE_APP_BATT)
#include "app_batt.h"             // Battery Module Definition
#include "bass_task.h"
#endif //(BLE_APP_BATT)

#if (BLE_APP_HID)
#include "app_hid.h"              // HID Module Definition
#include "hogpd_task.h"
#endif //(BLE_APP_HID)


#if (BLE_RDTSS_SERVER)
#include "app_rdtss.h"              // RDTSS Application Definitions
#endif //(BLE_RDTSS_SERVER)

#if (BLE_APP_NS_IUS)
#include "app_ns_ius.h"              // NS IUS Application Definitions
#include "ns_dfu_ble.h"
#endif //(BLE_RDTSS_SERVER)


#include "ahi_task.h"
#include "co_hci.h"
#include "hci.h"
#include "stdio.h"
#include "attm.h"
#include "atts.h"

#include "ns_delay.h"
#include "gapm_int.h"
#include "llm_int.h"
#include "ns_sleep.h"

#include "prf.h"
#if (NS_TIMER_ENABLE)
#include "ns_timer.h"
#endif //NS_TIMER_ENABLE



/*
 * LOCAL FUNCTION DEFINITIONS
 **/
 
void user_exe_operation_t(void)
{
  
} 
 
static int appm_gattc_cmp_evt_handler(ke_msg_id_t const msgid, struct gattc_cmp_evt const *gattc_cmp_evt,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{    
    switch(gattc_cmp_evt->operation){
    
        case GATTC_MTU_EXCH:{
                    if(app_env.manual_mtu_update){
                        app_env.manual_mtu_update = 0;
                        ns_dfu_ble_handler_mtu_update(gattc_cmp_evt->status);
                    }          
                    
                    
                    
        }
                break;    
    
    } 

    return (KE_MSG_CONSUMED);
}



static uint8_t app_get_handler(const struct app_subtask_handlers *handler_list_desc,
                               ke_msg_id_t msgid,
                               void *p_param,
                               ke_task_id_t src_id)
{
    // Counter
    uint8_t counter;
    // Get the message handler function by parsing the message table
    for (counter = handler_list_desc->msg_cnt; 0 < counter; counter--)
    {
        struct ke_msg_handler handler
                = /*(struct ke_msg_handler)*/(*(handler_list_desc->p_msg_handler_tab + counter - 1));
        
        if ((handler.id == msgid) ||
            (handler.id == KE_MSG_DEFAULT_HANDLER))
        {
            // If handler is NULL, message should not have been received in this state
            ASSERT_ERR(handler.func);

            return (uint8_t)(handler.func(msgid, p_param, TASK_APP, src_id));
        }
    }

    // If we are here no handler has been found, drop the message
    return (KE_MSG_CONSUMED);
}


/*
 * MESSAGE HANDLERS
 **/
#if (BLE_APP_PRF)
/**
 * @brief Handles GAPM_ACTIVITY_CREATED_IND event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 **/
static int gapm_activity_created_ind_handler(ke_msg_id_t const msgid,
                                             struct gapm_activity_created_ind const *p_param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    if (app_env.adv_state == APP_ADV_STATE_CREATING)
    {
        // Store the advertising activity index
        app_env.adv_actv_idx = p_param->actv_idx;
    }

    return (KE_MSG_CONSUMED);
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
 **/
static int gapm_activity_stopped_ind_handler(ke_msg_id_t const msgid,
                                             struct gapm_activity_stopped_ind const *p_param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    if (app_env.adv_state == APP_ADV_STATE_STARTED)
    {
        // Act as if activity had been stopped by the application
        app_env.adv_state = APP_ADV_STATE_STOPPING;

        // Perform next operation
        app_adv_fsm_next();
    }

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_APP_PRF)

/**
 * @brief Handles GAPM_PROFILE_ADDED_IND event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 **/
static int gapm_profile_added_ind_handler(ke_msg_id_t const msgid,
                                          struct gapm_profile_added_ind *p_param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
  
    // Current State
    ke_state_t state = ke_state_get(dest_id);
    
    if (state == APP_CREATE_DB)
    {
        switch (p_param->prf_task_id)
        {


            default: /* Nothing to do */ break;
        }
    }
    else
    {
        ASSERT_INFO(0, state, src_id);
    }

    return (KE_MSG_CONSUMED);
}

/**
 * @brief Handles GAP manager command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 **/
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *p_param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    #if (NVDS_SUPPORT)
    uint8_t key_len = KEY_LEN;
    #endif //(NVDS_SUPPORT)


    switch(p_param->operation)
    {

        // Reset completed
        case (GAPM_RESET):
        {
            if(p_param->status == GAP_ERR_NO_ERROR)
            {
                //reset prf_init
                prf_init(RWIP_1ST_RST);

                #if (NVDS_SUPPORT)
                nvds_tag_len_t len;
                #endif //(NVDS_SUPPORT)
                #if (BLE_APP_HID)
                app_hid_start_mouse();
                #endif //(BLE_APP_HID)

                // Set Device configuration
                struct gapm_set_dev_config_cmd* p_cmd = KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD,
                                                                   TASK_GAPM, TASK_APP,
                                                                   gapm_set_dev_config_cmd);
                // Set the operation
                p_cmd->operation = GAPM_SET_DEV_CONFIG;
                #if (BLE_APP_PRF)
                // Set the device role - Peripheral
                p_cmd->role = GAP_ROLE_PERIPHERAL;
                #endif //(BLE_APP_PRF)

                p_cmd->pairing_mode = GAPM_PAIRING_LEGACY;
                
                // Set Data length parameters
                p_cmd->sugg_max_tx_octets = BLE_MIN_OCTETS; 
                p_cmd->max_mtu = 517; //app_env.rsp_max_mtu;
                p_cmd->sugg_max_tx_time   = BLE_MIN_TIME;
                
                #if (BLE_APP_HID)
                // Enable Slave Preferred Connection Parameters present
                p_cmd->att_cfg = 0;
                SETF(p_cmd->att_cfg, GAPM_ATT_SLV_PREF_CON_PAR_EN, 1);
                #endif //(BLE_APP_HID)

                // Host privacy enabled by default
                p_cmd->privacy_cfg = 0;

                #if (NVDS_SUPPORT)
                if (nvds_get(NVDS_TAG_BD_ADDRESS, &len, &p_cmd->addr.addr[0]) == NVDS_OK)
                {
                    // Check if address is a static random address
                    if (p_cmd->addr.addr[5] & 0xC0)
                    {
                        // Host privacy enabled by default
                        p_cmd->privacy_cfg |= GAPM_PRIV_CFG_PRIV_ADDR_BIT;
                    }
                }
                #endif //(NVDS_SUPPORT)

                #if (NVDS_SUPPORT)
                #if (BLE_APP_PRF)
                if ((app_sec_get_bond_status()==true) &&
                    (nvds_get(NVDS_TAG_LOC_IRK, &key_len, app_env.loc_irk) == NVDS_OK))
                {
                    memcpy(p_cmd->irk.key, app_env.loc_irk, 16);
                }
                else

                #endif // (BLE_APP_PRF)
                #endif //(NVDS_SUPPORT)
                {
                    memset((void *)&p_cmd->irk.key[0], 0x00, KEY_LEN);
                }

                
                // Send message
                ke_msg_send(p_cmd);
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
        break;

        case (GAPM_PROFILE_TASK_ADD):
        {                                    
            #if (BLE_APP_PRF)
//            if (app_sec_get_bond_status()==true) 
//            {
//                #if (NVDS_SUPPORT)
//                // If Bonded retrieve the local IRK from NVDS
//                if (nvds_get(NVDS_TAG_LOC_IRK, &key_len, app_env.loc_irk) == NVDS_OK)
//                {
//                    // Set the IRK in the GAP
//                    struct gapm_set_irk_cmd *p_cmd = KE_MSG_ALLOC(GAPM_SET_IRK_CMD,
//                                                                TASK_GAPM, TASK_APP,
//                                                                gapm_set_irk_cmd);
//                    ///  - GAPM_SET_IRK: 
//                    p_cmd->operation = GAPM_SET_IRK;
//                    memcpy(&p_cmd->irk.key[0], &app_env.loc_irk[0], KEY_LEN);
//                    ke_msg_send(p_cmd);
//                }
//                else
//                #endif //(NVDS_SUPPORT)
//                {
//                    // If cannot read IRK from NVDS ASSERT
//                    ASSERT_ERR(0);
//                }
//            }
//            else // Need to start the generation of new IRK
            {
                struct gapm_gen_rand_nb_cmd *p_cmd = KE_MSG_ALLOC(GAPM_GEN_RAND_NB_CMD,
                                                                TASK_GAPM, TASK_APP,
                                                                gapm_gen_rand_nb_cmd);
                p_cmd->operation   = GAPM_GEN_RAND_NB;
                app_env.rand_cnt = 1;
                ke_msg_send(p_cmd);
            }

            #endif // (BLE_APP_PRF)
        }
        break;

        #if (BLE_APP_PRF)
        case (GAPM_GEN_RAND_NB) :
        {
            if (app_env.rand_cnt == 1)
            {
                // Generate a second random number
                app_env.rand_cnt++;
                struct gapm_gen_rand_nb_cmd *p_cmd = KE_MSG_ALLOC(GAPM_GEN_RAND_NB_CMD,
                                                                TASK_GAPM, TASK_APP,
                                                                gapm_gen_rand_nb_cmd);
                p_cmd->operation = GAPM_GEN_RAND_NB;
                ke_msg_send(p_cmd);
            }
            else
            {
                struct gapm_set_irk_cmd *p_cmd = KE_MSG_ALLOC(GAPM_SET_IRK_CMD,
                                                        TASK_GAPM, TASK_APP,
                                                        gapm_set_irk_cmd);
                app_env.rand_cnt=0;
                ///  - GAPM_SET_IRK
                p_cmd->operation = GAPM_SET_IRK;
                memcpy(&p_cmd->irk.key[0], &app_env.loc_irk[0], KEY_LEN);
                ke_msg_send(p_cmd);
            }
        }
        break;
        #endif //(BLE_APP_PRF)

        #if (BLE_APP_PRF)
        case (GAPM_SET_IRK):
        {
            // ASSERT_INFO(p_param->status == GAP_ERR_NO_ERROR, p_param->operation, p_param->status);
            // If not Bonded already store the generated value in NVDS
//            if (app_sec_get_bond_status()==false)
//            {
//                #if (NVDS_SUPPORT)
//                if (nvds_put(NVDS_TAG_LOC_IRK, KEY_LEN, (uint8_t *)&app_env.loc_irk) != NVDS_OK)
//                #endif //(NVDS_SUPPORT)
//                {
//                    ASSERT_INFO(0, 0, 0);
//                }
//            }

            app_env.rand_cnt = 0;
            // Add the next requested service
            if (!app_add_svc())
            {
                // Go to the ready state
                ke_state_set(TASK_APP, APP_READY);

                // No more service to add, start advertising
                app_update_adv_state(true);
            }
        }
        break;
        #endif //(BLE_APP_PRF)

        // Device Configuration updated
        case (GAPM_SET_DEV_CONFIG):
        {
            prf_init(RWIP_1ST_RST);
            ASSERT_INFO(p_param->status == GAP_ERR_NO_ERROR, p_param->operation, p_param->status);

            // Go to the create db state
            ke_state_set(TASK_APP, APP_CREATE_DB);

            // Add the first required service in the database
            // and wait for the PROFILE_ADDED_IND
            app_add_svc();
        }
        break;

        #if (BLE_APP_PRF)
        case (GAPM_CREATE_ADV_ACTIVITY):
        case (GAPM_STOP_ACTIVITY):
        case (GAPM_START_ACTIVITY):
        case (GAPM_DELETE_ACTIVITY):
        case (GAPM_SET_ADV_DATA):
        case (GAPM_SET_SCAN_RSP_DATA):
        {
            // Sanity checks
            ASSERT_INFO(app_env.adv_op == p_param->operation, app_env.adv_op, p_param->operation);
            ASSERT_INFO(p_param->status == GAP_ERR_NO_ERROR, p_param->status, app_env.adv_op);

            
            // Perform next operation
            app_adv_fsm_next();

        } break;

        case (GAPM_DELETE_ALL_ACTIVITIES) :
        {
            // Re-Invoke Advertising
            app_env.adv_state = APP_ADV_STATE_IDLE;
        } break;
        #endif //(BLE_APP_PRF)

        default:
        {
            // Drop the message
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}

static int gapc_get_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_get_dev_info_req_ind const *p_param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    
   switch(p_param->req)
    {
        case GAPC_DEV_NAME:
        {
            struct gapc_get_dev_info_cfm * cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM,
                                                    src_id, dest_id,
                                                    gapc_get_dev_info_cfm, APP_DEVICE_NAME_MAX_LEN);
            cfm->req = p_param->req;
            cfm->info.name.length = app_get_dev_name(cfm->info.name.value);

            // Send message
            ke_msg_send(cfm);
        } break;

        case GAPC_DEV_APPEARANCE:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                                             src_id, dest_id,
                                                             gapc_get_dev_info_cfm);
            cfm->req = p_param->req;
            // Set the device appearance
            #if (BLE_APP_HT)
            // Generic Thermometer 
            cfm->info.appearance = 728;
            #elif (BLE_APP_HID)
            // HID Mouse
            cfm->info.appearance = 962;
            #else
            // No appearance
            cfm->info.appearance = 0;
            #endif

            // Send message
            ke_msg_send(cfm);
        } break;

        case GAPC_DEV_SLV_PREF_PARAMS:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                    src_id, dest_id,
                                                            gapc_get_dev_info_cfm);
            cfm->req = p_param->req;
            // Slave preferred Connection interval Min
            cfm->info.slv_pref_params.con_intv_min = 8;
            // Slave preferred Connection interval Max
            cfm->info.slv_pref_params.con_intv_max = 10;
            // Slave preferred Connection latency
            cfm->info.slv_pref_params.slave_latency  = 0;
            // Slave preferred Link supervision timeout
            cfm->info.slv_pref_params.conn_timeout    = 200;  // 2s (500*10ms)

            // Send message
            ke_msg_send(cfm);
        } break;

        default: /* Do Nothing */ break;
    }

    return (KE_MSG_CONSUMED);
}

#if (BLE_APP_PRF)
/**
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 **/
static int gapc_set_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_set_dev_info_req_ind const *p_param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{

    // Set Device configuration
    struct gapc_set_dev_info_cfm* cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM, src_id, dest_id,
                                                     gapc_set_dev_info_cfm);

        // Reject to change parameters
    cfm->status = GAP_ERR_REJECTED;
    cfm->req = p_param->req;
    // Send message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

#endif //(BLE_APP_PRF)

/**
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 **/
static int gapc_connection_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_connection_req_ind const *p_param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{

    app_env.conidx = KE_IDX_GET(src_id);

    // Check if the received connection index is valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        // Allocate connection confirmation
        struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM,
                KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                gapc_connection_cfm);

        // Store received connection handle
        app_env.conhdl = p_param->conhdl;

//        #if(BLE_APP_SEC)
//        cfm->pairing_lvl      = app_sec_get_bond_status() ? GAP_AUTH_REQ_NO_MITM_BOND : GAP_AUTH_REQ_NO_MITM_NO_BOND;   //TODO  TODOOOOOO 2021_01_28
//        #else // !(BLE_APP_SEC)
//        cfm->auth      = app_env.cfm_auth;
//        #endif // (BLE_APP_SEC)

        cfm->pairing_lvl      = GAP_AUTH_REQ_NO_MITM_NO_BOND;
        // Send the message
        ke_msg_send(cfm);

        /*--------------------------------------------------------------
         * ENABLE REQUIRED PROFILES
         *--------------------------------------------------------------*/

        #if (BLE_APP_BATT)
        // Enable Battery Service
        app_batt_enable_prf(app_env.conidx);
        #endif //(BLE_APP_BATT)

        #if (BLE_APP_HID)
        // Enable HID Service
        app_hid_enable_prf(app_env.conidx);
        #endif //(BLE_APP_HID)

        // We are now in connected State
        ke_state_set(dest_id, APP_CONNECTED);

    }
    else
    {
        #if (BLE_APP_PRF)
        // No connection has been established, restart advertising
        app_update_adv_state(true);

        #endif //(BLE_APP_PRF)
    }


    return (KE_MSG_CONSUMED);
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
 **/
static int gapc_param_update_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_param_update_req_ind const *p_param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    app_env.conidx = KE_IDX_GET(src_id);

    // Check if the received Connection Handle was valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        // Send connection confirmation
        struct gapc_param_update_cfm *cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM,
                KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                gapc_param_update_cfm);

        cfm->accept = true;
        cfm->ce_len_min = 2;
        cfm->ce_len_max = 4;

        // Send message
        ke_msg_send(cfm);
    }
    else
    {
#if (BLE_APP_PRF)
        // No connection has been established, restart advertising
        app_update_adv_state(true);
#endif // (BLE_APP_PRF)
    }

    return (KE_MSG_CONSUMED);
}

#if (BLE_APP_PRF)
/**
 * @brief Handles GAP controller command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 **/
static int gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *p_param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
  
    
    switch(p_param->operation)
    {
        case (GAPC_UPDATE_PARAMS):
        {
                    
                    if(app_env.manual_conn_param_update){
                        app_env.manual_conn_param_update = 0;
                        ns_dfu_ble_handler_conn_param_update(p_param->status);
                    }        
                    
            if (p_param->status != GAP_ERR_NO_ERROR)
            {

                            // it's application specific what to do when the Param Upd request is rejected
                            //TODO


            }
        }
        case (GAPC_SET_LE_PKT_SIZE):
        {
            if (p_param->status != GAP_ERR_NO_ERROR)
            {

            }
        }
        case (GAPC_SECURITY_REQ):
        {
            if (p_param->status == GAP_ERR_NO_ERROR)
            {
                
            }
            else
            {

            }
            
        }
        
        
        break;

        default:
        {
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_APP_PRF)

/**
 * @brief Handles disconnection complete event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 **/
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                      struct gapc_disconnect_ind const *p_param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
  
    // Go to the ready state
    ke_state_set(TASK_APP, APP_READY);


    #if (BLE_APP_PRF)
    #if (BLE_APP_HT)
    // Stop interval timer
    app_stop_timer();
    #endif //(BLE_APP_HT)

    #if (BLE_ISO_MODE_0_PROTOCOL)
    app_env.adv_state = APP_ADV_STATE_CREATING;
    #endif //(BLE_ISO_MODE_0_PROTOCOL)

    #if (!BLE_APP_HID)
    // Restart Advertising
    app_update_adv_state(true);
    #endif //(!BLE_APP_HID)


    #endif //(BLE_APP_PRF)
    
    #if (BLE_RDTSS_SERVER)
        //disable usart receive
        
    #endif //(BLE_RDTSS_SERVER)
    // Restart Advertising
    app_update_adv_state(true);        


    return (KE_MSG_CONSUMED);
}


/**
 * @brief Handles reception of all messages sent from the lower layers to the application
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 **/

extern struct ke_task_desc TASK_DESC_APP_DIS;
extern const struct app_subtask_handlers app_sec_handlers;

static int app_msg_handler(ke_msg_id_t const msgid,
                            void *p_param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id,
                            enum ke_msg_status_tag *msg_ret)
{
  
    // Retrieve identifier of the task from received message
    ke_task_id_t src_task_id = MSG_T(msgid);
    // Message policy
    uint8_t msg_pol = KE_MSG_CONSUMED;
    
    switch (src_task_id)
    {
        case (TASK_ID_GAPC):
        {
            #if (BLE_APP_SEC)
            if ((msgid >= GAPC_BOND_CMD) &&
                (msgid <= GAPC_SECURITY_IND))
            {
                // Call the Security Module
                msg_pol = app_get_handler(&app_sec_handlers, msgid, p_param, src_id);
            }
            #endif //(BLE_APP_SEC)
            // else drop the message
        } break;

        case (TASK_ID_GATTC):
        {
            // Service Changed - Drop
        } break;

        #if (BLE_APP_HT)
        case (TASK_ID_HTPT):
        {
            // Call the Health Thermometer Module
            msg_pol = app_get_handler(&app_ht_handlers, msgid, p_param, src_id);
        } break;
        #endif //(BLE_APP_HT)

        #if (BLE_APP_DIS)
        case (TASK_ID_DISS):
        {
            // Call the Device Information Module
            msg_pol = app_get_handler(&app_dis_handlers, msgid, p_param, src_id);
        } break;
        #endif //(BLE_APP_DIS)

        #if (BLE_APP_HID)
        case (TASK_ID_HOGPD):
        {
            // Call the HID Module
            msg_pol = app_get_handler(&app_hid_handlers, msgid, p_param, src_id);
        } break;
        #endif //(BLE_APP_HID)


        #if (BLE_APP_BATT)
        case (TASK_ID_BASS):
        {
            // Call the Battery Module
            msg_pol = app_get_handler(&app_batt_handlers, msgid, p_param, src_id);
        } break;
        #endif //(BLE_APP_BATT)

        
        #if(BLE_RDTSS_SERVER)
        case (TASK_ID_RDTSS):
        {
            // Call the rdtss Module
            msg_pol = app_get_handler(&app_rdtss_handlers, msgid, p_param, src_id);
        } break;
        #endif //(TASK_ID_RDTSS)

        #if(BLE_APP_NS_IUS)
        case (TASK_ID_NS_IUS):
        {
            msg_pol = app_get_handler(&ns_ius_app_handlers, msgid, p_param, src_id);
        } break;
        #endif //(BLE_APP_NS_IUS)

        default:
        {
            #if (BLE_APP_HT)
            if (msgid == APP_HT_MEAS_INTV_TIMER)
            {
                msg_pol = app_get_handler(&TASK_DESC_APP_HT, msgid, p_param, src_id);
            }
            #endif //(BLE_APP_HT)

            #if (BLE_APP_HID)
            if (msgid == APP_HID_MOUSE_TIMEOUT_TIMER)
            {
                msg_pol = app_get_handler(&app_hid_handlers, msgid, p_param, src_id);
            }
            #endif //(BLE_APP_HID)


        } break;
    }

    return (msg_pol);
}

#if (BLE_APP_PRF)
/**
 * @brief Handles reception of random number generated message
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 **/
static int gapm_gen_rand_nb_ind_handler(ke_msg_id_t const msgid, struct gapm_gen_rand_nb_ind *p_param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
  
    if (app_env.rand_cnt==1)      // First part of IRK
    {
        memcpy(&app_env.loc_irk[0], &p_param->randnb.nb[0], 8);
    }
    else if (app_env.rand_cnt==2) // Second part of IRK
    {
        memcpy(&app_env.loc_irk[8], &p_param->randnb.nb[0], 8);
    }
    return (KE_MSG_CONSUMED);
}

#endif //(BLE_APP_PRF)

__STATIC int gapc_param_updated_ind_handler(ke_msg_id_t const msgid,
                             struct gapc_param_updated_ind *p_param,
                                         ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{

    return (KE_MSG_CONSUMED);
}

__STATIC int gapc_le_pkt_size_ind_handler(ke_msg_id_t const msgid,
                             struct gapc_le_pkt_size_ind *p_param,
                                       ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{

    return (KE_MSG_CONSUMED);
}

__STATIC int gattc_mtu_changed_ind_handler(ke_msg_id_t const msgid,
                             struct gattc_mtu_changed_ind *p_param,
                                        ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{

    return (KE_MSG_CONSUMED);
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
 **/
static int app_conn_params_update_evt_handler(ke_msg_id_t const msgid, void *p_param,//struct app_adv_timer_evt_t *p_param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    
      // Prepare the GAPC_PARAM_UPDATE_CMD message
    struct gapc_param_update_cmd *p_cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                                                     KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                     gapc_param_update_cmd);
        
    p_cmd->operation  = GAPC_UPDATE_PARAMS;
    p_cmd->intv_min   = MSECS_TO_UNIT(MIN_CONN_INTERVAL, MSECS_UNIT_1_25_MS);
    p_cmd->intv_max   = MSECS_TO_UNIT(MAX_CONN_INTERVAL, MSECS_UNIT_1_25_MS);
    p_cmd->latency    = SLAVE_LATENCY;
    p_cmd->time_out   = MSECS_TO_UNIT(CONN_SUP_TIMEOUT, MSECS_UNIT_10_MS);
        
    // not used by a slave device
    p_cmd->ce_len_min = 0xFFFF;
    p_cmd->ce_len_max = 0xFFFF;

    // Send the message
    ke_msg_send(p_cmd);

    return (KE_MSG_CONSUMED);
}


int app_entry_point_handler(ke_msg_id_t const msgid,
                            void const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    enum ke_msg_status_tag process_msg_handling_result;

    //add others user handler
    //TODO  


#if (NS_TIMER_ENABLE)
    ns_timer_api_process_handler(msgid, param, dest_id, src_id,&process_msg_handling_result);
    if(process_msg_handling_result == KE_MSG_CONSUMED)
         return (KE_MSG_CONSUMED);
#endif  //NS_TIMER_ENABLE
    app_msg_handler(msgid, (void *)param, dest_id, src_id, &process_msg_handling_result);
    if(process_msg_handling_result == KE_MSG_CONSUMED)
         return (KE_MSG_CONSUMED);
    
    return (KE_MSG_CONSUMED);
}



/**
 * @brief Function called when the APP_HID_DATA_PROCESS_EVT expires.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 **/
static int app_hid_data_process_handler(ke_msg_id_t const msgid,
                                                                             void const *param,
                                                                             ke_task_id_t const dest_id,
                                                                             ke_task_id_t const src_id)
{
    

    // Relaunch the timer
    ke_timer_set(APP_HID_DATA_PROCESS_EVT, TASK_APP, 1000);


    return (KE_MSG_CONSUMED);
}


/*
 * GLOBAL VARIABLES DEFINITION
 **/
extern int app_dfu_ble_reset_handler(ke_msg_id_t const msgid, void const *p_param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
/* Default State handlers definition. */
KE_MSG_HANDLER_TAB(app)
{

    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,    (ke_msg_func_t)app_entry_point_handler},


    // GAPM messages
    #if (BLE_APP_PRF)
    {GAPM_ACTIVITY_CREATED_IND, (ke_msg_func_t)gapm_activity_created_ind_handler},
    {GAPM_ACTIVITY_STOPPED_IND, (ke_msg_func_t)gapm_activity_stopped_ind_handler},
    {GAPM_GEN_RAND_NB_IND,      (ke_msg_func_t)gapm_gen_rand_nb_ind_handler},
    #endif //(BLE_APP_PRF)
    {GAPM_CMP_EVT,              (ke_msg_func_t)gapm_cmp_evt_handler},
    {GAPM_PROFILE_ADDED_IND,    (ke_msg_func_t)gapm_profile_added_ind_handler},

    // GATTC messages
    {GATTC_MTU_CHANGED_IND,     (ke_msg_func_t)gattc_mtu_changed_ind_handler},

    // GAPC messages
    {GAPC_CONNECTION_REQ_IND,   (ke_msg_func_t)gapc_connection_req_ind_handler},
    {GAPC_PARAM_UPDATE_REQ_IND, (ke_msg_func_t)gapc_param_update_req_ind_handler},
    {GAPC_PARAM_UPDATED_IND,    (ke_msg_func_t)gapc_param_updated_ind_handler},
    {GAPC_LE_PKT_SIZE_IND,      (ke_msg_func_t)gapc_le_pkt_size_ind_handler},
    {GAPC_DISCONNECT_IND,       (ke_msg_func_t)gapc_disconnect_ind_handler},
    {GAPC_GET_DEV_INFO_REQ_IND, (ke_msg_func_t)gapc_get_dev_info_req_ind_handler},    
        
        
    #if (BLE_APP_PRF)
    {GAPC_SET_DEV_INFO_REQ_IND, (ke_msg_func_t)gapc_set_dev_info_req_ind_handler},
    {GAPC_CMP_EVT,              (ke_msg_func_t)gapc_cmp_evt_handler},

    #endif //(BLE_APP_PRF)
   
    {GATTC_CMP_EVT,             (ke_msg_func_t)appm_gattc_cmp_evt_handler},
        
    //user event
    {APP_PARAMS_UPDATE_EVT,     (ke_msg_func_t)app_conn_params_update_evt_handler},
    {APP_HID_DATA_PROCESS_EVT,  (ke_msg_func_t)app_hid_data_process_handler},
        
        
    {APP_DFU_BLE_RESET_TIMER,            (ke_msg_func_t)app_dfu_ble_reset_handler},        
        
};

/* Defines the place holder for the states of all the task instances. */
ke_state_t app_state[APP_IDX_MAX];

// Application task descriptor
const struct ke_task_desc TASK_DESC_APP_M = {app_msg_handler_tab, app_state, APP_IDX_MAX, ARRAY_LEN(app_msg_handler_tab)};

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
