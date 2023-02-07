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
 * @version v1.0.3
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** 
 * @addtogroup APPTASK
 * @{ 
 */

/* Includes ------------------------------------------------------------------*/
#include "rwip_config.h"          // SW configuration

#if (BLE_APP_PRESENT)
#include "ns_ble_task.h"             // Application Manager Task API
#include "ns_ble.h"                  // Application Manager Definition
#if (BLE_APP_SEC)
#include "ns_sec.h"              // Security Module Definition
#endif //(BLE_APP_SEC)
#if (NS_TIMER_ENABLE)
#include "ns_timer.h"
#endif //NS_TIMER_ENABLE
#if (BLE_APP_NS_IUS)
#include "ns_dfu_ble.h"
#endif //BLE_APP_NS_IUS
#include "ns_sleep.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef CFG_PRF_GAP
#define CFG_PRF_GAP 1
#endif

#ifndef CFG_PRF_GATT
#define CFG_PRF_GATT 1
#endif

/* Private constants ---------------------------------------------------------*/
#if (CFG_PRF_GAP)
/// GAP Attribute database description
static const struct attm_desc gapm_att_db[GAP_IDX_NUMBER] =
{
    // GAP_IDX_PRIM_SVC - GAP service
    [GAP_IDX_PRIM_SVC]                 =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_CHAR_DEVNAME - device name declaration
    [GAP_IDX_CHAR_DEVNAME]             =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_DEVNAME - device name definition
    [GAP_IDX_DEVNAME]                  =   {ATT_CHAR_DEVICE_NAME, (PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE)), PERM(RI, ENABLE), GAP_MAX_NAME_SIZE},
    // GAP_IDX_CHAR_ICON - appearance declaration
    [GAP_IDX_CHAR_ICON]                =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_ICON -appearance
    [GAP_IDX_ICON]                     =   {ATT_CHAR_APPEARANCE, PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(uint16_t)},
    // GAP_IDX_CHAR_SLAVE_PREF_PARAM - Peripheral parameters declaration
    [GAP_IDX_CHAR_SLAVE_PREF_PARAM]    =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_SLAVE_PREF_PARAM - Peripheral parameters definition
    [GAP_IDX_SLAVE_PREF_PARAM]         =   {ATT_CHAR_PERIPH_PREF_CON_PARAM, PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(struct gap_slv_pref)},
    // GAP_IDX_CHAR_ADDR_RESOL - Central Address Resolution declaration
    [GAP_IDX_CHAR_CNT_ADDR_RESOL]      =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_ADDR_RESOL_SUPP - Central Address Resolution supported
    [GAP_IDX_CNT_ADDR_RESOL]           =   {ATT_CHAR_CTL_ADDR_RESOL_SUPP, PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(uint8_t)},
    // GAP_IDX_CHAR_RSLV_PRIV_ADDR_ONLY - Resolvable Private Address Only declaration
    [GAP_IDX_CHAR_RSLV_PRIV_ADDR_ONLY] =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_CHAR_RSLV_PRIV_ADDR_ONLY - Resolvable Private Address Only declaration supported
    [GAP_IDX_RSLV_PRIV_ADDR_ONLY]      =   {ATT_CHAR_RSLV_PRIV_ADDR_ONLY, PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(uint8_t)},
};
#endif
#if (CFG_PRF_GATT)
/// GATT Attribute database description
static const struct attm_desc gattm_att_db[GATT_IDX_NUMBER] =
{
    // GATT service
    [GATT_IDX_PRIM_SVC]          = { ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0 },

    // Service Changed
    [GATT_IDX_CHAR_SVC_CHANGED]  = { ATT_DECL_CHARACTERISTIC,  PERM(RD, ENABLE), 0, 0},
    [GATT_IDX_SVC_CHANGED]       = { ATT_CHAR_SERVICE_CHANGED, PERM(IND, ENABLE), PERM(RI, ENABLE), sizeof(uint16_t) *2},
    [GATT_IDX_SVC_CHANGED_CFG]   = { ATT_DESC_CLIENT_CHAR_CFG, (PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE)), 0, sizeof(uint16_t) },

    // Client Supported Features
    [GATT_IDX_CHAR_CLI_SUP_FEAT] = { ATT_DECL_CHARACTERISTIC,  PERM(RD, ENABLE), 0, 0 },
    [GATT_IDX_CLI_SUP_FEAT]      = { ATT_CHAR_CLI_SUP_FEAT,    (PERM(RD, ENABLE)| PERM(WRITE_REQ, ENABLE)), PERM(RI, ENABLE), ATT_MAX_CLI_FEAT_LEN },

    // Database Hash
    [GATT_IDX_CHAR_DB_HASH]      = { ATT_DECL_CHARACTERISTIC,  PERM(RD, ENABLE), 0, 0 },
    [GATT_IDX_DB_HASH]           = { ATT_CHAR_DB_HASH,         PERM(RD, ENABLE), PERM(RI, ENABLE), GAP_KEY_LEN },
};
#endif
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

static int gattc_cmp_evt_handler(ke_msg_id_t const msgid, struct gattc_cmp_evt const *gattc_cmp_evt,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{    
 
    NS_LOG_DEBUG("%s status = %x,  operation = %x\r\n",__func__,gattc_cmp_evt->status,gattc_cmp_evt->operation);
    if(app_env.ble_msg_handler)
    {
        struct ble_msg_t ble_msg = {APP_BLE_NULL_MSG,NULL,NULL};
        ble_msg.msg_id = APP_BLE_GATTC_CMP_EVT;
        ble_msg.msg.p_gattc_cmp = gattc_cmp_evt;
        app_env.ble_msg_handler((void const*)&ble_msg);
    }
    switch(gattc_cmp_evt->operation)
    {
        case (GATTC_MTU_EXCH):
        {
            #if (BLE_APP_NS_IUS)
            if(app_env.manual_mtu_update)
            {
                app_env.manual_mtu_update = 0;
                ns_dfu_ble_handler_mtu_update(gattc_cmp_evt->status);
            }
            #endif                
            if (gattc_cmp_evt->status != ATT_ERR_NO_ERROR)
            {
            }
        }
        break;
        case GATTC_NOTIFY:
            
        break;
        case GATTC_INDICATE:
            
        break;
        case (GATTC_DISC_ALL_SVC):
        {

        }break;
        
        case (GATTC_DISC_ALL_CHAR):
        {

        }break;
        
        case (GATTC_DISC_DESC_CHAR):
        {
            
        }break;

        case (GATTC_WRITE_NO_RESPONSE):
        {
            NS_LOG_DEBUG("GATTC_WRITE_NO_RESPONSE CMP\r\n");

        }break;
        
        case (GATTC_READ):
        {

        }break;
        default:
        {
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
    NS_LOG_DEBUG("%s,sid:%d\r\n",__func__,src_id);
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
 */
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
 */
static int gapm_activity_created_ind_handler(ke_msg_id_t const msgid,
                                             struct gapm_activity_created_ind const *p_param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    if(p_param->actv_type == GAPM_ACTV_TYPE_ADV)
    {
        if (app_env.adv_state == APP_ADV_STATE_CREATING)
        {
            // Store the advertising activity index
            app_env.adv_actv_idx = p_param->actv_idx;
        }
    }
    else if(p_param->actv_type == GAPM_ACTV_TYPE_SCAN)
    {
        app_env.scan_actv_idx = p_param->actv_idx;
    }
    else if(p_param->actv_type == GAPM_ACTV_TYPE_INIT)
    {
        app_env.init_actv_idx = p_param->actv_idx;
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
 */
static int gapm_activity_stopped_ind_handler(ke_msg_id_t const msgid,
                                             struct gapm_activity_stopped_ind const *p_param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    if(p_param->actv_type == GAPM_ACTV_TYPE_ADV)
    {
        NS_LOG_DEBUG("ADV STOP\r\n");
        if (app_env.adv_state == APP_ADV_STATE_STARTED)
        {
            // Act as if activity had been stopped by the application
            app_env.adv_state = APP_ADV_STATE_STOPPING;

            // Perform next operation
            ns_ble_adv_fsm_next();
        }
    }
    else if(p_param->actv_type == GAPM_ACTV_TYPE_SCAN)
    {
        NS_LOG_DEBUG("SCAN STOP:%d\r\n",app_env.scan_actv_idx);
        if( app_env.scan_actv_idx != 0xFF)
        {
            ns_ble_delete_scan();
            app_env.scan_actv_idx = 0xFF;
        }
    }
    else if(p_param->actv_type == GAPM_ACTV_TYPE_INIT)
    {
        NS_LOG_DEBUG("INIT STOP:%d\r\n",app_env.init_actv_idx );
        if( app_env.init_actv_idx != 0xFF)
        {
            ns_ble_delete_init();
            app_env.init_actv_idx = 0xFF;
        }
    }
    else
    {
        NS_LOG_DEBUG("OTHER STOP\r\n");
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
 */
static int gapm_profile_added_ind_handler(ke_msg_id_t const msgid,
                                          struct gapm_profile_added_ind *p_param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
  
    // Current State
    ke_state_t state = ke_state_get(dest_id);
    NS_LOG_DEBUG("%s,state %x\r\n", __func__,state);
    
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
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *p_param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    if(p_param->status != CO_ERROR_NO_ERROR)
    {
        NS_LOG_ERROR("%s, op = %x, status = %x\r\n", __func__, p_param->operation, p_param->status);
    }
    else{
        NS_LOG_DEBUG("%s, op = %x, status = %x\r\n", __func__, p_param->operation, p_param->status);
    }
    
    switch(p_param->operation)
    {

        // Reset completed
        case (GAPM_RESET):
        {
            if(p_param->status == GAP_ERR_NO_ERROR)
            {
                //enalbe lsi measurement
                if(app_env.lsc_cfg != BLE_LSC_LSE_32768HZ)   
                {
                    //start lis calib timer
                    ke_timer_set(APP_LSI_CALIB_EVT,TASK_APP,1);
                }
                
                //reset prf_init
                prf_init(RWIP_1ST_RST);

                // Set Device configuration
                struct gapm_set_dev_config_cmd* p_cmd = KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD,
                                                                   TASK_GAPM, TASK_APP,
                                                                   gapm_set_dev_config_cmd);
                // Set the operation
                p_cmd->operation = GAPM_SET_DEV_CONFIG;
                #if (BLE_APP_PRF)
                // Set the device role - Peripheral
                p_cmd->role = gap_env.dev_role;
                #endif //(BLE_APP_PRF)

                #if (BLE_APP_SEC_CON)
                // The Max MTU is increased to support the Public Key exchange
                // HOWEVER, with secure connections enabled you cannot sniff the 
                // LEAP and LEAS protocols
                p_cmd->max_mtu = app_env.rsp_max_mtu;
                p_cmd->pairing_mode = GAPM_PAIRING_SEC_CON | GAPM_PAIRING_LEGACY;
                #else // !(BLE_APP_SEC_CON)
                // Do not support secure connections
                p_cmd->pairing_mode = GAPM_PAIRING_LEGACY;
                #endif //(BLE_APP_SEC_CON)
                
                // Set Data length parameters
                p_cmd->sugg_max_tx_octets = 251; 
                p_cmd->max_mtu = 517; //app_env.rsp_max_mtu;
                p_cmd->sugg_max_tx_time   = 2120;
                
                p_cmd->att_cfg  = gap_env.att_cfg;

                // Host privacy enabled by default
                p_cmd->privacy_cfg = 0;

                memcpy(p_cmd->addr.addr,gap_env.mac_addr.addr,BD_ADDR_LEN);
                // Check if address is a static random address
                if (gap_env.mac_addr.addr[5] & 0xC0 && (!(gap_env.dev_role&GAP_ROLE_CENTRAL))) //central can't set PRIV
                {
                    // Host privacy enabled by default
                    p_cmd->privacy_cfg |= GAPM_PRIV_CFG_PRIV_ADDR_BIT;
                }
                p_cmd->privacy_cfg |= GAPM_PRIV_CFG_PRIV_EN_BIT;
                p_cmd->renew_dur = 900;
                if (gap_env.mac_addr_type == GAPM_GEN_RSLV_ADDR)
                {                    
                    memcpy(&p_cmd->irk.key[0], &app_env.loc_irk[0], KEY_LEN);
                }
                else
                {
                    memset((void *)&p_cmd->irk.key[0], 0x00, KEY_LEN);
                }
                if(app_env.ble_msg_handler)
                {
                    struct ble_msg_t ble_msg = {APP_BLE_NULL_MSG,NULL,NULL};
                    ble_msg.msg_id = APP_BLE_OS_READY;
                    ble_msg.msg.p_gapm_cmp = p_param;
                    ble_msg.cmd.p_dev_config = p_cmd;
                    app_env.ble_msg_handler((void const*)&ble_msg);
                }
                llhwc_modem_setmode(GAP_PHY_1MBPS);
                // Send message
                ke_msg_send(p_cmd);
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
        break;

        // Device Configuration updated
        case (GAPM_SET_DEV_CONFIG):
        {
            struct gap_ral_dev_info dev_info={0};
            #if (BLE_APP_SEC)  
            if (ns_sec_get_bond_status()==true)
            {
                ns_bond_last_bonded_ral_info(&dev_info);
                ns_ble_list_set_ral(&dev_info,1);
            }
            else
            #endif
            {
                //use device addr to set ral list as defauft 
                memcpy(dev_info.addr.addr.addr,&gap_env.mac_addr,GAP_BD_ADDR_LEN);
                dev_info.addr.addr_type = 0;
                memcpy(dev_info.peer_irk,&app_env.loc_irk[0],GAP_KEY_LEN);
                if (gap_env.mac_addr_type == GAPM_GEN_RSLV_ADDR)
                {
                   memcpy(dev_info.local_irk,&app_env.loc_irk[0],GAP_KEY_LEN);
                }
                else
                {
                    memset(dev_info.local_irk, 0, GAP_KEY_LEN);
                }
                ns_ble_list_set_ral(&dev_info,1);
            }
        }
            break;
        case (GAPM_SET_RAL):
        {
            if (!app_env.rand_cnt)
            {
                struct gapm_set_irk_cmd *p_cmd = KE_MSG_ALLOC(GAPM_SET_IRK_CMD,
                                                TASK_GAPM, TASK_APP,
                                                gapm_set_irk_cmd);
                ///  - GAPM_SET_IRK
                p_cmd->operation = GAPM_SET_IRK;
                if (gap_env.mac_addr_type == GAPM_GEN_RSLV_ADDR)
                {
                    memcpy(&p_cmd->irk.key[0],&app_env.loc_irk[0], KEY_LEN);
                }
                else
                {
                    memset(&p_cmd->irk.key[0], 0x00, KEY_LEN);
                }
                ke_msg_send(p_cmd);
            }
            app_env.rand_cnt = 0;
        } break;
        
        case (GAPM_SET_IRK):                
        {
            prf_init(RWIP_1ST_RST);
            ASSERT_INFO(p_param->status == GAP_ERR_NO_ERROR, p_param->operation, p_param->status);

            // Go to the create db state
            ke_state_set(TASK_APP, APP_CREATE_DB);
            
            uint8_t status = GAP_ERR_NO_ERROR;

            #if (CFG_PRF_GAP)
            /* initialize GAP Start Handle */
            gapm_env.svc_start_hdl = 0x1;
            uint32_t feat_gap = 0x001F;//| 0x0060 | 0x0180;
            
            if(GETF(gap_env.att_cfg, GAPM_ATT_SLV_PREF_CON_PAR_EN))
            {
                feat_gap |= 0x0060;
            }
            if (gap_env.dev_role & GAP_ROLE_CENTRAL)
            {
                //if (param->privacy_cfg & GAPM_PRIV_CFG_PRIV_EN_BIT)
                {
                    feat_gap |= 0x0180;
                }
            }
            // Enable presence of Resolvable private address only characteristic.
            if(GETF(gap_env.att_cfg, GAPM_ATT_RSLV_PRIV_ADDR_ONLY))
            {
                feat_gap |= 0x0600;
            }
                    
            /* Create GAP Attribute Database */
            status = attm_svc_create_db(&(gapm_env.svc_start_hdl), ATT_SVC_GENERIC_ACCESS, (uint8_t*) &feat_gap,
            GAP_IDX_NUMBER, NULL, TASK_GAPC, &gapm_att_db[0],
            PERM(SVC_MI, ENABLE));

            if(status != GAP_ERR_NO_ERROR)
            {
                break;
            }
            #endif // defined(CFG_PRF_GAP)

            #if (CFG_PRF_GATT)                        
            /* initialize GATT Start Handle */
            gattm_env.svc_start_hdl = 0x0;
            uint32_t feat_gatt = 0x00FF;
            
            /* Create GATT Attribute Database */
            status = attm_svc_create_db(&(gattm_env.svc_start_hdl), ATT_SVC_GENERIC_ATTRIBUTE, (uint8_t*) &feat_gatt, GATT_IDX_NUMBER,
                    NULL, TASK_GATTC, &gattm_att_db[0], PERM(SVC_MI, ENABLE));
            
            if(status != GAP_ERR_NO_ERROR)
            {
                break;
            }
            #endif // defined(CFG_PRF_GATT)
            (void)status;
            // Add the first required service in the database
            // and wait for the PROFILE_ADDED_IND
            ns_ble_add_svc();
        }
        break;    
        case (GAPM_PROFILE_TASK_ADD):
        #if (BLE_APP_PRF)
        {
            // ASSERT_INFO(p_param->status == GAP_ERR_NO_ERROR, p_param->operation, p_param->status);
            // If not Bonded already store the generated value in NVDS
            #if(BLE_APP_SEC)
            if (ns_sec_get_bond_status()==false)
            {
            }
            #endif
            app_env.rand_cnt = 0;
            // Add the next requested service
            if (!ns_ble_add_svc())
            {
                // Go to the ready state
                ke_state_set(TASK_APP, APP_READY);
                // No more service to add, start advertising or scan
                if(app_env.adv_mode == APP_ADV_MODE_ENABLE)
                {
                    ns_ble_adv_start();
                }
                else{
                    ns_ble_adv_stop();
                    
                    //avoid adv and scan create at the same time
                    if(scan_env.scan_enable)
                    {
                        ns_ble_start_scan();
                    }
                }
            }
        }
        #endif // (BLE_APP_PRF)
        break;                
  

        #if (BLE_APP_PRF)

        case (GAPM_START_ACTIVITY):
        case (GAPM_DELETE_ACTIVITY): 
        case (GAPM_CREATE_ADV_ACTIVITY):
        case (GAPM_SET_ADV_DATA):
        case (GAPM_SET_SCAN_RSP_DATA):
        case (GAPM_STOP_ACTIVITY):      
        {
            //handle adv operation only
            if( (app_env.current_op >= CURRENT_OP_CREATE_ADV) && 
                (app_env.current_op <= CURRENT_OP_DELETE_ADV))
            {
                // Perform next operation
                ns_ble_adv_fsm_next();
            }
            
            if(p_param->operation == GAPM_START_ACTIVITY ||
               p_param->operation == GAPM_DELETE_ACTIVITY)
            {
                //active advertising mode callback
                switch (app_env.current_op)
                {
                    case CURRENT_OP_START_ADV:
                    case CURRENT_OP_DELETE_ADV:
                        if(adv_env.ble_adv_msg_handler)
                        {
                            adv_env.ble_adv_msg_handler(app_env.adv_mode);
                        }
                        break;
                    case CURRENT_OP_START_SCAN:
                        if(scan_env.ble_scan_state_handler != NULL)
                        {
                            scan_env.ble_scan_state_handler(SCAN_STATE_SCANING);
                        }
                        break;
                    case CURRENT_OP_START_INIT:
                        if(scan_env.ble_scan_state_handler != NULL)
                        {
                            scan_env.ble_scan_state_handler(SCAN_STATE_CONNECTING);
                        }
                        break;
                    default:
                        break;
                }
            }
            
        } break;

        case (GAPM_DELETE_ALL_ACTIVITIES) :
        {
            // Re-Invoke Advertising
            app_env.adv_state = APP_ADV_STATE_IDLE;
            app_env.adv_actv_idx = 0xFF;
            app_env.scan_actv_idx = 0xFF;
            app_env.init_actv_idx = 0xFF;
        } break;
        #endif //(BLE_APP_PRF)

        case (GAPM_CREATE_SCAN_ACTIVITY):
        {
            if( (app_env.scan_actv_idx != 0xFF) && (p_param->status == CO_ERROR_NO_ERROR) )
            {
                if(scan_env.scan_enable)
                {
                    ns_ble_start_scan();
                }
            }
            else
            {
                ns_ble_delete_scan();
            }
        } break;
        
        case (GAPM_CREATE_INIT_ACTIVITY):
        {
            if( (app_env.init_actv_idx != 0xFF) && (p_param->status == CO_ERROR_NO_ERROR) )
            {
                if(app_env.target_found && scan_env.connect_enable )
                {
                    ns_ble_start_init(app_env.target_addr.addr, app_env.target_addr_type);
                }
            }
            else
            {
                ns_ble_delete_init();
            }
        } break;
        
        default:
        {
            // Drop the message
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}


/*
 * @brief Handles GAPC_GET_DEV_INFO_REQ_IND event.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
*/
static int gapc_get_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_get_dev_info_req_ind const *p_param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s, req:%d\r\n",__func__,p_param->req);
    switch(p_param->req)
    {
        case GAPC_DEV_NAME:
        {
            struct gapc_get_dev_info_cfm * cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM,
                                                    src_id, dest_id,
                                                    gapc_get_dev_info_cfm, APP_DEVICE_NAME_MAX_LEN);
            cfm->req = p_param->req;
            cfm->info.name.length = gap_env.dev_name_len;
            memcpy(cfm->info.name.value,gap_env.dev_name,gap_env.dev_name_len);

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
            // No appearance
            cfm->info.appearance = gap_env.appearance;
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
            cfm->info.slv_pref_params.con_intv_min  = gap_env.dev_conn_param.intv_min;
            // Slave preferred Connection interval Max
            cfm->info.slv_pref_params.con_intv_max  = gap_env.dev_conn_param.intv_max;
            // Slave preferred Connection latency
            cfm->info.slv_pref_params.slave_latency = gap_env.dev_conn_param.latency;
            // Slave preferred Link supervision timeout
            cfm->info.slv_pref_params.conn_timeout  = gap_env.dev_conn_param.time_out;

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
 */
static int gapc_set_dev_info_req_ind_handler(ke_msg_id_t const msgid,
                                            struct gapc_set_dev_info_req_ind const *p_param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{

    // Set Device configuration
    struct gapc_set_dev_info_cfm* cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM, src_id, dest_id,
                                                     gapc_set_dev_info_cfm);
    NS_LOG_DEBUG("%s\r\n",__func__);

    gap_env.dev_name_len = p_param->info.name.length;
    memcpy(gap_env.dev_name,p_param->info.name.value,p_param->info.name.length);

    cfm->status = GAP_ERR_NO_ERROR;
    cfm->req = p_param->req;
    // Send message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

#endif //(BLE_APP_PRF)

/** 
 * @brief Handles connection complete event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gapc_connection_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_connection_req_ind const *p_param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    NS_LOG_INFO("%s, sid:%d,conidx:%d\r\n",__func__,src_id,KE_IDX_GET(src_id));
    app_env.conidx = KE_IDX_GET(src_id);

    // Check if the received connection index is valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        // Allocate connection confirmation
        struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM,
                KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                gapc_connection_cfm);

        app_env.conn_env[app_env.conidx].conidx = app_env.conidx;
        app_env.conn_env[app_env.conidx].conhdl = p_param->conhdl;
        app_env.conn_env[app_env.conidx].peer_addr_type = p_param->peer_addr_type;
        memcpy(app_env.conn_env[app_env.conidx].peer_addr.addr, p_param->peer_addr.addr, BD_ADDR_LEN);
        if (p_param->role == ROLE_SLAVE)
        {
            app_env.conn_env[app_env.conidx].role = ROLE_SLAVE;
        }
        else if (p_param->role == ROLE_MASTER)
        {
            app_env.conn_env[app_env.conidx].role = ROLE_MASTER;                
        }
        // Store received connection handle
        app_env.conhdl = p_param->conhdl;
        app_env.peer_addr_type = p_param->peer_addr_type;
        memcpy(app_env.peer_addr.addr, p_param->peer_addr.addr, BD_ADDR_LEN);
        
        #if(BLE_APP_SEC)
        if(ns_sec_get_bond_status())
        {
            switch (ns_sec_get_iocap())
            {
                case GAP_IO_CAP_NO_INPUT_NO_OUTPUT:
                    cfm->pairing_lvl = GAP_PAIRING_BOND_UNAUTH;
                    break;
                case GAP_IO_CAP_DISPLAY_ONLY:
                case GAP_IO_CAP_DISPLAY_YES_NO:
                case GAP_IO_CAP_KB_ONLY:
                case GAP_IO_CAP_KB_DISPLAY:
                    cfm->pairing_lvl = GAP_PAIRING_BOND_AUTH;
                    break;
                default:
                    cfm->pairing_lvl = GAP_PAIRING_NO_BOND;
                    break;
            }
        }
        else
        {
            cfm->pairing_lvl = GAP_PAIRING_NO_BOND;
        }
        #else // !(BLE_APP_SEC)
        cfm->pairing_lvl = GAP_PAIRING_NO_BOND;
        #endif // (BLE_APP_SEC)
        if(app_env.ble_msg_handler)
        {
            struct ble_msg_t ble_msg = {APP_BLE_NULL_MSG,NULL,NULL};
            ble_msg.msg_id = APP_BLE_GAP_CONNECTED;
            ble_msg.msg.p_connection_ind = p_param;
            ble_msg.cmd.p_connection_cfm = cfm;
            app_env.ble_msg_handler((void const*)&ble_msg);
        }
        // Send the message
        ke_msg_send(cfm);

        /*--------------------------------------------------------------
         * ENABLE REQUIRED PROFILES
         *--------------------------------------------------------------*/

        // We are now in connected State
        ke_state_set(dest_id, APP_CONNECTED);
        
        //init MTU as 23 (user data len 20)
        app_env.max_mtu = 23;
        app_env.conn_env[app_env.conidx].max_mtu = 23;
        
        //actiive connection params update timer
        if(gap_env.conn_param_update_delay)
        {
            ke_timer_set(APP_PARAMS_UPDATE_EVT, TASK_APP, gap_env.conn_param_update_delay);
        }

    }
    else
    {
        #if (BLE_APP_PRF)
        // No connection has been established, restart advertising
        ns_ble_adv_start();

        #endif //(BLE_APP_PRF)
    }
    return (KE_MSG_CONSUMED);
}
/** 
 * @brief Handles connection parameters update require event from the GAP. 
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gapc_param_update_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_param_update_req_ind const *p_param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n", __func__);

    // Check if the received Connection Handle was valid
    if (KE_IDX_GET(src_id) != GAP_INVALID_CONIDX)
    {
        // Send connection confirmation
        struct gapc_param_update_cfm *cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM,
                KE_BUILD_ID(TASK_GAPC, KE_IDX_GET(src_id)), TASK_APP,
                gapc_param_update_cfm);
        //set default value
        cfm->accept = true;
        cfm->ce_len_min = 2;
        cfm->ce_len_max = 4; 
        if(app_env.ble_msg_handler)
        {
            //call user code
            struct ble_msg_t ble_msg = {APP_BLE_NULL_MSG,NULL,NULL};
            ble_msg.msg_id = APP_BLE_GAP_PARAMS_REQUEST;
            ble_msg.msg.p_param_req = p_param;
            ble_msg.cmd.p_param_cfm = cfm;
            app_env.ble_msg_handler((void const*)&ble_msg);
        }

        // Send message
        ke_msg_send(cfm);
    }
    else
    {
#if (BLE_APP_PRF)
        // No connection has been established, restart advertising
        ns_ble_adv_start();
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
 */
static int gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *p_param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s, operation =%x status = %x\r\n", __func__,p_param->operation, p_param->status);
    if(app_env.ble_msg_handler)
    {
        struct ble_msg_t ble_msg = {APP_BLE_NULL_MSG,NULL,NULL};
        ble_msg.msg_id = APP_BLE_GAP_CMP_EVT;
        ble_msg.msg.p_gapc_cmp = p_param;
        app_env.ble_msg_handler((void const*)&ble_msg);
    }
    
    switch(p_param->operation)
    {
        case (GAPC_UPDATE_PARAMS):
        {
            #if (BLE_APP_NS_IUS)
            if(app_env.manual_conn_param_update)
            {
                app_env.manual_conn_param_update = 0;
                ns_dfu_ble_handler_conn_param_update(p_param->status);
            }
            #endif
        }
        case (GAPC_SET_LE_PKT_SIZE):
        {
            if (p_param->status != GAP_ERR_NO_ERROR)
            {
            }
        }break;

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
        
        case (GAPC_GET_PEER_FEATURES):
        {

        } break;
        
        case (GAPC_GET_PEER_VERSION):
        {

        } break;

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
 */
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                      struct gapc_disconnect_ind const *p_param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
  
    NS_LOG_INFO("BLE Disconnected,reson:%x,conidx:%d \r\n",p_param->reason,KE_IDX_GET(src_id));    
    
    app_env.conn_env[KE_IDX_GET(src_id)].conidx = GAP_INVALID_CONIDX;    
    if(ns_ble_get_connection_num() == 0)
    {
        ke_state_set(TASK_APP, APP_READY);// Go to the ready state
    }
    if(app_env.ble_msg_handler)
    {
        struct ble_msg_t ble_msg = {APP_BLE_NULL_MSG,NULL,NULL};
        ble_msg.msg_id = APP_BLE_GAP_DISCONNECTED;
        ble_msg.msg.p_disconnect_ind = p_param;
        app_env.ble_msg_handler((void const*)&ble_msg);
    }
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
 */
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
    
    NS_LOG_DEBUG("%s, task = %x, msg = %x \r\n",__func__, src_task_id,MSG_I(msgid));
    
    //for ns sec
    if(src_task_id == TASK_ID_GAPC)
    {
        #if (BLE_APP_SEC)
        if ((msgid >= GAPC_BOND_CMD) &&
            (msgid <= GAPC_SECURITY_IND))
        {
            // Call the Security Module
            msg_pol = app_get_handler(&app_sec_handlers, msgid, p_param, src_id);
        }
        #endif //(BLE_APP_SEC)
    }
    else{
        //app prf sub task
        for(uint8_t id = 0; id < ble_prf_evn.prf_num;id++)
        {
            //find the prf task id
            if(src_task_id == ble_prf_evn.prf_task_list[id].prf_task_id )
            {
                //found prf
                msg_pol = app_get_handler(ble_prf_evn.prf_task_list[id].prf_task_handler, msgid, p_param, src_id);
            }
        }
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
 */
static int gapm_gen_rand_nb_ind_handler(ke_msg_id_t const msgid, struct gapm_gen_rand_nb_ind *p_param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
  
    NS_LOG_DEBUG("%s\r\n",__func__);
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


__STATIC int gapm_addr_solved_ind_handler(ke_msg_id_t const msgid, 
                                          struct gapm_addr_solved_ind *p_param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n",__func__);
    
        
//    NS_LOG_DEBUG("This mac solved:%02X %02X %02X %02X %02X %02X ",
//                                          p_param->addr.addr[0],
//                                          p_param->addr.addr[1],
//                                          p_param->addr.addr[2],
//                                          p_param->addr.addr[3],
//                                          p_param->addr.addr[4],
//                                          p_param->addr.addr[5]);
    
    
    return (KE_MSG_CONSUMED);
}
#endif //(BLE_APP_PRF)

__STATIC int gapc_param_updated_ind_handler(ke_msg_id_t const msgid,
                             struct gapc_param_updated_ind *p_param,
                                         ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s,%d,%d,%d\r\n",__func__,p_param->con_interval,p_param->con_latency,p_param->sup_to);
    if(app_env.ble_msg_handler)
    {
        struct ble_msg_t ble_msg = {APP_BLE_NULL_MSG,NULL,NULL};
        ble_msg.msg_id = APP_BLE_GAP_PARAMS_IND;
        ble_msg.msg.p_param_updated = p_param;
        app_env.ble_msg_handler((void const*)&ble_msg);
    }
    return (KE_MSG_CONSUMED);
}

__STATIC int gapc_le_pkt_size_ind_handler(ke_msg_id_t const msgid,
                             struct gapc_le_pkt_size_ind *p_param,
                                       ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n",__func__);
    app_env.tx_pkt_size = p_param->max_tx_octets;
    app_env.conn_env[KE_IDX_GET(src_id)].tx_pkt_size = p_param->max_tx_octets;    
    return (KE_MSG_CONSUMED);
}

__STATIC int gattc_mtu_changed_ind_handler(ke_msg_id_t const msgid,
                             struct gattc_mtu_changed_ind *p_param,
                                        ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{

    NS_LOG_DEBUG("%s,mtu:%x, num:%x \r\n",__func__, p_param->mtu,p_param->seq_num);
    app_env.max_mtu = p_param->mtu;
    app_env.conn_env[KE_IDX_GET(src_id)].max_mtu = p_param->mtu;    
  
    if(app_env.ble_msg_handler)
    {
        struct ble_msg_t ble_msg = {APP_BLE_NULL_MSG,NULL,NULL};
        ble_msg.msg_id = APP_BLE_GATTC_MTU_IND;
        ble_msg.msg.p_gattc_mtu = p_param;
        app_env.ble_msg_handler((void const*)&ble_msg);
    }
    //fix to set mut again
    extern struct gattc_env_tag* gattc_env[];
    if(gattc_env[KE_IDX_GET(src_id)] != NULL)
    {
        gattc_env[KE_IDX_GET(src_id)]->mtu_exch = false;
    }
    return (KE_MSG_CONSUMED);
}

__STATIC int gapc_le_phy_ind_handler(ke_msg_id_t const msgid,
                             struct gapc_le_phy_ind *p_param,
                                   ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n",__func__);
    NS_LOG_INFO("tx_phy:%d, rx_phy:%d\r\n",p_param->tx_phy,p_param->rx_phy);
    
    llhwc_modem_setmode(p_param->tx_phy);
    
    return (KE_MSG_CONSUMED);
}


/** 
 * @brief Handles user define update connection params 
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int app_conn_params_update_evt_handler(ke_msg_id_t const msgid, void *p_param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
  
    NS_LOG_DEBUG("%s\r\n",__func__);
    
    ns_ble_update_param(&gap_env.dev_conn_param);

    return (KE_MSG_CONSUMED);
}

/** 
 * @brief Handles user lsi calib
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int app_lsi_calib_evt_handler(ke_msg_id_t const msgid, void *p_param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    static uint16_t  calib_times = LSI_CLOCK_CALIB_TIMES;
    if((RCC->OSCFCSR) & 0x01)
    {
        uint32_t lsicnt = RCC->OSCFCLSICNT;
        extern uint32_t g_lsi_1_syscle_cal_value; 
        int32_t lpc_dif = lsicnt - (LSI_CLOCK_CNT_CYCLES*g_lsi_1_syscle_cal_value); 
        calib_times--;
        if(lpc_dif < -600 || lpc_dif > 600)
        {
            NS_LOG_DEBUG("lsi config,%d,%d \r\n",lsicnt,lpc_dif);
            calib_times=0;
            //config lis
            RCC->LSCTRL &= ~1; //off lsi
            if((*(uint32_t*)0x40011000) & 0xC000)
            {
                *(uint32_t*)0x40011000 &= ~0xC000;
            }
            else
            {
                *(uint32_t*)0x40011000 |= 0xC000; 
            }
            ns_ble_lsc_config(app_env.lsc_cfg);
        }
    }
    
    if(calib_times)
    {
        RCC->OSCFCCR |= 1;
        ke_timer_set(APP_LSI_CALIB_EVT,TASK_APP,LSI_CLOCK_EVENT_INTV);
    }
    else
    {
        RCC->OSCFCCR &= ~(0xFF<< 8);
        RCC->OSCFCCR |= (20 <<8);  //write count n syscle  
        RCC->OSCFCCR |= 1;
        ns_sleep_lock_release(); //release when lsi calib finsh
    }   

    return (KE_MSG_CONSUMED);
}


/** 
 * @brief Handles rssi event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] p_param   Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not. 
 */
static int gapc_conn_rssi_ind_handler(ke_msg_id_t const msgid,
                                        struct gapc_con_rssi_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    NS_LOG_DEBUG("%s\r\n",__func__);
    if(src_id != 0xff)
    {
        NS_LOG_DEBUG("RSSI = %d\r\n", param->rssi);
        if(app_env.ble_msg_handler)
        {
            struct ble_msg_t ble_msg = {APP_BLE_NULL_MSG,NULL,NULL};
            ble_msg.msg_id = APP_BLE_GAP_RSSI_IND;
            ble_msg.msg.p_gapc_rssi = param;
            app_env.ble_msg_handler((void const*)&ble_msg);
        }
        
        if(ke_state_get(TASK_APP) == APP_CONNECTED)
        {
            if(app_env.rssi_intv)
            {  
                //active this event again
                ke_timer_set(GAPC_CON_RSSI_IND,TASK_APP,app_env.rssi_intv);
            }
        }
    }
    else{
        //repeat get rssi event
        struct gapc_get_info_cmd* info_cmd = KE_MSG_ALLOC(GAPC_GET_INFO_CMD,
                KE_BUILD_ID(TASK_GAPC, KE_IDX_GET(src_id)), TASK_APP,
                gapc_get_info_cmd);

        // request RSSI
        info_cmd->operation = GAPC_GET_CON_RSSI;
        // send command
        ke_msg_send(info_cmd);   
    }
    return (KE_MSG_CONSUMED);
}


int app_entry_point_handler(ke_msg_id_t const msgid,
                            void const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    
    enum ke_msg_status_tag process_msg_handling_result;

    //add others user handler 
    
#if (NS_TIMER_ENABLE)
    ns_timer_api_process_handler(msgid, param, dest_id, src_id,&process_msg_handling_result);
    if(process_msg_handling_result == KE_MSG_CONSUMED)
         return (KE_MSG_CONSUMED);
#endif    //NS_TIMER_ENABLE
    
    app_msg_handler(msgid, (void *)param, dest_id, src_id, &process_msg_handling_result);
    if(process_msg_handling_result == KE_MSG_CONSUMED)
         return (KE_MSG_CONSUMED);

    //keep it as last event handler 
    if(app_env.user_msg_handler)
    {
        app_env.user_msg_handler(msgid, param);
    }
    
    return (KE_MSG_CONSUMED);
}




static int gapm_ext_adv_repoer_ind_handler(ke_msg_id_t const msgid, struct gapm_ext_adv_report_ind const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    bool target_found = false;
    NS_LOG_DEBUG("\r\nReport %x, ", param->trans_addr.addr_type);
    for(int i = 0; i < 6; i++)
    {
        NS_LOG_DEBUG("%02x ", param->trans_addr.addr.addr[i]);
    }


    switch (scan_env.filter_type)
    {
        case SCAN_FILTER_DISABLE:
            target_found = true;
            break;
        case SCAN_FILTER_BY_ADDRESS:
            if(memcmp((uint8_t *)param->trans_addr.addr.addr, scan_env.filter_data, 6) == 0)
            {
                target_found = true;
            }
            break;
        case SCAN_FILTER_BY_NAME:
            if(ns_ble_scan_data_find(GAP_AD_TYPE_COMPLETE_NAME,scan_env.filter_data,(uint8_t*)param->data,param->length))
            {
                target_found = true;
            }
            break;
        case SCAN_FILTER_BY_UUID128:
            if(ns_ble_scan_data_find(GAP_AD_TYPE_SERVICE_128_BIT_DATA,scan_env.filter_data,(uint8_t*)param->data,param->length))
            {
                target_found = true;
            }
            break;
        case SCAN_FILTER_BY_UUID16:
            if(ns_ble_scan_data_find(GAP_AD_TYPE_SERVICE_16_BIT_DATA,scan_env.filter_data,(uint8_t*)param->data,param->length))
            {
                target_found = true;
            }
            break;
        case SCAN_FILTER_BY_APPEARANCE:
            if(ns_ble_scan_data_find(GAP_AD_TYPE_APPEARANCE,scan_env.filter_data,(uint8_t*)param->data,param->length))
            {
                target_found = true;
            }
            break;
        default:
            break;
    }
    if(target_found && (!app_env.target_found) )
    {
        NS_LOG_DEBUG("Found target device\r\n");
        app_env.target_addr_type = param->trans_addr.addr_type;
        memcpy(app_env.target_addr.addr, param->trans_addr.addr.addr, 6);      
        app_env.target_found            = true;
        
        ns_ble_stop_scan();
        ns_ble_start_init((uint8_t*)param->trans_addr.addr.addr, param->trans_addr.addr_type);
        
        if(scan_env.ble_scan_state_handler != NULL)
        {
            scan_env.ble_scan_state_handler(SCAN_STATE_FOUND);
        }
    }

    if(scan_env.ble_scan_data_handler != NULL)
    {
        scan_env.ble_scan_data_handler(param);
    }    

    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLES DEFINITION 
 */
#if (BLE_APP_NS_IUS)
extern int app_dfu_ble_reset_handler(ke_msg_id_t const msgid, void const *p_param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
#endif
/* Default State handlers definition. */
KE_MSG_HANDLER_TAB(app)
{

    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,    (ke_msg_func_t)app_entry_point_handler},

    // GAPM messages
    {GAPM_ACTIVITY_CREATED_IND, (ke_msg_func_t)gapm_activity_created_ind_handler}, 
    {GAPM_ACTIVITY_STOPPED_IND, (ke_msg_func_t)gapm_activity_stopped_ind_handler},
    {GAPM_GEN_RAND_NB_IND,      (ke_msg_func_t)gapm_gen_rand_nb_ind_handler},
    {GAPM_ADDR_SOLVED_IND,      (ke_msg_func_t)gapm_addr_solved_ind_handler},
    {GAPM_CMP_EVT,              (ke_msg_func_t)gapm_cmp_evt_handler},
    {GAPM_PROFILE_ADDED_IND,    (ke_msg_func_t)gapm_profile_added_ind_handler},

    // GATTC messages
    {GATTC_MTU_CHANGED_IND,     (ke_msg_func_t)gattc_mtu_changed_ind_handler},
    {GATTC_CMP_EVT,             (ke_msg_func_t)gattc_cmp_evt_handler},

    // GAPC messages
    {GAPC_CONNECTION_REQ_IND,   (ke_msg_func_t)gapc_connection_req_ind_handler},
    {GAPC_PARAM_UPDATE_REQ_IND, (ke_msg_func_t)gapc_param_update_req_ind_handler},
    {GAPC_PARAM_UPDATED_IND,    (ke_msg_func_t)gapc_param_updated_ind_handler},
    {GAPC_LE_PKT_SIZE_IND,      (ke_msg_func_t)gapc_le_pkt_size_ind_handler},
    {GAPC_DISCONNECT_IND,       (ke_msg_func_t)gapc_disconnect_ind_handler},
    {GAPC_GET_DEV_INFO_REQ_IND, (ke_msg_func_t)gapc_get_dev_info_req_ind_handler},    
    {GAPC_LE_PHY_IND,           (ke_msg_func_t)gapc_le_phy_ind_handler},    
    {GAPC_SET_DEV_INFO_REQ_IND, (ke_msg_func_t)gapc_set_dev_info_req_ind_handler},
    {GAPC_CMP_EVT,              (ke_msg_func_t)gapc_cmp_evt_handler},
    {GAPC_CON_RSSI_IND,         (ke_msg_func_t)gapc_conn_rssi_ind_handler},

    {APP_LSI_CALIB_EVT,         (ke_msg_func_t)app_lsi_calib_evt_handler},
    {APP_PARAMS_UPDATE_EVT,     (ke_msg_func_t)app_conn_params_update_evt_handler},
    #if (BLE_APP_SEC)
    {APP_BOND_STORE_EVT,         (ke_msg_func_t)ns_sec_bond_store_evt_handler},
    #endif //(BLE_APP_SEC)
    #if (BLE_APP_NS_IUS)
    {APP_DFU_BLE_RESET_TIMER,   (ke_msg_func_t)app_dfu_ble_reset_handler},        
    #endif    
    
    {GAPM_EXT_ADV_REPORT_IND,   (ke_msg_func_t)gapm_ext_adv_repoer_ind_handler},
    
};

/* Defines the place holder for the states of all the task instances. */
ke_state_t app_state[APP_IDX_MAX];

// Application task descriptor
const struct ke_task_desc TASK_DESC_APP_M = {app_msg_handler_tab, app_state, APP_IDX_MAX, ARRAY_LEN(app_msg_handler_tab)};

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
