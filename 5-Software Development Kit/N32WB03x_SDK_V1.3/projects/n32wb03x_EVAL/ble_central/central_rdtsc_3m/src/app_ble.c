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
 * @file app_ble.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include <string.h>
#include "n32wb03x.h"
#include "gapm_task.h"               // GAP Manager Task API
#include "app_ble.h"
#include "app_usart.h"
#include "app_gpio.h"
#include "ns_sec.h"
#include "app_rdtsc.h"
#include "app_user_config.h"
/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void app_ble_connected(void);
void app_ble_disconnected(void);

/**
 * @brief  user message handler
 * @param  
 * @return 
 * @note   
 */
void app_user_msg_handler(ke_msg_id_t const msgid, void const *p_param)
{
    
    switch (msgid)
    {
    	case APP_UART_TX_EVT:
            app_usart_tx_process();
    		break;
//    	case :
//    		break;
    	default:
    		break;
    }
 

}

/**
 * @brief  ble message handler
 * @param  
 * @return 
 * @note   
 */
void app_ble_msg_handler(struct ble_msg_t const *p_ble_msg)
{
    switch (p_ble_msg->msg_id)
    {
        case APP_BLE_OS_READY:
            NS_LOG_INFO("APP_BLE_OS_READY\r\n");
            break;
        case APP_BLE_GAP_CONNECTED:
            app_rdtsc_enable_prf(app_env.conidx);
            app_ble_connected();
            break;
        case APP_BLE_GAP_DISCONNECTED:
            app_ble_disconnected();
            break;
         case APP_BLE_GAP_PARAMS_IND:
             // restart scan if not reach max connection
            if (ns_ble_get_connection_num() < BLE_MASTER_CONN)
            {
                NS_LOG_INFO("restart scan\r\n");
                ns_ble_start_scan();
            }
            break;
        default:
            break;
    }

}

/**
 * @brief  scan data handler
 * @param  
 * @return 
 * @note   
 */
void app_ble_scan_data_handler(struct gapm_ext_adv_report_ind const* p_param)
{
//    NS_LOG_INFO("Found:%02x%02x%02x%02x%02x%02x, rssi:%d \r\n",
//                              p_param->trans_addr.addr.addr[0],
//                              p_param->trans_addr.addr.addr[1],
//                              p_param->trans_addr.addr.addr[2],
//                              p_param->trans_addr.addr.addr[3],
//                              p_param->trans_addr.addr.addr[4],
//                              p_param->trans_addr.addr.addr[5],
//                              p_param->rssi);

}

/**
 * @brief  scan state handler
 * @param  
 * @return 
 * @note   
 */
void app_ble_scan_state_handler(enum scan_state_t state)
{
    switch (state)
    {
    	case SCAN_STATE_SCANING:
            NS_LOG_INFO("Scaning.\r\n");
    		break;
    	case SCAN_STATE_FOUND:
            NS_LOG_INFO("Found target.\r\n");
    		break;
    	default:
    		break;
    }

}
    

/**
 * @brief  ble GAP initialization
 * @param  
 * @return 
 * @note   
 */
void app_ble_gap_params_init(void)
{
    struct ns_gap_params_t dev_info = {0};
    uint8_t *p_mac = SystemGetMacAddr();
    //get UUID from trim stored
    if(p_mac != NULL)
    {
        //set the uuid as mac address
        memcpy(dev_info.mac_addr.addr, p_mac , BD_ADDR_LEN); 
    }
    else{
        memcpy(dev_info.mac_addr.addr, "\x01\x02\x03\x0C\x05\x06" , BD_ADDR_LEN);
    }
    

    /* init params*/
    dev_info.mac_addr_type = GAPM_STATIC_ADDR;
    dev_info.appearance = 0;
    dev_info.dev_role = GAP_ROLE_ALL;
    
    dev_info.dev_name_len = sizeof(CUSTOM_DEVICE_NAME)-1;
    memcpy(dev_info.dev_name, CUSTOM_DEVICE_NAME, dev_info.dev_name_len); 
   
    dev_info.dev_conn_param.intv_min = MSECS_TO_UNIT(MIN_CONN_INTERVAL,MSECS_UNIT_1_25_MS);
    dev_info.dev_conn_param.intv_max = MSECS_TO_UNIT(MAX_CONN_INTERVAL,MSECS_UNIT_1_25_MS);
    dev_info.dev_conn_param.latency  = SLAVE_LATENCY;
    dev_info.dev_conn_param.time_out = MSECS_TO_UNIT(CONN_SUP_TIMEOUT,MSECS_UNIT_10_MS);
    dev_info.conn_param_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    
    ns_ble_gap_init(&dev_info);
    
}


/**
 * @brief  ble scan initialization
 * @param  
 * @return 
 * @note   
 */
void app_ble_scan_init(void)
{
    struct ns_scan_params_t init = {0};
//    static const uint8_t target_addr[] = {"\x11\x11\x11\x11\x11\x11"};
    static const uint8_t target_name[] = {"NS_RDTSS"};
//    static const uint16_t target_uuid16 = 0x180A;
    
    init.type               = SCAN_PARAM_TYPE;
    init.dup_filt_pol       = SCAN_PARAM_DUP_FILT_POL;
    init.connect_enable     = SCAN_PARAM_CONNECT_EN;
    init.prop_active_enable = SCAN_PARAM_PROP_ACTIVE;
    init.scan_intv          = SCAN_PARAM_INTV;
    init.scan_wd            = SCAN_PARAM_WD;
    init.duration           = SCAN_PARAM_DURATION;
    
    init.filter_type    = SCAN_FILTER_BY_NAME;//SCAN_FILTER_BY_ADDRESS;
    init.filter_data    = (uint8_t*)&target_name;
    
    init.ble_scan_data_handler  = app_ble_scan_data_handler;
    init.ble_scan_state_handler = app_ble_scan_state_handler;
    
    ns_ble_scan_init(&init);
}

void app_ble_sec_init(void)
{
    struct ns_sec_init_t sec_init = {0};
    
    sec_init.rand_pin_enable = false;
    sec_init.pin_code = 123456;
    
    sec_init.pairing_feat.auth      = ( SEC_PARAM_BOND | (SEC_PARAM_MITM<<2) | (SEC_PARAM_LESC<<3) | (SEC_PARAM_KEYPRESS<<4) );
    sec_init.pairing_feat.iocap     = SEC_PARAM_IO_CAPABILITIES;
    sec_init.pairing_feat.key_size  = SEC_PARAM_KEY_SIZE;
    sec_init.pairing_feat.oob       = SEC_PARAM_OOB;
    sec_init.pairing_feat.ikey_dist = SEC_PARAM_IKEY;
    sec_init.pairing_feat.rkey_dist = SEC_PARAM_RKEY;
    sec_init.pairing_feat.sec_req   = SEC_PARAM_SEC_MODE_LEVEL;
    
    sec_init.bond_enable            = BOND_STORE_ENABLE;
    sec_init.bond_db_addr           = BOND_DATA_BASE_ADDR;
    sec_init.bond_max_peer          = MAX_BOND_PEER;
    sec_init.bond_sync_delay        = 2000;
    
    sec_init.ns_sec_msg_handler     = NULL;
    
    ns_sec_init(&sec_init);
}

void app_ble_prf_init(void)
{
    //add raw data transmit server(rdts)
    ns_ble_add_prf_func_register(app_rdtsc_add_rdts);
}


/**
 * @brief  ble initialization
 * @param  
 * @return 
 * @note   
 */
void app_ble_init(void)
{

    struct ns_stack_cfg_t app_handler = {0};
    app_handler.ble_msg_handler  = app_ble_msg_handler;
    app_handler.user_msg_handler = app_user_msg_handler;
    //initialization ble stack
    ns_ble_stack_init(&app_handler);
    
    app_ble_gap_params_init();
    app_ble_sec_init();
    app_ble_scan_init();
    app_ble_prf_init();
    //start scan
    ns_ble_start_scan();
}

/**
 * @brief  ble connected
 * @param  
 * @return 
 * @note   
 */
void app_ble_connected(void)
{
    //enable usart receive
    app_usart_dma_enable(ENABLE);
    LedOn(LED2_PORT,LED2_PIN);   
    
    #if (BLE_APP_BATT)
    // Enable Battery Service
    app_batt_enable_prf(app_env.conidx);
    #endif //(BLE_APP_BATT)
}

/**
 * @brief  ble disconnected
 * @param  
 * @return 
 * @note   
 */
void app_ble_disconnected(void)
{
    // Restart scan

    if (ns_ble_get_connection_num() < BLE_MASTER_CONN)
    {
        ns_ble_start_scan();
    }
    if (ns_ble_get_connection_num() == 0)
    {		
    //disable usart receive
    app_usart_dma_enable(DISABLE);
    LedOff(LED2_PORT,LED2_PIN);
    }
}





/**
 * @}
 */
