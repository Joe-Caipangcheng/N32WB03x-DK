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
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "app_user_config.h"
#include "global_func.h"
#include "ns_sec.h"
#include "app_ble.h"
#include "app_dis.h"
#include "app_usart.h"
/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define POWER_ON_ENTER_CW
#define POWER_ON_CW_CH  0 //2402 Hz
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
    static uint8_t  uart_cmd[2];
    switch (msgid)
    {
    	case GAPM_LE_TEST_END_IND:
        {
            extern uint8_t usart_sending;
            extern uint8_t m_stop_rx_rsp;
            ///respond cmd to usart
            struct gapm_le_test_end_ind* p = (struct gapm_le_test_end_ind*)p_param;
            uint16_t m_event;
            m_event = 0x8000 | p->nb_packet_received;
            uart_cmd[0] = (m_event>>8)& 0xFF;
            uart_cmd[1] = (m_event)& 0xFF;
            while(usart_sending){}
            
            usart_tx_dma_send(uart_cmd,2);
            m_stop_rx_rsp = true;
        }

    		break;
    	case APP_TIMER0:
        {
            extern uint8_t usart_sending;
            extern uint8_t m_stop_rx_rsp;
            if(!m_stop_rx_rsp)
            {
                //if not send respond yet 
                uart_cmd[0] = 0;
                uart_cmd[1] = 0;
                while(usart_sending){}
                usart_tx_dma_send(uart_cmd,2);
                m_stop_rx_rsp = true;
            }
        }
    		break;
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
        {
            NS_LOG_INFO("APP_BLE_OS_READY\r\n");
            #ifdef POWER_ON_ENTER_CW
            NS_LOG_INFO("POWER_ON_ENTER_CW\r\n");
            struct gapm_le_test_mode_ctrl_cmd   rf_params = {0};
            rf_params.channel               = POWER_ON_CW_CH; 
            rf_params.operation             = GAPM_LE_TEST_TX_START;
            rf_params.tx_data_length        = 1;
            rf_params.tx_pkt_payload        = GAP_PKT_PLD_PRBS9;
            rf_params.phy                   = GAP_PHY_1MBPS;
            rf_params.slot_dur              = 1;
            rf_params.modulation_idx        = GAP_MODULATION_STANDARD;
            rf_params.switching_pattern_len = MIN_SWITCHING_PATTERN_LEN;
            ns_ble_prod_test_cmd_send(&rf_params,true);//tx cw
            #endif
        
        }   break;
        case APP_BLE_GAP_CONNECTED:
            app_ble_connected();
            break;
        case APP_BLE_GAP_DISCONNECTED:
            app_ble_disconnected();
            break;

        default:
            break;
    }

}

/**
 * @brief  advertising message handler
 * @param  
 * @return 
 * @note   
 */
void app_ble_adv_msg_handler(enum app_adv_mode adv_mode)
{
    switch (adv_mode)
    {
        case APP_ADV_MODE_DIRECTED:
            
            break;
        case APP_ADV_MODE_FAST:
            
            break;
        case APP_ADV_MODE_SLOW:
            
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
        memcpy(dev_info.mac_addr.addr, "\x01\x02\x03\x04\x05\x06" , BD_ADDR_LEN);
    }
    

    /* init params*/
    dev_info.mac_addr_type = GAPM_STATIC_ADDR;
    dev_info.appearance = 0;
    dev_info.dev_role = GAP_ROLE_PERIPHERAL;
    
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
 * @brief  ble advertising initialization
 * @param  
 * @return 
 * @note   
 */
void app_ble_adv_init(void)
{
    struct ns_adv_params_t user_adv = {0};
    
    //init advertising data 
    user_adv.adv_data_len = CUSTOM_USER_ADVERTISE_DATA_LEN;
    memcpy(user_adv.adv_data,CUSTOM_USER_ADVERTISE_DATA,CUSTOM_USER_ADVERTISE_DATA_LEN);
    user_adv.scan_rsp_data_len = CUSTOM_USER_ADV_SCNRSP_DATA_LEN;
    memcpy(user_adv.scan_rsp_data,CUSTOM_USER_ADV_SCNRSP_DATA,CUSTOM_USER_ADV_SCNRSP_DATA_LEN);
    
    user_adv.attach_appearance  = false;
    user_adv.attach_name        = true;
    user_adv.ex_adv_enable      = false;
    user_adv.adv_phy            = PHY_1MBPS_VALUE;
    
    //init advertising params
    user_adv.directed_adv.enable = false;

    user_adv.fast_adv.enable    = true;
    user_adv.fast_adv.duration  = CUSTOM_ADV_FAST_DURATION;
    user_adv.fast_adv.adv_intv  = CUSTOM_ADV_FAST_INTERVAL;
    
    user_adv.slow_adv.enable    = true;  
    user_adv.slow_adv.duration  = CUSTOM_ADV_SLOW_DURATION;
    user_adv.slow_adv.adv_intv  = CUSTOM_ADV_SLOW_INTERVAL;
    
    user_adv.ble_adv_msg_handler = app_ble_adv_msg_handler;

    ns_ble_adv_init(&user_adv);
    

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
    //add device informaiton server
    ns_ble_add_prf_func_register(app_dis_add_dis);
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
    app_handler.lsc_cfg          = BLE_LSC_LSI_32768HZ;
    //initialization ble stack
    ns_ble_stack_init(&app_handler);
    
    app_ble_gap_params_init();
    app_ble_sec_init();
    app_ble_adv_init();
    app_ble_prf_init();

}

/**
 * @brief  ble connected
 * @param  
 * @return 
 * @note   
 */
void app_ble_connected(void)
{
 

}

/**
 * @brief  ble disconnected
 * @param  
 * @return 
 * @note   
 */
void app_ble_disconnected(void)
{
    // Restart Advertising
//    ns_ble_adv_start();

}





/**
 * @}
 */
