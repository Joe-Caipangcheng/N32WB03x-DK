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
 * @file ns_ble.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)

#include <string.h>
#include <stdio.h>
#include "ns_ble_task.h"                // Application task Definition
#include "ns_ble.h"                     // Application Definition
#include "gap.h"                     // GAP Definition
#include "gapm_task.h"               // GAP Manager Task API
#include "gapc_task.h"               // GAP Controller Task API
#include "gapm_int.h"
#include "co_math.h"                 // Common Maths Definition
#include "ble_stack_common.h"
#if (BLE_APP_SEC)
#include "ns_sec.h"                 // Application security Definition
#endif // (BLE_APP_SEC)

#if (BLE_APP_HT)
#include "app_ht.h"                  // Health Thermometer Application Definitions
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
#include "app_dis.h"                 // Device Information Service Application Definitions
#endif //(BLE_APP_DIS)

#if (BLE_APP_BATT)
#include "app_batt.h"                // Battery Application Definitions
#endif //(BLE_APP_DIS)

#if (BLE_APP_HID)
#include "app_hid.h"                 // HID Application Definitions
#endif //(BLE_APP_HID)


#if (BLE_RDTSS_SERVER)
#include "app_rdtss.h"                 // RDTSS Application Definitions
#endif //(BLE_RDTSS_SERVER)

#if (BLE_APP_NS_IUS)
#include "app_ns_ius.h"                 // NS_IUS Application Definitions
#endif //(BLE_APP_NS_IUS)


#if (NS_TIMER_ENABLE)
#include "ns_timer.h"
#endif //NS_TIMER_ENABLE

#include "ke_timer.h"

#include <stdio.h>
#include "global_var.h"

/* Define ------------------------------------------------------------*/
/// Default Device Name
#ifdef  CUSTOM_DEVICE_NAME
#define APP_DFLT_DEVICE_NAME            CUSTOM_DEVICE_NAME
#define APP_DFLT_DEVICE_NAME_LEN        (sizeof(CUSTOM_DEVICE_NAME)-1)
#elif   DEFAULT_DEVICE_NAME
#define APP_DFLT_DEVICE_NAME            DEFAULT_DEVICE_NAME
#define APP_DFLT_DEVICE_NAME_LEN        (sizeof(DEFAULT_DEVICE_NAME)-1)
#endif

 /**
 * Define advertise data
 */
#ifdef  CUSTOM_USER_ADVERTISE_DATA
#define USER_ADVERTISE_DATA          CUSTOM_USER_ADVERTISE_DATA
#define USER_ADVERTISE_DATA_LEN      CUSTOM_USER_ADVERTISE_DATA_LEN
#elif   DEFAULT_ADVERTISE_DATA
#define USER_ADVERTISE_DATA          DEFAULT_USER_ADVERTISE_DATA
#define USER_ADVERTISE_DATA_LEN      DEFAULT_USER_ADVERTISE_DATA_LEN
#endif
        
/**
 * Default Scan response data
 */
#ifdef  CUSTOM_USER_ADV_SCNRSP_DATA
#define USER_ADV_SCNRSP_DATA          CUSTOM_USER_ADV_SCNRSP_DATA 
#define USER_ADV_SCNRSP_DATA_LEN      CUSTOM_USER_ADV_SCNRSP_DATA_LEN
#elif   DEFAULT_USER_ADV_SCNRSP_DATA
#define USER_ADV_SCNRSP_DATA          DEFAULT_USER_ADV_SCNRSP_DATA
#define USER_ADV_SCNRSP_DATA_LEN      DEFAULT_USER_ADV_SCNRSP_DATA_LEN
#endif        



/**
 * Advertising Parameters
 */
#if (BLE_APP_HID)
/// Default Advertising duration - 30s (in multiple of 10ms)
#define APP_DFLT_ADV_DURATION   (3000)
#endif //(BLE_APP_HID)
/// Advertising channel map - 37, 38, 39
#define APP_ADV_CHMAP           (0x07)
/// Advertising minimum interval - 40ms (64*0.625ms)
#define APP_ADV_INT_MIN         (64)
/// Advertising maximum interval - 40ms (64*0.625ms)
#define APP_ADV_INT_MAX         (64)
/// Fast advertising interval
#define APP_ADV_FAST_INT        (32)

/* Private define ------------------------------------------------------------*/

typedef void (*app_add_svc_func_t)(void);

/* Typedef -----------------------------------------------------------*/

/// List of service to add in the database
enum app_svc_list
{
    #if (BLE_APP_HT)
    APP_SVC_HTS,
    #endif //(BLE_APP_HT)
    #if (BLE_APP_DIS)
    APP_SVC_DIS,
    #endif //(BLE_APP_DIS)
    #if (BLE_APP_BATT)
    APP_SVC_BATT,
    #endif //(BLE_APP_BATT)
    #if (BLE_APP_HID)
    APP_SVC_HIDS,
    #endif //(BLE_APP_HID)
    #if (BLE_RDTSS_SERVER)
    APP_SVC_RDTS,
    #endif //(BLE_RDTSS_SERVER)
    #if (BLE_APP_NS_IUS)
    APP_SVC_NS_IUS,
    #endif //(BLE_APP_NS_IUS)   
    APP_SVC_LIST_STOP,
};

/*
 * LOCAL VARIABLES DEFINITIONS
 **/

/// Application Task Descriptor
extern const struct ke_task_desc TASK_DESC_APP_M;

/// List of functions used to create the database  
static const app_add_svc_func_t app_add_svc_func_list[APP_SVC_LIST_STOP] =
{
    #if (BLE_APP_HT)
    (app_add_svc_func_t)app_ht_add_hts,
    #endif //(BLE_APP_HT)
    #if (BLE_APP_DIS)
    (app_add_svc_func_t)app_dis_add_dis,
    #endif //(BLE_APP_DIS)
    #if (BLE_APP_BATT)
    (app_add_svc_func_t)app_batt_add_bas,
    #endif //(BLE_APP_BATT)
    #if (BLE_APP_HID)
    (app_add_svc_func_t)app_hid_add_hids,
    #endif //(BLE_APP_HID)
    #if (BLE_RDTSS_SERVER)
    (app_add_svc_func_t)app_rdtss_add_rdts,
    #endif //(BLE_RDTSS_SERVER)
    #if (BLE_APP_NS_IUS)
    (app_add_svc_func_t)app_ns_ius_add_ns_ius,
    #endif //(BLE_APP_NS_IUS)        
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 **/

/// Application Environment Structure
struct app_env_tag app_env;
struct ns_scan_params_t scan_env;
/* 
 * LOCAL FUNCTION DEFINITIONS
 **/

#if (BLE_APP_PRF)

static void app_build_adv_data(uint16_t max_length, uint16_t *p_length, uint8_t *p_buf)
{
    // Remaining Length
    uint8_t rem_len = max_length;
         
      #ifdef CUSTOM_USER_ADVERTISE_DATA
      ASSERT_ERR( CUSTOM_USER_ADVERTISE_DATA >= rem_len );
    
      memcpy(p_buf,CUSTOM_USER_ADVERTISE_DATA, CUSTOM_USER_ADVERTISE_DATA_LEN);
      *p_length += CUSTOM_USER_ADVERTISE_DATA_LEN;
      p_buf += CUSTOM_USER_ADVERTISE_DATA_LEN;
        
      #elif DEFAULT_ADVERTISE_DATA
      ASSERT_ERR( DEFAULT_ADVERTISE_DATA >= rem_len );
    
      memcpy(p_buf,DEFAULT_ADVERTISE_DATA,DEFAULT_ADVERTISE_DATA_LEN);
      *p_length += DEFAULT_ADVERTISE_DATA_LEN;
      p_buf += DEFAULT_ADVERTISE_DATA_LEN;
    
      #endif

    // Sanity check
    ASSERT_ERR(rem_len >= max_length);

    // Get remaining space in the Advertising Data - 2 bytes are used for name length/flag
    rem_len -= *p_length;

    // Check if additional data can be added to the Advertising data - 2 bytes needed for type and length
    if (rem_len > 2)
    {
        uint8_t dev_name_length = co_min(app_env.dev_name_len, (rem_len - 2));

        // Device name length
        *p_buf = dev_name_length + 1;
        // Device name flag (check if device name is complete or not)
        *(p_buf + 1) = (dev_name_length == app_env.dev_name_len) ? '\x09' : '\x08';
        // Copy device name
        memcpy(p_buf + 2, app_env.dev_name, dev_name_length);

        // Update advertising data length
        *p_length += (dev_name_length + 2);
    }
}

 void app_start_advertising_with_mode(void)
{  
    // Prepare the GAPM_ACTIVITY_START_CMD message
    struct gapm_activity_start_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_START_CMD,
                                                         TASK_GAPM, TASK_APP,
                                                         gapm_activity_start_cmd);

    p_cmd->operation = GAPM_START_ACTIVITY;
    p_cmd->actv_idx = app_env.adv_actv_idx;

    switch (app_env.adv_mode)
    {
        case APP_ADV_MODE_DIRECTED_HIGH_DUTY:
            p_cmd->u_param.adv_add_param.duration = SECS_TO_UNIT(CUSTOM_ADV_FAST_DURATION, SECS_UNIT_10MS);                      
            break;
        case APP_ADV_MODE_DIRECTED:
            p_cmd->u_param.adv_add_param.duration = SECS_TO_UNIT(CUSTOM_ADV_FAST_DURATION, SECS_UNIT_10MS);    
            break;
        case APP_ADV_MODE_FAST:
            p_cmd->u_param.adv_add_param.duration = SECS_TO_UNIT(CUSTOM_ADV_FAST_DURATION, SECS_UNIT_10MS);
            break;
        case APP_ADV_MODE_SLOW:
            p_cmd->u_param.adv_add_param.duration = SECS_TO_UNIT(CUSTOM_ADV_SLOW_DURATION, SECS_UNIT_10MS);
            break;
        default:
            p_cmd->u_param.adv_add_param.duration = 0;
            break;
    }

    p_cmd->u_param.adv_add_param.max_adv_evt = 0;
        
    // Send the message
    ke_msg_send(p_cmd);

    // Keep the current operation
    app_env.adv_state = APP_ADV_STATE_STARTING;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_START_ACTIVITY;
        
        
}


 void app_stop_advertising(void)
{

    // Prepare the GAPM_ACTIVITY_STOP_CMD message
    struct gapm_activity_stop_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_STOP_CMD,
                                                      TASK_GAPM, TASK_APP,
                                                      gapm_activity_stop_cmd);
    // Fill the allocated kernel message
    p_cmd->operation = GAPM_STOP_ACTIVITY;
    p_cmd->actv_idx = app_env.adv_actv_idx;

    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_STOPPING;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_STOP_ACTIVITY;
}


static void app_set_adv_data(void)
{

    // Prepare the GAPM_SET_ADV_DATA_CMD message
    struct gapm_set_adv_data_cmd *p_cmd = KE_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
                                                           TASK_GAPM, TASK_APP,
                                                           gapm_set_adv_data_cmd,
                                                           ADV_DATA_LEN);
    // Fill the allocated kernel message
    p_cmd->operation = GAPM_SET_ADV_DATA;
    p_cmd->actv_idx = app_env.adv_actv_idx;

    p_cmd->length = 0;
    // GAP will use 3 bytes for the AD Type
    app_build_adv_data(ADV_DATA_LEN - 3, &p_cmd->length, &p_cmd->data[0]);

    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_SETTING_ADV_DATA;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_SET_ADV_DATA;
}

static void app_set_scan_rsp_data(void)
{
    // Prepare the GAPM_SET_ADV_DATA_CMD message
    struct gapm_set_adv_data_cmd *p_cmd = KE_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
                                                           TASK_GAPM, TASK_APP,
                                                           gapm_set_adv_data_cmd,
                                                           ADV_DATA_LEN);
    // Fill the allocated kernel message
    p_cmd->operation = GAPM_SET_SCAN_RSP_DATA;
    p_cmd->actv_idx = app_env.adv_actv_idx;

    p_cmd->length = USER_ADV_SCNRSP_DATA_LEN;
    memcpy(&p_cmd->data[0], USER_ADV_SCNRSP_DATA, USER_ADV_SCNRSP_DATA_LEN);

    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_SETTING_SCAN_RSP_DATA;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_SET_SCAN_RSP_DATA;
}

#endif //(BLE_APP_PRF)

 void app_send_gapm_reset_cmd(void)
{
    // Reset the stack
    struct gapm_reset_cmd *p_cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
                                                TASK_GAPM, TASK_APP,
                                                gapm_reset_cmd);

    p_cmd->operation = GAPM_RESET;
    ke_msg_send(p_cmd);
}



void ns_ble_lsc_config(ble_lsc_cfg_t lsc_set)
{
    //Config the LSC for stack
    if(lsc_set == BLE_LSC_LSE_32768HZ)
    {
        //enable LSE and select LSC as LSE
        *(uint32_t*)0x40011008 |= 0xF00;
        RCC->LSCTRL |= RCC_LSCTRL_LSEEN;         
        while(!(RCC->LSCTRL & RCC_LSCTRL_LSERD));
        RCC->LSCTRL |= RCC_LSCTRL_LSXSEL_LSE;       
        
        g_lsi_1_syscle_cal_value = 1000;
        //lse frequency 32768
        g_lsi_1_syscle_cnt_value = 976;  //32000000/32768 = 976.56 //must fixed
    }
    else
    {
        //LSC select LSI
        RCC->LSCTRL |= RCC_LSCTRL_LSIEN;         
        while(!(RCC->LSCTRL & RCC_LSCTRL_LSIRD));
        RCC->LSCTRL &= ~RCC_LSCTRL_LSXSEL_LSE;  
        /* set lsi frequency, (32m/lsc_freq) 32k:1000 , 28.8k 1111.11 , 32.768: 976.56 */
        /* g_lsi_1_syscle_cal_value = 32000000/lsc_freq; */
        switch (lsc_set)
        {
            case BLE_LSC_LSI_32768HZ:
                g_lsi_1_syscle_cal_value = 976;
                break;
            case BLE_LSC_LSI_28800HZ:
                g_lsi_1_syscle_cal_value = 1111;
                break;
            case BLE_LSC_LSI_32000HZ:
            default:
                g_lsi_1_syscle_cal_value = 1000;
                break;
        }
        g_lsi_1_syscle_cnt_value = calib_lsi_clk(); //take 7.5ms
    }
}



/*
 * GLOBAL FUNCTION DEFINITIONS
 **/
extern struct bd_addr g_co_default_bdaddr;
void app_init()
{
    uint8_t *p_mac = SystemGetMacAddr();
    uint8_t dfu_mac[BD_ADDR_LEN];
    // Reset the application manager environment
    memset(&app_env, 0, sizeof(app_env));
    
    app_env.scan_actv_idx = 0xff;
    app_env.adv_actv_idx = 0xff;
    
    app_env.lsc_cfg = BLE_LSC_LSI_32000HZ;
    ns_ble_lsc_config(app_env.lsc_cfg);
    NS_BLE_STACK_INIT();
    
    //get UUID from trim stored
    if(p_mac != NULL)
    {
        //set the uuid as mac address        
        memcpy(dfu_mac,p_mac, BD_ADDR_LEN); 
        dfu_mac[0]++;
        memcpy(g_co_default_bdaddr.addr, dfu_mac , BD_ADDR_LEN); 
    }
    else{
        memcpy(g_co_default_bdaddr.addr, CUSTOM_BLE_MAC_ADDRESS, 6);
    }

    // Create APP task
    ke_task_create(TASK_APP, &TASK_DESC_APP_M);

    // Initialize Task state
    ke_state_set(TASK_APP, APP_INIT);


    #if (NVDS_SUPPORT)
    // Get the Device Name to add in the Advertising Data (Default one or NVDS one)
    app_env.dev_name_len = APP_DEVICE_NAME_MAX_LEN;
    if (nvds_get(NVDS_TAG_DEVICE_NAME, &(app_env.dev_name_len), app_env.dev_name) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        // Get default Device Name (No name if not enough space)
        memcpy(app_env.dev_name, APP_DFLT_DEVICE_NAME, APP_DFLT_DEVICE_NAME_LEN);
        app_env.dev_name_len = APP_DFLT_DEVICE_NAME_LEN;

        // TODO update this value per profiles
    }

    /*------------------------------------------------------
     * INITIALIZE ALL MODULES
     *------------------------------------------------------*/

    // load device information:

    #if (BLE_APP_SEC)
    // Security Module
    app_sec_init();
    #endif // (BLE_APP_SEC)

    #if (BLE_APP_HT)
    // Health Thermometer Module
    app_ht_init();
    #endif //(BLE_APP_HT)

    #if (BLE_APP_DIS)
    // Device Information Module
    app_dis_init();
    #endif //(BLE_APP_DIS)

    #if (BLE_APP_HID)
    // HID Module
    app_hid_init();
    #endif //(BLE_APP_HID)

    #if (BLE_APP_BATT)
    // Battery Module
    app_batt_init();
    #endif //(BLE_APP_BATT)


    
    #if (BLE_RDTSS_SERVER)
    // rdtss Module
    app_rdtss_init();
    #endif //(BLE_RDTSS_SERVER)

    #if (BLE_APP_NS_IUS)
    // DFU OTA Service
    app_ns_ius_init();
    #endif //(BLE_APP_NS_IUS)

    // Reset the stack
    app_send_gapm_reset_cmd();
}

bool app_add_svc(void)
{
    // Indicate if more services need to be added in the database
    bool more_svc = false;
    // Check if another should be added in the database
    if (app_env.next_svc != APP_SVC_LIST_STOP)
    {
        ASSERT_INFO(app_add_svc_func_list[app_env.next_svc] != NULL, app_env.next_svc, 1);

        // Call the function used to add the required service
        app_add_svc_func_list[app_env.next_svc]();

        // Select following service to add
        app_env.next_svc++;
        more_svc = true;
    }

    return more_svc;
}

uint8_t app_get_dev_name(uint8_t* name)
{
    // copy name to provided pointer
    memcpy(name, app_env.dev_name, app_env.dev_name_len);
    // return name length
    return app_env.dev_name_len;
}

#if (BLE_APP_PRF)
void app_disconnect(void)
{
    struct gapc_disconnect_cmd *p_cmd = KE_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                                                   KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                   gapc_disconnect_cmd);

    p_cmd->operation = GAPC_DISCONNECT;
    p_cmd->reason    = CO_ERROR_REMOTE_USER_TERM_CON;

    // Send the message
    ke_msg_send(p_cmd);
}



 void app_create_advertising_with_mode(void)
{
    if (app_env.adv_state == APP_ADV_STATE_IDLE)
    {
        // Prepare the GAPM_ACTIVITY_CREATE_CMD message
        struct gapm_activity_create_adv_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
                                                                  TASK_GAPM, TASK_APP,
                                                                  gapm_activity_create_adv_cmd);

        // Set operation code
        p_cmd->operation = GAPM_CREATE_ADV_ACTIVITY;

        // Fill the allocated kernel message
        p_cmd->own_addr_type = GAPM_STATIC_ADDR;
        p_cmd->adv_param.type = GAPM_ADV_TYPE_LEGACY;
        p_cmd->adv_param.prop = GAPM_ADV_PROP_UNDIR_CONN_MASK;
        p_cmd->adv_param.filter_pol = ADV_ALLOW_SCAN_ANY_CON_ANY;
        p_cmd->adv_param.prim_cfg.chnl_map = APP_ADV_CHMAP;
        p_cmd->adv_param.prim_cfg.phy = GAP_PHY_LE_1MBPS;

                
        switch (app_env.adv_mode)
        {
            case APP_ADV_MODE_DIRECTED_HIGH_DUTY:
            case APP_ADV_MODE_DIRECTED:
            case APP_ADV_MODE_FAST:
                p_cmd->adv_param.prim_cfg.adv_intv_min = CUSTOM_ADV_FAST_INTERVAL;
                p_cmd->adv_param.prim_cfg.adv_intv_max = CUSTOM_ADV_FAST_INTERVAL;
                p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_GEN_DISC;
                break;
            case APP_ADV_MODE_SLOW:
                p_cmd->adv_param.prim_cfg.adv_intv_min = CUSTOM_ADV_SLOW_INTERVAL;
                p_cmd->adv_param.prim_cfg.adv_intv_max = CUSTOM_ADV_SLOW_INTERVAL;
                p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_GEN_DISC;
                break;                    
            default:
                break;
        }

        // Send the message
        ke_msg_send(p_cmd);

        // Keep the current operation
        app_env.adv_state = APP_ADV_STATE_CREATING;
        // And the next expected operation code for the command completed event
        app_env.adv_op = GAPM_CREATE_ADV_ACTIVITY;
    }
}

 void app_create_advertising(void)
{
    if (app_env.adv_state == APP_ADV_STATE_IDLE)
    {
        // Prepare the GAPM_ACTIVITY_CREATE_CMD message
        struct gapm_activity_create_adv_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
                                                                  TASK_GAPM, TASK_APP,
                                                                  gapm_activity_create_adv_cmd);

        // Set operation code
        p_cmd->operation = GAPM_CREATE_ADV_ACTIVITY;

        // Fill the allocated kernel message
        p_cmd->own_addr_type = GAPM_STATIC_ADDR;
        p_cmd->adv_param.type = GAPM_ADV_TYPE_LEGACY;
        p_cmd->adv_param.prop = GAPM_ADV_PROP_UNDIR_CONN_MASK;
        p_cmd->adv_param.filter_pol = ADV_ALLOW_SCAN_ANY_CON_ANY;
        p_cmd->adv_param.prim_cfg.chnl_map = APP_ADV_CHMAP;
        p_cmd->adv_param.prim_cfg.phy = GAP_PHY_LE_1MBPS;

        #if (BLE_APP_HID)
        p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_LIM_DISC;

        /*
         * If the peripheral is already bonded with a central device, use the direct advertising
         * procedure (BD Address of the peer device is stored in NVDS.
         */
        if (app_sec_get_bond_status())
        {
            p_cmd->adv_param.prim_cfg.adv_intv_min = APP_ADV_FAST_INT;
            p_cmd->adv_param.prim_cfg.adv_intv_max = APP_ADV_FAST_INT;
        }
        else
        {
            p_cmd->adv_param.prim_cfg.adv_intv_min = APP_ADV_INT_MIN;
            p_cmd->adv_param.prim_cfg.adv_intv_max = APP_ADV_INT_MAX;
        }
        #else //(BLE_APP_HID)
        p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_GEN_DISC;
        p_cmd->adv_param.prim_cfg.adv_intv_min = APP_ADV_INT_MIN; //app_env.adv_interval;  //APP_ADV_INT_MIN;
        p_cmd->adv_param.prim_cfg.adv_intv_max = APP_ADV_INT_MAX; //app_env.adv_interval;  //APP_ADV_INT_MAX;
        #endif //(BLE_APP_HID)


        // Send the message
        ke_msg_send(p_cmd);

        // Keep the current operation
        app_env.adv_state = APP_ADV_STATE_CREATING;
        // And the next expected operation code for the command completed event
        app_env.adv_op = GAPM_CREATE_ADV_ACTIVITY;
    }
}

void app_delete_advertising(void)
{

    // Prepare the GAPM_ACTIVITY_CREATE_CMD message
    struct gapm_activity_delete_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_DELETE_CMD,
                                                              TASK_GAPM, TASK_APP,
                                                              gapm_activity_delete_cmd);
    // Set operation code
    p_cmd->operation = GAPM_DELETE_ALL_ACTIVITIES;

    // Send the message
    ke_msg_send(p_cmd);

    // Keep the current operation
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_DELETE_ALL_ACTIVITIES;
}

void app_adv_fsm_next(void)
{
    
    switch (app_env.adv_state)
    {
        case (APP_ADV_STATE_IDLE):
        {
            // Create advertising
            //app_create_advertising();
            app_create_advertising_with_mode();
        } break;

        case (APP_ADV_STATE_CREATING):
        {
            // Set advertising data
            app_set_adv_data();
        } break;

        case (APP_ADV_STATE_SETTING_ADV_DATA):
        {
            // Set scan response data
            app_set_scan_rsp_data();
        } break;

        case (APP_ADV_STATE_CREATED):
        case (APP_ADV_STATE_SETTING_SCAN_RSP_DATA):
        {
            // Start advertising activity
            app_start_advertising_with_mode();
        } break;

        case (APP_ADV_STATE_STARTING):
        {

                    

            // Go to started state
            app_env.adv_state = APP_ADV_STATE_STARTED;
        } break;

        case (APP_ADV_STATE_STARTED):
        {
            // Stop advertising activity
            app_stop_advertising();
        } break;

        case (APP_ADV_STATE_STOPPING):
        {

            // Go created state
            //app_env.adv_state = APP_ADV_STATE_CREATED;

        if(ke_state_get(TASK_APP) != APP_CONNECTED)
        {
            if(app_env.adv_mode == APP_ADV_MODE_FAST)
            {
                app_env.adv_mode = APP_ADV_MODE_SLOW;
                app_env.adv_state = APP_ADV_STATE_IDLE;
                app_create_advertising_with_mode();
            }
            else if(app_env.adv_mode == APP_ADV_MODE_SLOW)
            {
                app_env.adv_state = APP_ADV_STATE_CREATED;
            }
        }
        else
        {
            app_env.adv_state = APP_ADV_STATE_CREATED;
        }

        } break;

        default:
        {
            ASSERT_ERR(0);
        } break;
    }
}

void ns_ble_update_param(struct gapc_conn_param *p_conn_param)
{
    // Prepare the GAPC_PARAM_UPDATE_CMD message
    struct gapc_param_update_cmd *p_cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                             KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                             gapc_param_update_cmd);

    
    p_cmd->operation  = GAPC_UPDATE_PARAMS;
    p_cmd->intv_min   = p_conn_param->intv_min;
    p_cmd->intv_max   = p_conn_param->intv_max;
    p_cmd->latency    = p_conn_param->latency;
    p_cmd->time_out   = p_conn_param->time_out;

    // not used by a slave device
    p_cmd->ce_len_min = 0xFFFF;
    p_cmd->ce_len_max = 0xFFFF;

    // Send the message
    ke_msg_send(p_cmd);
}

void app_update_adv_state(bool start)
{
    // TODO [LT] - Check current advertising state
      if(start)
      {
          app_env.adv_mode = APP_ADV_MODE_FAST;
      }
            
    // Start or stop advertising
    app_adv_fsm_next();
}

void ns_ble_mtu_set(uint16_t mtu)
{
    gapm_env.max_mtu = mtu;
    struct gattc_exc_mtu_cmd *cmd = KE_MSG_ALLOC(GATTC_EXC_MTU_CMD,
                                    KE_BUILD_ID(TASK_GATTC, app_env.conidx), TASK_APP,
                                    gattc_exc_mtu_cmd);
    cmd->operation = GATTC_MTU_EXCH;
    cmd->seq_num = 0;
    ke_msg_send(cmd);
}



#endif //(BLE_APP_PRF)
#endif //(BLE_APP_PRESENT)

/// @} APP
