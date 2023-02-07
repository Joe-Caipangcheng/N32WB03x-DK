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
 * @version v1.0.3
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** 
 * @addtogroup APP
 * @{ 
 */

/* Includes ------------------------------------------------------------------*/
#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)
#include "ns_sleep.h"
#include "ns_ble.h"
#include "ns_ble_task.h"
#if (BLE_APP_SEC)
#include "ns_sec.h"                 // Application security Definition
#endif // (BLE_APP_SEC)
#if (NS_TIMER_ENABLE)
#include "ns_timer.h"
#endif //NS_TIMER_ENABLE


/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static __align(4) unsigned char  rwip_fifo_isr_codeArray[] ={
0x03,0xb4,0x01,0x48,0x01,0x90,0x01,0xbd,0xed,0xc3,0x02,0x00,0x10,0xb5,0x07,0x4c,
0xe0,0x69,0x00,0x04,0x08,0xd5,0xff,0xf7,0xf3,0xff,0x05,0x48,0x00,0x78,0x00,0x28,
0x02,0xd1,0x01,0x20,0xc0,0x03,0x20,0x62,0x10,0xbd,0x00,0x00,0x00,0x80,0x02,0x40,
0xc6,0x00,0x00,0x20};

/// List of functions used to create the database  
struct ns_ble_add_prf_evn_t  add_prf_evn;
struct ns_ble_prf_evn_t      ble_prf_evn;

/// Application Environment Structure
struct app_env_tag      app_env;
struct ns_gap_params_t  gap_env;
struct ns_adv_params_t  adv_env;
struct ns_scan_params_t scan_env;
/* Private function prototypes -----------------------------------------------*/
/// Application Task Descriptor
extern const struct ke_task_desc TASK_DESC_APP_M;
/* Private functions ---------------------------------------------------------*/

IRQ_HANNDLE_FUN user_HardFault_Handler = NULL;
IRQ_HANNDLE_FUN user_EXTI4_12_IRQHandler = NULL;
void ns_HardFault_Handler(void)
{
    uint32_t *stack_top = (uint32_t *)__get_MSP();
    NS_LOG_INIT();  
    NS_LOG_ERROR("**************HardFault ERROR:REG INFO**************.\r\n");                                                                                                                            \
    NS_LOG_ERROR("msp = 0x%08x.\r\n", (uint32_t)stack_top);                                                                                                                                                        \
    NS_LOG_ERROR("r0  = 0x%08x, \tr1   = 0x%08x.\r\n", stack_top[0], stack_top[1]);                                                                                                                    \
    NS_LOG_ERROR("r2  = 0x%08x, \tr3   = 0x%08x.\r\n", stack_top[2], stack_top[3]);                                                                                                                    \
    NS_LOG_ERROR("r12 = 0x%08x, \tlr   = 0x%08x.\r\n", stack_top[4], stack_top[5]);                                                                                                                    \
    NS_LOG_ERROR("pc  = 0x%08x, \txpsr = 0x%08x.\r\n", stack_top[6], stack_top[7]);  
   
    
    if(user_HardFault_Handler )
    {
        user_HardFault_Handler();//call user HardFault_Handler
    }
    
    NS_LOG_ERROR("%s\r\n",__func__);
    
    NVIC_SystemReset();    
    while(1)
    {}
}

void ns_EXTI4_12_IRQHandler(void)
{
    /*  EXTI line 11 for BLE Stack */
    if (RESET != EXTI_GetITStatus(EXTI_LINE11))
    {
        EXTI_ClrITPendBit(EXTI_LINE11);
    }
    /* NRST Wakeup Interrupt through EXTI line 12 */
    if (RESET != EXTI_GetITStatus(EXTI_LINE12))
    {
        EXTI_ClrITPendBit(EXTI_LINE12);   
        NS_LOG_INIT();
        NS_LOG_INFO("NRST->NVIC_SystemReset\r\n");
        NVIC_SystemReset();
    }
    
    if(user_EXTI4_12_IRQHandler)
    {
        user_EXTI4_12_IRQHandler();//call user EXTI4_12_IRQHandler
    }
}

static void ns_NRST_config(void)
{
    EXTI_InitType EXTI_InitStructure = {0};
    /* Enable the NRST Wakeup Interrupt through EXTI line 12 */
    EXTI_InitStructure.EXTI_Line    = EXTI_LINE12;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);
}


static void ns_ble_stack_vtor_init(void)
{
    extern uint32_t __Vectors;
    /*note: SPI1_IRQn and IRC_IRQn Interrupt not availed. */
    //init vtor address
    volatile uint32_t* p_IrqFun  = (void*)0x200009c0;
    uint32_t* user_vtor = (&__Vectors);

    //system Interrupt
    REG32(0x200000ec) = (uint32_t)user_vtor[16+NonMaskableInt_IRQn];
    REG32(0x200000f0) = (uint32_t)ns_HardFault_Handler;
    REG32(0x200000e8) = (uint32_t)user_vtor[16+SVCall_IRQn];
    REG32(0x200000f4) = (uint32_t)user_vtor[16+SysTick_IRQn];
    REG32(0x200000f8) = (uint32_t)user_vtor[16+PendSV_IRQn];
    //user Interrupt
    memcpy((void*)p_IrqFun,&user_vtor[16],124);
    //clear ble Interrupt
    p_IrqFun[BLE_SW_IRQn]               = 0;
    p_IrqFun[BLE_HSLOT_IRQn]            = 0;
    p_IrqFun[BLE_FINETGT_IRQn]          = 0;
    p_IrqFun[BLE_FIFO_IRQn]             = (uint32_t)(&rwip_fifo_isr_codeArray[0x0d]);
    p_IrqFun[BLE_ERROR_IRQn]            = 0;
    p_IrqFun[BLE_CRYPT_IRQn]            = 0;
    p_IrqFun[BLE_TIMESTAMP_TGT1_IRQn]   = 0;
    p_IrqFun[BLE_TIMESTAMP_TGT2_IRQn]   = 0;
    p_IrqFun[BLE_SLP_IRQn]              = (uint32_t)rwip_slp_isr; 
    
    p_IrqFun[EXTI4_12_IRQn]             = (uint32_t)ns_EXTI4_12_IRQHandler;
    
    //Register updated ble fifo irq function
    g_sch_prog_fifo_iar_branch = false;
    rwip_prog_delay = 3;
    //enable Interrupt of remaped in flash
    NVIC_EnableIRQ(BLE_FIFO_IRQn);
    NVIC_EnableIRQ(BLE_SLP_IRQn);
    NVIC_EnableIRQ(EXTI4_12_IRQn);

    //register HardFault and EXTI4_12 Interrupt if user define it
    uint32_t temp_addr  =(*(uint32_t *)&user_vtor[3]) - 1;
    uint16_t temp_cmd   = *((uint16_t*)temp_addr);
    if( temp_cmd != 0xe7fe) // ASM cmd B 
    {
        user_HardFault_Handler = (IRQ_HANNDLE_FUN)(*(uint32_t *)&user_vtor[3]);;
    }
    temp_addr  =(*(uint32_t *)&user_vtor[24]) - 1;
    temp_cmd   = *((uint16_t*)temp_addr);
    if( temp_cmd != 0xe7fe) // ASM cmd B 
    {
        user_EXTI4_12_IRQHandler = (IRQ_HANNDLE_FUN)(*(uint32_t *)&user_vtor[24]);
    }
    // enable NRST pin Interrupt
    ns_NRST_config();
}




static void app_create_advertising(void)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    if (app_env.adv_state == APP_ADV_STATE_IDLE )
    {
        // Prepare the GAPM_ACTIVITY_CREATE_CMD message
        struct gapm_activity_create_adv_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
                                                                  TASK_GAPM, TASK_APP,
                                                                  gapm_activity_create_adv_cmd);

        // Set operation code
        p_cmd->operation = GAPM_CREATE_ADV_ACTIVITY;

        // Fill the allocated kernel message
        p_cmd->own_addr_type                = gap_env.mac_addr_type;
        p_cmd->adv_param.type               = GAPM_ADV_TYPE_LEGACY;
        p_cmd->adv_param.prop               = GAPM_ADV_PROP_UNDIR_CONN_MASK;
        p_cmd->adv_param.filter_pol         = ADV_ALLOW_SCAN_ANY_CON_ANY;
        p_cmd->adv_param.disc_mode          = GAPM_ADV_MODE_GEN_DISC;        
        p_cmd->adv_param.prim_cfg.chnl_map  = ADV_ALL_CHNLS_EN;
        p_cmd->adv_param.prim_cfg.phy       = adv_env.adv_phy;
        
        if(adv_env.beacon_enable)
        {
            p_cmd->adv_param.prop           = GAPM_ADV_PROP_NON_CONN_NON_SCAN_MASK;
            p_cmd->adv_param.disc_mode      = GAPM_ADV_MODE_BEACON;
        }else
        if(adv_env.ex_adv_enable)
        {
            if( adv_env.adv_phy == PHY_2MBPS_VALUE)
            {
                p_cmd->adv_param.prim_cfg.phy   = PHY_1MBPS_VALUE;
            }
            p_cmd->adv_param.type               = GAPM_ADV_TYPE_EXTENDED;
            p_cmd->adv_param.prop               = ADV_CON;
            p_cmd->adv_param.second_cfg.phy     = adv_env.adv_phy;
            p_cmd->adv_param.second_cfg.adv_sid = 1;
            p_cmd->adv_param.second_cfg.max_skip = 0;
        }
        #if (BLE_APP_SEC)        
        if (ns_sec_get_bond_status())
        {
            ns_bond_last_bonded_peer_id(&p_cmd->adv_param.peer_addr);
        }
        else
        #endif
        {
            memcpy(p_cmd->adv_param.peer_addr.addr.addr,&gap_env.mac_addr,GAP_BD_ADDR_LEN);
            p_cmd->adv_param.peer_addr.addr_type = 0;
            
        }
        switch (app_env.adv_mode)
        {
            case APP_ADV_MODE_DIRECTED:
                /*
                 * If the peripheral is already bonded with a central device, use the direct advertising
                 * procedure (BD Address of the peer device is stored in NVDS.
                 */
                #if(BLE_APP_SEC)
                if (ns_sec_get_bond_status())
                {
                    p_cmd->adv_param.prop                   = GAPM_ADV_PROP_DIR_CONN_MASK;
                    p_cmd->adv_param.disc_mode              = GAPM_ADV_MODE_NON_DISC;  
                    p_cmd->adv_param.prim_cfg.adv_intv_min  = adv_env.directed_adv.adv_intv;
                    p_cmd->adv_param.prim_cfg.adv_intv_max  = adv_env.directed_adv.adv_intv;

                }
                else
                #endif    
                {
                    app_env.adv_mode = APP_ADV_MODE_FAST;
                    p_cmd->adv_param.prim_cfg.adv_intv_min = adv_env.fast_adv.adv_intv;
                    p_cmd->adv_param.prim_cfg.adv_intv_max = adv_env.fast_adv.adv_intv;
                }

                break;
            case APP_ADV_MODE_FAST:
                p_cmd->adv_param.prim_cfg.adv_intv_min = adv_env.fast_adv.adv_intv;
                p_cmd->adv_param.prim_cfg.adv_intv_max = adv_env.fast_adv.adv_intv;
                break;
            case APP_ADV_MODE_SLOW:
                p_cmd->adv_param.prim_cfg.adv_intv_min = adv_env.slow_adv.adv_intv;
                p_cmd->adv_param.prim_cfg.adv_intv_max = adv_env.slow_adv.adv_intv;
               
                break;                    
            default:
                break;
        }

        // Send the message
        ke_msg_send(p_cmd);

        // Keep the current operation
        app_env.adv_state = APP_ADV_STATE_CREATING;
        // And the next expected operation code for the command completed event
        app_env.current_op = CURRENT_OP_CREATE_ADV;
    }
}

static void app_delete_activity(void)
{

    // Prepare the GAPM_ACTIVITY_CREATE_CMD message
    struct gapm_activity_delete_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_DELETE_CMD,
                                                              TASK_GAPM, TASK_APP,
                                                              gapm_activity_delete_cmd);
    NS_LOG_DEBUG("%s\r\n", __func__);
    // Set operation code
    p_cmd->operation = GAPM_DELETE_ACTIVITY;
    p_cmd->actv_idx  = app_env.adv_actv_idx;

    // Send the message
    ke_msg_send(p_cmd);

    // Keep the current operation
    // And the next expected operation code for the command completed event
    app_env.current_op   = CURRENT_OP_DELETE_ADV;
   
    app_env.adv_state = APP_ADV_STATE_IDLE; //reset state
    //set next adv mode
    if(ke_state_get(TASK_APP) == APP_CONNECTED)
    {
        //connected, stop adv
        app_env.adv_mode = APP_ADV_MODE_STOP;
    }
    else
    {    //not connect yet, start next adv
        switch (app_env.adv_mode)
        {
            case APP_ADV_MODE_ENABLE:
                //restart adv
            case APP_ADV_MODE_DIRECTED:
                if(adv_env.fast_adv.enable)
                {
                    app_env.adv_mode = APP_ADV_MODE_FAST;
                    break;
                }
            case APP_ADV_MODE_FAST:
                if(adv_env.slow_adv.enable)
                {
                    app_env.adv_mode = APP_ADV_MODE_SLOW;
                    break;
                }
            case APP_ADV_MODE_SLOW:
                app_env.adv_mode = APP_ADV_MODE_STOP;
                break;
            default:
                app_env.adv_mode = APP_ADV_MODE_STOP;
                break;
        }
    }
    
}

static void app_start_advertising(void)
{  
    NS_LOG_DEBUG("%s\r\n", __func__);
    // Prepare the GAPM_ACTIVITY_START_CMD message
    struct gapm_activity_start_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_START_CMD,
                                                         TASK_GAPM, TASK_APP,
                                                         gapm_activity_start_cmd);

    p_cmd->operation = GAPM_START_ACTIVITY;
    p_cmd->actv_idx = app_env.adv_actv_idx;

    switch (app_env.adv_mode)
    {
        case APP_ADV_MODE_DIRECTED:
            NS_LOG_DEBUG("APP_ADV_MODE_DIRECTED ");
            p_cmd->u_param.adv_add_param.duration = SECS_TO_UNIT(adv_env.directed_adv.duration , SECS_UNIT_10MS);    
            break;
        case APP_ADV_MODE_FAST:
            NS_LOG_DEBUG("APP_ADV_MODE_FAST ");
            p_cmd->u_param.adv_add_param.duration = SECS_TO_UNIT(adv_env.fast_adv.duration, SECS_UNIT_10MS);
            break;
        case APP_ADV_MODE_SLOW:
            NS_LOG_DEBUG("APP_ADV_MODE_SLOW ");
            p_cmd->u_param.adv_add_param.duration = SECS_TO_UNIT(adv_env.slow_adv.duration, SECS_UNIT_10MS);
            break;
        default:
            p_cmd->u_param.adv_add_param.duration = 0;
            break;
    }
    NS_LOG_DEBUG(", duration: %d x10 ms\r\n",p_cmd->u_param.adv_add_param.duration);

    p_cmd->u_param.adv_add_param.max_adv_evt = 0;
    
    llhwc_modem_setmode(adv_env.adv_phy);
        
    // Send the message
    ke_msg_send(p_cmd);

    // Keep the current operation
    app_env.adv_state = APP_ADV_STATE_STARTING;
    // And the next expected operation code for the command completed event
    app_env.current_op = CURRENT_OP_START_ADV;
        
        
}


static void app_stop_advertising(void)
{

    // Prepare the GAPM_ACTIVITY_STOP_CMD message
    struct gapm_activity_stop_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_STOP_CMD,
                                                      TASK_GAPM, TASK_APP,
                                                      gapm_activity_stop_cmd);
    NS_LOG_DEBUG("%s\r\n", __func__);
    // Fill the allocated kernel message
    p_cmd->operation = GAPM_STOP_ACTIVITY;
    p_cmd->actv_idx = app_env.adv_actv_idx;

    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_STOPPING;
    // And the next expected operation code for the command completed event
    app_env.current_op   = CURRENT_OP_STOP_ADV;
}



static void app_set_adv_data(void)
{
    uint8_t data_len;
    if(adv_env.ex_adv_enable)
    {
        data_len = adv_env.ex_adv_data_len;
    }
    else{
        data_len = adv_env.adv_data_len;
    }
    // Prepare the GAPM_SET_ADV_DATA_CMD message
    struct gapm_set_adv_data_cmd *p_cmd = KE_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
                                                           TASK_GAPM, TASK_APP,
                                                           gapm_set_adv_data_cmd,
                                                           data_len);
    NS_LOG_DEBUG("%s\r\n", __func__);
    // Fill the allocated kernel message
    p_cmd->operation = GAPM_SET_ADV_DATA;
    p_cmd->actv_idx = app_env.adv_actv_idx;

    p_cmd->length = 0;

    // GAP will use 3 bytes for the AD Type
    if(adv_env.ex_adv_enable)
    {
        memcpy(&p_cmd->data[0],adv_env.ex_adv_p_data, adv_env.ex_adv_data_len);
    }
    else{
        memcpy(&p_cmd->data[0],adv_env.adv_data, adv_env.adv_data_len);
    }
    p_cmd->length += data_len;

    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_SETTING_ADV_DATA;
    // And the next expected operation code for the command completed event
    app_env.current_op   = CURRENT_OP_SET_ADV_DATA;
}

static void app_set_scan_rsp_data(void)
{
    uint8_t rem_len = ADV_DATA_LEN;
    
    // Prepare the GAPM_SET_ADV_DATA_CMD message
    struct gapm_set_adv_data_cmd *p_cmd = KE_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
                                                           TASK_GAPM, TASK_APP,
                                                           gapm_set_adv_data_cmd,
                                                           ADV_DATA_LEN);
    NS_LOG_DEBUG("%s\r\n", __func__);
    // Fill the allocated kernel message
    p_cmd->operation = GAPM_SET_SCAN_RSP_DATA;
    p_cmd->actv_idx = app_env.adv_actv_idx;

    p_cmd->length = adv_env.scan_rsp_data_len;
    memcpy(&p_cmd->data[0], adv_env.scan_rsp_data, adv_env.scan_rsp_data_len);

    // Get remaining space in the Advertising Data - 2 bytes are used for name length/flag
    rem_len -= p_cmd->length;
    
    uint8_t *p_buf = &p_cmd->data[p_cmd->length];
    // Check if additional data can be added to the Advertising data - 2 bytes needed for type and length
    if (rem_len > 3 && adv_env.attach_appearance)
    {
        // appearance length
        *p_buf = 3 ;
        // appearance flag
        *(p_buf + 1) = 0x19;//ADV_TYPE_APPEARANCE
        // Copy device name
        *(p_buf + 3) = (gap_env.appearance>>8)&0xff; //hight 8bit
        *(p_buf + 2) = gap_env.appearance&0xff;      //low 8bit
        // Update advertising data length
        p_cmd->length += 4;
        p_buf += 4;
        //Update Remaining Length
        rem_len -= 4;
    }
    // Check if additional data can be added to the Advertising data - 2 bytes needed for type and length
    if (rem_len > 2 && adv_env.attach_name)
    {
        uint8_t dev_name_length = co_min(gap_env.dev_name_len, (rem_len - 2));

        // Device name length
        *p_buf = dev_name_length + 1;
        // Device name flag (check if device name is complete or not)
        *(p_buf + 1) = (dev_name_length == gap_env.dev_name_len) ? '\x09' : '\x08';
        // Copy device name
        memcpy(p_buf + 2, gap_env.dev_name, dev_name_length);

        // Update advertising data length
        p_cmd->length += (dev_name_length + 2);
    }

    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_SETTING_SCAN_RSP_DATA;
    // And the next expected operation code for the command completed event
    app_env.current_op   = CURRENT_OP_SET_RSP_DATA;
}



static void app_send_gapm_reset_cmd(void)
{
    // Reset the stack
    struct gapm_reset_cmd *p_cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
                                                TASK_GAPM, TASK_APP,
                                                gapm_reset_cmd);

    p_cmd->operation = GAPM_RESET;
    ke_msg_send(p_cmd);
}


/**
 * @brief Initialize the BLE application task.
 **/
static void ns_ble_task_init(void)
{
    NS_LOG_DEBUG("%s\r\n", __func__);

    // Reset the application manager environment
    memset(&app_env, 0, sizeof(app_env));
    memset(&gap_env, 0, sizeof(gap_env));
    memset(&adv_env, 0, sizeof(adv_env));
    memset(&scan_env, 0, sizeof(scan_env));
    
    // Create APP task
    ke_task_create(TASK_APP, &TASK_DESC_APP_M);

    // Initialize Task state
    ke_state_set(TASK_APP, APP_INIT);

    app_env.adv_actv_idx = 0xFF;
    app_env.scan_actv_idx = 0xFF;
    app_env.init_actv_idx = 0xFF;

    for (uint8_t i =0;i<APP_CON_IDX_MAX;i++)
    {
        app_env.conn_env[i].conidx = GAP_INVALID_CONIDX;
    }
  
    // Reset the stack
    app_send_gapm_reset_cmd();
    
}

/**
 * @brief Add a required service in the database
 **/
bool ns_ble_add_svc(void)
{
    // Indicate if more services need to be added in the database
    bool more_svc = false;

    // Check if another should be added in the database
    if (app_env.next_svc < add_prf_evn.prf_num)
    {
        ASSERT_INFO(app_add_svc_func_list[app_env.next_svc] != NULL, app_env.next_svc, 1);

        // Call the function used to add the required service
        NS_LOG_DEBUG("%s\r\n", __func__);
        add_prf_evn.add_prf_func_list[app_env.next_svc]();

        // Select following service to add
        app_env.next_svc++;
        more_svc = true;
    }

    return more_svc;
}

bool ns_ble_add_prf_func_register(ns_ble_add_prf_func_t func)
{
    if(add_prf_evn.prf_num < BLE_NB_PROFILES)
    {
        //add profile to list 
        add_prf_evn.add_prf_func_list[add_prf_evn.prf_num] = func;
        add_prf_evn.prf_num++;
        return true;
    }

    return false;
}



bool ns_ble_prf_task_register(struct prf_task_t *prf)
{
    if(ble_prf_evn.prf_num < BLE_NB_PROFILES)
    {
        //add profile to list 
        memcpy(&ble_prf_evn.prf_task_list[ble_prf_evn.prf_num],
                prf,sizeof(struct prf_task_t));
        ble_prf_evn.prf_num++;
        return true;
    }

    return false;
}




/**
 * @brief Set slave latency without connection parameters update
 **/
void ns_ble_set_slave_latency(uint16_t latency_cfg)
{
    // Prepare the GAPC_SET_PREF_SLAVE_LATENCY message
    struct gapc_set_pref_slave_latency_cmd *p_cmd = KE_MSG_ALLOC(GAPC_SET_PREF_SLAVE_LATENCY_CMD,
                                                     KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                     gapc_set_pref_slave_latency_cmd);

    p_cmd->operation  = GAPC_SET_PREF_SLAVE_LATENCY;
    p_cmd->latency    = latency_cfg; 

    // Send the message
    ke_msg_send(p_cmd);
}

/**
 * @brief Send to request to update the connection parameters
 **/
bool ns_ble_update_param(struct gapc_conn_param *p_conn_param)
{
    // Prepare the GAPC_PARAM_UPDATE_CMD message
    struct gapc_param_update_cmd *p_cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                                                     KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                     gapc_param_update_cmd);

    NS_LOG_DEBUG("%s\r\n",__func__);
    
    /********** connection parameters requirement *************
    Interval Max * (Slave Latency + 1) <= 4 s
    Interval Max >= 7.5 ms
    Interval Min <= Interval Max
    ConnSupervisionTimeout <= 10 s
    Interval Max * ( Slave Latency + 1) * 3 < ConnSupervisionTimeout
    ****************************************************************/
    
    if(( p_conn_param->intv_max*(p_conn_param->latency+1) > 3200/* 4s */)  ||
       ( p_conn_param->intv_max < 6 /* 7.5ms */) ||     
       ( p_conn_param->intv_max < p_conn_param->intv_min) || 
       ( p_conn_param->time_out > 1000 /* 10s */)||  
       ( p_conn_param->intv_max*(p_conn_param->latency+1)*3) > (p_conn_param->time_out*8 /* 10/1.25 */))
    {
        NS_LOG_WARNING("request parameters error\r\n");
        KE_MSG_FREE(p_cmd);
        return false;
    }
    else{
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
    
    return true;
}

/**
 * @brief master device get peer info
 *  op:  GAPC_GET_PEER_FEATURES
 *       GAPC_GET_PEER_VERSION
 */
void ns_ble_get_peer_info(uint8_t op)
{
    NS_LOG_DEBUG("%s,app_env.conidx= %d\r\n", __func__,app_env.conidx);
    struct gapc_get_info_cmd *cmd = KE_MSG_ALLOC(GAPC_GET_INFO_CMD,
                                                     KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                     gapc_get_info_cmd);
    cmd->operation = op;
    // Send the message
    ke_msg_send(cmd);
}

/**
 * @brief set ble dle
 **/
void ns_ble_dle_set(uint16_t tx_octets, uint16_t tx_time)
{
    struct gapc_set_le_pkt_size_cmd *cmd = KE_MSG_ALLOC(GAPC_SET_LE_PKT_SIZE_CMD,
                                                     KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                     gapc_set_le_pkt_size_cmd);
    cmd->operation = GAPC_SET_LE_PKT_SIZE;
    cmd->tx_octets = tx_octets;
    cmd->tx_time   = tx_time;
    // Send the message
    ke_msg_send(cmd);
}


/**
 * @brief set ble mtu
 **/
void ns_ble_mtu_set(uint16_t mtu)
{
    
    struct gattc_exc_mtu_cmd *cmd = KE_MSG_ALLOC(GATTC_EXC_MTU_CMD,
                                                KE_BUILD_ID(TASK_GATTC, app_env.conidx), TASK_APP,
                                                gattc_exc_mtu_cmd);
    //ROM bug fix
    gapm_env.max_mtu = mtu;
        
    cmd->operation = GATTC_MTU_EXCH;
    cmd->seq_num = 0;
    ke_msg_send(cmd);
}



/**
 * @brief set ble phy 
 **/
void ns_ble_phy_set(enum gap_phy_val phy)
{
    struct gapc_set_phy_cmd *cmd = KE_MSG_ALLOC(GAPC_SET_PHY_CMD,
                                                KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                gapc_set_phy_cmd);

    cmd->operation = GAPC_SET_PHY;
    switch (phy)
    {
        case GAP_PHY_1MBPS:
        case GAP_PHY_2MBPS:
            cmd->rx_phy = phy;
            cmd->tx_phy = phy; 
            cmd->phy_opt = 0;
            break;
        case GAP_PHY_125KBPS:
            cmd->rx_phy = GAP_PHY_LE_CODED;
            cmd->tx_phy = GAP_PHY_LE_CODED; 
            cmd->phy_opt = PHY_OPT_S8_LE_CODED_TX_PREF;
            break;
        case GAP_PHY_500KBPS:
            cmd->rx_phy = GAP_PHY_LE_CODED;
            cmd->tx_phy = GAP_PHY_LE_CODED; 
            cmd->phy_opt = PHY_OPT_S2_LE_CODED_TX_PREF;
            break;
        default:
            cmd->rx_phy = GAP_PHY_LE_1MBPS;
            cmd->tx_phy = GAP_PHY_LE_1MBPS; 
            cmd->phy_opt = 0;
            break;
    }

    ke_msg_send(cmd);
}

/**
 * @brief avtive get ble RSSI with an interval in ms. 0ms mean active one time only
 **/
void ns_ble_active_rssi(uint32_t interval)
{
    struct gapc_get_info_cmd* info_cmd = KE_MSG_ALLOC(GAPC_GET_INFO_CMD,
                    KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                    gapc_get_info_cmd);

    // request RSSI
    info_cmd->operation = GAPC_GET_CON_RSSI;
    // send command
    ke_msg_send(info_cmd);   
    
    app_env.rssi_intv = interval;
}


void ns_ble_adv_fsm_next(void)
{
    NS_LOG_DEBUG("%s,adv_state:%d \r\n",__func__,app_env.adv_state);
    switch (app_env.adv_state)
    {
        case (APP_ADV_STATE_IDLE):
        {
            if(app_env.adv_mode != APP_ADV_MODE_STOP)
            {
                // Create advertising
                app_create_advertising();
            }
        } break;

        case (APP_ADV_STATE_CREATING):
        {
            if(app_env.adv_mode == APP_ADV_MODE_DIRECTED)
            {
                // Start advertising activity
                app_start_advertising();
            }
            else{
                // Set advertising data
                app_set_adv_data();
            }
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
              app_start_advertising();
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
            //delete activity when advertising stop
            app_delete_activity();

        } break;

        default:
        {
            ASSERT_ERR(0);
        } break;
    }
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

void ns_ble_stack_init(struct ns_stack_cfg_t const* p_init)
{
    ble_lsc_cfg_t lsc_set;
    while(!(RCC->CTRL&RCC_CTRL_HSERDF));//wait HSE ready
    ns_ble_stack_vtor_init();

    if( p_init->lsc_cfg <= BLE_LSC_LSE_32768HZ )
    {
        lsc_set = p_init->lsc_cfg;
    } 
    else
    {
        //set LSI 32000HZ as default
        lsc_set = BLE_LSC_LSI_32000HZ;
    }
    ns_ble_lsc_config(lsc_set);
    //enalbe lsi measurement
    if(lsc_set != BLE_LSC_LSE_32768HZ)   
    {
        ns_sleep_lock_acquire();//pending sleep for calib
        RCC->OSCFCCR &= ~(0xFF<< 8);
        RCC->OSCFCCR |= (LSI_CLOCK_CNT_CYCLES <<8);
        RCC->OSCFCCR |= 1;
    }
    NS_BLE_PATCH_INIT();
    NS_BLE_STACK_INIT();
    ns_ble_task_init();
    prf_init(RWIP_INIT);

    app_env.lsc_cfg = lsc_set; //store the lsc selected
    app_env.ble_msg_handler = p_init->ble_msg_handler;
    app_env.user_msg_handler = p_init->user_msg_handler;
    
}

void ns_ble_gap_init(struct ns_gap_params_t const* p_init)
{
    if(p_init != NULL)
    {
        memcpy(&gap_env,p_init,sizeof(struct ns_gap_params_t));
    }
    
    //generate local IRK
    uint8_t tmp_irk[KEY_LEN];
    memcpy(&tmp_irk[0], SEC_DEFAULT_IRK, KEY_LEN);
    for (uint8_t i=0;i<BD_ADDR_LEN;i++)
    {
    tmp_irk[i]|= gap_env.mac_addr.addr[i];
    }
    memcpy(&app_env.loc_irk[0], &tmp_irk[0], KEY_LEN);        

    //set ble address
    memcpy(g_co_default_bdaddr.addr, &gap_env.mac_addr, BD_ADDR_LEN);
    
}


void ns_ble_adv_init(struct ns_adv_params_t const* p_init)
{
    if(p_init != NULL)
    {
        memcpy(&adv_env,p_init, sizeof(struct ns_adv_params_t)); 
    }

    if(adv_env.adv_data_len > ADV_DATA_LEN-3)
    {
        adv_env.adv_data_len = ADV_DATA_LEN-3;
    }
    if(adv_env.scan_rsp_data_len > ADV_DATA_LEN)
    {
        adv_env.scan_rsp_data_len = ADV_DATA_LEN;
    }
    if(adv_env.ex_adv_data_len > EXT_ADV_DATA_MAX_LEN-3)
    {
        adv_env.ex_adv_data_len = EXT_ADV_DATA_MAX_LEN-3;
    }
   
}

void ns_ble_adv_data_set(uint8_t* p_dat, uint16_t len)
{
    adv_env.adv_data_len = len;
    if(adv_env.adv_data_len > ADV_DATA_LEN-3)
    {
        adv_env.adv_data_len = ADV_DATA_LEN-3;
    }
    
    memcpy(adv_env.adv_data,p_dat,adv_env.adv_data_len);
    
    if(app_env.adv_mode == APP_ADV_MODE_FAST || 
       app_env.adv_mode == APP_ADV_MODE_SLOW)
    {
        app_set_adv_data();
    }
}

void ns_ble_scan_rsp_data_set(uint8_t* p_dat, uint16_t len)
{
    adv_env.scan_rsp_data_len = len;
    if(adv_env.scan_rsp_data_len > ADV_DATA_LEN)
    {
        adv_env.scan_rsp_data_len = ADV_DATA_LEN;
    }
    
    memcpy(adv_env.scan_rsp_data,p_dat,adv_env.scan_rsp_data_len);
    
    if(app_env.adv_mode == APP_ADV_MODE_FAST || 
       app_env.adv_mode == APP_ADV_MODE_SLOW)
    {
        app_set_scan_rsp_data();
    }
}

void ns_ble_ex_adv_data_set(uint8_t* p_dat, uint16_t len)
{
    adv_env.ex_adv_data_len = len;
    if(adv_env.ex_adv_data_len > EXT_ADV_DATA_MAX_LEN-3)
    {
        adv_env.ex_adv_data_len = EXT_ADV_DATA_MAX_LEN-3;
    }
    adv_env.ex_adv_p_data = p_dat;
}

void ns_ble_adv_start(void)
{
    if(app_env.adv_state == APP_ADV_STATE_STOPPING)
    {
        app_env.adv_mode = APP_ADV_MODE_ENABLE;
        return;
    }

    switch (app_env.adv_mode)
    {
        case APP_ADV_MODE_IDLE:
            //on idle mode then enable.            
            app_env.adv_mode = APP_ADV_MODE_ENABLE;
            break;

        case APP_ADV_MODE_ENABLE:
        case APP_ADV_MODE_STOP:
            //on ready mode then start
            if(adv_env.directed_adv.enable)
            {
                app_env.adv_mode = APP_ADV_MODE_DIRECTED;
            }
            else if(adv_env.fast_adv.enable)
            {
                app_env.adv_mode = APP_ADV_MODE_FAST;
            }
            else if(adv_env.slow_adv.enable)
            {
                app_env.adv_mode = APP_ADV_MODE_SLOW;
            }
            else
            {
                app_env.adv_mode = APP_ADV_MODE_STOP;
            }
            app_create_advertising();
            break;
        default:
            //on started mode do nothing 
            break;
    }
    
    
}

void ns_ble_adv_stop(void)
{
    switch (app_env.adv_mode)
    {
        case APP_ADV_MODE_IDLE:
        case APP_ADV_MODE_ENABLE:
            //on idle and enable mode then disable
            app_env.adv_mode = APP_ADV_MODE_STOP;
            break;
        case APP_ADV_MODE_STOP:
            //on stop mode then do nothing     
            break;        
        case APP_ADV_MODE_DIRECTED:
        case APP_ADV_MODE_FAST:
        case APP_ADV_MODE_SLOW:
        default:
            //on started mode then stop
            app_env.adv_mode = APP_ADV_MODE_STOP;
            app_stop_advertising();
            break;
    }
}

void ns_ble_disconnect(void)
{
    struct gapc_disconnect_cmd *p_cmd = KE_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                                                   KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                   gapc_disconnect_cmd);

    NS_LOG_DEBUG("%s\r\n", __func__);
    p_cmd->operation = GAPC_DISCONNECT;
    p_cmd->reason    = CO_ERROR_REMOTE_USER_TERM_CON;

    // Send the message
    ke_msg_send(p_cmd);
}




/* Master device API*/

void ns_ble_scan_init(struct ns_scan_params_t *p_init)
{
    if(p_init != NULL)
    {
        memcpy(&scan_env,p_init, sizeof(struct ns_scan_params_t)); 
    }
    
}



void ns_ble_create_scan(void)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    // Prepare the GAPM_ACTIVITY_CREATE_CMD message
    struct gapm_activity_create_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
                                                              TASK_GAPM, TASK_APP,
                                                              gapm_activity_create_cmd);

    // Set operation code
    p_cmd->operation = GAPM_CREATE_SCAN_ACTIVITY;


    // Fill the allocated kernel message
    p_cmd->own_addr_type = gap_env.mac_addr_type;//GAPM_STATIC_ADDR;

    // Send the message
    ke_msg_send(p_cmd);
}

void ns_ble_start_scan(void)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    
    if(app_env.scan_actv_idx == 0xFF)
    {
        scan_env.scan_enable = true;
        //creat scan active if not created yet
        ns_ble_create_scan();
        return;
    }
    else if(!scan_env.scan_enable)
    {
        //scan already started
        return;
    }
    scan_env.scan_enable = false;
    // Prepare the GAPM_ACTIVITY_START_CMD message
    struct gapm_activity_start_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_START_CMD,
                                                         TASK_GAPM, TASK_APP,
                                                         gapm_activity_start_cmd);

    p_cmd->operation = GAPM_START_ACTIVITY;
    p_cmd->actv_idx = app_env.scan_actv_idx;

    p_cmd->u_param.scan_param.type                      = scan_env.type;
    p_cmd->u_param.scan_param.dup_filt_pol              = scan_env.dup_filt_pol;
    p_cmd->u_param.scan_param.scan_param_1m.scan_intv   = scan_env.scan_intv;
    p_cmd->u_param.scan_param.scan_param_1m.scan_wd     = scan_env.scan_wd;
    p_cmd->u_param.scan_param.duration                  = scan_env.duration;
    p_cmd->u_param.scan_param.period                    = scan_env.period;   
    
    if(scan_env.phy_coded_enable)
    {
        p_cmd->u_param.scan_param.prop                         = GAPM_SCAN_PROP_PHY_CODED_BIT;
        p_cmd->u_param.scan_param.scan_param_coded.scan_intv   = scan_env.scan_intv;
        p_cmd->u_param.scan_param.scan_param_coded.scan_wd     = scan_env.scan_wd;
        llhwc_modem_setmode(GAP_PHY_CODED);
    }
    else {
        p_cmd->u_param.scan_param.prop = GAPM_SCAN_PROP_PHY_1M_BIT;
        llhwc_modem_setmode(GAP_PHY_1MBPS);
    }
    if(scan_env.prop_active_enable)
    {
        p_cmd->u_param.scan_param.prop |= GAPM_SCAN_PROP_ACTIVE_1M_BIT|GAPM_SCAN_PROP_ACTIVE_CODED_BIT;
    }    

    // Send the message
    ke_msg_send(p_cmd);
    
    app_env.current_op      = CURRENT_OP_START_SCAN;
    app_env.target_found    = false;
}

void ns_ble_delete_scan(void)
{
    NS_LOG_DEBUG("%s\r\n", __func__);

    struct gapm_activity_delete_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_DELETE_CMD,
                                                          TASK_GAPM, TASK_APP,
                                                          gapm_activity_delete_cmd);

    // Set operation code
    p_cmd->operation = GAPM_DELETE_ACTIVITY;

    p_cmd->actv_idx = app_env.scan_actv_idx;
    
    // Send the message
    ke_msg_send(p_cmd);   
    app_env.current_op   = CURRENT_OP_NULL;
    app_env.scan_actv_idx = 0xFF;
}

void ns_ble_stop_scan(void)
{
    NS_LOG_DEBUG("%s\r\n", __func__);

    struct gapm_activity_stop_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_STOP_CMD,
                                                          TASK_GAPM, TASK_APP,
                                                          gapm_activity_stop_cmd);

    // Set operation code
    p_cmd->operation = GAPM_STOP_ACTIVITY;

    p_cmd->actv_idx = app_env.scan_actv_idx;
    
    // Send the message
    ke_msg_send(p_cmd);  
}

void ns_ble_create_init(void)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    // Prepare the GAPM_ACTIVITY_CREATE_CMD message
    struct gapm_activity_create_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
                                                          TASK_GAPM, TASK_APP,
                                                          gapm_activity_create_cmd);

    // Set operation code
    p_cmd->operation = GAPM_CREATE_INIT_ACTIVITY;


    // Fill the allocated kernel message
    p_cmd->own_addr_type = gap_env.mac_addr_type;//GAPM_STATIC_ADDR;

    // Send the message
    ke_msg_send(p_cmd);   
}

void ns_ble_start_init(uint8_t *addr, uint8_t addr_type)
{
    NS_LOG_DEBUG("%s\r\n", __func__);
    
    if(app_env.init_actv_idx == 0xFF)
    {
        //creat init active if not created yet        
        ns_ble_create_init();
        
        app_env.target_addr_type = addr_type;
        memcpy(app_env.target_addr.addr, addr, 6);       
        app_env.target_found            = true;
        scan_env.connect_enable   = true;
        return;
    }
    
    // Prepare the GAPM_ACTIVITY_START_CMD message
    struct gapm_activity_start_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_START_CMD,
                                                         TASK_GAPM, TASK_APP,
                                                         gapm_activity_start_cmd);

    p_cmd->operation = GAPM_START_ACTIVITY;
    p_cmd->actv_idx = app_env.init_actv_idx;

    p_cmd->u_param.init_param.type = GAPM_INIT_TYPE_DIRECT_CONN_EST;
    p_cmd->u_param.init_param.prop = GAPM_INIT_PROP_1M_BIT;
    p_cmd->u_param.init_param.scan_param_1m.scan_intv       = 0x320;
    p_cmd->u_param.init_param.scan_param_1m.scan_wd         = 0x320;
    p_cmd->u_param.init_param.conn_param_1m.conn_intv_min   = gap_env.dev_conn_param.intv_min; 
    p_cmd->u_param.init_param.conn_param_1m.conn_intv_max   = gap_env.dev_conn_param.intv_max;
    p_cmd->u_param.init_param.conn_param_1m.conn_latency    = gap_env.dev_conn_param.latency; 
    p_cmd->u_param.init_param.conn_param_1m.supervision_to  = gap_env.dev_conn_param.time_out; 
    p_cmd->u_param.init_param.conn_param_1m.ce_len_min      = 0;       
    p_cmd->u_param.init_param.conn_param_1m.ce_len_max      = 0x140; 

    p_cmd->u_param.init_param.peer_addr.addr_type = addr_type;

    memcpy(p_cmd->u_param.init_param.peer_addr.addr.addr, addr, 6);
    
    if(p_cmd->u_param.init_param.prop & GAPM_INIT_PROP_2M_BIT)
    {
        llhwc_modem_setmode(GAP_PHY_2MBPS);
    }
    else if(p_cmd->u_param.init_param.prop & GAPM_INIT_PROP_CODED_BIT)
    {
        llhwc_modem_setmode(GAP_PHY_CODED);
    }
    else{
        llhwc_modem_setmode(GAP_PHY_1MBPS);
    }
    // Send the message
    ke_msg_send(p_cmd);
    
    app_env.current_op = CURRENT_OP_START_INIT;
}

void ns_ble_delete_init(void)
{
    NS_LOG_DEBUG("%s\r\n", __func__);

    struct gapm_activity_delete_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_DELETE_CMD,
                                                          TASK_GAPM, TASK_APP,
                                                          gapm_activity_delete_cmd);

    // Set operation code
    p_cmd->operation = GAPM_DELETE_ACTIVITY;

    p_cmd->actv_idx = app_env.init_actv_idx;
    
    // Send the message
    ke_msg_send(p_cmd);
    
    app_env.current_op   = CURRENT_OP_NULL;
    app_env.init_actv_idx = 0xFF;
}


bool ns_ble_scan_data_find(uint8_t types, const uint8_t *p_filter_data, uint8_t *p_data, uint8_t len )
{
    uint8_t pack_len;
    uint8_t pack_type;
    while(len)
    {
        // check if pack incomplete
        if(len < (*p_data)+1 || len < 3)
        {
            break;
        }
        pack_len = (*p_data)-1; // get pack len
        p_data++;
        pack_type = *p_data; // get pack type
        p_data++;
        if(pack_type == types) //
        {
            if(memcmp(p_data,p_filter_data,pack_len) == 0)
            {
                return true;
            }
        }
        p_data += pack_len; //next pack
        len    -= (pack_len+2);
    }

    return false;
}

/* Used before connection operation or set connection parameter*/
void ns_ble_set_active_connection (uint8_t  conidx)
{
    app_env.conidx = conidx;
}

uint8_t ns_ble_get_active_connection (void)
{
    return app_env.conidx;
}

/* check the connection status by indx: true is connected,false is disconnected */
bool ns_ble_get_connection_state(uint8_t conidx)
{
    bool status = false;
    if ((app_env.conn_env[conidx].conidx != GAP_INVALID_CONIDX ) && (conidx < APP_CON_IDX_MAX) )
    {
        status = true;
    }
    return status;
}

/* Get the connection number*/
uint8_t ns_ble_get_connection_num (void)
{
    uint8_t  conidx_cnt = 0;
    for (uint8_t i =0;i<APP_CON_IDX_MAX;i++)
    {
        if (app_env.conn_env[i].conidx != GAP_INVALID_CONIDX)
            conidx_cnt ++;
    }
    return conidx_cnt;
}

/* Get the slave connection number, use at one master one slave example only*/
uint8_t ns_ble_get_conidx_by_role (uint8_t role)
{
    uint8_t  conidx = GAP_INVALID_CONIDX;
    for (uint8_t i =0;i<APP_CON_IDX_MAX;i++)
    {
        if ((app_env.conn_env[i].conidx != GAP_INVALID_CONIDX) && (app_env.conn_env[i].role == role))
        {
            conidx = app_env.conn_env[i].conidx;
            break;
        }
    }
    return conidx;
}
/* Get the master connection number, use at one master one slave example only*/
uint8_t ns_ble_master_connection_num (void)
{
    uint8_t  conidx_cnt = 0;
    for (uint8_t i =0;i<APP_CON_IDX_MAX;i++)
    {
        if ((app_env.conn_env[i].conidx != GAP_INVALID_CONIDX) && (app_env.conn_env[i].role == ROLE_MASTER))
                conidx_cnt ++;
    }
    return conidx_cnt;
}

/* Get the slave connection number*/
uint8_t ns_ble_slave_connection_num (void)
{
    uint8_t  conidx_cnt = 0;
    for (uint8_t i =0;i<APP_CON_IDX_MAX;i++)
    {
        if ((app_env.conn_env[i].conidx != GAP_INVALID_CONIDX) && (app_env.conn_env[i].role == ROLE_SLAVE))
                conidx_cnt ++;
    }
    return conidx_cnt;
}


void ns_ble_resolv_addr(struct gap_sec_key *irk, uint8_t nb_key,struct bd_addr* addr)
{
    // Prepare the GAPM_RESOLV_ADDR_CMD message
    struct gapm_resolv_addr_cmd *p_cmd = KE_MSG_ALLOC_DYN(GAPM_RESOLV_ADDR_CMD,
                                                     TASK_GAPM, TASK_APP,
                                                     gapm_resolv_addr_cmd,KEY_LEN *nb_key);
    // Set operation code
    p_cmd->operation  = GAPM_RESOLV_ADDR;
    p_cmd->nb_key  = nb_key;

    memcpy(&(p_cmd->addr.addr[0]), addr, sizeof(bd_addr_t));
    memcpy(&(p_cmd->irk[0]), irk, KEY_LEN*nb_key);
    NS_LOG_DEBUG("%s\r\n", __func__);

    // Send the message
    ke_msg_send(p_cmd);
}

void ns_ble_list_set_ral(struct gap_ral_dev_info *ral_list,uint8_t ral_cnt)
{
    struct gapm_list_set_ral_cmd *p_cmd = KE_MSG_ALLOC_DYN(GAPM_LIST_SET_CMD,
                                                       TASK_GAPM, TASK_APP,
                                                       gapm_list_set_ral_cmd,ral_cnt * sizeof(struct gap_ral_dev_info));
    
    
    NS_LOG_DEBUG("%s,ral_cnt:%d\r\n", __func__,ral_cnt);
    p_cmd->operation = GAPM_SET_RAL;
    p_cmd->size = ral_cnt;
    for (uint8_t cnt = 0; cnt < ral_cnt; cnt++)
    {
        ral_list[cnt].priv_mode = 0;
        memcpy(&p_cmd->ral_info[cnt], &ral_list[cnt], sizeof(struct gap_ral_dev_info));  
        NS_LOG_DEBUG("set_ral:%d,%d,%d,%d,%d\r\n",ral_list[cnt].priv_mode,ral_list[cnt].local_irk[0],ral_list[cnt].peer_irk[0],\
        ral_list[cnt].addr.addr_type,ral_list[cnt].addr.addr.addr[0]);
    }

    ke_msg_send(p_cmd);
}


void ns_ble_prod_test_cmd_send(struct gapm_le_test_mode_ctrl_cmd *p_params, bool cw_mode)
{
    struct gapm_le_test_mode_ctrl_cmd *p_cmd = KE_MSG_ALLOC(GAPM_LE_TEST_MODE_CTRL_CMD,
                                                     TASK_GAPM, TASK_APP,
                                                     gapm_le_test_mode_ctrl_cmd);
    
    if(cw_mode)
    {
        REG32(MODEM_BASE + 0xc6 * 4)  = 0;
        REG32(MODEM_BASE + 0xc7 * 4)  = 0;
        REG32(MODEM_BASE + 0xc8 * 4)  = 0;
        ble_rftestcntl_infinitetx_setf(1);
        p_params->tx_pkt_payload = 0;
        p_params->operation = GAPM_LE_TEST_TX_START;
    }
    
    memcpy(p_cmd,p_params,sizeof(struct gapm_le_test_mode_ctrl_cmd));
    llhwc_modem_setmode(p_params->phy);
    // Send the message
    ke_msg_send(p_cmd);
}  

#endif //(BLE_APP_PRESENT)

/// @} APP
