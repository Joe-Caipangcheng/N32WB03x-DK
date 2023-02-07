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
 * @file main.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "n32wb03x.h"
#include "ble_stack_common.h"
#include "prf.h"
#include "ns_sleep.h"
#include <co_bt.h>
#include "bsp_led.h"
#include "bsp_lpuart.h"
#include "bsp_button.h"
#include "bsp_wdt.h"
#include "app.h"
#include "reg_ipcore.h"
#include "ns_delay.h"
#include "mesh_app.h"
#include "mesh_app_task.h"
#include "utils.h"
#include "rwip.h"
#include "app_task.h"
#include "Typedefine.h"
#include "app_onoff.h"
#include "generic_onoff_client.h"
#include "ke_timer.h"
#include "global_var.h"
#include "rwip_int.h"



extern struct rwip_env_tag rwip_env;
extern otp1_stored_t otp1_stored;
extern void system_opt_read(uint8_t* p_data,uint32_t byte_length);

static void models_init(void);
static void HardFault_Handler_CC(void);
static uint8_t tid = 0;
static void button_evt(uint8_t evt);





/**
 * @brief  A method to get the mac address.
 * @note   This function should only be called at initialization, since the address will be used later.
 */
void make_ble_mac(void)
{
    system_opt_read((uint8_t*)&otp1_stored,sizeof(otp1_stored));
    memcpy(g_co_default_bdaddr.addr,&otp1_stored.flash_uuid[5],6);
    
    
}




/**
 * @brief  Main Entry.
 */
int main(void)
{
    //Redirect Hardfault function.
    REG32(0x200000f0) =(uint32_t) HardFault_Handler_CC;
    //Initialize ble stack.
    NS_BLE_STACK_INIT();
    //Setting RF TX Power
    rf_tx_power_set(TX_POWER_0_DBM);
    //Create a unique ble mac address.
    make_ble_mac(); 
    //Initialize uart for logging.
    bsp_lpuart_init();
    printf("application start...\r\n"); 
    //Initialize evaluation board button driver. 
    bsp_button_init(button_evt);    
    //Initialize evaluation board led driver.
    bsp_leds_config();
    //Initialize watch dog timer.
    bsp_wdt_init();
    //Initialize Q-spi Flash driver.
    Qflash_Init();
    //If Flash Content Causing Reset, Uncomment The Function Below. After Recover, Comment It Again.
//    ns_mesh_nvds_clear();
    //Initalize ble app task.
    app_init();   
    //Initialze mesh stack.
    ns_mesh_stack_init(models_init);
    //Enable mesh stack
    ns_mesh_stack_enable();
    //Start watch dog timer
    bsp_wdt_start();
   
    while(1){
        //Fork and handle events.
        rwip_schedule();
    }
}



__asm int return_sp()
{
    MOV r0, sp
    BX LR
}   
__asm uint32_t getStackedReg(uint32_t regnum, uint32_t u32SP)
{
    lsls    r0, r0, #2
    adds    r0, r0, r1
    ldr     r0, [r0]
    bx      lr
}
/**
 * @brief  Handle the exceptions.
 */
static void HardFault_Handler_CC(void)
{

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);    
    uint32_t sp = return_sp();
	printf("HardFault_Handler_CC --> 0x%08X\r\n",sp);   
    printf("r0 --> 0x%08X\r\n", getStackedReg(0, sp));
    printf("r1 --> 0x%08X\r\n", getStackedReg(1, sp));
    printf("r2 --> 0x%08X\r\n", getStackedReg(2, sp));
    printf("r3 --> 0x%08X\r\n", getStackedReg(3, sp));  
    printf("r12 --> 0x%08X\r\n",getStackedReg(4, sp));
    printf("lr --> 0x%08X\r\n", getStackedReg(5, sp));
    printf("pc --> 0x%08X\r\n", getStackedReg(6, sp));
    printf("psr --> 0x%08X\r\n",getStackedReg(7, sp));

    NVIC_SystemReset();

	while(1);
}



/**
 * @brief	An application function used to change the LED state. After the generic onoff server model received
 *			a set message, this function will be called at application layer.
 * @param  	p_server Pointer to app_onoff_server_t context.
 * @param  	onoff onoff state to be present
 * @note 	User can change this function to use the custom LED pins.
 */               
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff)
{
    if(onoff){
        bsp_led_on(LED2_GPIO_PORT, LED_GPIO2_PIN);
    }else{
        bsp_led_off(LED2_GPIO_PORT, LED_GPIO2_PIN);
    }
}
/**
 * @brief	After the generic onoff server model received a get message, this function will be called at application layer.
 * @param  	p_server Pointer to app_onoff_server_t context.
 * @param  	p_present_onoff current onoff state value
*/
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff)
{
    *p_present_onoff = bsp_led_read(LED2_GPIO_PORT, LED_GPIO2_PIN);
}
APP_ONOFF_SERVER_DEF(m_onoff_server_0,
                     false,
                     MESH_TRANSMIC_SIZE_SMALL,
                     app_onoff_server_set_cb,
                     app_onoff_server_get_cb)

static generic_onoff_client_t m_clients = {
    .settings.p_callbacks = NULL,
    .settings.timeout = 0,
    .settings.force_segmented = false,
    .settings.transmic_size = MESH_TRANSMIC_SIZE_SMALL,
};
/**
 * @brief	Generic onoff model init. In this function, generic onoff server and client models will be created.
*/
static void models_init(void)
{
    app_onoff_init(&m_onoff_server_0, 0);    
    generic_onoff_client_init(&m_clients, 0);    
}
/**
 * @brief  Send out an onoff set message at the generic onoff client model.
 * @param  onoff if true, let the light be on, if false, the light will be off.
 */
static void generic_onoff_set(bool onoff)
{
    static uint8_t tid = 0;
    generic_onoff_set_params_t set_params;
    set_params.on_off = onoff;  
    set_params.tid = tid++;
    generic_onoff_client_set_unack(&m_clients, &set_params, NULL, 0);        
}




/**
 * @brief  Handle button events.
 * @param  evt button event type 
 */
static void button_evt(uint8_t evt)
{
    switch(evt){
        case BUTTON_EVT_ONE_SHORT_RELEASE:{
            generic_onoff_set(true);           
        }break;
        case BUTTON_EVT_TWO_SHORT_RELEASE:{
            generic_onoff_set(false);
        }break;    
        case BUTTON_EVT_TWO_LONG_PUSH:{
            uint32_t was_masked;
            _DISABLE_IRQS(was_masked);              
            ns_mesh_nvds_clear();  
            NVIC_SystemReset();
        }break;        
        
    }
}


