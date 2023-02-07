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
 * @file ns_sleep.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "rwip.h"
#include "global_var.h"
#include "global_func.h"
#include "Typedefine.h"
#include "ns_sleep.h"
#include "bsp_led.h"
#include "ns_delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define IP_DEEPSLCNTL_ADDR   BASEBAND_REG_BASE +0x30

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t ns_sleep_lock = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Acquire a sleep lock, it will prevent the os enter sleep mode. 
 *         We should call ns_sleep_lock_release function when this lock can be release. 
 * @param  
 * @return 
 * @note   
 */
uint8_t ns_sleep_lock_acquire(void)
{
    if(ns_sleep_lock++ == 0)
    {
        //overflow
        return false;
    }
    return true;
}

/**
 * @brief  Release a sleep lock, if all the lock has been released, os will enter sleep mode 
 *         when run out of task.
 * @param  
 * @return 
 * @note   
 */
uint8_t ns_sleep_lock_release(void)
{
    if(ns_sleep_lock)
    {
        ns_sleep_lock--;
        return true;
    }
    return false;
;
}


/**
 * @brief  check_calib_clk
 * @param  
 * @return 
 * @note   
 */
void check_calib_clk(void)
{
    int count_value = 0;
    int delta = 0;    

    if((RCC->OSCFCSR & 0x02))
    {
        count_value = RCC->OSCFCHSICNT;     
        delta = abs(count_value - g_cal_hsi_cnt_value); 
        if(delta > g_cal_hsi_cnt_value * 3/100)
        {
            g_recalib_hsi_flag = 1;
        }
        count_value = RCC->OSCFCLSICNT;  
    }

    if((RCC->OSCFCSR & 0x01))
    {
        count_value = RCC->OSCFCLSICNT; 
        g_lsi_1_syscle_cnt_value = (count_value / g_lsi_count_n_syscle) + (count_value % g_lsi_count_n_syscle) / (g_lsi_count_n_syscle/2);   
    }
    g_lsi_1_syscle_cnt_value = 1000;
}

/**
 * @brief  entry_wakeup
 * @param  
 * @return 
 * @note   
 */
void entry_wakeup(void)
{
    //re write LSI TRIM VAL
    uint32_t temp_RCC_LSCTRL = RCC->LSCTRL ;
    temp_RCC_LSCTRL = temp_RCC_LSCTRL & (~(0x7FF << 8));
    temp_RCC_LSCTRL = temp_RCC_LSCTRL | (g_rc32k_trim << 8);
    RCC->LSCTRL = temp_RCC_LSCTRL;    
        
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    PWR->CR2 |= 1<<8;
}

/**
 * @brief  entry_sleep
 * @param  
 * @return 
 * @note   
 */
void entry_sleep(void)
{
    g_sleep_status_flag = 1;
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    REG32(0x40028030) |=  0x07;

    while(PWR->CR1&0x20);

    EXTI_PA11_Configuration();  

    PWR->CR1 |= 0x0A;
    SCB->SCR |= 0x04;
    __WFI(); 

    entry_wakeup();
}

/**
 * @brief  entry_standby
 * @param  
 * @return 
 * @note   
 */
void entry_standby(void)
{
    EXTI_PA11_Configuration(); 

    SCB->SCR &= 0xFB;
    __WFI();
}


/**
 * @brief  sleep task function, usually run after rwip_schedule function in main loop.
 * @param  
 * @return 
 * @note   
 */

void ns_sleep(void)
{
    GLOBAL_INT_DISABLE();    
    switch(rwip_sleep())
    {
        case RWIP_DEEP_SLEEP:
        {
            bsp_led_on(LED2_GPIO_PORT, LED_GPIO2_PIN);
            entry_sleep();
            bsp_led_off(LED2_GPIO_PORT, LED_GPIO2_PIN);
        }
        break;
        case RWIP_CPU_SLEEP:
        {
            entry_standby();
        }
        break;
        case RWIP_ACTIVE:
        default:
        {

        }
        break;
    }

    GLOBAL_INT_RESTORE();  
}

/**
 * @brief  User code beofre enter sleep mode
 * @param  
 * @return 
 * @note   
 */
__weak void app_sleep_prepare_proc(void)
{
    
}

/**
 * @brief  User code after out of sleep mode. This function run after interrupt
 *         handler function if any interrupt pending when sleep.
 * @param  
 * @return 
 * @note   
 */
__weak void app_sleep_resume_proc(void)
{
    
}

/**
 * @}
 */

