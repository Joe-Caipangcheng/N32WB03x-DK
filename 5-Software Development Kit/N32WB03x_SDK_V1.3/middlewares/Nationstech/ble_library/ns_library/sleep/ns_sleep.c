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
 * @version v1.0.3
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include "ns_sleep.h"
#include "ns_ble.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t ns_sleep_lock = 0;
volatile uint16_t *p_prevent_sleep = &rwip_env.prevent_sleep;
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
}


/**
 * @brief  entry_sleep
 * @param  
 * @return 
 * @note   
 */
void entry_sleep(void)
{
    RCC->APB1PCLKEN |= RCC_APB1_PERIPH_PWR; // PWR enable    
    REG32(0x40028030) |=  0x07;
    REG32(0x40011004) = 0x00; 
   
    while(PWR->CR1&0x20); //wait ble sleep

    EXTI_PA11_Configuration();  
    g_sleep_status_flag = 1;

    PWR->CR1  = 0x0A;
    SCB->SCR |= 0x04;
    __WFI(); 

    RCC->LSCTRL |= 1;   //rewrite LSCTRL
    RCC->CFG |=  RCC_HCLK_DIV2; //APB1 = HCLK/2, APB1 max is 32M

    RCC->APB1PCLKEN |= RCC_APB1_PERIPH_PWR; // PWR enable
    PWR->CR2 |= 0x100; //1<<8
    while(!(RCC->CTRL&RCC_CTRL_HSERDF));//wait HSE ready
    while(!(PWR->CR1&0x20));//wait ble active
}

/**
 * @brief  entry Idle mode
 * @param  
 * @return 
 * @note   
 */
void entry_idle(void)
{
    EXTI_PA11_Configuration(); 
    PWR->CR1 &= 0xF0;
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
    
    if(ns_sleep_lock != 0)
    {
        return;
    }
    GLOBAL_INT_DISABLE();
    app_sleep_prepare_proc();
    NS_LOG_DEINIT();
    switch(rwip_sleep())
    {
        case RWIP_DEEP_SLEEP:
        {
            entry_sleep();
        }
        break;
        case RWIP_CPU_SLEEP:
        {
            entry_idle();
        }
        break;
        case RWIP_ACTIVE:
        default:
        {

        }
        break;
    }
    NS_LOG_INIT();
    GLOBAL_INT_RESTORE();

    /* check ble out of sleep */
    if(((*p_prevent_sleep) & (RW_WAKE_UP_ONGOING|RW_DEEP_SLEEP) )) 
    {
        uint32_t wait_sleep = 12800; //2*800us
        while((*p_prevent_sleep) & (RW_WAKE_UP_ONGOING|RW_DEEP_SLEEP))
        {
            wait_sleep--;
            if(wait_sleep == 0)
            {
                break;
            }
        }
        rwip_time_t current_time = rwip_time_get();
        // check if 1ms timer is active and make sure ke timer active.
        if(rwip_env.timer_1ms_target.hs != RWIP_INVALID_TARGET_TIME)
        {
            int32_t duration = CLK_DIFF(current_time.hs, rwip_env.timer_1ms_target.hs);
            if(duration < 0 )
            {
                // Mark that 1ms timer is over
                ke_event_set(KE_EVENT_KE_TIMER);
            }
        }
    }
    
    app_sleep_resume_proc(); 

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

