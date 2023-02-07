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
 * @file bsp_button.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

#include "bsp_button.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "n32wb03x.h"
#include "toolchain.h"
#include "stdio.h"
#include "mesh_app_task.h"
#include "ke_timer.h"
#include "co_utils.h"





typedef struct
{
    GPIO_Module* GPIOx; //GPIO port
    uint16_t Pin;       //GPIO pin
    uint8_t PortSource; //port source
    uint8_t PinSource;  //pin source
    uint32_t EXTI_Line; //exti line
    IRQn_Type NVIC_IRQChannel; //nvic channel
    uint8_t evt;
}KeyInit_t; //Key exti interrupt structure
static KeyInit_t *m_button;
static uint32_t m_count = 0;
static void (*m_callback)(uint8_t button_evt);
#define BTN_LONG_PUSH_COUNT       30



KeyInit_t key_1 = {
	.GPIOx = GPIOB,
	.Pin = GPIO_PIN_1,
	.PortSource = GPIOB_PORT_SOURCE,
	.PinSource = GPIO_PIN_SOURCE1,
	.EXTI_Line = EXTI_LINE1,
	.NVIC_IRQChannel = EXTI0_1_IRQn,
    .evt = BUTTON_EVT_ONE_SHORT_RELEASE,
};
KeyInit_t key_2 = {
	.GPIOx = GPIOB,
	.Pin = GPIO_PIN_2,
	.PortSource = GPIOB_PORT_SOURCE,
	.PinSource = GPIO_PIN_SOURCE2,
	.EXTI_Line = EXTI_LINE2,
	.NVIC_IRQChannel = EXTI2_3_IRQn,
    .evt = BUTTON_EVT_TWO_SHORT_RELEASE,
};
static void gpio_config(KeyInit_t *init);

/*Button delay timer handler*/
int mesh_task_button_timer_handler(ke_msg_id_t const msgid,void *p_param,ke_task_id_t const dest_id,ke_task_id_t const src_id,enum ke_msg_status_tag *msg_ret)
{
    bool polarity = GPIO_ReadInputDataBit(m_button->GPIOx, m_button->Pin);
    if(polarity == 1)
    {
        m_count = 0;
        m_callback(m_button->evt);
    }
    else
    {
        m_count++;
        if(m_count > BTN_LONG_PUSH_COUNT)
        {     
            m_count = 0;
            m_callback(m_button->evt+100);		
        }else{
            ke_timer_set(MESH_TASK_BUTTON_TIMER, TASK_MESH, 50); 
        }
    }
    return KE_MSG_CONSUMED;
}

static inline void button_handler(KeyInit_t *key)
{
    m_button = key;
    m_count = 0;
    ke_timer_set(MESH_TASK_BUTTON_TIMER, TASK_MESH, 50);     
}
/*Gpio exti handler*/
static void EXTI0_1_IRQHandler(void)
{
    if (RESET != EXTI_GetITStatus(key_1.EXTI_Line))
    {
        button_handler(&key_1);
        EXTI_ClrITPendBit(key_1.EXTI_Line);     
    }	
}
/*Gpio exti handler*/
static void EXTI2_3_IRQHandler(void)
{
    if (RESET != EXTI_GetITStatus(key_2.EXTI_Line))
    {
        button_handler(&key_2);   
        EXTI_ClrITPendBit(key_2.EXTI_Line);   
    }	
}
/*Button init*/
void bsp_button_init(void (* callback)(uint8_t button_evt))
{
	m_callback = callback;
	ModuleIrqRemoval(EXTI2_3_IRQn);
	ModuleIrqRegister(EXTI2_3_IRQn, EXTI2_3_IRQHandler);        
	ModuleIrqRemoval(EXTI0_1_IRQn);
	ModuleIrqRegister(EXTI0_1_IRQn, EXTI0_1_IRQHandler);  
    gpio_config(&key_1);
    gpio_config(&key_2);

}



/*Gpio exti init*/
static void gpio_config(KeyInit_t *init)
{
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    if (init->GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_AFIO, ENABLE);
    }
    else if (init->GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);
    }
    else
    {
        return;
    }

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin          = init->Pin;
    GPIO_InitStructure.GPIO_Pull    = GPIO_PULL_UP;
    GPIO_InitPeripheral(init->GPIOx, &GPIO_InitStructure);
    
    GPIO_ConfigEXTILine(init->PortSource, init->PinSource);

    EXTI_InitStructure.EXTI_Line    = init->EXTI_Line;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = init->NVIC_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


















