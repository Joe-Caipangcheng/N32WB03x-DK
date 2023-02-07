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
 * @file app_gpio.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "app_gpio.h"
#include <stdio.h>
#include "global_func.h"
#include "app_hid.h"
#include "ns_log.h"
#include "ns_timer.h"
#include "ns_delay.h"
#include "app_ble.h"
/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define KEY_PRESS_DELAY 100
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t key1_irq_actived = 0;
uint8_t key2_irq_actived = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/**
 * @brief  Configures LED GPIO.
 * @param Led Specifies the Led to be configured.
 *   This parameter can be GPIO_PIN_0~GPIO_PIN_13.
 */
void LedInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    /* Enable the GPIO Clock */
    if (GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    else
    {
        return;
    }

    /* Configure the GPIO pin as output push-pull */
    if (Pin <= GPIO_PIN_ALL)
    {
        GPIO_InitStruct(&GPIO_InitStructure);
        GPIO_InitStructure.Pin = Pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
    }
}
/**
 * @brief  Turns selected Led on.
 * @param GPIOx x can be A to B to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_13.
 */
void LedOn(GPIO_Module* GPIOx, uint16_t Pin)
{    
    GPIO_SetBits(GPIOx, Pin);
}

/**
 * @brief  Turns selected Led Off.
 * @param GPIOx x can be A to B to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_13.
 */
void LedOff(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_ResetBits(GPIOx, Pin);
}

/**
 * @brief  Toggles the selected Led.
 * @param GPIOx x can be A to B to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_13.
 */
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_TogglePin(GPIOx, Pin);
}

/**
 * @brief  Configures key port.
 */
void app_key_configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    /* Enable the GPIO Clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);

    /*Configure the GPIO pin as input floating */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin          = KEY1_INPUT_PIN;
    GPIO_InitStructure.GPIO_Pull    = GPIO_PULL_UP;
    GPIO_InitPeripheral(KEY1_INPUT_PORT, &GPIO_InitStructure);

    /* Enable the GPIO Clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_AFIO, ENABLE);

    /*Configure the GPIO pin as input floating*/
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin          = KEY2_INPUT_PIN;
    GPIO_InitStructure.GPIO_Pull    = GPIO_PULL_UP;
    GPIO_InitPeripheral(KEY2_INPUT_PORT, &GPIO_InitStructure);

    /*Configure key EXTI Line to key input Pin*/
    GPIO_ConfigEXTILine(KEY1_INPUT_PORT_SOURCE, KEY1_INPUT_PIN_SOURCE);
    GPIO_ConfigEXTILine(KEY2_INPUT_PORT_SOURCE, KEY2_INPUT_PIN_SOURCE);


    /*Configure key EXTI line*/
    EXTI_InitStructure.EXTI_Line    = KEY1_INPUT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    /*Set key input interrupt priority*/
    NVIC_InitStructure.NVIC_IRQChannel                   = KEY1_INPUT_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
        
    /*Configure key EXTI line*/
    EXTI_InitStructure.EXTI_Line    = KEY2_INPUT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    /*Set key input interrupt priority*/
    NVIC_InitStructure.NVIC_IRQChannel                   = KEY2_INPUT_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);        
    
}

/**
 * @brief  re-configures key port aafter sleep.
 */
void app_key_reinit_after_sleep(void)
{
    /* Enable the GPIO Clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);

}

/**
 * @brief key press timer handler.
 */
void app_key_press_timeout_handler(void)
{
 
    if(key1_irq_actived == 1)
    {
        LedBlink(LED1_PORT, LED1_PIN);
        #if (CFG_APP_HID)
        //send move x up
        struct ps2_mouse_msg msg = {0};
        msg.x = 10;
        app_hid_send_mouse_report(msg);

        #endif                        
        NS_LOG_INFO("Button 1, mouse moving\r\n");
        key1_irq_actived = 0;
    }
    else if(key1_irq_actived == 2 ) 
    {
        //re-start timer if got irq after first created timer
        ns_timer_create(KEY_PRESS_DELAY,app_key_press_timeout_handler);
        key1_irq_actived = 1;
    }
    
    if(key2_irq_actived == 1)
    {
        LedBlink(LED1_PORT, LED1_PIN);
        #if (CFG_APP_HID)   

        uint8_t key = 0;
        key = 0x08; //Volume Down
//            key = 0x10; //Volume up
//            key = 0x20; //Mute
        app_hid_send_consumer_report((uint8_t*)&key);              
        key = 0x0; //send release key
        app_hid_send_consumer_report((uint8_t*)&key); 
        #endif
        NS_LOG_INFO("Button 2, Volume Down\r\n");
        key2_irq_actived = 0;
    }
    else if(key2_irq_actived == 2) 
    {
        //re-start timer if got irq after first created timer
        ns_timer_create(KEY_PRESS_DELAY,app_key_press_timeout_handler);
        key2_irq_actived = 1;
    }

}

/**
 * @brief  External lines 1 interrupt.
 */
void EXTI0_1_IRQHandler(void)
{
    
    if ( EXTI_GetITStatus(KEY1_INPUT_EXTI_LINE)!= RESET)
    {
        
        if(key1_irq_actived == 0)
        {
            ke_msg_send_basic(APP_KEY_DETECTED, TASK_APP, TASK_APP); 
            key1_irq_actived = 1;
        }
        else if(key1_irq_actived == 1)
        {
            //if timer started, update flag
            key1_irq_actived = 2;

        }
        EXTI_ClrITPendBit(KEY1_INPUT_EXTI_LINE);
    }
}

/**
 * @brief  External lines 2 interrupt.
 */
void EXTI2_3_IRQHandler(void)
{
    if ( EXTI_GetITStatus(KEY2_INPUT_EXTI_LINE)!= RESET)
    {
        
        if(key2_irq_actived == 0)
        {
            ke_msg_send_basic(APP_KEY_DETECTED, TASK_APP, TASK_APP); 
            key2_irq_actived = 1;
        }
        else if(key2_irq_actived == 1)
        {
            //if timer started, update flag
            key2_irq_actived = 2;
        }
        EXTI_ClrITPendBit(KEY2_INPUT_EXTI_LINE);
    }
}


/**
 * @}
 */
