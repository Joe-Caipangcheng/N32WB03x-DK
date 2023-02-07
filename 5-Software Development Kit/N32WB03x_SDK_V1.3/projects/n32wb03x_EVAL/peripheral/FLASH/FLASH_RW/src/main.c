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
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "n32wb03x.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LED1_PORT GPIOB
#define LED1_PIN  GPIO_PIN_0
#define LED2_PORT GPIOA
#define LED2_PIN  GPIO_PIN_6

#define FLASH_TEST_ADDRESS       0x1020000
#define BUFFER_SIZE              4096
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t m_buffer[BUFFER_SIZE];
//Allocate a buffer for read flash memory and for compare with a static buffer.
static uint8_t buffer[BUFFER_SIZE];
GPIO_InitType GPIO_InitStructure;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static void assert_handle(void)
{
    GPIO_SetBits(LED1_PORT, LED1_PIN);
    while(1){

    }
}


/**
 * @brief  Main program.
 */
int main(void)
{
    //Initialize dev board leds
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);    
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);    
    GPIO_InitStructure.Pin = LED1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;
    GPIO_InitPeripheral(LED1_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = LED2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;    
    GPIO_InitPeripheral(LED2_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(LED1_PORT, LED1_PIN);
    GPIO_ResetBits(LED2_PORT, LED2_PIN);    
    
    //Initialize Qflash.
    Qflash_Init();
    //Erase one Flash Sector
    Qflash_Erase_Sector(FLASH_TEST_ADDRESS);
    //Read test flash sector into buffer ram
    Qflash_Read(FLASH_TEST_ADDRESS, buffer, BUFFER_SIZE);
    for(uint32_t i=0;i<BUFFER_SIZE;i++){
        if(buffer[i] != 0xFF){
            assert_handle();
        }
    }
    //Assign value to m_buffer
    for(uint32_t i=0;i<BUFFER_SIZE;i++){
        m_buffer[i] = i;
    }    
    //Write buffer data to flash.
    Qflash_Write(FLASH_TEST_ADDRESS, m_buffer, BUFFER_SIZE); 
    //Read test flash sector into buffer ram
    Qflash_Read(FLASH_TEST_ADDRESS, buffer, BUFFER_SIZE);
    if(memcmp(buffer,m_buffer,BUFFER_SIZE) != 0){
        assert_handle();
    }
    GPIO_SetBits(LED2_PORT, LED2_PIN);
    while(1){
        
    }
    
    
    
}



/**
 * @}
 */
