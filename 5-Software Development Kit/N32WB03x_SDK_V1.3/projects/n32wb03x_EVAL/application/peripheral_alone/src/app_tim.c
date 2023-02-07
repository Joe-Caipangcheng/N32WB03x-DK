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
 * @file app_tim.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "app_tim.h"
#include "main.h"
#include <stdio.h>


/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SRC_BUF_SIZE  161
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t CCR1_Val       = 300;
uint16_t CCR2_Val       = 200;
uint16_t CCR3_Val       = 100;
uint16_t CCR4_Val       = 50;

uint16_t SRC_Buffer[SRC_BUF_SIZE] = {0};
/* Private function prototypes -----------------------------------------------*/



/* Private functions ---------------------------------------------------------*/


/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration_TIM3(void)
{
     /* TIM3 clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);

    /* GPIOB clock enable */
    RCC_EnableAPB2PeriphClk( RCC_APB2_PERIPH_GPIOB| RCC_APB2_PERIPH_AFIO,   ENABLE);
}




/**
 * @brief  Configure the TIM3 Ouput Channels.
 */
void GPIO_Configuration_TIM3(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);

    /* GPIOB Configuration:TIM3 Channel 1 and 2 as  alternate function push-pull */
    GPIO_InitStructure.Pin        = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

}

/**
 * @brief  Configure the TIM3 as PWM mode and enable output
 */
void TIM3_PWM_Configuration(void)
{
    uint16_t PrescalerValue = 0;
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    OCInitType TIM_OCInitStructure;
    
    
    /* System Clocks Configuration */
    RCC_Configuration_TIM3();

    /* GPIO Configuration */
    GPIO_Configuration_TIM3();

    /* -----------------------------------------------------------------------
    TIM1 Configuration: generate 2 PWM signals with 2 different duty cycles:
    The TIM1CLK frequency is set to SystemCoreClock (Hz), to get TIM1 counter
    clock at 16 MHz the Prescaler is computed as following:
     - Prescaler = (TIM1CLK / TIM1 counter clock) - 1
    SystemCoreClock is set to 32(HSE) or 64(HSI) MHz for N32WB03X device

    The TIM1 is running at 40 KHz: TIM1 Frequency = TIM1 counter clock/(AR + 1)
                                                  = 16 MHz / 400 = 40 KHz
    TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR)* 100 = 75%
    TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR)* 100 = 50%
    ----------------------------------------------------------------------- */
    /* Compute the prescaler value */
    PrescalerValue = (uint16_t)(SystemCoreClock / 16000000) - 1; 
    /* Time base configuration*/
    TIM_TimeBaseStructure.Period    = (400-1);
    TIM_TimeBaseStructure.Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;

    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR1_Val;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;

    TIM_InitOc1(TIM3, &TIM_OCInitStructure);

    TIM_EnableOc1Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR2_Val;

    TIM_InitOc2(TIM3, &TIM_OCInitStructure);

    TIM_ConfigOc2Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);
        
        
        /* TIM3 enable AR Preload */
    TIM_ConfigArPreload(TIM3, ENABLE);

    /* TIM3 enable counter */
    TIM_Enable(TIM3, ENABLE);


}


/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration_TIM1(void)
{

    /* TIM3 and GPIOB clock enable */
    RCC_EnableAPB2PeriphClk( RCC_APB2_PERIPH_GPIOB| RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_TIM1, ENABLE);
}




/**
 * @brief  Configure the TIM1 Ouput Channels.
 */
void GPIO_Configuration_TIM1(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);

    /* GPIOB Configuration:TIM1 Channel 2,3 and 4 as  alternate function push-pull */
    GPIO_InitStructure.Pin        = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF1_TIM1;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

}


/**
 * @brief  Configures the DMA.
 */
void DMA_Configuration(void)
{
    DMA_InitType DMA_InitStructure;
        uint8_t i =0;
      /* init SRC_Buffer data*/
     for( i =0; i < SRC_BUF_SIZE; i++ )
     {
         SRC_Buffer[i] = i;
     }

    /* DMA Channel5 Config */
    DMA_DeInit(DMA_CH5);

    DMA_InitStructure.PeriphAddr     = (uint32_t)&TIM1->CCDAT2;  //TIM1_CCR2_Address; 
    DMA_InitStructure.MemAddr        = (uint32_t)SRC_Buffer;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize        = SRC_BUF_SIZE;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.CircularMode   = DMA_MODE_CIRCULAR;
    DMA_InitStructure.Priority       = DMA_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;

    DMA_Init(DMA_CH5, &DMA_InitStructure);
    DMA_RequestRemap(DMA_REMAP_TIM1_UP, DMA, DMA_CH5, ENABLE);
    /* DMA Channel5 enable */
    DMA_EnableChannel(DMA_CH5, ENABLE);
}

/**
 * @brief  Configure the TIM1 as PWM mode and enable output
 */
void TIM1_PWM_Configuration(void)
{
    uint16_t PrescalerValue = 0;
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    OCInitType TIM_OCInitStructure;


    /* System Clocks Configuration */
    RCC_Configuration_TIM1();

    /* GPIO Configuration */
    GPIO_Configuration_TIM1();
    
      /* DMA Configuration */
    DMA_Configuration();

    /* -----------------------------------------------------------------------
    TIM1 Configuration: generate 2 PWM signals with 2 different duty cycles:
    The TIM1CLK frequency is set to SystemCoreClock (Hz), to get TIM1 counter
    clock at 16 MHz the Prescaler is computed as following:
     - Prescaler = (TIM1CLK / TIM1 counter clock) - 1
    SystemCoreClock is set to 32(HSE) or 64(HSI) MHz for N32WB03X device

    The TIM1 is running at 40 KHz: TIM1 Frequency = TIM1 counter clock/(AR + 1)
                                                  = 32 MHz / 160 = 200 KHz
    TIM1 Channel2 duty cycle = (TIM1_CCR1/ TIM1_ARR)* 100 = 0-100%
    TIM1 Channel3 duty cycle = (TIM1_CCR2/ TIM1_ARR)* 100 = 63%
        TIM1 Channel3 duty cycle = (TIM1_CCR2/ TIM1_ARR)* 100 = 32%
    ----------------------------------------------------------------------- */
    /* Compute the prescaler value */
    PrescalerValue = (uint16_t)(SystemCoreClock / 32000000) - 1; 
    /* Time base configuration*/
    TIM_TimeBaseStructure.Period    = (160-1);
    TIM_TimeBaseStructure.Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_TimeBaseStructure.RepetCnt  = 0;

    TIM_InitTimeBase(TIM1, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.OcMode       = TIM_OCMODE_PWM2;
    TIM_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.OutputNState = TIM_OUTPUT_NSTATE_DISABLE;
    TIM_OCInitStructure.Pulse        = SRC_Buffer[0];
    TIM_OCInitStructure.OcPolarity   = TIM_OC_POLARITY_LOW;

    TIM_InitOc2(TIM1, &TIM_OCInitStructure);

    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.Pulse       = CCR3_Val;
    TIM_InitOc3(TIM1, &TIM_OCInitStructure);
        
        /* PWM1 Mode configuration: Channel4 */
    TIM_OCInitStructure.Pulse       = CCR4_Val;
    TIM_InitOc4(TIM1, &TIM_OCInitStructure);
        
    /* TIM1 Update DMA Request enable */
    TIM_EnableDma(TIM1, TIM_DMA_UPDATE, ENABLE);        

    /* TIM1 enable counter */
    TIM_Enable(TIM1, ENABLE);
        
        /* TIM1 Main Output Enable */
    TIM_EnableCtrlPwmOutputs(TIM1, ENABLE);

}





/**
 * @}
 */
