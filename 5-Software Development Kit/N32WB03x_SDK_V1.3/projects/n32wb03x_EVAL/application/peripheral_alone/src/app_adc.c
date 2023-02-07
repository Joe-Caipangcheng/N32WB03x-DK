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
 * @file app_adc.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "app_adc.h"
#include "main.h"
#include <stdio.h>


/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/



/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration_ADC(void)
{
    /* Enable peripheral clocks */
    /* Enable GPIOB clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    /* Enable ADC clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);
    RCC_Enable_ADC_CLK_SRC_AUDIOPLL(ENABLE);    
    
    RCC_ConfigAdcClk(RCC_ADCCLK_SRC_AUDIOPLL);

}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration_ADC(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /* Configure PB.10 (ADC Channel1) as analog input --------*/
    GPIO_InitStructure.Pin       = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief  Configures the ADC and Read CH1 CH2.
 */
void ADC_Configuration(void)
{
    RCC_Configuration_ADC();
    GPIO_Configuration_ADC();

    /* Select the ADC channel */
    ADC_ConfigChannel(ADC, ADC_CTRL_CH_1);
    
    ADC_EnableBypassFilter(ADC, ENABLE);


}

/**
 * @brief  Configures and read the selected ADC channel
 * @param  ADC_Channel the ADC channel to read.
 *   This parameter can be any combination of the following values:
 *     @arg ADC_CTRL_CH_0 ADC Channel0 selected (PGA PB11/PB13)
 *     @arg ADC_CTRL_CH_1 ADC Channel1 selected (PB10)
 *     @arg ADC_CTRL_CH_2 ADC Channel2 selected (PB9)
 *     @arg ADC_CTRL_CH_3 ADC Channel3 selected (PB8)
 *     @arg ADC_CTRL_CH_4 ADC Channel4 selected (PB7)
 *     @arg ADC_CTRL_CH_5 ADC Channel5 selected (PB6)
 *     @arg ADC_CTRL_CH_6 ADC Channel6 selected (external voltage)
 *     @arg ADC_CTRL_CH_7 ADC Channel7 selected (temperature)
 */
uint16_t ADC_GetDataBlocking(uint32_t ADC_Channel)
{
    uint16_t dat;
    uint32_t timer = 0;

    /* Select the ADC channel */
    ADC_ConfigChannel(ADC, ADC_Channel);
    /* Enable ADC */
    ADC_Enable(ADC, ENABLE);

    while(ADC_GetFlagStatus(ADC,ADC_FLAG_DONE)== RESET)
    {
      if(++timer > 0xfffff)
            return 0xffff;
    }
    ADC_ClearFlag(ADC,ADC_FLAG_DONE);
    dat=ADC_GetDat(ADC);
    return dat;
}


/**
 * @}
 */

