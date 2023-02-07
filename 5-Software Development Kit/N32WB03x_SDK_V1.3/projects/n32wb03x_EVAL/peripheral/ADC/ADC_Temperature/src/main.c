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
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "log.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC_BUF_SIZE     (10)

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t ADCConvertedBuffer[ADC_BUF_SIZE];
uint16_t ADCConvertedValue;
uint32_t Voltage_mA; 
float Temperature; 
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void ADC_Configuration(void);
/* Private functions ---------------------------------------------------------*/


/**
 * @brief  Main program
 */
int main(void)
{
    DMA_InitType DMA_InitStructure;
    uint32_t adc_sum,i;
    
    log_init();
    log_info("\nthis is adc temperature sensor read demo.\n");
    
    /* System Clocks Configuration */
    RCC_Configuration();
    
    /* Configure the GPIO ports */
    GPIO_Configuration();
    
    /* DMA channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA_CH1);
    DMA_InitStructure.PeriphAddr     = (uint32_t)&ADC->DAT;
    DMA_InitStructure.MemAddr        = (uint32_t)ADCConvertedBuffer;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize        = ADC_BUF_SIZE;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.CircularMode   = DMA_MODE_CIRCULAR;
    DMA_InitStructure.Priority       = DMA_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(DMA_CH1, &DMA_InitStructure);
    DMA_RequestRemap(DMA_REMAP_ADC, DMA, DMA_CH1, ENABLE);

    /* Enable DMA channel 1 */
    DMA_EnableChannel(DMA_CH1, ENABLE);
    ADC_SetOverSampleCounter(ADC,3);
    ADC_EnableBypassFilter(ADC, ENABLE);
    ADC_ConfigChannel(ADC, ADC_CTRL_CH_7);
    ADC_ConfigContinuousMode(ADC, ENABLE);
    ADC_EnableDMA(ADC, ENABLE);
    ADC_EnableTS(ADC, ENABLE);
    ADC_Enable(ADC, ENABLE);

    while (1)
    {

        /* Transfer complete */
        while (!DMA_GetFlagStatus(DMA_FLAG_TC1, DMA));
        adc_sum = 0;
        for(i=0;i<ADC_BUF_SIZE;i++)
        {
            adc_sum += ADCConvertedBuffer[i];
        }
        ADCConvertedValue = adc_sum/ADC_BUF_SIZE;
        Temperature = ADC_ConverValueToTemperature(ADCConvertedValue);
        log_info("ADC temperature sensor: %.2f C\r\n",Temperature);  

        Delay_ms(1000);
    }
}


/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable peripheral clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA, ENABLE);
    /* Enable GPIOB clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    /* Enable ADC clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);
        
    RCC_ConfigAdcClk(RCC_ADCCLK_SRC_AUDIOPLL);

    /* enable ADC 4M clock */
    RCC_Enable_ADC_CLK_SRC_AUDIOPLL(ENABLE);
}


/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    /* Configure PB.10 (ADC Channel1) PB.7 (ADC Channel4) as analog input --------*/
    GPIO_InitStructure.Pin       = GPIO_PIN_10|GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
}


#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file pointer to the source file name
 * @param line assert_param error line source number
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    while (1)
    {
    }
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/*************** (C) COPYRIGHT Nations Technologies Inc *****END OF FILE***************/


