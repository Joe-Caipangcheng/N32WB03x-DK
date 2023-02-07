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
#include "main.h"
#include "log.h"

void RCC_Configuration(void);
void GPIO_Configuration(void);
void ADC_Configuration(void);

uint16_t ADCConvertedValue[2];
uint32_t voltage[2] = {0}; 
/**
 * @brief  Main program
 */
int main(void)
{
	log_init();
    log_info("\nthis is adc single read Demo.\n");
	log_info("Please make sure J15 and J16 connect the IO to pin on board!\n");
	/* System Clocks Configuration */
	RCC_Configuration();
	
	/* Configure the GPIO ports */
    GPIO_Configuration();
	
	ADC_EnableBypassFilter(ADC, ENABLE);
    while (1)
    {
		ADC_ConfigChannel(ADC, ADC_CTRL_CH_3);
		ADC_Enable(ADC, ENABLE);
		while(ADC_GetFlagStatus(ADC,ADC_FLAG_DONE) == RESET);
		ADC_ClearFlag(ADC,ADC_FLAG_DONE);  
		ADCConvertedValue[1] = ADC_GetDat(ADC);
    
		voltage[1] = ADC_ConverValueToVoltage(ADCConvertedValue[1], ADC_CTRL_CH_3);
		log_info("ADC CH3 value: %4d  |  ADC CH3 vol_mV: %4d .\r\n",ADCConvertedValue[1],voltage[1]);  
	
		ADC_ConfigChannel(ADC, ADC_CTRL_CH_1);
		ADC_Enable(ADC, ENABLE);
		while(ADC_GetFlagStatus(ADC,ADC_FLAG_DONE) == RESET);
		ADC_ClearFlag(ADC,ADC_FLAG_DONE);  
		ADCConvertedValue[0] = ADC_GetDat(ADC);

		voltage[0] = ADC_ConverValueToVoltage(ADCConvertedValue[0], ADC_CTRL_CH_1);
		log_info("ADC CH1 value: %4d  |  ADC CH1 vol_mV: %4d .\r\n\r\n",ADCConvertedValue[0],voltage[0]);    
		Delay_ms(1000);
    }
}


/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
	/* Enable peripheral clocks */
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
    /* Configure PB.10 (ADC Channel1) PB.8 (ADC Channel3) as analog input --------*/
    GPIO_InitStructure.Pin       = GPIO_PIN_10|GPIO_PIN_8;
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


