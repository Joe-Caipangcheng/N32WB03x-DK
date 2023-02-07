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

/** @addtogroup N32WB03X_StdPeriph_Examples
 * @{
 */

/** @addtogroup Simplex_Interrupt
 * @{
 */

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

#define BufferSize 32

SPI_InitType SPI_InitStructure;
uint8_t MASTER_Buffer_Tx[BufferSize] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
                                        0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
                                        0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20};
uint8_t SLAVE_Buffer_Rx[BufferSize];
__IO uint8_t TxIdx = 0, RxIdx = 0;
volatile TestStatus TransferStatus = FAILED;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

void RCC_ConfigMcoPllClk(uint32_t RCC_MCOPLLCLKPrescaler)
{
    uint32_t tmpregister = 0;
    /* Check the parameters */
    assert_param(IS_RCC_MCOPLLCLKPRE(RCC_MCOPLLCLKPrescaler));

    tmpregister = RCC->CFG;
    /* Clear MCOPRE[3:0] bits */
    tmpregister &= ((uint32_t)0x0FFFFFFF);
    /* Set MCOPRE[3:0] bits according to RCC_ADCHCLKPrescaler value */
    tmpregister |= RCC_MCOPLLCLKPrescaler;

    /* Store the new value */
    RCC->CFG = tmpregister;
}                                        
                                        

                                        
/**
 * @brief  Main program.
 */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_n32wb03x.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_n32wb03x.c file
     */
    log_init();
    log_info("\n this is a SPI Simplex Interrupt SLAVE Demo.\n");
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();
    /* NVIC configuration ------------------------------------------------------*/
    NVIC_Configuration();

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();
    
    /* SPI_SLAVE configuration ------------------------------------------------*/
    SPI_I2S_DeInit(SPI_SLAVE);
    SPI_InitStructure.DataDirection = SPI_DIR_SINGLELINE_RX;
    SPI_InitStructure.SpiMode       = SPI_MODE_SLAVE;
    SPI_InitStructure.DataLen       = SPI_DATA_SIZE_8BITS;
    SPI_InitStructure.CLKPOL        = SPI_CLKPOL_LOW;
    SPI_InitStructure.CLKPHA        = SPI_CLKPHA_SECOND_EDGE;
    SPI_InitStructure.NSS           = SPI_NSS_SOFT;
    SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_16;
    SPI_InitStructure.FirstBit      = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly       = 7;
    SPI_Init(SPI_SLAVE, &SPI_InitStructure);
    
    /* Enable SPI_SLAVE RNE interrupt */
    SPI_I2S_EnableInt(SPI_SLAVE, SPI_I2S_INT_RNE, ENABLE);

    /* Enable SPI_SLAVE */
    SPI_Enable(SPI_SLAVE, ENABLE);

    /* Transfer procedure */
    while (RxIdx < BufferSize)
    {
    }
    
    if (RxIdx == BufferSize)
    {
        SPI_I2S_EnableInt(SPI_SLAVE, SPI_I2S_INT_RNE, DISABLE);
    }
    
    log_buff(SLAVE_Buffer_Rx, RxIdx);
    /* Check the correctness of written dada */
    TransferStatus = Buffercmp(SLAVE_Buffer_Rx, MASTER_Buffer_Tx, BufferSize);
    /* TransferStatus = PASSED, if the transmitted and received data
       are equal */
    /* TransferStatus = FAILED, if the transmitted and received data
       are different */
    if(TransferStatus == PASSED)
    {
        log_info("SPI Simplex Interrupt SLAVE Demo test Succeed.\n");
    }
    else
    {
        log_info("SPI Simplex Interrupt SLAVE Demo test Fail.\n");
    }
    
    while (1)
    {
    }
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* Enable peripheral clocks --------------------------------------------------*/

    /* Enable GPIO clock for SPI_SLAVE */
    RCC_EnableAPB2PeriphClk(SPI_SLAVE_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);

    /* Enable SPI_SLAVE Periph clock */
    RCC_EnableAPB2PeriphClk(SPI_SLAVE_CLK, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    
    /* Configure SPI_SLAVE pins: SCK and MISO*/
    GPIO_InitStructure.Pin                = SPI_SLAVE_PIN_SCK | SPI_SLAVE_PIN_MISO;
    GPIO_InitStructure.GPIO_Mode          = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed         = GPIO_SPEED_HIGH;
    GPIO_InitStructure.GPIO_Alternate     = SPI_SLAVE_GPIO_AF;
    GPIO_InitStructure.GPIO_Pull          = GPIO_NO_PULL;
    GPIO_InitPeripheral(SPI_SLAVE_GPIO, &GPIO_InitStructure);
}

/**
 * @brief  Configure the nested vectored interrupt controller.
 */
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Configure and enable SPI_SLAVE interrupt --------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel                   = SPI_SLAVE_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer1, pBuffer2: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer1 identical to pBuffer2
 *         FAILED: pBuffer1 differs from pBuffer2
 */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return FAILED;
        }

        pBuffer1++;
        pBuffer2++;
    }

    return PASSED;
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

    /* Infinite loop */
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
