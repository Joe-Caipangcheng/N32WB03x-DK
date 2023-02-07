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
 * @file n32wb03x_rcc.h
 * @author Nations Firmware Team
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __N32WB03X_RCC_H__
#define __N32WB03X_RCC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32wb03x.h"

/** @addtogroup N32WB03X_StdPeriph_Driver
 * @{
 */

/** @addtogroup RCC
 * @{
 */

/** @addtogroup RCC_Exported_Types
 * @{
 */

typedef struct
{
    uint32_t SysclkFreq;    /*!< returns SYSCLK clock frequency expressed in Hz */
    uint32_t HclkFreq;      /*!< returns HCLK clock frequency expressed in Hz */
    uint32_t Pclk1Freq;     /*!< returns PCLK1 clock frequency expressed in Hz */
    uint32_t Pclk2Freq;     /*!< returns PCLK2 clock frequency expressed in Hz */
    uint32_t AdcclkFreq;   /*!< returns ADCCLK clock frequency expressed in Hz */
} RCC_ClocksType;

/**
 * @}
 */

/** @addtogroup RCC_Exported_Constants
 * @{
 */

/** @addtogroup HSE_configuration
 * @{
 */

#define RCC_HSE_DISABLE     ((uint32_t)0x00000000)
#define RCC_HSE_ENABLE      ((uint32_t)0x00010000)

#define IS_RCC_HSE(HSE) (((HSE) == RCC_HSE_DISABLE) || ((HSE) == RCC_HSE_ENABLE))
                      
/**
 * @}
 */

/** @addtogroup HSI_configuration
 * @{
 */

#define RCC_HSI_DISABLE ((uint32_t)0x00000000)
#define RCC_HSI_ENABLE  ((uint32_t)0x00000001)
#define IS_RCC_HSI(HSI) (((HSI) == RCC_HSI_DISABLE) || ((HSI) == RCC_HSI_ENABLE))

/**
 * @}
 */

/** @addtogroup System_clock_source
 * @{
 */

#define RCC_SYSCLK_SRC_HSI    ((uint32_t)0x00000000)
#define RCC_SYSCLK_SRC_HSE    ((uint32_t)0x00000001)
#define IS_RCC_SYSCLK_SRC(SOURCE)    (((SOURCE) == RCC_SYSCLK_SRC_HSI) || ((SOURCE) == RCC_SYSCLK_SRC_HSE))                                              

/**
 * @}
 */

/** @addtogroup AHB_clock_source
 * @{
 */

#define RCC_SYSCLK_DIV1   ((uint32_t)0x00000000)
#define RCC_SYSCLK_DIV2   ((uint32_t)0x00000020)
#define RCC_SYSCLK_DIV4   ((uint32_t)0x00000030)
#define IS_RCC_SYSCLK_DIV(HCLK)   (((HCLK) == RCC_SYSCLK_DIV1) || ((HCLK) == RCC_SYSCLK_DIV2) || ((HCLK) == RCC_SYSCLK_DIV4))             

/**
 * @}
 */

/** @addtogroup APB1_APB2_clock_source
 * @{
 */

#define RCC_HCLK_DIV1  ((uint32_t)0x00000000)
#define RCC_HCLK_DIV2  ((uint32_t)0x00000400)
#define RCC_HCLK_DIV4  ((uint32_t)0x00000500)
#define RCC_HCLK_DIV8  ((uint32_t)0x00000600)
#define RCC_HCLK_DIV16 ((uint32_t)0x00000700)
#define IS_RCC_HCLK_DIV(PCLK)                                                                                          \
    (((PCLK) == RCC_HCLK_DIV1) || ((PCLK) == RCC_HCLK_DIV2) || ((PCLK) == RCC_HCLK_DIV4) || ((PCLK) == RCC_HCLK_DIV8)  \
     || ((PCLK) == RCC_HCLK_DIV16))
/**
 * @}
 */

/** @addtogroup RCC_Interrupt_source
 * @{
 */

#define RCC_INT_LSIRDIF ((uint8_t)0x01)
#define RCC_INT_LSERDIF ((uint8_t)0x02)
#define RCC_INT_HSIRDIF ((uint8_t)0x04)
#define RCC_INT_HSERDIF ((uint8_t)0x08)

#define IS_RCC_INT(IT)                                                                                                 \
    (((IT) == RCC_INT_LSIRDIF) || ((IT) == RCC_INT_LSERDIF) || ((IT) == RCC_INT_HSIRDIF) || ((IT) == RCC_INT_HSERDIF))


#define IS_RCC_GET_INT(IT)                                                                                             \
    (((IT) == RCC_INT_LSIRDIF) || ((IT) == RCC_INT_LSERDIF) || ((IT) == RCC_INT_HSIRDIF) || ((IT) == RCC_INT_HSERDIF))

#define RCC_CLR_LSIRDIF ((uint32_t)0x00010000)
#define RCC_CLR_LSERDIF ((uint32_t)0x00020000)
#define RCC_CLR_HSIRDIF ((uint32_t)0x00040000)
#define RCC_CLR_HSERDIF ((uint32_t)0x00080000)

#define IS_RCC_CLR_INTF(IT)                                                                                             \
    (((IT) == RCC_CLR_LSIRDIF) || ((IT) == RCC_CLR_LSERDIF) || ((IT) == RCC_CLR_HSIRDIF) || ((IT) == RCC_CLR_HSERDIF))

/**
 * @}
 */

/** @addtogroup RCC_CFG2_Config
 * @{
 */
#define RCC_TIM1CLK_SRC_TIM1CLK  ((uint32_t)0x00000000)
#define RCC_TIM1CLK_SRC_SYSCLK   ((uint32_t)0x80000000)
#define IS_RCC_TIM1CLKSRC(TIM1CLK) (((TIM1CLK) == RCC_TIM1CLK_SRC_TIM1CLK) || ((TIM1CLK) == RCC_TIM1CLK_SRC_SYSCLK))


#define RCC_ADCCLK_SRC_AUDIOPLL         ((uint32_t)0x00000000)
#define RCC_ADCCLK_SRC_HSE_DIV8         ((uint32_t)0x00000010)
#define IS_RCC_ADCCLKSRC(ADCMCLK) (((ADCMCLK) == RCC_ADCCLK_SRC_AUDIOPLL) || ((ADCMCLK) == RCC_ADCCLK_SRC_HSE_DIV8))

/**
 * @}
 */

/** @addtogroup LSE_configuration
 * @{
 */

#define RCC_LSE_DISABLE ((uint32_t)0x00000000)
#define RCC_LSE_ENABLE  ((uint32_t)0x00000004)
#define RCC_LSE_BYPASS  ((uint32_t)0x00000010)
#define IS_RCC_LSE(LSE) (((LSE) == RCC_LSE_DISABLE) || ((LSE) == RCC_LSE_ENABLE) || ((LSE) == RCC_LSE_BYPASS))
/**
 * @}
 */ 

/** @addtogroup RTC_clock_source
 * @{
 */

#define RCC_RTCCLK_SRC_LSE        ((uint32_t)0x00000020)
#define RCC_RTCCLK_SRC_LSI        ((uint32_t)0x00000000)
#define IS_RCC_RTCCLK_SRC(SOURCE)    (((SOURCE) == RCC_RTCCLK_SRC_LSE) || ((SOURCE) == RCC_RTCCLK_SRC_LSI))       

/**
 * @}
 */

/** @addtogroup AHB_peripheral
 * @{
 */

#define RCC_AHB_PERIPH_DMA    ((uint32_t)0x00000001)
#define RCC_AHB_PERIPH_SRAM   ((uint32_t)0x00000004)
#define RCC_AHB_PERIPH_CRC    ((uint32_t)0x00000040)
#define RCC_AHB_PERIPH_ADC    ((uint32_t)0x00001000)
#define RCC_AHB_PERIPH_IRC    ((uint32_t)0x00004000)

#define IS_RCC_AHB_PERIPH(PERIPH) ((((PERIPH)&0xFFFFAF8A) == 0x00) && ((PERIPH) != 0x00))
#define IS_RCC_AHB_PERIPH_RESET(PERIPH) ((((PERIPH)&0xFFFFAF8A) == 0x00) && ((PERIPH) != 0x00))

/**
 * @}
 */

/** @addtogroup APB2_peripheral
 * @{
 */

#define RCC_APB2_PERIPH_AFIO   ((uint32_t)0x00000001)
#define RCC_APB2_PERIPH_GPIOA  ((uint32_t)0x00000004)
#define RCC_APB2_PERIPH_GPIOB  ((uint32_t)0x00000008)
#define RCC_APB2_PERIPH_SPI1   ((uint32_t)0x00000200)
#define RCC_APB2_PERIPH_SPI2   ((uint32_t)0x00000400)
#define RCC_APB2_PERIPH_TIM1   ((uint32_t)0x00001000)
#define RCC_APB2_PERIPH_USART1 ((uint32_t)0x00004000)

#define IS_RCC_APB2_PERIPH(PERIPH) ((((PERIPH)&0xFFFFA9F2) == 0x00) && ((PERIPH) != 0x00))
/**
 * @}
 */

/** @addtogroup APB1_peripheral
 * @{
 */

#define RCC_APB1_PERIPH_TIM3      ((uint32_t)0x00000002)
#define RCC_APB1_PERIPH_TIM6      ((uint32_t)0x00000010)
#define RCC_APB1_PERIPH_WWDG      ((uint32_t)0x00000800)
#define RCC_APB1_PERIPH_USART2    ((uint32_t)0x00020000)
#define RCC_APB1_PERIPH_I2C1      ((uint32_t)0x00200000)
#define RCC_APB1_PERIPH_PWR       ((uint32_t)0x10000000)

#define IS_RCC_APB1_PERIPH(PERIPH) ((((PERIPH)&0xEFDDF7ED) == 0x00) && ((PERIPH) != 0x00))
#define IS_RCC_APB1_PERIPH_RESET(PERIPH) ((((PERIPH)&0xEFDDF7ED) == 0x00) && ((PERIPH) != 0x00))

/**
 * @}
 */
 
/** @addtogroup LPUART
 * @{
 */

#define RCC_LPUART1CLK            ((uint32_t)0x00000000)
#define RCC_LPUARTCLK_SRC_APB1    ((uint32_t)0x00000000)
#define RCC_LPUARTCLK_SRC_LSI_LSE ((uint32_t)0x04000000)
#define IS_RCC_LPUART_CLK(LPUARTCLK) ((LPUARTCLK == RCC_LPUARTCLK_SRC_APB1)||(LPUARTCLK == RCC_LPUARTCLK_SRC_LSI_LSE))
#define IS_RCC_LPUARTX(RCC_LPUARTx) (RCC_LPUARTx == RCC_LPUART1CLK)


/** @addtogroup Clock_source_to_output_on_MCO_pin
 * @{
 */

#define RCC_MCO_NOCLK       ((uint8_t)0x00)
#define RCC_MCO_LSI         ((uint8_t)0x01)
#define RCC_MCO_LSE         ((uint8_t)0x02)
#define RCC_MCO_SYSCLK      ((uint8_t)0x03)
#define RCC_MCO_HSI         ((uint8_t)0x04)
#define RCC_MCO_HSE         ((uint8_t)0x05)
#define RCC_MCO_HCLK        ((uint8_t)0x06)
#define RCC_MCO_AUDIOPLL    ((uint8_t)0x06)

#define IS_RCC_MCO(MCO)       \
    (((MCO) == RCC_MCO_NOCLK)  || ((MCO) == RCC_MCO_LSI)  || ((MCO) == RCC_MCO_LSE)  || \
    ((MCO) == RCC_MCO_SYSCLK) || ((MCO) == RCC_MCO_HSI) || ((MCO) == RCC_MCO_HSE)   ||  \
    ((MCO) == RCC_MCO_HCLK) || ((MCO) == RCC_MCO_AUDIOPLL))

/**
 * @}
 */

/** @addtogroup RCC_Flag
 * @{
 */
#define RCC_CTRL_FLAG_HSIRDF        ((uint8_t)0x21)
#define RCC_CTRL_FLAG_HSERDF        ((uint8_t)0x31)

#define RCC_LSCTRL_FLAG_LSIRD       ((uint8_t)0x41)
#define RCC_LSCTRL_FLAG_LSERD       ((uint8_t)0x43)

#define RCC_CTRLSTS_FLAG_PINRSTF    ((uint8_t)0x83)
#define RCC_CTRLSTS_FLAG_PORRSTF    ((uint8_t)0x84)
#define RCC_CTRLSTS_FLAG_SFTRSTF    ((uint8_t)0x85)
#define RCC_CTRLSTS_FLAG_IWDGRSTF   ((uint8_t)0x86)
#define RCC_CTRLSTS_FLAG_WWDGRSTF   ((uint8_t)0x87)

#define IS_RCC_FLAG(FLAG)                                                                                                       \
      (((FLAG) == RCC_CTRL_FLAG_HSIRDF)      || ((FLAG) == RCC_CTRL_FLAG_HSERDF)           || ((FLAG) == RCC_LSCTRL_FLAG_LSIRD)      \
        || ((FLAG) == RCC_CTRLSTS_FLAG_PINRSTF)  || ((FLAG) == RCC_CTRLSTS_FLAG_PORRSTF)  || ((FLAG) == RCC_LSCTRL_FLAG_LSERD)          \
        || ((FLAG) == RCC_CTRLSTS_FLAG_SFTRSTF)  || ((FLAG) == RCC_CTRLSTS_FLAG_IWDGRSTF) || ((FLAG) == RCC_CTRLSTS_FLAG_WWDGRSTF))

#define RCC_FLAG_MASK  ((uint8_t)0x1F)


/** @addtogroup DBGMCU_Exported_Constants
 * @{
 */
#define DBG_IWDG_STOP         ((uint32_t)0x00000001)
#define DBG_WWDG_STOP         ((uint32_t)0x00000002)
#define DBG_TIM1_STOP         ((uint32_t)0x00000004)
#define DBG_TIM3_STOP         ((uint32_t)0x00000008)
#define DBG_TIM6_STOP         ((uint32_t)0x00000010)
#define DBG_I2C1SMBUS_TIMEOUT ((uint32_t)0x00000020)


/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup RCC_Exported_Macros
 * @{
 */

/**
 * @}
 */

/** @addtogroup RCC_Exported_Functions
 * @{
 */

void RCC_DeInit(void);
void RCC_ConfigHse(uint32_t RCC_HSE);
ErrorStatus RCC_WaitHseStable(void);
void RCC_ConfigHsi(uint32_t RCC_HSI);
ErrorStatus RCC_WaitHsiStable(void);
void RCC_SetHsiCalibValue(uint8_t HSICalibrationValue);
void RCC_EnableHsi(FunctionalState Cmd);

void RCC_ConfigSysclk(uint32_t RCC_SYSCLKSource);
uint32_t RCC_GetSysclkSrc(void);
void RCC_ConfigHclk(uint32_t RCC_SYSCLK);
void RCC_ConfigPclk1(uint32_t RCC_HCLK);
void RCC_ConfigPclk2(uint32_t RCC_HCLK);
void RCC_ConfigInt(uint8_t RccInt, FunctionalState Cmd);

void RCC_ConfigTim1Clk(uint32_t RCC_TIM1CLKSource);
void RCC_ConfigAdcClk(uint32_t RCC_ADCCLKSource);
    
void RCC_ConfigLpuartClk(uint32_t RCC_LPUARTx, uint32_t RCC_LPUARTCLKSource);
uint32_t RCC_GetLpuartClkSrc(uint32_t RCC_LPUARTx);
void RCC_EnableLpuartClk(FunctionalState Cmd);
void RCC_ConfigLse(uint8_t RCC_LSE);
void RCC_EnableLsi(FunctionalState Cmd);
void RCC_ConfigLSXSEL(uint32_t RCC_LS_Source);
void RCC_EnableRtcClk(FunctionalState Cmd);
void RCC_EnableRTCReset(FunctionalState Cmd);
uint32_t RCC_GetRTCClkSrc(void);
void RCC_GetClocksFreqValue(RCC_ClocksType* RCC_Clocks);
void RCC_EnableAHBPeriphClk(uint32_t RCC_AHBPeriph, FunctionalState Cmd);
void RCC_EnableAPB2PeriphClk(uint32_t RCC_APB2Periph, FunctionalState Cmd);
void RCC_EnableAPB1PeriphClk(uint32_t RCC_APB1Periph, FunctionalState Cmd);

void RCC_EnableAHBPeriphReset(uint32_t RCC_AHBPeriph, FunctionalState Cmd);
void RCC_EnableAPB2PeriphReset(uint32_t RCC_APB2Periph, FunctionalState Cmd);
void RCC_EnableAPB1PeriphReset(uint32_t RCC_APB1Periph, FunctionalState Cmd);

void RCC_EnableClockSecuritySystem(FunctionalState Cmd);
void RCC_ConfigMcoClkPre(uint32_t RCC_MCOCLKPrescaler);
void RCC_ConfigMco(uint8_t RCC_MCO);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClrFlag(void);
INTStatus RCC_GetIntStatus(uint8_t RccInt);
void RCC_ClrIntPendingBit(uint32_t RccClrInt);
void RCC_Enable_ADC_CLK_SRC_AUDIOPLL(FunctionalState Cmd);
void RCC_EnableRAMCParityErrorReset(FunctionalState Cmd);
void DBG_ConfigPeriph(uint32_t DBG_Periph, FunctionalState Cmd);

#ifdef __cplusplus
}
#endif

#endif /* __N32WB03X_RCC_H__ */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
