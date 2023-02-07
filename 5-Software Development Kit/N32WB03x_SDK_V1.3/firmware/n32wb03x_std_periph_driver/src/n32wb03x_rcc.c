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
 * @file n32wb03x_rcc.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32wb03x_rcc.h"

/** @addtogroup n32wb03x_StdPeriph_Driver
 * @{
 */

/** @addtogroup RCC
 * @brief RCC driver modules
 * @{
 */

/** @addtogroup RCC_Private_TypesDefinitions
 * @{
 */

/**
 * @}
 */

/** @addtogroup RCC_Private_Defines
 * @{
 */


/**
 * @}
 */

/** @addtogroup RCC_Private_Macros
 * @{
 */

/**
 * @}
 */

/** @addtogroup RCC_Private_Variables
 * @{
 */
static const uint8_t s_ApbPresTable[8]     = {0, 0, 0, 0, 1, 2, 3, 4};
static const uint8_t s_AhbPresTable[4]     = {0, 0, 1, 2};
/**
 * @}
 */

/** @addtogroup RCC_Private_FunctionPrototypes
 * @{
 */

/**
 * @}
 */

/** @addtogroup RCC_Private_Functions
 * @{
 */

/**
 * @brief  Resets the RCC clock configuration to the default reset state.
 */
void RCC_DeInit(void)
{
    /* Set HSIEN bit */
    RCC->CTRL |= (uint32_t)0x00000001;

    /* Reset SCLKSW, AHBPRE, APB1PRES, APB2PRES, MCO and MCOPRES bits */
    RCC->CFG &= (uint32_t)0x21FF0000;

    /* Reset HSEEN, CLKSSEN and PLLEN bits */
    RCC->CTRL &= (uint32_t)0xFEF6FFFF;

    /* Reset PLLBP, PLLOUTEN bit */
    RCC->CTRL &= (uint32_t)0xFFFBFFFF;
  
    /* Reset PLLMULFCT, PLLPRE, PLLOUTDIV and PLLSRC bits */
    RCC->CFG &= (uint32_t)0xFE00FFFF;

    /* Reset CFG2 register */
    RCC->CFG2 = 0x00003800;

    /* Disable all interrupts and clear pending bits  */
    RCC->CLKINT = 0x00BF0000;
}

/**
 * @brief  Configures the External High Speed oscillator (HSE).
 * @note   HSE can not be stopped if it is used directly or through the PLL as system clock.
 * @param RCC_HSE specifies the new state of the HSE.
 *   This parameter can be one of the following values:
 *     @arg RCC_HSE_DISABLE    HSE oscillator OFF
 *     @arg RCC_HSE_ENABLE     HSE oscillator ON 
 */
void RCC_ConfigHse(uint32_t RCC_HSE)
{
    /* Check the parameters */
    assert_param(IS_RCC_HSE(RCC_HSE));
    /* Reset HSEON and HSEIOSEL bits before configuring the HSE ------------------*/
    /* Reset HSEON bit */
    RCC->CTRL &= ~RCC_CTRL_HSEEN;
    /* Configure HSE (RCC_HSE_DISABLE is already covered by the code section above) */
    switch (RCC_HSE)
    {
        case RCC_HSE_ENABLE:
            /* Set HSEEN bit */
            RCC->CTRL |= RCC_CTRL_HSEEN;
            break;
        default:
            break;
    }
}

/**
 * @brief  Waits for HSE start-up.
 * @return An ErrorStatus enumuration value:
 * - SUCCESS: HSE oscillator is stable and ready to use
 * - ERROR: HSE oscillator not yet ready
 */
ErrorStatus RCC_WaitHseStable(void)
{
    __IO uint32_t StartUpCounter = 0;
    ErrorStatus status           = ERROR;
    FlagStatus HSEStatus         = RESET;

    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
        HSEStatus = RCC_GetFlagStatus(RCC_CTRL_FLAG_HSERDF);
        StartUpCounter++;
    } while ((StartUpCounter != HSE_STARTUP_TIMEOUT) && (HSEStatus == RESET));

    if (RCC_GetFlagStatus(RCC_CTRL_FLAG_HSERDF) != RESET)
    {
        status = SUCCESS;
    }
    else
    {
        status = ERROR;
    }
    return (status);
}

/**
 * @brief  Configures the Internal High Speed oscillator (HSI).
 * @note   HSI can not be stopped if it is used directly or through the PLL as system clock.
 * @param RCC_HSI specifies the new state of the HSI.
 *   This parameter can be one of the following values:
 *     @arg RCC_HSI_DISABLE HSI oscillator OFF
 *     @arg RCC_HSI_ENABLE HSI oscillator ON
 */
void RCC_ConfigHsi(uint32_t RCC_HSI)
{
    /* Check the parameters */
    assert_param(IS_RCC_HSI(RCC_HSI));
    /* Reset HSIEN bit */
    RCC->CTRL &= ~RCC_CTRL_HSIEN;
    /* Configure HSI */
    switch (RCC_HSI)
    {
        case RCC_HSI_ENABLE:
            /* Set HSIEN bit */
            RCC->CTRL |= RCC_CTRL_HSIEN;
            break;

        default:
            break;
    }
}

/**
 * @brief  Waits for HSI start-up.
 * @return An ErrorStatus enumuration value:
 * - SUCCESS: HSI oscillator is stable and ready to use
 * - ERROR: HSI oscillator not yet ready
 */
ErrorStatus RCC_WaitHsiStable(void)
{
    __IO uint32_t StartUpCounter = 0;
    ErrorStatus status           = ERROR;
    FlagStatus HSIStatus         = RESET;

    /* Wait till HSI is ready and if Time out is reached exit */
    do
    {
        HSIStatus = RCC_GetFlagStatus(RCC_CTRL_FLAG_HSIRDF);
        StartUpCounter++;
    } while ((StartUpCounter != HSI_STARTUP_TIMEOUT) && (HSIStatus == RESET));

    if (RCC_GetFlagStatus(RCC_CTRL_FLAG_HSIRDF) != RESET)
    {
        status = SUCCESS;
    }
    else
    {
        status = ERROR;
    }
    return (status);
}


/**
 * @brief  Enables or disables the Internal High Speed oscillator (HSI).
 * @note   HSI can not be stopped if it is used directly or through the PLL as system clock.
 * @param Cmd new state of the HSI. This parameter can be: ENABLE or DISABLE.
 */
void RCC_EnableHsi(FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

   if(Cmd == ENABLE)
   {
       /* Set HSIEN bit */
       RCC->CTRL |= RCC_CTRL_HSIEN;
   }
   else
   {
       /* Reset HSIEN bit */
       RCC->CTRL &= ~RCC_CTRL_HSIEN;
   }
}


/**
 * @brief  Configures the system clock (SYSCLK).
 * @param RCC_SYSCLKSource specifies the clock source used as system clock.
 *   This parameter can be one of the following values:
 *     @arg RCC_SYSCLK_SRC_HSI    HSI selected as system clock
 *     @arg RCC_SYSCLK_SRC_HSE    HSE selected as system clock
 */
void RCC_ConfigSysclk(uint32_t RCC_SYSCLKSource)
{
    uint32_t tmpregister = 0;
    /* Check the parameters */
    assert_param(IS_RCC_SYSCLK_SRC(RCC_SYSCLKSource));
    tmpregister = RCC->CFG;
    /* Clear SW[2:0] bits */
    tmpregister &= RCC_CFG_SCLKSW;
    /* Set SW[2:0] bits according to RCC_SYSCLKSource value */
    tmpregister |= RCC_SYSCLKSource;
    /* Store the new value */
    RCC->CFG = tmpregister;
}

/**
 * @brief  Returns the clock source used as system clock.
 * @return The clock source used as system clock. The returned value can
 *   be one of the following:
 *     - RCC_CFG_SCLKSTS_HSI: HSI used as system clock
 *     - RCC_CFG_SCLKSTS_HSE: HSE used as system clock
 */
uint32_t RCC_GetSysclkSrc(void)
{
    return ((uint32_t)(RCC->CFG & RCC_CFG_SCLKSTS));
}

/**
 * @brief  Configures the AHB clock (HCLK).
 * @param RCC_SYSCLK defines the AHB clock divider. This clock is derived from
 *   the system clock (SYSCLK).
 *   This parameter can be one of the following values:
 *     @arg RCC_SYSCLK_DIV1 AHB clock = SYSCLK
 *     @arg RCC_SYSCLK_DIV2 AHB clock = SYSCLK/2
 *     @arg RCC_SYSCLK_DIV4 AHB clock = SYSCLK/4
 */
void RCC_ConfigHclk(uint32_t RCC_SYSCLK)
{
    uint32_t tmpregister = 0;
    /* Check the parameters */
    assert_param(IS_RCC_SYSCLK_DIV(RCC_SYSCLK));
    tmpregister = RCC->CFG;
    /* Clear HPRE[3:0] bits */
    tmpregister &= ~RCC_CFG_AHBPRES;
    /* Set HPRE[3:0] bits according to RCC_SYSCLK value */
    tmpregister |= RCC_SYSCLK;
    /* Store the new value */
    RCC->CFG = tmpregister;
}

/**
 * @brief  Configures the Low Speed APB clock (PCLK1).
 * @param RCC_HCLK defines the APB1 clock divider. This clock is derived from
 *   the AHB clock (HCLK).
 *   This parameter can be one of the following values:
 *     @arg RCC_HCLK_DIV1 APB1 clock = HCLK
 *     @arg RCC_HCLK_DIV2 APB1 clock = HCLK/2
 *     @arg RCC_HCLK_DIV4 APB1 clock = HCLK/4
 *     @arg RCC_HCLK_DIV8 APB1 clock = HCLK/8
 *     @arg RCC_HCLK_DIV16 APB1 clock = HCLK/16
 */
void RCC_ConfigPclk1(uint32_t RCC_HCLK)
{
    uint32_t tmpregister = 0;
    /* Check the parameters */
    assert_param(IS_RCC_HCLK_DIV(RCC_HCLK));
    tmpregister = RCC->CFG;
    /* Clear PPRE1[2:0] bits */
    tmpregister &= ~RCC_CFG_APB1PRES;
    /* Set PPRE1[2:0] bits according to RCC_HCLK value */
    tmpregister |= RCC_HCLK;
    /* Store the new value */
    RCC->CFG = tmpregister;
}

/**
 * @brief  Configures the High Speed APB clock (PCLK2).
 * @param RCC_HCLK defines the APB2 clock divider. This clock is derived from
 *   the AHB clock (HCLK).
 *   This parameter can be one of the following values:
 *     @arg RCC_HCLK_DIV1 APB2 clock = HCLK
 *     @arg RCC_HCLK_DIV2 APB2 clock = HCLK/2
 *     @arg RCC_HCLK_DIV4 APB2 clock = HCLK/4
 *     @arg RCC_HCLK_DIV8 APB2 clock = HCLK/8
 *     @arg RCC_HCLK_DIV16 APB2 clock = HCLK/16
 */
void RCC_ConfigPclk2(uint32_t RCC_HCLK)
{
    uint32_t tmpregister = 0;
    /* Check the parameters */
    assert_param(IS_RCC_HCLK_DIV(RCC_HCLK));
    tmpregister = RCC->CFG;
    /* Clear PPRE2[2:0] bits */
    tmpregister &= ~RCC_CFG_APB2PRES;
    /* Set PPRE2[2:0] bits according to RCC_HCLK value */
    tmpregister |= RCC_HCLK << 3;
    /* Store the new value */
    RCC->CFG = tmpregister;
}

/**
 * @brief  Enables or disables the specified RCC interrupts.
 * @param RccInt specifies the RCC interrupt sources to be enabled or disabled.
 *
 *   this parameter can be any combination of the following values
 *     @arg RCC_INT_LSIRDIF LSI ready interrupt
 *     @arg RCC_INT_LSERDIF LSE ready interrupt
 *     @arg RCC_INT_HSIRDIF HSI ready interrupt
 *     @arg RCC_INT_HSERDIF HSE ready interrupt
 *     @arg RCC_INT_PLLRDIF PLL ready interrupt
 *     @arg RCC_INT_RAMCPIF RAMC parity interrupt
 *
 * @param Cmd new state of the specified RCC interrupts.
 *   This parameter can be: ENABLE or DISABLE.
 */
void RCC_ConfigInt(uint8_t RccInt, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_RCC_INT(RccInt));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    if (Cmd != DISABLE)
    {
        /* Perform Byte access to RCC_CLKINT bits to enable the selected interrupts */
        RCC->LSCTRL |= (((uint32_t)RccInt) << 8);
    }
    else
    {
        /* Perform Byte access to RCC_CLKINT bits to disable the selected interrupts */
        RCC->LSCTRL &= (~(((uint32_t)RccInt) << 8));
    }
}

/**
 * @brief  Configures the TIM1 clock (TIM1CLK).
 * @param RCC_TIM1CLKSource specifies the TIM1 clock source.
 *   This parameter can be one of the following values:
 *     @arg RCC_TIM1CLK_SRC_TIM1CLK
 *     @arg RCC_TIM1CLK_SRC_SYSCLK
 */
void RCC_ConfigTim1Clk(uint32_t RCC_TIM1CLKSource)
{
    uint32_t tmpregister = 0;
    /* Check the parameters */
    assert_param(IS_RCC_TIM1CLKSRC(RCC_TIM1CLKSource));

    tmpregister = RCC->CFG2;
    /* Clear TIMCLK_SEL bits */
    tmpregister &= ~RCC_CFG2_TIMCLKSEL;
    /* Set TIMCLK_SEL bits according to RCC_TIM18CLKSource value */
    tmpregister |= RCC_TIM1CLKSource;

    /* Store the new value */
    RCC->CFG2 = tmpregister;
}


/**
 * @brief  Configures the ADC clock.
 * @param RCC_ADCCLKSource specifies the ADC clock source.
 *   This parameter can be on of the following values:
 *     @arg RCC_ADCCLK_SRC_AUDIOPLL 
 *     @arg RCC_ADCCLK_SRC_HSE_DIV8 
 */
void RCC_ConfigAdcClk(uint32_t RCC_ADCCLKSource)
{
    uint32_t tmpregister = 0;
    /* Check the parameters */
    assert_param(IS_RCC_ADCCLKSRC(RCC_ADCCLKSource));

    tmpregister = RCC->CFG2;
 
    tmpregister &= ~RCC_CFG2_ADCSEL;
    /* Set ADCSEL bits according to RCC_ADCCLKSource value */
    tmpregister |= RCC_ADCCLKSource;

    /* Store the new value */
    RCC->CFG2 = tmpregister;
}


/**
 * @brief  Configures the LPUART clock (LPUARTCLK).
 * @param RCC_LPUARTx x = 1.
 *   This parameter can be one of the following values:
 *     @arg RCC_LPUART1CLK   LPUART1
 * @param RCC_LPUARTCLKSource specifies the LPUART clock source.
 *   This parameter can be one of the following values:
 *     @arg RCC_LPUARTCLK_SRC_APB1   APB1 clock selected as LPUART clock
 *     @arg RCC_LPUARTCLK_SRC_LSI_LSE LSI_LSE selected as LPUART clock
 */
void RCC_ConfigLpuartClk(uint32_t RCC_LPUARTx, uint32_t RCC_LPUARTCLKSource)
{
    /* Check the parameters */
    assert_param(IS_RCC_LPUARTX(RCC_LPUARTx));
    assert_param(IS_RCC_LPUART_CLK(RCC_LPUARTCLKSource));

    /* LPUART1 */
    if(RCC_LPUARTx == RCC_LPUART1CLK)
    {
        /* Clear the LPUART1 clock source */
        RCC->LSCTRL &= ~RCC_LSCTRL_LPUARTSEL;

        /* Select the LPTIM clock source */
        RCC->LSCTRL |= RCC_LPUARTCLKSource;
    }
}

/**
 * @brief  Returns the clock source used as LPUART clock.
 * @param RCC_LPUARTx x = 1.
 *   This parameter can be one of the following values:
 *     @arg RCC_LPUART1CLK   LPUART1
 * @return The clock source used as system clock. The returned value can
 *   be one of the following:
 *     - RCC_LPUARTCLK_SRC_APB1:   APB1 used as LPUART clock
 *     - RCC_LPUARTCLK_SRC_LSI_LSE: LSI_LSE used as LPUART clock
 */

uint32_t RCC_GetLpuartClkSrc(uint32_t RCC_LPUARTx)
{
    return ((uint32_t)(RCC->LSCTRL & (RCC_LSCTRL_LPUARTSEL_LSI_LSE)));
}

/**
 * @brief  Enables or disables the LPUART clock.
 * @note   This function must be used only after the LPUART clock was selected using the RCC_ConfigLpuartClk function.
 * @param Cmd new state of the LPUART clock. This parameter can be: ENABLE or DISABLE.
 */
void RCC_EnableLpuartClk(FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if(Cmd == ENABLE)
    {
        /* Set RTCEN bit */
        RCC->LSCTRL |= RCC_LSCTRL_LPUARTEN;
    }
    else
    {
        /* Reset RTCEN bit */
        RCC->LSCTRL &= ~RCC_LSCTRL_LPUARTEN;
    }
}

/**
 * @brief  Configures the External Low Speed oscillator (LSE).
 * @param RCC_LSE specifies the new state of the LSE.
 *   This parameter can be one of the following values:
 *     @arg RCC_LSE_DISABLE LSE oscillator OFF
 *     @arg RCC_LSE_ENABLE LSE oscillator ON
 *     @arg RCC_LSE_BYPASS LSE oscillator bypassed with external clock
 */
void RCC_ConfigLse(uint8_t RCC_LSE)
{
    /* Check the parameters */
    assert_param(IS_RCC_LSE(RCC_LSE));
    /* Reset LSEEN and LSEBYP bits before configuring the LSE ------------------*/
    RCC->LSCTRL &= (~(RCC_LSCTRL_LSEEN | RCC_LSCTRL_LSEBP));
    /* Configure LSE (RCC_LSE_DISABLE is already covered by the code section above) */
    switch (RCC_LSE)
    {
        case RCC_LSE_ENABLE:
            /* Set LSEON bit */
            RCC->LSCTRL |= RCC_LSE_ENABLE;
            break;

        case RCC_LSE_BYPASS:
            /* Set LSEBYP and LSEON bits */
            RCC->LSCTRL |= (RCC_LSE_BYPASS | RCC_LSE_ENABLE);
            break;

        default:
            break;
    }
}

/**
 * @brief  Enables or disables the Internal Low Speed oscillator (LSI).
 * @note   LSI can not be disabled if the IWDG is running.
 * @param Cmd new state of the LSI. This parameter can be: ENABLE or DISABLE.
 */
void RCC_EnableLsi(FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
  
    if(Cmd == ENABLE)
    {
        /* Set LSIEN bit */
        RCC->LSCTRL |= RCC_LSCTRL_LSIEN;
    }
    else
    {
        /* Reset PLLEN bit */
        RCC->LSCTRL &= ~RCC_LSCTRL_LSIEN;
    }
}

/**
 * @brief  Configures the Low Speed Clock Source as LSE or LSI.
 * @note   Once the Low Speed Clock Source is selected it can't be changed unless the LowPower domain is reset.
 * @param RCC_LSCTRLCLKSource specifies the LSCTRL clock source.
 *   This parameter can be one of the following values:
 *     @arg RCC_LSCTRL_LSXSEL_LSE:        LSE selected as Low Speed Clock Source
 *     @arg RCC_LSCTRL_LSXSEL_LSI:        LSI selected as Low Speed Clock Source
 */
void RCC_ConfigLSXSEL(uint32_t RCC_LS_Source)
{
    /* Check the parameters */
    assert_param(IS_RCC_RTCCLK_SRC(RCC_LS_Source));

    /* Clear the RTC clock source */
    RCC->LSCTRL &= (~RCC_LSCTRL_LSXSEL);

    /* Select the RTC clock source */
    RCC->LSCTRL |= RCC_LS_Source;
}

/**
 * @brief  Returns the clock source used as RTC clock (RTCCLK).
 * @return The clock source used as system clock. The returned value can
 *   be one of the following:
 *     - RCC_RTCCLK_SRC_LSE:        LSE used as RTC clock (RTCCLK)
 *     - RCC_RTCCLK_SRC_LSI:        LSI used as RTC clock (RTCCLK)
 */
uint32_t RCC_GetRTCClkSrc(void)
{
    return ((uint32_t)(RCC->LSCTRL & RCC_LSCTRL_LSXSEL));
}

/**
 * @brief  Enables or disables the RTC clock.
 * @note   This function must be used only after the RTC clock was selected using the RCC_ConfigRtcClk function.
 * @param Cmd new state of the RTC clock. This parameter can be: ENABLE or DISABLE.
 */
void RCC_EnableRtcClk(FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if(Cmd == ENABLE)
    {
        /* Set RTCEN bit */
        RCC->LSCTRL |= RCC_LSCTRL_RTCEN;
    }
    else
    {
        /* Reset RTCEN bit */
        RCC->LSCTRL &= ~RCC_LSCTRL_RTCEN;
    }
}



/**
 * @brief Forces or releases RTC reset.
 * @param Cmd new state of the specified peripheral reset. This parameter can be ENABLE or DISABLE.
 */
void RCC_EnableRTCReset(FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        RCC->LSCTRL |= RCC_LSCTRL_RTCRST;
    }
    else
    {
        RCC->LSCTRL &= ~RCC_LSCTRL_RTCRST;
    }
}


/**
 * @brief  Returns the frequencies of different on chip clocks.
 * @param RCC_Clocks pointer to a RCC_ClocksType structure which will hold
 *         the clocks frequencies.
 * @note   The result of this function could be not correct when using
 *         fractional value for HSE crystal.
 */
void RCC_GetClocksFreqValue(RCC_ClocksType* RCC_Clocks)
{
    uint32_t tmp = 0, presc = 0;

    /* Get SYSCLK source -------------------------------------------------------*/
    tmp = RCC->CFG & RCC_CFG_SCLKSTS;

    switch (tmp)
    {
        case RCC_CFG_SCLKSTS_HSI: /* HSI used as system clock */
            RCC_Clocks->SysclkFreq = HSI_VALUE;
            break;
        case RCC_CFG_SCLKSTS_HSE: /* HSE used as system clock */
            RCC_Clocks->SysclkFreq = HSE_VALUE;
            break;
        default:
            RCC_Clocks->SysclkFreq = HSI_VALUE;
            break;
    }

    /* Compute HCLK, PCLK1, PCLK2 and ADCCLK clocks frequencies ----------------*/
    /* Get HCLK prescaler */
    tmp   = RCC->CFG & RCC_CFG_AHBPRES;
    tmp   = tmp >> 4;

    presc = s_AhbPresTable[tmp];
    /* HCLK clock frequency */
    RCC_Clocks->HclkFreq = RCC_Clocks->SysclkFreq >> presc;
    /* Get PCLK1 prescaler */
    tmp   = RCC->CFG & RCC_CFG_APB1PRES;
    tmp   = tmp >> 8;
    presc = s_ApbPresTable[tmp];
    /* PCLK1 clock frequency */
    RCC_Clocks->Pclk1Freq = RCC_Clocks->HclkFreq >> presc;

    /* Get PCLK2 prescaler */
    tmp   = RCC->CFG & RCC_CFG_APB2PRES;
    tmp   = tmp >> 11;
    presc = s_ApbPresTable[tmp];
    /* PCLK2 clock frequency */
    RCC_Clocks->Pclk2Freq = RCC_Clocks->HclkFreq >> presc;
    
    /* HSE_DIV8 is the ADC clock source */ 
    if(RCC->CFG2 & RCC_ADCCLK_SRC_HSE_DIV8 )
    {
        RCC_Clocks->AdcclkFreq = HSE_VALUE/8;
    }
    /* AUDIOPLL is the ADC clock source */ 
    else
    {            
        RCC_Clocks->AdcclkFreq = ADC_AUDIOPLLCLK_VALUE;
    }    
}

/**
 * @brief  Enables or disables the AHB peripheral clock.
 * @param RCC_AHBPeriph specifies the AHB peripheral to gates its clock.
 *
 *   this parameter can be any combination of the following values:
 *     @arg RCC_AHB_PERIPH_DMA
 *     @arg RCC_AHB_PERIPH_SRAM
 *     @arg RCC_AHB_PERIPH_CRC
 *     @arg RCC_AHB_PERIPH_ADC
 *
 * @param Cmd new state of the specified peripheral clock.
 *   This parameter can be: ENABLE or DISABLE.
 */
void RCC_EnableAHBPeriphClk(uint32_t RCC_AHBPeriph, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_RCC_AHB_PERIPH(RCC_AHBPeriph));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        RCC->AHBPCLKEN |= RCC_AHBPeriph;
    }
    else
    {
        RCC->AHBPCLKEN &= ~RCC_AHBPeriph;
    }
}

/**
 * @brief  Enables or disables the High Speed APB (APB2) peripheral clock.
 * @param RCC_APB2Periph specifies the APB2 peripheral to gates its clock.
 *   This parameter can be any combination of the following values:
 *     @arg RCC_APB2_PERIPH_AFIO, RCC_APB2_PERIPH_GPIOA, RCC_APB2_PERIPH_GPIOB,
 *          RCC_APB2_PERIPH_SPI1, RCC_APB2_PERIPH_SPI2,
 *          RCC_APB2_PERIPH_TIM1, 
 *          RCC_APB2_PERIPH_USART1
 * @param Cmd new state of the specified peripheral clock.
 *   This parameter can be: ENABLE or DISABLE.
 */
void RCC_EnableAPB2PeriphClk(uint32_t RCC_APB2Periph, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_RCC_APB2_PERIPH(RCC_APB2Periph));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    if (Cmd != DISABLE)
    {
        RCC->APB2PCLKEN |= RCC_APB2Periph;
    }
    else
    {
        RCC->APB2PCLKEN &= ~RCC_APB2Periph;
    }
}

/**
 * @brief  Enables or disables the Low Speed APB (APB1) peripheral clock.
 * @param RCC_APB1Periph specifies the APB1 peripheral to gates its clock.
 *   This parameter can be any combination of the following values:
 *     @arg RCC_APB1_PERIPH_TIM3, 
 *          RCC_APB1_PERIPH_TIM6, 
 *          RCC_APB1_PERIPH_WWDG, 
 *          RCC_APB1_PERIPH_USART2,  
 *          RCC_APB1_PERIPH_I2C1, 
 *          RCC_APB1_PERIPH_PWR
 * @param Cmd new state of the specified peripheral clock.
 *   This parameter can be: ENABLE or DISABLE.
 */
void RCC_EnableAPB1PeriphClk(uint32_t RCC_APB1Periph, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_RCC_APB1_PERIPH(RCC_APB1Periph));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    if (Cmd != DISABLE)
    {
        RCC->APB1PCLKEN |= RCC_APB1Periph;
    }
    else
    {
        RCC->APB1PCLKEN &= ~RCC_APB1Periph;
    }
}

/**
 * @brief Forces or releases AHB peripheral reset.
 * @param RCC_AHBPeriph specifies the AHB peripheral to reset.
 *   This parameter can be any combination of the following values:
 *     @arg   RCC_AHB_PERIPH_ADC
 *            RCC_AHB_PERIPH_IRC
 * @param Cmd new state of the specified peripheral reset. This parameter can be ENABLE or DISABLE.
 */
void RCC_EnableAHBPeriphReset(uint32_t RCC_AHBPeriph, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_RCC_AHB_PERIPH_RESET(RCC_AHBPeriph));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    if (Cmd != DISABLE)
    {
        RCC->AHBPRST |= RCC_AHBPeriph;
    }
    else
    {
        RCC->AHBPRST &= ~RCC_AHBPeriph;
    }
}

/**
 * @brief  Forces or releases High Speed APB (APB2) peripheral reset.
 * @param RCC_APB2Periph specifies the APB2 peripheral to reset.
 *   This parameter can be any combination of the following values:
 *     @arg RCC_APB2_PERIPH_AFIO, RCC_APB2_PERIPH_GPIOA, RCC_APB2_PERIPH_GPIOB,
 *          RCC_APB2_PERIPH_SPI1, RCC_APB2_PERIPH_SPI2,          
 *          RCC_APB2_PERIPH_TIM1,  
 *          RCC_APB2_PERIPH_USART1
 * @param Cmd new state of the specified peripheral reset.
 *   This parameter can be: ENABLE or DISABLE.
 */
void RCC_EnableAPB2PeriphReset(uint32_t RCC_APB2Periph, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_RCC_APB2_PERIPH(RCC_APB2Periph));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    if (Cmd != DISABLE)
    {
        RCC->APB2PRST |= RCC_APB2Periph;
    }
    else
    {
        RCC->APB2PRST &= ~RCC_APB2Periph;
    }
}

/**
 * @brief  Forces or releases Low Speed APB (APB1) peripheral reset.
 * @param RCC_APB1Periph specifies the APB1 peripheral to reset.
 *   This parameter can be any combination of the following values:
 *     @arg RCC_APB1_PERIPH_TIM3, 
 *          RCC_APB1_PERIPH_TIM6, 
 *          RCC_APB1_PERIPH_USART2, 
 *          RCC_APB1_PERIPH_I2C1, 
 *          RCC_APB1_PERIPH_PWR
 * @param Cmd new state of the specified peripheral clock.
 *   This parameter can be: ENABLE or DISABLE.
 */
void RCC_EnableAPB1PeriphReset(uint32_t RCC_APB1Periph, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_RCC_APB1_PERIPH_RESET(RCC_APB1Periph));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    if (Cmd != DISABLE)
    {
        RCC->APB1PRST |= RCC_APB1Periph;
    }
    else
    {
        RCC->APB1PRST &= ~RCC_APB1Periph;
    }
    
}

/**
 * @brief  Selects the clock source to output on MCO pin.
 * @param RCC_MCO specifies the clock source to output.
 *
 *   this parameter can be one of the following values:
 *     @arg RCC_MCO_NOCLK       No clock selected
 *     @arg RCC_MCO_SYSCLK      System clock selected
 *     @arg RCC_MCO_HSI         HSI oscillator clock selected
 *     @arg RCC_MCO_HSE         HSE oscillator clock selected
 *     @arg RCC_MCO_PLLCLK_PRES PLL clock prescaler
 *
 */
void RCC_ConfigMco(uint8_t RCC_MCO)
{
    uint32_t tmpregister = 0;
    /* Check the parameters */
    assert_param(IS_RCC_MCO(RCC_MCO));

    tmpregister = RCC->CFG;
    /* Clear MCO[2:0] bits */
    tmpregister &= (~RCC_CFG_MCO);
    /* Set MCO[2:0] bits according to RCC_MCO value */
    tmpregister |= ((uint32_t)(RCC_MCO << 25));

    /* Store the new value */
    RCC->CFG = tmpregister;
}

/**
 * @brief  Checks whether the specified RCC flag is set or not.
 * @param RCC_FLAG specifies the flag to check.
 *
 *   this parameter can be one of the following values:
 *     @arg RCC_CTRL_FLAG_HSIRDF        HSI oscillator clock ready
 *     @arg RCC_CTRL_FLAG_HSERDF        HSE oscillator clock ready
 *     @arg RCC_LSCTRL_FLAG_LSIRD       LSI oscillator clock ready
 *     @arg RCC_LSCTRL_FLAG_LSERD       LSE oscillator clock ready
 *     @arg RCC_CTRLSTS_FLAG_PINRSTF    Pin reset
 *     @arg RCC_CTRLSTS_FLAG_PORRSTF    POR reset
 *     @arg RCC_CTRLSTS_FLAG_SFTRSTF    Software reset
 *     @arg RCC_CTRLSTS_FLAG_IWDGRSTF   Independent Watchdog reset
 *     @arg RCC_CTRLSTS_FLAG_WWDGRSTF   Window Watchdog reset
 *
 * @return The new state of RCC_FLAG (SET or RESET).
 */
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG)
{
    uint32_t tmp         = 0;
    uint32_t statusreg   = 0;
    FlagStatus bitstatus = RESET;
    /* Check the parameters */
    assert_param(IS_RCC_FLAG(RCC_FLAG));

    /* Get the RCC register index */
    tmp = RCC_FLAG >> 5;
    if (tmp == 1) /* The flag to check is in CTRL register */
    {
        statusreg = RCC->CTRL;
    }
    else if (tmp == 2) /* The flag to check is in BDCTRL register */
    {
        statusreg = RCC->LSCTRL;
    }
    else /* The flag to check is in CTRLSTS register */
    {
        statusreg = RCC->CTRLSTS;
    }

    /* Get the flag position */
    tmp = RCC_FLAG & RCC_FLAG_MASK;
    if ((statusreg & ((uint32_t)1 << tmp)) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    /* Return the flag status */
    return bitstatus;
}

/**
 * @brief  Clears the RCC reset flags.
 * @note   The reset flags are: RCC_CTRLSTS_FLAG_PINRSTF, RCC_CTRLSTS_FLAG_PORRSTF, 
 *                              RCC_CTRLSTS_FLAG_SFTRSTF, RCC_CTRLSTS_FLAG_IWDGRSTF, 
 *                              RCC_CTRLSTS_FLAG_WWDGRSTF
 */
void RCC_ClrFlag(void)
{
    /* Set RMVF bit to clear the reset flags */
    RCC->CTRLSTS |= RCC_CTRLSTS_RMRSTF;
    /* RMVF bit should be reset */
    RCC->CTRLSTS &= ~RCC_CTRLSTS_RMRSTF;
}

/**
 * @brief  Checks whether the specified RCC interrupt has occurred or not.
 * @param RccInt specifies the RCC interrupt source to check.
 *
 *   this parameter can be one of the following values:
 *     @arg RCC_INT_LSIRDIF LSI ready interrupt
 *     @arg RCC_INT_LSERDIF LSE ready interrupt
 *     @arg RCC_INT_HSIRDIF HSI ready interrupt
 *     @arg RCC_INT_HSERDIF HSE ready interrupt
 *
 * @return The new state of RccInt (SET or RESET).
 */
INTStatus RCC_GetIntStatus(uint8_t RccInt)
{
    INTStatus bitstatus = RESET;
    /* Check the parameters */
    assert_param(IS_RCC_GET_INT(RccInt));

    /* Check the status of the specified RCC interrupt */
    if ((RCC->CLKINT & RccInt) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    /* Return the RccInt status */
    return bitstatus;
}

/**
 * @brief  Clears the RCC's interrupt pending bits.
 * @param RccInt specifies the interrupt pending bit to clear.
 *
 *   this parameter can be any combination of the
 *   following values:
 *     @arg RCC_CLR_LSIRDIF Clear LSI ready interrupt flag
 *     @arg RCC_CLR_LSERDIF Clear LSE ready interrupt flag
 *     @arg RCC_CLR_HSIRDIF Clear HSI ready interrupt flag
 *     @arg RCC_CLR_HSERDIF Clear HSE ready interrupt flag
 */
void RCC_ClrIntPendingBit(uint32_t RccClrInt)
{
    /* Check the parameters */
    assert_param(IS_RCC_CLR_INTF(RccClrInt));
   /* Software set this bit to clear INT flag. */
    RCC->CLKINT |= RccClrInt;
}

/**
 * @brief  Enables or disables the ADC clock source AUDIOPLL.
 * @param  This parameter can be: ENABLE or DISABLE.
 */
void RCC_Enable_ADC_CLK_SRC_AUDIOPLL(FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if(Cmd == ENABLE)
   {
       /* Set RAMCERRRST bit */
       RCC->CTRL |= RCC_CTRL_AUDIOPLLEN;
   }
   else
   {
       /* Reset RAMCERRRST bit */
       RCC->CTRL &= ~RCC_CTRL_AUDIOPLLEN;
   }
}

/**
 * @brief  Configures the specified peripheral and low power mode behavior
 *   when the MCU under Debug mode.
 * @param DBG_Periph specifies the peripheral and low power mode.
 *   This parameter can be any combination of the following values:
 *     @arg DBG_IWDG_STOP Debug IWDG stopped when Core is halted
 *     @arg DBG_WWDG_STOP Debug WWDG stopped when Core is halted
 *     @arg DBG_TIM1_STOP TIM1 counter stopped when Core is halted
 *     @arg DBG_TIM3_STOP TIM3 counter stopped when Core is halted
 *     @arg DBG_I2C1SMBUS_TIMEOUT I2C1 SMBUS timeout mode stopped when Core is halted
 *     @arg DBG_TIM6_STOP TIM6 counter stopped when Core is halted
 * @param Cmd new state of the specified peripheral in Debug mode.
 *   This parameter can be: ENABLE or DISABLE.
 */
void DBG_ConfigPeriph(uint32_t DBG_Periph, FunctionalState Cmd)
{

    if (Cmd != DISABLE)
    {
        RCC->DBGMCU_CR |= DBG_Periph;
    }
    else
    {
        RCC->DBGMCU_CR &= ~DBG_Periph;
    }
}

/**
 * @}
 */ 

/**
 * @}
 */

/**
 * @}
 */
