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
 * @file app_rtc.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "app_rtc.h"
#include "main.h"
#include <stdio.h>


/** @addtogroup 
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define RCT_LOG_ENABLE
#ifdef RCT_LOG_ENABLE
#define log_info printf
#else
#define log_info(str, ...) 
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RTC_DateType RTC_DateStructure;
RTC_DateType RTC_DateDefault;
RTC_TimeType RTC_TimeStructure;
RTC_TimeType RTC_TimeDefault;
RTC_InitType RTC_InitStructure;
RTC_AlarmType RTC_AlarmStructure;
uint32_t SynchPrediv, AsynchPrediv;

/* Private function prototypes -----------------------------------------------*/
void RTC_CLKSourceConfig(uint8_t ClkSrc);
ErrorStatus RTC_TimeRegulate(void);
ErrorStatus RTC_DateRegulate(void);
void RTC_DateAndTimeDefaultVale(void);
static void RTC_PrescalerConfig(void);
void EXTI9_RTCWKUP_Configuration(FunctionalState Cmd);
void WakeUpClockSelect(uint8_t WKUPClkSrcSel);




/* Private functions ---------------------------------------------------------*/

/**
 * @brief  app rtc init
 */
void RTC_Configuration(void)
{
     if((RTC->INITSTS&RTC_INITSTS_INITSF) == 0)
     {
        /* RTC date time alarm default value*/
        RTC_DateAndTimeDefaultVale();

        /* RTC clock source select 1:LSE 2:LSI*/
        RTC_CLKSourceConfig(1);
        RTC_PrescalerConfig();

        /* Adjust time by values entered by the user on the hyperterminal */
        RTC_DateRegulate();
        RTC_TimeRegulate();
     }
     else
     {
        /* Enable the RTC Clock */
        RCC_EnableRtcClk(ENABLE);
        RTC_WaitForSynchro();
        RTC_TimeShow();
     }
     
    
    #ifdef TEST_200MS_WAKEUP    
    /* wake up clock select */
    WakeUpClockSelect(1);
    /* wake up timer value */
    RTC_SetWakeUpCounter( 1000 );//400->200ms ,1000->500ms
    #else

    /* wake up clock select */
    WakeUpClockSelect(5);
    /* wake up timer value */
    RTC_SetWakeUpCounter(10);
    #endif         

    EXTI9_RTCWKUP_Configuration(ENABLE);
    /* Enable the RTC Wakeup Interrupt */
    RTC_ConfigInt(RTC_INT_WUT, ENABLE);
    
        

}


/**
 * @brief  Wake up clock config.
 */
void WakeUpClockSelect(uint8_t WKUPClkSrcSel)
{
    /* Configure the RTC WakeUp Clock source: CK_SPRE (1Hz) */
    if (WKUPClkSrcSel == 0x01)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV16);
    else if (WKUPClkSrcSel == 0x02)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV8);
    else if (WKUPClkSrcSel == 0x03)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV4);
    else if (WKUPClkSrcSel == 0x04)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV2);
    else if (WKUPClkSrcSel == 0x05)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_CK_SPRE_16BITS);
    else if (WKUPClkSrcSel == 0x06)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_CK_SPRE_17BITS);
}
/**
 * @brief  Config RTC wake up Interrupt.
 */
void EXTI9_RTCWKUP_Configuration(FunctionalState Cmd)
{
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    EXTI_ClrITPendBit(EXTI_LINE9);
    EXTI_InitStructure.EXTI_Line = EXTI_LINE9;
#ifdef __TEST_SEVONPEND_WFE_NVIC_DIS__
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
#else
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
#endif
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    /* Enable the RTC WakeUp Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = Cmd;
    NVIC_Init(&NVIC_InitStructure);
}



/**
 * @brief  Display the current Date on the Hyperterminal.
 */
void RTC_DateShow(void)
{
    /* Get the current Date */
    RTC_GetDate(RTC_FORMAT_BIN, &RTC_DateStructure);
    log_info("The current date (WeekDay-Date-Month-Year) is : %0.2d-%0.2d-%0.2d-%0.2d\r\n",
             RTC_DateStructure.WeekDay,
             RTC_DateStructure.Date,
             RTC_DateStructure.Month,
             RTC_DateStructure.Year);
}

/**
 * @brief  Display the current time on the Hyperterminal.
 */
void RTC_TimeShow(void)
{
    /* Get the current Time and Date */
    RTC_GetTime(RTC_FORMAT_BIN, &RTC_TimeStructure);
    log_info("The current time (Hour-Minute-Second) is : %0.2d:%0.2d:%0.2d \r\n",
             RTC_TimeStructure.Hours,
             RTC_TimeStructure.Minutes,
             RTC_TimeStructure.Seconds);
    /* Unfreeze the RTC DAT Register */
    (void)RTC->DATE;
}
/**
 * @brief  RTC initalize default value.
 */
void RTC_DateAndTimeDefaultVale(void)
{ // Date
    RTC_DateDefault.WeekDay = 3;
    RTC_DateDefault.Date    = 20;
    RTC_DateDefault.Month   = 11;
    RTC_DateDefault.Year    = 19;
    // Time
    RTC_TimeDefault.H12     = RTC_AM_H12;
    RTC_TimeDefault.Hours   = 4;
    RTC_TimeDefault.Minutes = 5;
    RTC_TimeDefault.Seconds = 1;
}
/**
 * @brief  RTC date regulate with the default value.
 */
ErrorStatus RTC_DateRegulate(void)
{
    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;

    tmp_hh = RTC_DateDefault.WeekDay;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStructure.WeekDay = tmp_hh;
    }
    tmp_hh = 0xFF;
    tmp_hh = RTC_DateDefault.Date;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStructure.Date = tmp_hh;
    }
    tmp_mm = RTC_DateDefault.Month;
    if (tmp_mm == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStructure.Month = tmp_mm;
    }
    tmp_ss = RTC_DateDefault.Year;
    if (tmp_ss == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStructure.Year = tmp_ss;
    }

    /* Configure the RTC date register */
    if (RTC_SetDate(RTC_FORMAT_BIN, &RTC_DateStructure) == ERROR)
    {
        log_info(">> !! RTC Set Date failed. !! <<\n\r");
        return ERROR;
    }
    else
    {
        RTC_DateShow();
        return SUCCESS;
    }
}
/**
 * @brief  RTC time regulate with the default value.
 */
ErrorStatus RTC_TimeRegulate(void)
{
    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;

    RTC_TimeStructure.H12 = RTC_TimeDefault.H12;
    tmp_hh = RTC_TimeDefault.Hours;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_TimeStructure.Hours = tmp_hh;
    }
    tmp_mm = RTC_TimeDefault.Minutes;
    if (tmp_mm == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_TimeStructure.Minutes = tmp_mm;
    }

    tmp_ss = RTC_TimeDefault.Seconds;
    if (tmp_ss == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_TimeStructure.Seconds = tmp_ss;
    }


    /* Configure the RTC time register */
    if (RTC_ConfigTime(RTC_FORMAT_BIN, &RTC_TimeStructure) == ERROR)
    {
        log_info(">> !! RTC Set Time failed. !! <<\n\r");
        return ERROR;
    }
    else
    {
        RTC_TimeShow();
        return SUCCESS;
    }
}

/**
 * @brief  RTC prescaler config.
 */
static void RTC_PrescalerConfig(void)
{
    /* Configure the RTC data register and RTC prescaler */
    RTC_InitStructure.RTC_AsynchPrediv = AsynchPrediv;
    RTC_InitStructure.RTC_SynchPrediv  = SynchPrediv;
    RTC_InitStructure.RTC_HourFormat   = RTC_24HOUR_FORMAT;

    /* Check on RTC init */
    if (RTC_Init(&RTC_InitStructure) == ERROR)
    {
        log_info("//******* RTC Prescaler Config failed **********// \r\n");
    }
}

/**
 * @brief  Configures RTC.
 *   Configure the RTC peripheral by selecting the clock source
 */

void RTC_CLKSourceConfig(uint8_t ClkSrc)
{
    assert_param(IS_CLKSRC_VALUE(ClkSrc));
    assert_param(IS_FLCFG_VALUE(FirstLastCfg));

    /* Enable the PWR clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

    /* Disable RTC clock */
    RCC_EnableRtcClk(DISABLE);


    if (ClkSrc == 0x01)
    {

#if (_TEST_LSE_BYPASS_)
        RCC_ConfigLse(RCC_LSE_BYPASS);
#else
        RCC_ConfigLse(RCC_LSE_ENABLE);
#endif
        
        while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSERD) == RESET)
        {
        }
        
        RCC_ConfigLSXSEL(RCC_RTCCLK_SRC_LSE);
        RCC_EnableLsi(DISABLE);

        SynchPrediv  = 0xFF; // 32.768KHz
        AsynchPrediv = 0x7F; // value range: 0-7F
    }
    else if (ClkSrc == 0x02)
    {
        RCC_EnableLsi(ENABLE);
        while (RCC_GetFlagStatus(RCC_LSCTRL_FLAG_LSIRD) == RESET)
        {
        }
        
        RCC_ConfigLSXSEL(RCC_RTCCLK_SRC_LSI);
        
        SynchPrediv  = 0xFF; // 32.768KHz
        AsynchPrediv = 0x7F;  // value range: 0-7F
    }
    else
    {
        log_info("RTC_ClkSrc Value is error! \r\n");
    }

    /* Enable the RTC Clock */
    RCC_EnableRtcClk(ENABLE);
    RTC_WaitForSynchro();
}


/**
 * @}
 */
