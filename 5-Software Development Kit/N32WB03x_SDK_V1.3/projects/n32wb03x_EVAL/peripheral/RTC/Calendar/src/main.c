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
 *5
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
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include "log.h"
#include "n32wb03x_rtc.h"
#include "n32wb03x.h"
/** @addtogroup RTC_Calendar
 * @{
 */
RTC_DateType RTC_DateStructure;
RTC_DateType RTC_DateDefault;
RTC_TimeType RTC_TimeStructure;
RTC_TimeType RTC_TimeDefault;
RTC_InitType RTC_InitStructure;
RTC_AlarmType RTC_AlarmStructure;
uint32_t SynchPrediv, AsynchPrediv;

void RTC_CLKSourceConfig(uint8_t ClkSrc);
ErrorStatus RTC_TimeRegulate(void);
ErrorStatus RTC_DateRegulate(void);
void RTC_DateAndTimeDefaultVale(void);
static void RTC_PrescalerConfig(void);
void TIM3_Configuration(void);
void RTC_TimeShow(void);
void RTC_DateShow(void);
    
/**
 * @brief  Main program.
 */
int main(void)
{
    /* Initialize debug uart*/
    log_init();
    log_info("\r\nRTC calendar demo\r\n");
    
    /* RTC date time alarm default value*/
    RTC_DateAndTimeDefaultVale();

    /* RTC clock source select 1:LSE 2:LSI*/
    RTC_CLKSourceConfig(2);
    RTC_PrescalerConfig();
    log_info("RTC configured....");

    /* Adjust time by values entered by the user on the hyperterminal */
    /* Disable the RTC Wakeup Interrupt and delay more than 100us before set data and time */
    RTC_ConfigInt(RTC_INT_WUT, DISABLE);
    RTC_EnableWakeUp(DISABLE);
    RTC_DateRegulate();
    RTC_TimeRegulate();

    /* Configure the tim3 to generate an Interrupt with a period of 1s*/
    TIM3_Configuration();

    log_info("\r\n RTC Config end....\r\n\r\n\r\n\r\n\r\n\r\n\r\n");

    while (1)
        ;
}

void TIM3_Configuration(void)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    NVIC_InitType NVIC_InitStructure;
    uint16_t PrescalerValue = 0;
    
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);
    
    /* Compute the prescaler value */
    PrescalerValue = (uint16_t) (SystemCoreClock / 1000) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructure.Period    = 1000-1;
    TIM_TimeBaseStructure.Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);

    /* TIM3 enable update irq */
    TIM_ConfigInt(TIM3, TIM_INT_UPDATE, ENABLE);

    /* TIM3 enable counter */
    TIM_Enable(TIM3, ENABLE);
    
    /* Enable the TIM3 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Display the current Date on the Hyperterminal.
 */
void RTC_DateShow(void)
{
    /* Get the current Date */
    RTC_GetDate(RTC_FORMAT_BIN, &RTC_DateStructure);
    log_info("\n\r //=========== Current Date Display ==============// \n\r");
    log_info("\n\r The current date (WeekDay-Date-Month-Year) is :  %0.2d-%0.2d-%0.2d-%0.2d \n\r",
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
    log_info("\n\r //============ Current Time Display ===============// \n\r");
    log_info("\n\r The current time (Hour-Minute-Second) is :  %0.2d:%0.2d:%0.2d \n\r",
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
    log_info("\n\r //=============Date Settings================// \n\r");

    log_info("\n\r Please Set WeekDay (01-07) \n\r");
    tmp_hh = RTC_DateDefault.WeekDay;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStructure.WeekDay = tmp_hh;
    }
    log_info(": %0.2d\n\r", tmp_hh);

    tmp_hh = 0xFF;
    log_info("\n\r Please Set Date (01-31) \n\r");
    tmp_hh = RTC_DateDefault.Date;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStructure.Date = tmp_hh;
    }
    log_info(": %0.2d\n\r", tmp_hh);

    log_info("\n\r Please Set Month (01-12)\n\r");
    tmp_mm = RTC_DateDefault.Month;
    if (tmp_mm == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStructure.Month = tmp_mm;
    }
    log_info(": %0.2d\n\r", tmp_mm);

    log_info("\n\r Please Set Year (00-99)\n\r");
    tmp_ss = RTC_DateDefault.Year;
    if (tmp_ss == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStructure.Year = tmp_ss;
    }
    log_info(": %0.2d\n\r", tmp_ss);

    /* Configure the RTC date register */
    if (RTC_SetDate(RTC_FORMAT_BIN, &RTC_DateStructure) == ERROR)
    {
        log_info("\n\r>> !! RTC Set Date failed. !! <<\n\r");
        return ERROR;
    }
    else
    {
        log_info("\n\r>> !! RTC Set Date success. !! <<\n\r");
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
    log_info("\n\r //==============Time Settings=================// \n\r");

    RTC_TimeStructure.H12 = RTC_TimeDefault.H12;

    log_info("\n\r Please Set Hours \n\r");
    tmp_hh = RTC_TimeDefault.Hours;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_TimeStructure.Hours = tmp_hh;
    }
    log_info(": %0.2d\n\r", tmp_hh);

    log_info("\n\r Please Set Minutes \n\r");
    tmp_mm = RTC_TimeDefault.Minutes;
    if (tmp_mm == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_TimeStructure.Minutes = tmp_mm;
    }
    log_info(": %0.2d\n\r", tmp_mm);

    log_info("\n\r Please Set Seconds \n\r");
    tmp_ss = RTC_TimeDefault.Seconds;
    if (tmp_ss == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_TimeStructure.Seconds = tmp_ss;
    }
    log_info(": %0.2d\n\r", tmp_ss);

    /* Configure the RTC time register */
    if (RTC_ConfigTime(RTC_FORMAT_BIN, &RTC_TimeStructure) == ERROR)
    {
        log_info("\n\r>> !! RTC Set Time failed. !! <<\n\r");
        return ERROR;
    }
    else
    {
        log_info("\n\r>> !! RTC Set Time success. !! <<\n\r");
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
        log_info("\r\n //******* RTC Prescaler Config failed **********// \r\n");
    }
}

/**
 * @brief  Configures RTC.
 *   Configure the RTC peripheral by selecting the clock source
 */

void RTC_CLKSourceConfig(uint8_t ClkSrc)
{
    assert_param(IS_CLKSRC_VALUE(ClkSrc));

    /* Enable the PWR clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

    /* Disable RTC clock */
    RCC_EnableRtcClk(DISABLE);

    if (ClkSrc == 0x01)
    {
        log_info("\r\n RTC_ClkSrc Is Set LSE! \r\n");

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
        log_info("\r\n RTC_ClkSrc Is Set LSI! \r\n");

        /* Enable the LSI OSC */
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
        log_info("\r\n RTC_ClkSrc Value is error! \r\n");
    }

    /* Enable the RTC Clock */
    RCC_EnableRtcClk(ENABLE);
    RTC_WaitForSynchro();
}
