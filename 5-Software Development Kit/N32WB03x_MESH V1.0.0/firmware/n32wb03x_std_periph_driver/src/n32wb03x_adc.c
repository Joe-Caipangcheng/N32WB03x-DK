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
 * @file n32wb03x_adc.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32wb03x_adc.h"
#include "n32wb03x_rcc.h"

/** @addtogroup N32WB03X_StdPeriph_Driver
 * @{
 */

/** @addtogroup ADC
 * @brief ADC driver modules
 * @{
 */

/** @addtogroup ADC_Private_TypesDefinitions
 * @{
 */

/**
 * @}
 */

/** @addtogroup ADC_Private_Defines
 * @{
 */


/**
 * @}
 */

/** @addtogroup ADC_Private_Macros
 * @{
 */

/**
 * @}
 */

/** @addtogroup ADC_Private_Variables
 * @{
 */

/**
 * @}
 */

/** @addtogroup ADC_Private_FunctionPrototypes
 * @{
 */

/**
 * @}
 */

/** @addtogroup ADC_Private_Functions
 * @{
 */
extern otp1_stored_t otp1_stored;
extern void system_opt_read(uint8_t* p_data,uint32_t byte_length);
/**
 * @brief  Deinitializes the ADCx peripheral registers to their default reset values.
 * @param ADCx = ADC, to select the ADC peripheral.
 */
void ADC_DeInit(ADC_Module* ADCx)
{
    /* Check the parameters */
    assert_param(IsAdcModule(ADCx));

    if (ADCx == ADC)
    {
    /* Enable ADC reset state */
    RCC_EnableAHBPeriphReset(RCC_AHB_PERIPH_ADC, ENABLE);
    /* Release ADC from reset state */
    RCC_EnableAHBPeriphReset(RCC_AHB_PERIPH_ADC, DISABLE);
    }
}


/**
 * @brief  Enables or disables the specified ADC peripheral.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Cmd new state of the ADCx peripheral.
 *   This parameter can be: ENABLE or DISABLE.
 */
void ADC_Enable(ADC_Module* ADCx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IsAdcModule(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    if (Cmd != DISABLE)
    {
        /* Enable GPADC */
        ADCx->CTRL |= ADC_CTRL_EN;
    }
    else
    {
        /* Disable GPADC */
        ADCx->CTRL &= ~ADC_CTRL_EN;
    }
}

/**
 * @brief  Enables or disables the specified ADC DMA request.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Cmd new state of the selected ADC DMA transfer.
 *   This parameter can be: ENABLE or DISABLE.
 */
void ADC_EnableDMA(ADC_Module* ADCx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IsAdcDmaModule(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    if (Cmd != DISABLE)
    {
        /* Enable the selected ADC DMA request */
        ADCx->CTRL |= ADC_CTRL_DMA_MODE_EN;
    }
    else
    {
        /* Disable the selected ADC DMA request */
        ADCx->CTRL &= ~ADC_CTRL_DMA_MODE_EN;
    }
}

/**
 * @brief  Enables or disables the specified ADC TS request.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Cmd new state of the selected ADC DMA transfer.
 *   This parameter can be: ENABLE or DISABLE.
 */
void ADC_EnableTS(ADC_Module* ADCx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IsAdcDmaModule(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    if (Cmd != DISABLE)
    {
        /* Enable the selected Temperature Sensor */
        ADCx->CTRL |= ADC_CTRL_TS_EN;
    }
    else
    {
        /* Disable the selected Temperature Sensor */
        ADCx->CTRL &= ~ADC_CTRL_TS_EN;
    }
}

/**
 * @brief  Enables or disables the specified Analog watchdog.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Cmd new state of the selected Analog watchdog.
 *   This parameter can be: ENABLE or DISABLE.
 */
void ADC_EnableAWD(ADC_Module* ADCx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IsAdcDmaModule(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    if (Cmd != DISABLE)
    {
        /* Enable the selected Analog watchdog */
        ADCx->CTRL |= ADC_CTRL_AWD_EN;
    }
    else
    {
        /* Disable the selected Analog watchdog */
        ADCx->CTRL &= ~ADC_CTRL_AWD_EN;
    }
}

/**
 * @brief  Enables or disables the specified ADC interrupts.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  ADC_IT specifies the ADC interrupt sources to be enabled or disabled.
 *   This parameter can be any combination of the following values:
 *     @arg ADC_INT_DONE GPADC Done Interrupt Enable
 *     @arg ADC_INT_AWD Analog watchdog interrupt mask
 *     @arg ADC_INT_PGARDY Used to enable the interrupt PGAREADY to interrupt processor when PGA fails
 * @param Cmd new state of the specified ADC interrupts.
 *   This parameter can be: ENABLE or DISABLE.
 */
void ADC_ConfigInt(ADC_Module* ADCx, uint16_t ADC_IT, FunctionalState Cmd)
{
    uint16_t itmask = 0;
    /* Check the parameters */
    assert_param(IsAdcModule(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));
    assert_param(IsAdcInt(ADC_IT));
    /* Get the ADC IT index */
    itmask = (uint16_t)ADC_IT;
    if (Cmd != DISABLE)
    {
        /* Enable the selected ADC interrupts */
        ADCx->CTRL |= itmask;
    }
    else
    {
        /* Disable the selected ADC interrupts */
        ADCx->CTRL &= (~(uint16_t)itmask);
    }
}

/**
 * @brief  Configures for the selected ADC channel
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  ADC_Channel the ADC channel to configure.
 *   This parameter can be any combination of the following values:
 *     @arg ADC_CTRL_CH_0 ADC Channel0 selected
 *     @arg ADC_CTRL_CH_1 ADC Channel1 selected
 *     @arg ADC_CTRL_CH_2 ADC Channel2 selected
 *     @arg ADC_CTRL_CH_3 ADC Channel3 selected
 *     @arg ADC_CTRL_CH_4 ADC Channel4 selected
 *     @arg ADC_CTRL_CH_5 ADC Channel5 selected
 *     @arg ADC_CTRL_CH_6 ADC Channel6 selected
 *     @arg ADC_CTRL_CH_7 ADC Channel7 selected
 */
void ADC_ConfigChannel(ADC_Module* ADCx, uint16_t Channel)
{
    uint16_t itmask = 0;
    
    /* Check the parameters */
    assert_param(IsAdcModule(ADCx));
    /* Get the old register value */
    itmask = ADCx->CTRL;
    /* Clear the old channel value */
    itmask &= ~ADC_CTRL_CH_SEL;
    /* Set the new channel value */
    itmask |= Channel;
    /* Store the new register value */
    ADCx->CTRL = itmask;
}

/**
 * @brief  Configures for the selected ADC Mode.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Cmd new state of the specified ADC Mode.
 *   This parameter can be: ENABLE or DISABLE.
 */
void ADC_ConfigContinuousMode(ADC_Module* ADCx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IsAdcDmaModule(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        /* Set ADC_MODE bit according to ContinueConvEn value */
        ADCx->CTRL |= ADC_CTRL_MODE;
    }
    else
    {
        /* Set ADC_MODE bit according to SingleEn value */
        ADCx->CTRL &= ~ADC_CTRL_MODE;
    }
}

/**
 * @brief  Configures the high and low thresholds of the analog watchdog.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  HighThreshold the ADC analog watchdog High threshold value.
 *   This parameter must be a 10bit value.
 * @param LowThreshold the ADC analog watchdog Low threshold value.
 *   This parameter must be a 10bit value.
 */
void ADC_ConfigAnalogWatchdogThresholds(ADC_Module* ADCx, uint32_t High_Threshold, uint32_t Low_Threshold)
{
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;

    /* Check the parameters */
    assert_param(IsAdcModule(ADCx));
    assert_param(IsAdcValid(High_Threshold));
    assert_param(IsAdcValid(Low_Threshold));
    /* Get the old register value */
    tmp1 = ADCx->WDT_THRES ;
    /* Clear the old value */
    tmp1 &= ~(ADC_WDT_THRES_HT | ADC_WDT_THRES_LT);
    /* Calculate the mask to set */
    tmp2 = High_Threshold << 18;
        /* Set the analog watchdog threshold */
    tmp1 |= (tmp2 | Low_Threshold);
    /* Store the new register value */
    ADCx->WDT_THRES = tmp1;
}

/**
 * @brief  Configures the high and low thresholds of the zero cross.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  HighThreshold the ADC zero cross High threshold value.
 *   This parameter must be a 8bit value.
 * @param LowThreshold the ADC zero cross Low threshold value.
 *   This parameter must be a 8bit value.
 */
void ADC_ConfigVoiceZcrThresholds(ADC_Module* ADCx, uint32_t High_Threshold, uint32_t Low_Threshold)
{
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;

    /* Check the parameters */
    assert_param(IsAdcDmaModule(ADCx));
    /* Get the old register value */
    tmp1 = ADCx->VOICE_ZCR_THRES;
    /* Clear the old value */
    tmp1 &= ~(ADC_VAD_ZCRD_HIGH_THRES | ADC_VAD_ZCRD_LOW_THRES);
    /* Calculate the mask to set */
    tmp2 = High_Threshold << 8;
    /* Set the zero cross threshold */
    tmp1 |= (tmp2 | Low_Threshold);
    /* Store the new register value */
    ADCx->VOICE_ZCR_THRES = tmp1;
}

/**
 * @brief  Configures the high and low thresholds of the energy detection.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  HighThreshold the ADC energy detection High threshold value.
 *   This parameter must be a 23bit value.
 * @param  LowThreshold the ADC energy detection Low threshold value.
 *   This parameter must be a 23bit value.
 */
void ADC_ConfigEDThreshold(ADC_Module* ADCx, uint32_t High_Threshold, uint32_t Low_Threshold)
{
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;

    /* Check the parameters */
    assert_param(IsAdcDmaModule(ADCx));
    /* Get the old register value */
    tmp1 = ADCx->VOICE_ED_UP_THRES;
    /* Get the old register value */
    tmp2 = ADCx->VOICE_ED_DWN_THRES;
    /* Clear the old value */
    tmp1 &= ~ADC_VOICE_ED_UP_THRES_GATE_MAX;
    /* Clear the old value */
    tmp2 &= ~ADC_VOICE_ED_DWN_THRES_GATE_MIN;
    /* Set the energy detection High threshold value */
    tmp1 |= High_Threshold;
    /* Set the energy detection Low threshold value */
    tmp2 |= Low_Threshold;
    /* Store the new register value */
    ADCx->VOICE_ED_UP_THRES = tmp1;
    /* Store the new register value */
    ADCx->VOICE_ED_DWN_THRES = tmp2;
}

/**
 * @brief  Set the environment noise level value.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  moffset the offset value for the selected environment noise level value.
 *   This parameter must be a 23bit value.
 */
void ADC_SetNoiseOffset(ADC_Module* ADCx, uint32_t moffset)
{
    uint32_t tmp1 = 0;
        /* Check the parameters */
        assert_param(IsAdcDmaModule(ADCx));
    /* Get the old register value */
    tmp1 = ADCx->VOICE_ED_THRES;
    /* Clear the old value */
    tmp1 &= ~ADC_VOICE_ED_THRES_MOFFSET;
    /* Set the environment noise level value */
    tmp1 |= moffset;
    /* Store the new register value */
    ADCx->VOICE_ED_THRES = tmp1;
}

/**
 * @brief  Returns the last ADCx conversion result data for channel.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @return The Data conversion value.
 */
uint16_t ADC_GetDat(ADC_Module* ADCx)
{
    /* Check the parameters */
    assert_param(IsAdcModule(ADCx));
    /* Return the selected ADC conversion value */
    return (uint16_t)ADCx->DAT;
}

/**
 * @brief  Checks whether the specified ADC flag is set or not.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  ADC_FLAG specifies the flag to check.
 *   This parameter can be one of the following values:
 *     @arg ADC_FLAG_DONE GP-ADC conversion complete status
 *     @arg ADC_FLAG_AWD Analog watchdog flag
 *     @arg ADC_FLAG_PGARDY PGA fail status
 * @return The new state of ADC_FLAG (SET or RESET).
 */
FlagStatus ADC_GetFlagStatus(ADC_Module* ADCx, uint8_t ADC_FLAG)
{
    FlagStatus bitstatus = RESET;
    /* Check the parameters */
    assert_param(IsAdcModule(ADCx));
    assert_param(IsAdcGetFlag(ADC_FLAG));
    /* Check the status of the specified ADC flag */
    if ((ADCx->SR & ADC_FLAG) != (uint8_t)RESET)
    {
        /* ADC_FLAG is set */
        bitstatus = SET;
    }
    else
    {
        /* ADC_FLAG is reset */
        bitstatus = RESET;
    }
    /* Return the ADC_FLAG status */
    return bitstatus;
}

/**
 * @brief  Clears the ADCx's pending flags.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  ADC_FLAG specifies the flag to clear.
 *   This parameter can be any combination of the following values:
 *     @arg ADC_FLAG_DONE GP-ADC conversion complete status
 *     @arg ADC_FLAG_AWD Analog watchdog flag
 *     @arg ADC_FLAG_PGARDY PGA fail status
 */
void ADC_ClearFlag(ADC_Module* ADCx, uint8_t ADC_FLAG)
{
    /* Check the parameters */
    assert_param(IsAdcModule(ADCx));
    assert_param(IsAdcClrFlag(ADC_FLAG));
    /* Clear the selected ADC flags */
    ADCx->SR = ~(uint8_t)ADC_FLAG;
}

/**
 * @brief  Enables or disables the selected ADC zero cross.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Cmd new state of the selected ADC zero cross.
 *   This parameter can be: ENABLE or DISABLE.
 */
void ADC_EnableZeroCross(ADC_Module* ADCx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IsAdcDmaModule(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        /* Enable zero cross */
        ADCx->VOICE_DET_CR |= ADC_VOICE_DET_CR_ZCRD_EN;
    }
    else
    {
        /* Disable zero cross */
        ADCx->VOICE_DET_CR &= ~ADC_VOICE_DET_CR_ZCRD_EN;
    }
}

/**
 * @brief  Enables or disables the selected ADC energy .
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Cmd new state of the selected ADC energy.
 *   This parameter can be: ENABLE or DISABLE.
 */
void ADC_EnableEnergy(ADC_Module* ADCx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IsAdcDmaModule(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        /* Enable energy */
        ADCx->VOICE_DET_CR |= ADC_VOICE_DET_CR_ED_EN;
    }
    else
    {
        /* Disable energy */
        ADCx->VOICE_DET_CR &= !ADC_VOICE_DET_CR_ED_EN;
    }
}

/**
 * @brief  Enables or disables the selected bypass the digital filtering and ED/ZCRD features .
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Cmd new state of the selected ADC bypass.
 *   This parameter can be: ENABLE or DISABLE.
 */
void ADC_EnableBypassFilter(ADC_Module* ADCx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IsAdcDmaModule(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        /* Enbale bypass mode */
        ADCx->VOICE_DET_CR |= ADC_VOICE_DET_CR_FILTTER_BYP;
    }
    else
    {
        /* Disbale bypass mode */
        ADCx->VOICE_DET_CR &= ADC_VOICE_DET_CR_FILTTER_BYP;
    }
}

/**
 * @brief  Enables or disables the selected the ADC MIC bias .
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Cmd new state of the selected the ADC MIC bias.
 *   This parameter can be: ENABLE or DISABLE.
 */
void ADC_EnableMICBIAS(ADC_Module* ADCx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IsAdcDmaModule(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        /* Enbale  MIC bias*/
        ADCx->FGA_CFG |= ADC_PGA_CFG_MICBIAS_EN;
    }
    else
    {
        /* Disbale  MIC bias*/
        ADCx->FGA_CFG &= ~ADC_PGA_CFG_MICBIAS_EN;
    }
}

/**
 * @brief  Set the MIC bias output voltage.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  MIC_VoltageValue the value for the selected the MIC bias output voltage.
 *   This parameter can be any combination of the following values:
 *     @arg MICBIAS_1V6 
 *     @arg MICBIAS_1V7 
 *     @arg MICBIAS_1V8 
 *     @arg MICBIAS_1V9 
 *     @arg MICBIAS_2V0 
 *     @arg MICBIAS_2V1 
 *     @arg MICBIAS_2V2 
 *     @arg MICBIAS_2V3 
 */
void ADC_SetMICOutputVoltage(ADC_Module* ADCx, uint32_t MIC_VoltageValue)
{
    uint32_t tmp = 0;
    /* Get the old register value */
    tmp = ADC->FGA_CFG;
    /* Clear the old value */
    tmp &= ~ADC_PGA_CFG_MICBIAS;
    /* Set the MIC bias output voltage */
    tmp |= MIC_VoltageValue;
    /* Store the new register value */
    ADC->FGA_CFG = tmp;
}

/**
 * @brief  Enables or disables the selected the Audio PGA fast charge control.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Cmd new state of the selected the Audio PGA fast charge control.
 *   This parameter can be: ENABLE or DISABLE.
 */
void ADC_EnableAUDIOPGA(ADC_Module* ADCx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IsAdcDmaModule(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        /* Enbale fast charge */
        ADCx->FGA_CFG |= ADC_PGA_CFG_INIT_ENA;
    }
    else
    {
        /* Disbale fast charge */
        ADCx->FGA_CFG &= ~ADC_PGA_CFG_INIT_ENA;
    }
}

/**
 * @brief  Set the Audio PGA drive signal amplitud.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  AMP_Value the value for the selected the Audio PGA drive signal amplitud.
 *   This parameter can be any combination of the following values:
 *     @arg AUDIOPGA_PEAK_0 
 *     @arg AUDIOPGA_PEAK_1 
 *     @arg AUDIOPGA_PEAK_2
 *     @arg AUDIOPGA_PEAK_3 
 */
void ADC_SetAUDIOAmplitude(ADC_Module* ADCx, uint32_t AMP_Value)
{
    uint32_t tmp = 0;
    /* Get the old register value */
    tmp = ADC->FGA_CFG;
    /* Clear the old value */
    tmp &= ~ADC_PGA_CFG_PEAK;
    /* Set the Audio PGA drive signal amplitud */
    tmp |= AMP_Value;
    /* Store the new register value */
    ADC->FGA_CFG = tmp;
}

/**
 * @brief  Set the Audio PGA drive current.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Drive_Value the value for the selected the Audio PGA drive current.
 *   This parameter can be any combination of the following values:
 *     @arg AUDIOPGA_DRIVE_0 
 *     @arg AUDIOPGA_DRIVE_1 
 *     @arg AUDIOPGA_DRIVE_2
 *     @arg AUDIOPGA_DRIVE_3 
 */
void ADC_SetAUDIOCurrent(ADC_Module* ADCx, uint32_t Drive_Value)
{
    uint32_t tmp = 0;
    /* Get the old register value */
    tmp = ADC->FGA_CFG;
    /* Clear the old value */
    tmp &= ~ADC_PGA_CFG_DRIVE;
    /* Set the Audio PGA drive current */
    tmp |= Drive_Value;
    /* Store the new register value */
    ADC->FGA_CFG = tmp;
}

/**
 * @brief  Enables or disables the selected the Audio PGA.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Cmd new state of the selected the Audio PGA.
 *   This parameter can be: ENABLE or DISABLE.
 */
void ADC_EnablePGA(ADC_Module* ADCx, FunctionalState Cmd)
{
    /* Check the parameters */
    assert_param(IsAdcDmaModule(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(Cmd));

    if (Cmd != DISABLE)
    {
        /* Enable Audio PGA */
        ADCx->FGA_CFG |= ADC_PGA_CFG_EN;
    }
    else
    {
        /* Disable Audio PGA */
        ADCx->FGA_CFG &= ~ADC_PGA_CFG_EN;
    }
}

/**
 * @brief  Set the Audio PGA gain configuration.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Gain_Value the value for the selected the Audio PGA gain configuration.
 *   This parameter can be any combination of the following values:
 *     @arg ADC_PGA_CFG_GAIN_0dB 
 *     @arg ADC_PGA_CFG_GAIN_6dB 
 *     @arg ADC_PGA_CFG_GAIN_12dB
 *     @arg ADC_PGA_CFG_GAIN_18dB 
 *     @arg ADC_PGA_CFG_GAIN_24dB 
 *     @arg ADC_PGA_CFG_GAIN_30dB 
 *     @arg ADC_PGA_CFG_GAIN_36dB
 *     @arg ADC_PGA_CFG_GAIN_42dB
 */
void ADC_SetPGAGain(ADC_Module* ADCx, uint32_t Gain_Value)
{
    uint32_t tmp = 0;
    /* Get the old register value */
    tmp = ADC->FGA_CFG;
    /* Clear the old value */
    tmp &= ~ADC_PGA_CFG_GAIN;
    /* Set the MIC bias output voltage */
    tmp |= Gain_Value;
    /* Store the new register value */
    ADC->FGA_CFG = tmp;
}

/**
 * @brief  Set the Over Sampling Counter.
 * @param  ADCx where x can be 1 to select the ADC peripheral.
 * @param  Cnt the value for the selected the Over Sampling Counter.
 *   This parameter must be a 5bit value.
 */
void ADC_SetOverSampleCounter(ADC_Module* ADCx, uint32_t Cnt)
{
    uint32_t tmp = 0;
    /* Get the old register value */
    tmp = ADC->OVR_SAMP_CNT;
    /* Clear the old value */
    tmp &= ~ADC_OS_CNT_LD_CNT;
    /* Set the MIC bias output voltage */
    tmp |= Cnt;
    /* Store the new register value */
    ADC->OVR_SAMP_CNT = tmp;
}
    
/**
 * @brief  Conver adc value to voltage
 * @param  adc_value the adv value want to conver to voltage.
 * @param  Channel the adc value is from which ADC channel(1 to 6) 
 * @return The voltage in mA unit after conver.
 */
uint32_t ADC_ConverValueToVoltage(uint16_t adc_value, uint16_t channel)
{

    uint16_t value_3400mv;
    uint16_t value_600mv;
    uint16_t value_850mv;
    uint16_t value_150mv;
    uint32_t voltage_mA = 0;

    /* Check if otp has been read to ram */
    if(otp1_stored.rc_gpadc_value_3400mv == 0xFFFFFFFF || otp1_stored.rc_gpadc_value_3400mv == 0)
    {
        system_opt_read((uint8_t*)&otp1_stored,sizeof(otp1_stored));
        /* Check if otp is empty */
        if(otp1_stored.rc_gpadc_value_3400mv == 0xFFFFFFFF)
        {
            /* default value*/
            otp1_stored.rc_gpadc_value_3400mv = 999;
            otp1_stored.rc_gpadc_value_600mv  = 61;
        }
    }

    value_3400mv = otp1_stored.rc_gpadc_value_3400mv;
    value_600mv  = otp1_stored.rc_gpadc_value_600mv;
    value_850mv  = otp1_stored.rc_gpadc_value_3400mv;
    value_150mv  = otp1_stored.rc_gpadc_value_600mv;
    if(adc_value < value_600mv)
    {
        adc_value = value_600mv;
    }
    else if(adc_value > value_3400mv)
    {
        adc_value = value_3400mv;
    }
    if( channel == ADC_CTRL_CH_1 || channel == ADC_CTRL_CH_2 )
    {
        /* CH1 and CH2 125mV to 850mA */
        voltage_mA =((uint32_t)(adc_value - value_150mv) * 700)/(value_850mv - value_150mv) + 150;
            
    }
    else if(channel == ADC_CTRL_CH_3 || channel == ADC_CTRL_CH_4|| channel == ADC_CTRL_CH_5 || channel == ADC_CTRL_CH_6)
    {
        /* CH3, CH4 and CH5 600mV to 3400mA */
        voltage_mA =((uint32_t)(adc_value - value_600mv)* 2800)/(value_3400mv - value_600mv) + 600;
    }
    return  voltage_mA;
}


/**
 * @brief  Conver adc value to temperature in centigrade
 * @param  adc_value the adv value want to conver to voltage.
 * @param  Channel the adc value is from which ADC channel(1 to 7) 
 * @return The voltage in mA unit after conver.
 */
float ADC_ConverValueToTemperature(uint16_t adc_value)
{
    float Temperature;
    /* Check if otp has been read to ram */
    if(otp1_stored.rc_adc_ts_25c == 0xFFFFFFFF || otp1_stored.rc_adc_ts_25c == 0)
    {
        system_opt_read((uint8_t*)&otp1_stored,sizeof(otp1_stored));
        /* Check if otp is empty */
        if(otp1_stored.rc_adc_ts_25c == 0xFFFFFFFF || otp1_stored.rc_adc_ts_25c == 0)
        {
            /* default value*/
            otp1_stored.rc_adc_ts_25c  = 750;
        }
    }
    
    Temperature = (((float)otp1_stored.rc_adc_ts_25c - adc_value)/(2.137)) + 25;
    return Temperature;
}



/**
 * @}
 */

/**
 * @}S
 */

/**
 * @}
 */
