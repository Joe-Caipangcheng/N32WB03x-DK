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
 * @file n32wb03x_adc.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __N32WB03X_ADC_H__
#define __N32WB03X_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32wb03x.h"
#include <stdbool.h>

/** @addtogroup N32WB03X_StdPeriph_Driver
 * @{
 */

/** @addtogroup ADC
 * @{
 */

/** @addtogroup ADC_Exported_Types
 * @{
 */

/**
 * @brief  ADC Init structure definition
 */
typedef struct
{

    FunctionalState MicbiasEn;      /*!< MIC BIAS enable control signal
                                           This parameter can be set to ENABLE or DISABLE */

    FunctionalState AudiopgaInitEn; /*!< Audio PGA off-chip capacitorfast charge control
                                            This parameter can be set to ENABLE or DISABLE. */

    FunctionalState PgaEn;          /*!< Audio PGA Enable
                                             This parameter can be set to ENABLE or DISABLE. */

    uint32_t Micbias;               /*!< MIC BIAS output voltage range setting, step=0.1V
                                             This parameter must range from 1.6 to 2.3 */

    uint32_t AudiopgaPeak;          /*!< Audio PGA drive signal amplitude setting. */                                

    uint32_t AudiopgaDrive;         /*!< Audio PGA drive current setting */
                                                                     
    uint32_t PgaGain;               /*!< Audio PGA gain configuration (0??42dB) */

} ADC_PGA_InitType;
/**
 * @}
 */

/** @addtogroup ADC_Exported_Constants
 * @{
 */

#define IsAdcModule(PERIPH) (((PERIPH) == ADC))

#define IsAdcDmaModule(PERIPH) (((PERIPH) == ADC))

/** @addtogroup MIC BIAS output voltage range setting, step=0.1V
 * @{
 */
 

/** @addtogroup Audio PGA drive signal amplitude setting
 * @{
 */

/** @addtogroup Audio PGA drive current setting
 * @{
 */

/** @addtogroup ADC_Exported_Constants
 * @{
 */


/**
 * @brief  ADC Init structure definition
 */
typedef struct
{
    FunctionalState VadZcrdEn; /*!< Audio PGA off-chip capacitorfast charge control
                                            This parameter can be set to ENABLE or DISABLE. */

    FunctionalState VadEdEn;          /*!< Audio PGA Enable
                                             This parameter can be set to ENABLE or DISABLE. */
    
        FunctionalState VadFilterByp;         /*!< Decides between Single and Continuous Conversion modes
                                             This parameter can be set to ENABLE or DISABLE. */
} ADC_DET_InitType;
             
/**
 * @}
 */

/** @addtogroup ADC_channels
 * @{
 */

/**
 * @}
 */


/** @addtogroup ADC_interrupts_definition
    the hign 8 bits (02,01,04) do not correspond to CTRL1 register, 
        the main poupose is to match the corresponding flag bit in the sts register 
 * @{
 */

#define ADC_INT_DONE     ((uint16_t)0x0100)
#define ADC_INT_AWD      ((uint16_t)0x0200)
#define ADC_INT_PGARDY   ((uint16_t)0x0400)

#define IsAdcInt(IT) ((((IT) & (uint16_t)0xF8FF) == 0x00) && ((IT) != 0x00))

#define IsAdcGetInt(IT) (((IT) == ADC_INT_DONE) || ((IT) == ADC_INT_AWD) || ((IT) == ADC_INT_PGARDY))
/**
 * @}
 */

/** @addtogroup ADC_flags_definition
 * @{
 */

#define ADC_FLAG_DONE       ((uint8_t)0x01)
#define ADC_FLAG_AWD          ((uint8_t)0x02)
#define ADC_FLAG_PGARDY     ((uint8_t)0x04)
#define IsAdcClrFlag(FLAG)  ((((FLAG) & (uint8_t)0xF8) == 0x00) && ((FLAG) != 0x00))
#define IsAdcGetFlag(FLAG)                                                                                             \
    (((FLAG) == ADC_FLAG_DONE) || ((FLAG) == ADC_FLAG_AWD) || ((FLAG) == ADC_FLAG_PGARDY))
/**
 * @}
 */
 
/**
 * @}
 */

/** @addtogroup ADC_thresholds
 * @{
 */
#define IsAdcValid(THRESHOLD) ((THRESHOLD) <= 0xFFF)

#define IsAdcVoiceZcrValid(THRESHOLD) ((THRESHOLD) <= 0x7F)
/**
 * @}
 */

/** @addtogroup ADC Latency wait config number
 * @{
 */

#define IsAdcLatencyWaitValue(NUMBER) ((NUMBER) <= 0x1FF)

/**
 * @}
 */

/** @addtogroup ADC_NoiseOffset
 * @{
 */

#define IsAdcNoiseOffsetValid(OFFSET) ((OFFSET) <= 0x00FFFFFF)

/** @addtogroup ADC FIR FLT COEFFICIENT
 * @{
 */

#define IsAdcFIRFLTValid(COEFFICIENT) ((COEFFICIENT) <= 0xFFFF)
/**
 * @}
 */

/**
 * @}
 */

/************************** fllowing bit seg in ex register  **********************/
/**@addtogroup ADC_channels_ex_style
 * @{
 */


/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup ADC_Exported_Functions
 * @{
 */

void ADC_DeInit(ADC_Module* ADCx);
void ADC_Enable(ADC_Module* ADCx, FunctionalState Cmd);
void ADC_EnableDMA(ADC_Module* ADCx, FunctionalState Cmd);
void ADC_EnableTS(ADC_Module* ADCx, FunctionalState Cmd);
void ADC_EnableAWD(ADC_Module* ADCx, FunctionalState Cmd);
void ADC_ConfigInt(ADC_Module* ADCx, uint16_t ADC_IT, FunctionalState Cmd);
void ADC_ConfigChannel(ADC_Module* ADCx, uint16_t Channel);
void ADC_ConfigContinuousMode(ADC_Module* ADCx, FunctionalState Cmd);
void ADC_ConfigAnalogWatchdogThresholds(ADC_Module* ADCx, uint32_t High_Threshold, uint32_t Low_Threshold);
void ADC_ConfigVoiceZcrThresholds(ADC_Module* ADCx, uint32_t High_Threshold, uint32_t Low_Threshold);
void ADC_ConfigEDThreshold(ADC_Module* ADCx, uint32_t High_Threshold, uint32_t Low_Threshold);
void ADC_SetNoiseOffset(ADC_Module* ADCx, uint32_t moffset);
uint16_t ADC_GetDat(ADC_Module* ADCx);
FlagStatus ADC_GetFlagStatus(ADC_Module* ADCx, uint8_t ADC_FLAG);
void ADC_ClearFlag(ADC_Module* ADCx, uint8_t ADC_FLAG);
void ADC_EnableZeroCross(ADC_Module* ADCx, FunctionalState Cmd);
void ADC_EnableEnergy(ADC_Module* ADCx, FunctionalState Cmd);
void ADC_EnableBypassFilter(ADC_Module* ADCx, FunctionalState Cmd);
void ADC_EnableMICBIAS(ADC_Module* ADCx, FunctionalState Cmd);
void ADC_SetMICOutputVoltage(ADC_Module* ADCx, uint32_t MIC_VoltageValue);
void ADC_EnableAUDIOPGA(ADC_Module* ADCx, FunctionalState Cmd);
void ADC_SetAUDIOAmplitude(ADC_Module* ADCx, uint32_t AMP_Value);
void ADC_SetAUDIOCurrent(ADC_Module* ADCx, uint32_t Drive_Value);
void ADC_EnablePGA(ADC_Module* ADCx, FunctionalState Cmd);
void ADC_SetPGAGain(ADC_Module* ADCx, uint32_t Gain_Value);
void ADC_SetOverSampleCounter(ADC_Module* ADCx, uint32_t Cnt);
uint32_t ADC_ConverValueToVoltage(uint16_t adc_value, uint16_t channel);
float ADC_ConverValueToTemperature(uint16_t adc_value);

#ifdef __cplusplus
}
#endif

#endif /*__N32WB03X_ADC_H__ */

/**
 * @}
 */
/**
 * @}
 */
