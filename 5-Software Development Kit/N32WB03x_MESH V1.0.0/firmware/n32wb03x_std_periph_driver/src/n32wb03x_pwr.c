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
 * @file n32wb03x_pwr.c
 * @author Nations Firmware Team 
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32wb03x_pwr.h"

/** @addtogroup N32WB03X_StdPeriph_Driver
 * @{
 */

/**
 * @}
 */

/** @addtogroup PWR_Private_Macros
 * @{
 */

/**
 * @}
 */

/** @addtogroup PWR_Private_Variables
 * @{
 */

/**
 * @}
 */

/** @addtogroup PWR_Private_FunctionPrototypes
 * @{
 */

/**
 * @}
 */

/** @addtogroup PWR_Private_Functions
 * @{
 */
 
 
/**
 * @brief  Deinitializes the PWR peripheral registers to their default reset values.
 */
void PWR_DeInit(void)
{
    RCC_EnableAPB1PeriphReset(RCC_APB1_PERIPH_PWR, ENABLE);
    RCC_EnableAPB1PeriphReset(RCC_APB1_PERIPH_PWR, DISABLE);
}

/**
  * @brief  Enters IDLE mode.
  * @param  SLEEPONEXIT: specifies the SLEEPONEXIT state in IDLE mode.
  *   This parameter can be one of the following values:
  *     @arg 0: SLEEP mode with SLEEPONEXIT disable
  *     @arg 1: SLEEP mode with SLEEPONEXIT enable
  * @param  PWR_STOPEntry: specifies if SLEEP mode in entered with WFI or WFE instruction.
  *   This parameter can be one of the following values:
  *     @arg PWR_IDLEENTRY_WFI: enter IDLE mode with WFI instruction
  *     @arg PWR_IDLEENTRY_WFE: enter IDLE mode with WFE instruction
  * @retval None
  */
void PWR_EnterIDLEMode(uint8_t IDLEONEXIT, uint8_t PWR_IDLEEntry)
{  
  
    /* CLEAR SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP); 

      /* Select SLEEPONEXIT mode entry --------------------------------------------------*/
    if(IDLEONEXIT == 1)
    {   
        /* the MCU enters Sleep mode as soon as it exits the lowest priority ISR */
        SCB->SCR |= SCB_SCR_SLEEPONEXIT;
    }
    else if(IDLEONEXIT == 0)
    {
        /* Sleep-now */
        SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPONEXIT);
    }       
      
    /* Select SLEEP mode entry --------------------------------------------------*/
    if(PWR_IDLEEntry == PWR_IDLEENTRY_WFI)
    {   
        /* Request Wait For Interrupt */
        __WFI();        
    }
    else
    {
        /* Request Wait For Event */
        __SEV();
        __WFE();
        __WFE();
    }    
}

/**
  * @brief  Enters SLEEP mode.
  * @param  PWR_SLEEPEntry: specifies if SLEEP mode in entered with WFI or WFE instruction.
  *   This parameter can be one of the following values:
  *     @arg PWR_SLEEPENTRY_WFI: enter SLEEP mode with WFI instruction
  *     @arg PWR_SLEEPENTRY_WFE: enter SLEEP mode with WFE instruction
  * @retval None
  */
void PWR_EnterSLEEPMode(uint8_t PWR_SleepEntry)
{
    uint32_t tmpreg = 0;
    /* Check the parameters */


    *(uint32_t *)(BLE_BASE + 0x30) = 0x07;
    while(PWR->CR1&PWR_CR1_OSC_EN);
      
    /* Select the regulator state in SLEEP mode ---------------------------------*/
    tmpreg = PWR->CR1;
    /* Clear PDDS and FLPDS bits */
    tmpreg &= ~PWR_CR1_MODE_SEL;
    /* Set FLPDS bit according to PWR_Regulator value */    
    tmpreg |= (PWR_CR1_MODE_SLEEP |PWR_CR1_MODE_EN);
    /* Store the new value */
    PWR->CR1 |= tmpreg;
    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP;

    /* Select SLEEP mode entry --------------------------------------------------*/
    if(PWR_SleepEntry == PWR_SLEEPENTRY_WFI)
    {   
        /* Request Wait For Interrupt */
        __WFI();
    }
    else
    {
        /* Request Wait For Event */
        __SEV();
        __WFE();
        __WFE();
    }

    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP);  
}

 /**
  * @brief  Enters PD mode.
  * @param  PWR_PDEntry: specifies if PD mode in entered with WFI or WFE instruction.
  *   This parameter can be one of the following values:
  *     @arg PWR_PDENTRY_WFI: enter PD mode with WFI instruction
  *     @arg PWR_PDENTRY_WFE: enter PD mode with WFE instruction
  * @retval None
  */
void PWR_EnterPDMode(uint8_t PWR_PDEntry)
{
    uint32_t tmpreg = 0; 
    /* Select the regulator state in SHUTDOWN mode ---------------------------------*/
    tmpreg = PWR->CR1;
    /* Clear PDDS  bits */
    tmpreg &= ~PWR_CR1_MODE_SEL;
    /* Set FLPDS bit according to PWR_Regulator value */    
    tmpreg |= PWR_CR1_MODE_PD|PWR_CR1_MODE_EN;
    /* Store the new value */
    PWR->CR1 |= tmpreg;    

    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP;
    /* This option is used to ensure that store operations are completed */
    #if defined ( __CC_ARM   )
    __force_stores();
    #endif
    /* Select SHUTDOWN mode entry --------------------------------------------------*/
    if(PWR_PDEntry == PWR_PDENTRY_WFI)
    {   
        /* Request Wait For Interrupt */
        __WFI();
    }
    else
    {
        /* Request Wait For Event */
        __SEV();
        __WFE();
        __WFE();
    }
  
}

void PWR_EnterStandbyMode(uint8_t PWR_StandbyEntry)
{
    uint32_t tmpreg = 0;
    /* Check the parameters */
    *(uint32_t *)(BLE_BASE + 0x30) = 0x07;
    while(PWR->CR1&PWR_CR1_OSC_EN);
      
    /* Select the regulator state in SLEEP mode ---------------------------------*/
    tmpreg = PWR->CR1;
    /* Clear PDDS and FLPDS bits */
    tmpreg &= ~PWR_CR1_MODE_SEL;
    /* Set FLPDS bit according to PWR_Regulator value */    
    tmpreg |= (PWR_CR1_MODE_PD |PWR_CR1_MODE_EN);
    /* Store the new value */
    PWR->CR1 = tmpreg;
    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP;

    /* Select SLEEP mode entry --------------------------------------------------*/
    if(PWR_StandbyEntry == PWR_SLEEPENTRY_WFI)
    {   
      /* Request Wait For Interrupt */
      __WFI();
    }
    else
    {
        /* Request Wait For Event */
        __SEV();
        __WFE();
        __WFE();
    }

    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP);  
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
