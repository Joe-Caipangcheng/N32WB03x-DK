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
 * @file n32wb03x_pwr.h
 * @author Nations Firmware Team
 * @version v1.1.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __N32WB03X_PWR_H__
#define __N32WB03X_PWR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32wb03x.h"

/** @addtogroup N32WB03X_StdPeriph_Driver
 * @{
 */

/**
 * @}
 */

/** @defgroup SLEEP_mode_entry 
  * @{
  */
#define PWR_IDLEENTRY_WFI         ((uint8_t)0x01)
#define PWR_IDLEENTRY_WFE         ((uint8_t)0x02)

 
/**
  * @}
  */


/** @defgroup SLEEP_mode_entry 
  * @{
  */

#define PWR_SLEEPENTRY_WFI        ((uint8_t)0x01)
#define PWR_SLEEPENTRY_WFE        ((uint8_t)0x02)

/** @defgroup Powerdown_mode_entry 
  * @{
  */

#define PWR_PDENTRY_WFI         ((uint8_t)0x01)
#define PWR_PDENTRY_WFE         ((uint8_t)0x02)


/** @addtogroup PWR_Exported_Functions
 * @{
 */

void PWR_DeInit(void);
void PWR_EnterIDLEMode(uint8_t IDLEONEXIT, uint8_t PWR_IdleEntry);
void PWR_EnterSLEEPMode(uint8_t PWR_SleepEntry);
void PWR_EnterPDMode(uint8_t PWR_PDEntry);
void PWR_EnterStandbyMode(uint8_t PWR_StanbyEntry);
#ifdef __cplusplus
}
#endif

#endif /* __N32WB03X_PWR_H__ */
       /**
        * @}
        */

/**
 * @}
 */

/**
 * @}
 */
