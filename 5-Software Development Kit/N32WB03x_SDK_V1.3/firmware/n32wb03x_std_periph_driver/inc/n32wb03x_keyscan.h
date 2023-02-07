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
 * @file n32wb03x_keyscan.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __N32WB03X__KEYSCAN_H__
#define __N32WB03X__KEYSCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32wb03x.h"

/** @addtogroup n32wb03x_StdPeriph_Driver
 * @{
 */

/** @addtogroup KEYSCAN
 * @{
 */

/** @addtogroup KEYSCAN_Exported_Types
 * @{
 */

/**
 * @brief  KEYSCAN Init Structure definition
 */
typedef struct
{
    uint32_t Mask; 
    uint16_t Mode; 
    uint16_t Wts; 
    uint16_t Dts; 
    uint16_t Int_en; 
    
} KEYSCAN_InitType;


/**
 * @brief  KEYSCAN NUM enumeration
 */
typedef enum 
{
    KEY_104,
    KEY_44,
    KEY_65,
}KEY_NUM;

/**
 * @brief  KEYSCAN NUM enumeration
 */
typedef enum 
{
    MODE_FIXED_INTV,
    MODE_SW_TRIG,
    MODE_PRESS_TRIG,
}KEY_MODE;

/**
 * @brief  KEYSCAN debounce time enumeration
 */
typedef enum 
{
    DTS_10MS,
    DTS_20MS,
    DTS_40MS,
    DTS_80MS,
    DTS_160MS,
    DTS_320MS,
    DTS_640MS,
    DTS_640MS_2,
}KEY_DTS;

/**
 * @brief  KEYSCAN scan wait time enumeration
 */
typedef enum 
{
    WTS_0MS,
    WTS_32MS,
    WTS_64MS,
    WTS_96MS,
    WTS_128MS,
    WTS_160MS,
    WTS_192MS,
    WTS_224MS,
}KEY_WTS;

/**
 * @brief  KEYSCAN INT trigger enumeration
 */
typedef enum 
{
    INT_DIS,
    INT_EN,
}KEY_INT_EN;
    
/**
 * @}
 */

/** @addtogroup KEYSCAN_Exported_Constants
 * @{
 */

/** @addtogroup KEYSCAN_Prescaler
 * @{
 */

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup KEYSCAN_Exported_Macros
 * @{
 */

#define KEYSCAN_EN_POS            0
#define KEY_MODE_POS              2
#define KEY_DTS_POS               4
#define KEY_WTS_POS               7
#define KEY_MASK_POS              10
#define KEY_SW_START_POS          12
#define KEY_INFO_CLR_POS          13
#define KEY_INT_EN_POS            22
#define KEY_IRP_POS               23

/**
 * @}
 */

/** @addtogroup KEYSCAN_Exported_Functions
 * @{
 */
void KEYSCAN_Init(KEYSCAN_InitType* KEYSCAN_InitStruct);
void KEYSCAN_Enable(FunctionalState Cmd);
void KEYSCAN_InfoClear(void);
void KEYSCAN_SoftwareStartScan(void);
void KEYSCAN_ClearInterrupt(void);
void KEYSCAN_ReadKeyData(uint32_t *key_data);
#ifdef __cplusplus
}
#endif

#endif /* __N32WB03X__KEYSCAN_H */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
