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
 * @file n32wb03x_qflash.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __N32WB03X_QFLASH_H__
#define __N32WB03X_QFLASH_H__

#ifdef __cplusplus
extern "C" {
#endif
    
#include "n32wb03x.h"
    

#define Qflash_Erase_Sector_Raw  Qflash_Erase_Sector 
#define Qflash_Write_Raw  Qflash_Write
    
#define FLASH_PAGE_SIZE       0x100
#define Sector_Offset         0x1000


typedef enum{
    FlashOperationSuccess,
    FlashWriteRegFailed,
    FlashTimeOut,
    FlashIsBusy,
    FlashQuadNotEnable,
    FlashAddressInvalid,
}ReturnMsg; 
    
    
    
    
    
    
void Qflash_Init(void);
void Qflash_Erase_Sector(uint32_t address);
//void Qflash_Erase_Sector_Raw(uint32_t address);
void Qflash_Write(uint32_t address, uint8_t* p_data, uint32_t len);
//void Qflash_Write_Raw(uint32_t address, uint8_t* p_data, uint32_t len);    
void Qflash_Read(uint32_t address, uint8_t* p_data, uint32_t len);
    
    
    
#ifdef __cplusplus
}
#endif

#endif /* __N32WB03X_QFLASH_H__ */
/**
 * @}
 */    

