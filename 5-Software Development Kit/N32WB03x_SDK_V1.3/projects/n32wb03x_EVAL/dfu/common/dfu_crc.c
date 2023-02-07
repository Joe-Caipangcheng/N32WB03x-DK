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
 * @file dfu_crc.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "dfu_crc.h"
#include "n32wb03x_crc.h"

/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/  

#define LITTLE_TO_BIG(u32)    \
(uint32_t) ((u32 & 0x000000FF) << 24) | \
(uint32_t) ((u32 & 0x0000FF00) << 8) | \
(uint32_t) ((u32 & 0x00FF0000) >> 8) | \
(uint32_t) ((u32 & 0xFF000000) >> 24)

/**
 * @brief Calculate crc32 for given data, using hardware crc32.
 * @param[in] p_data raw data.
 * @param[in] len raw data len. 
 * @return crc32 result.
 */
uint32_t dfu_crc32(uint8_t * p_data, uint32_t len)
{
    if(len%4 > 0)return 0;
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_CRC, ENABLE);
    CRC32_ResetCrc();
    uint32_t index = 0;
    uint32_t *u32p_data = (uint32_t *)p_data;
    for (index = 0; index < len/4; index++)
    {
            CRC->CRC32DAT = LITTLE_TO_BIG(u32p_data[index]);
    }
    return (CRC->CRC32DAT);
}


/**
 * @}
 */



