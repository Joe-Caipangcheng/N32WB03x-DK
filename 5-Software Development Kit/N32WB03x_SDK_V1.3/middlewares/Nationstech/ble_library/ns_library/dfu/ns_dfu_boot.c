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
 * @file ns_dfu_boot.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "ns_dfu_boot.h"
#include "n32wb03x.h"
#include "dfu_crc.h"
#include "ns_error.h"
/* Private typedef -----------------------------------------------------------*/
typedef void (*func_ptr_t)(void);
/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
NS_Bootsetting_t ns_bootsetting __attribute__((at(NS_BOOTSETTING_START_ADDRESS))) __attribute__((used));
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
 * @brief Inter program jump function.
 * @param[in] address program flash address.
 * @return none, function will not return
 */
void ns_dfu_boot_jump(uint32_t address)
{
    
    uint32_t JumpAddress;
    func_ptr_t JumpToApplication;
    JumpAddress = *(__IO uint32_t *)(address + 4);
    JumpToApplication = (func_ptr_t)JumpAddress;
    __set_MSP(*(__IO uint32_t *)address);
    JumpToApplication();

}


/**
 * @brief Write force usart image update variable in bootsetting.
 * @param[in] none.
 * @return none
 */
bool ns_dfu_boot_force_usart_dfu(void)
{
    NS_Bootsetting_t ns_bootsetting_tmp;
    memcpy(&ns_bootsetting_tmp,&ns_bootsetting,sizeof(NS_Bootsetting_t));
    ns_bootsetting_tmp.master_boot_force_update = NS_BOOTSETTING_MASTER_BOOT_FORCE_UPDATE_YES;
    ns_bootsetting_tmp.crc = dfu_crc32((uint8_t *)&ns_bootsetting_tmp.crc + 4, sizeof(NS_Bootsetting_t) - 4);
    Qflash_Erase_Sector(NS_BOOTSETTING_START_ADDRESS);
    Qflash_Write(NS_BOOTSETTING_START_ADDRESS, (uint8_t *)&ns_bootsetting_tmp, sizeof(NS_Bootsetting_t));            
    
    if(ns_bootsetting_tmp.crc == dfu_crc32((uint8_t *)((uint32_t *)(NS_BOOTSETTING_START_ADDRESS + 4)), sizeof(NS_Bootsetting_t) - 4))
    {
        NVIC_SystemReset();
        return ERROR_SUCCESS;
    }
    else
    {
        return ERROR_INTERNAL;
    }
}








