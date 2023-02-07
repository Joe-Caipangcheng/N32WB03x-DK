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
 * @file ns_dfu_ble.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

 /** @addtogroup 
 * @{
 */
#ifndef __NS_DFU_BLE_H__
#define __NS_DFU_BLE_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
typedef struct{
    uint32_t start_address;
    uint32_t size;
    uint32_t crc;
    uint32_t version;
}Dfu_setting_bank_t;

typedef struct{

    uint32_t crc;
    Dfu_setting_bank_t app1;
    Dfu_setting_bank_t app2;
    Dfu_setting_bank_t image_update;
    uint8_t signature[64];
}Dfu_setting_t;
/* Public define ------------------------------------------------------------*/  
/* Public constants ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void ns_dfu_ble_handler_cc(uint8_t const *input, uint8_t input_len, uint8_t *output, uint8_t *output_len);
void ns_dfu_ble_handler_rc(uint8_t const  *input, uint32_t input_len);
void ns_dfu_ble_handler_conn_param_update(uint8_t status);
void ns_dfu_ble_handler_mtu_update(uint8_t status);

#ifdef __cplusplus
}
#endif



#endif //__NS_DFU_BLE_H__
