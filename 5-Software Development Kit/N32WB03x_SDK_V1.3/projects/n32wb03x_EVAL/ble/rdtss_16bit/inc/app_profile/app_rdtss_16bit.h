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
 * @file app_rdtss_16bit.h
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#ifndef APP_RDTSS_16BIT_H_
#define APP_RDTSS_16BIT_H_

/**
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief Battery Application Module entry point
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"     // SW configuration

#if (BLE_RDTSS_16BIT_SERVER)

/// Manufacturer Name Value
#define APP_RDTSS_16BIT_MANUFACTURER_NAME       ("Nations")
#define APP_RDTSS_16BIT_MANUFACTURER_NAME_LEN   (7)



#define ATT_SERVICE_AM_SPEED_16          0xfffa           /*!< Service UUID */
#define ATT_CHAR_AM_SPEED_WRITE_16       0xfffb           /*!< Characteristic value UUID */
#define ATT_CHAR_AM_SPEED_NTF_16         0xfffc           /*!< Characteristic value UUID */

/// rdtss 16bit uuid Service Attributes Indexes
enum
{
    RDTSS_16BIT_IDX_SVC,
    
    RDTSS_16BIT_IDX_WRITE_CHAR,
    RDTSS_16BIT_IDX_WRITE_VAL,
    RDTSS_16BIT_IDX_WRITE_CFG,
    
    RDTSS_16BIT_IDX_NTF_CHAR,
    RDTSS_16BIT_IDX_NTF_VAL,
    RDTSS_16BIT_IDX_NTF_CFG,
    
    RDTSS_16BIT_IDX_NB,
};

/**
 * @brief Initialize Device Information Service Application
 **/
void app_rdtss_16bit_init(void);

/**
 * @brief Add a Device Information Service instance in the DB
 **/
void app_rdtss_16bit_add_rdtss_16bit(void);

void rdtss_16bit_send_notify(uint8_t *data, uint16_t length);


#endif //BLE_RDTSS_16BIT_SERVER

/// @} APP

#endif // APP_RDTSS_16BIT_H_
