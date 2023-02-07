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
 * @file main.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#include "n32wb03x.h"
#include "n32wb03x_conf.h"

#define I2S_SLAVE                       SPI1
#define I2S_SLAVE_CLK                   RCC_APB2_PERIPH_SPI1
#define I2S_SLAVE_PIN_CK                GPIO_PIN_1
#define I2S_SLAVE_PIN_SD                GPIO_PIN_2
#define I2S_SLAVE_PIN_WS                GPIO_PIN_0
#define I2S_SLAVE_PIN_PORT              GPIOA
#define I2S_SLAVE_PIN_AF                GPIO_AF1_SPI1

#define I2S_SLAVE_DMA            DMA
#define I2S_SLAVE_DMA_CLK        RCC_AHB_PERIPH_DMA
#define I2S_SLAVE_Rx_DMA_Channel DMA_CH2
#define I2S_SLAVE_Rx_DMA_FLAG    DMA_FLAG_TC2
#define I2S_SLAVE_Tx_DMA_Channel DMA_CH3
#define I2S_SLAVE_Tx_DMA_FLAG    DMA_FLAG_TC3


#endif /* __MAIN_H__ */
