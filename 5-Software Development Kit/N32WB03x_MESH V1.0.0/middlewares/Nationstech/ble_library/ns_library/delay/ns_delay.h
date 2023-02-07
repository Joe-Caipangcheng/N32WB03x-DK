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
 * @file ns_delay.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __NS_DELAY_H__
#define __NS_DELAY_H__

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h> // standard boolean definitions
#include <stdint.h>  // standard integer functions
/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/ 
/* Public constants ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/

/**
 * @brief  delay in cycles which is running 4 asm instructions in (10/110) us 
 * @param  ui32Cycles delay time in (10/110) us 
 * @return 
 * @note   
 */
void delay_cycles(uint32_t ui32Cycles);

/**
 * @brief  delay in 10us
 * @param  val delay time in 10us 
 * @return 
 * @note   
 */
void delay_n_10us(uint32_t val);

/**
 * @brief  delay in 100us
 * @param  val delay time in 100us 
 * @return 
 * @note   
 */
void delay_n_100us(uint32_t val);

/**
 * @brief  delay in ms
 * @param  val delay time in ms 
 * @return 
 * @note   
 */
void delay_n_ms(uint32_t val);

/**
 * @brief  delay in us
 * @param  val delay time in us 
 * @return 
 * @note   
 */
void delay_n_us(uint32_t val);


#endif //__NS_DELAY_H__
