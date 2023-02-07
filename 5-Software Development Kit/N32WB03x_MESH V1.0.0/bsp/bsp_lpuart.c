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
 * @file bsp_lpuart.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */


#include "bsp_lpuart.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>

#define LPUARTx             LPUART1
#define LPUARTx_CLK         RCC_LPUART1CLK
#define LPUARTx_GPIO        GPIOB
#define LPUARTx_GPIO_CLK    RCC_APB2_PERIPH_GPIOB
#define LPUARTx_TxPin       GPIO_PIN_12
#define LPUARTx_Tx_GPIO_AF  GPIO_AF2_LPUART1


/**
 * @brief LPUart RCC Configuration.
 * @param[in] clock src.
 * @return none.
 */
void RCC_Configuration(uint32_t LPUART_CLK_SRC)
{
    switch(LPUART_CLK_SRC)
    {
        case RCC_LPUARTCLK_SRC_LSI_LSE:
        {  
            /* Specifies the LPUARTx clock source, LSE selected as LPUARTx clock */
            RCC_ConfigLpuartClk(LPUARTx_CLK, RCC_LPUARTCLK_SRC_LSI_LSE);
        }
        break;
        default:
        {
            /* Specifies the LPUARTx clock source, APB1 selected as LPUARTx clock */
            RCC_ConfigLpuartClk(LPUARTx_CLK, RCC_LPUARTCLK_SRC_APB1);
        }
        break;
    }    
    
    /* Enable LPUARTx Clock */
    RCC_EnableLpuartClk(ENABLE);    
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(LPUARTx_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);
    
    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);  
    /* Configure LPUARTx Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = LPUARTx_TxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = LPUARTx_Tx_GPIO_AF;
    GPIO_InitPeripheral(LPUARTx_GPIO, &GPIO_InitStructure);

    
}



/**
 * @brief LPUart init.
 * @param[in] none.
 * @return none.
 */
void bsp_lpuart_init(void)
{
    LPUART_InitType LPUART_InitStructure;
    /* System Clocks Configuration */
#if 0    
    RCC_Configuration(RCC_LPUARTCLK_SRC_LSI_LSE);
#else
    RCC_Configuration(RCC_LPUARTCLK_SRC_APB1);
#endif
    
    /* Configure the GPIO ports */
    GPIO_Configuration();

    LPUART_DeInit(LPUARTx);
    /* LPUARTx configuration ------------------------------------------------------*/
    LPUART_InitStructure.BaudRate            = 1000000;
    LPUART_InitStructure.Parity              = LPUART_PE_NO;
    LPUART_InitStructure.RtsThreshold        = LPUART_RTSTH_FIFOFU;
    LPUART_InitStructure.HardwareFlowControl = LPUART_HFCTRL_NONE;
    LPUART_InitStructure.Mode                = LPUART_MODE_TX;
    /* Configure LPUARTx */
    LPUART_Init(LPUARTx, &LPUART_InitStructure);


}


/**
 * @brief Send one byte over uart.
 * @param[in] char to send.
 * @return none.
 */
static __inline void bsp_lpuart_send_byte(uint8_t ch)
{
    LPUART1->DAT = ch;
    while((LPUART1->STS & (1<<1)) == 0);
    LPUART1->STS |= (1<<1);
    while((LPUART1->STS & (1<<1)) != 0);
}
/**
 * @brief Send string over uart.
 * @param[in] byte array to send.
 * @param[in] length of byte array.
 * @return none.
 */
uint32_t bsp_lpuart_send(uint8_t *p_data, uint32_t len)
{
    for(uint32_t i=0;i<len;i++)
    {
        bsp_lpuart_send_byte(p_data[i]);
    }
    return len;
}


/* retarget the C library printf function to the LPUARTx */
int fputc(int ch, FILE* f)
{
    bsp_lpuart_send_byte(ch);
    return (ch);
}


#define RTT_RETRY_COUNTER 10
#define HEXDUMP_BYTES_PER_LINE               16
#define HEXDUMP_HEXBYTE_AREA                 3 // Two bytes for hexbyte and space to separate
#define HEX_BUFFER_SIZE 512

/* byte to hex format string */
static inline void byte2hex(const uint8_t c, char * p_out)
{
	uint8_t  nibble;
	uint32_t i = 2;
	while (i-- != 0)
	{
			nibble       = (c >> (4 * i)) & 0x0F;
			p_out[1 - i] = (nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble);
	}
}

/* Send variable lenght to serial port */
static inline void serial_tx(uint8_t * p_buf, uint32_t len)
{
	uint32_t idx    = 0;
	uint32_t length = len;
	uint32_t processed;	
	do
	{
			processed = bsp_lpuart_send((uint8_t *)&p_buf[idx], length);
			idx += processed;
			length -= processed;
	} while (length);	

}
/**
 * @brief Print hex format to serial port.
 * @param[in] byte array to send.
 * @param[in] length of byte array.
 * @return none.
 */
uint32_t log_hexdump(const uint8_t *p_data, uint16_t len)
{
    if(len <= 0)return 0;
    char     str[HEX_BUFFER_SIZE];
    char   * p_hex_part;
    char   * p_char_part;
    uint8_t  c;
    uint32_t byte_in_line;
    uint32_t buffer_len    = 0;
    uint32_t byte_cnt      = 0;
    uint32_t length        = len;
    do
    {
        uint32_t hex_part_offset  = buffer_len;
        uint32_t char_part_offset = hex_part_offset + HEXDUMP_BYTES_PER_LINE*HEXDUMP_HEXBYTE_AREA + 1;
        p_hex_part  = &str[hex_part_offset];
        p_char_part = &str[char_part_offset];
        for (byte_in_line = 0; byte_in_line < HEXDUMP_BYTES_PER_LINE; byte_in_line++)
        {
            if (byte_cnt >= length)
            {
                *p_hex_part++  = ' ';
                *p_hex_part++  = ' ';
                *p_hex_part++  = ' ';
                *p_char_part++ = ' ';
            }
            else
            {

                c = p_data[byte_cnt];
                byte2hex(c, p_hex_part);
                p_hex_part    += 2; 
                *p_hex_part++  = ' ';
                *p_char_part++ = ' ';
                byte_cnt++;
            }
        }
        *p_char_part++ = '\r';
        *p_char_part++ = '\n';
        *p_hex_part++  = ' ';
        buffer_len    += 0 +
                         (HEXDUMP_BYTES_PER_LINE * HEXDUMP_HEXBYTE_AREA + 1) + 
                         HEXDUMP_BYTES_PER_LINE +                              
                         2;                                                    


        serial_tx((uint8_t *)str, buffer_len);

        buffer_len = 0;
    }
    while (byte_cnt < length);	
	return SUCCESS;
}




