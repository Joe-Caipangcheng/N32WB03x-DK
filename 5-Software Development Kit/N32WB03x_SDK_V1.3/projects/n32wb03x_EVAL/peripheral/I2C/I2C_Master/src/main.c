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
 * @file main.c
 * @author Nations Firmware Team
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32wb03x.h"
#include "n32wb03x_i2c.h"
#include "main.h"
#include "stdio.h"
/** @addtogroup N32WB03X_StdPeriph_Examples
 * @{
 */

/** @addtogroup I2C_Master
 * @{
 */

#define TEST_BUFFER_SIZE  32
#define I2CT_FLAG_TIMEOUT ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT ((uint32_t)(20 * I2CT_FLAG_TIMEOUT))  ///((uint32_t)(20 * I2C_FLAG_TIMOUT))
#define I2C_MASTER_ADDR   0x30
#define I2C_SLAVE_ADDR    0xA0

uint8_t tx_buf[TEST_BUFFER_SIZE] = {0};
uint8_t rx_buf[TEST_BUFFER_SIZE] = {0};
volatile Status test_status      = FAILED;
uint32_t stsbuf[4];
Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

static __IO uint32_t I2CTimeout = I2CT_LONG_TIMEOUT;
void delay(uint32_t count)
{
    uint32_t i;
    for(i = 0; i< count; i++)
    {
    }
}
int i2c_master_init(void)
{
    I2C_InitType i2c1_master;
    GPIO_InitType i2c1_gpio;
    
    RCC_EnableAPB1PeriphClk(I2Cx_MASTER_CLK, ENABLE);
    RCC_EnableAPB2PeriphClk(I2Cx_MASTER_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);
    
    /*PB7 -- SCL; PB6 -- SDA*/
    GPIO_InitStruct(&i2c1_gpio);
    i2c1_gpio.Pin                = I2Cx_MASTER_SCL_PIN | I2Cx_MASTER_SDA_PIN;
    i2c1_gpio.GPIO_Speed         = GPIO_SPEED_HIGH; 
    i2c1_gpio.GPIO_Mode          = GPIO_MODE_AF_OD;            
    i2c1_gpio.GPIO_Alternate     = I2Cx_MASTER_GPIO_AF;
    i2c1_gpio.GPIO_Pull          = GPIO_PULL_UP;
    GPIO_InitPeripheral(I2Cx_MASTER_GPIO, &i2c1_gpio);

    I2C_DeInit(I2Cx_MASTER);
    i2c1_master.BusMode     = I2C_BUSMODE_I2C;
    i2c1_master.FmDutyCycle = I2C_FMDUTYCYCLE_1; 
    i2c1_master.OwnAddr1    = I2C_MASTER_ADDR;
    i2c1_master.AckEnable   = I2C_ACKEN;
    i2c1_master.AddrMode    = I2C_ADDR_MODE_7BIT;
    i2c1_master.ClkSpeed    = 100000;                //100k

    I2C_Init(I2Cx_MASTER, &i2c1_master);
    I2C_Enable(I2Cx_MASTER, ENABLE);
    return 0;
}

int i2c_master_send(uint8_t* data, int len)
{
    uint8_t* sendBufferPtr = data;
    I2CTimeout = I2CT_LONG_TIMEOUT;
    while (I2C_GetFlag(I2Cx_MASTER, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
            return 4;
    };

    
    I2C_ConfigAck(I2Cx_MASTER, ENABLE);

      I2C_GenerateStart(I2Cx_MASTER, ENABLE);        
    I2CTimeout = I2C_FLAG_TIMOUT;
    while (!I2C_CheckEvent(I2Cx_MASTER, I2C_EVT_MASTER_MODE_FLAG)) // EV5
    {
        if ((I2CTimeout--) == 0)
            return 5;
    };

    I2C_SendAddr7bit(I2Cx_MASTER, I2C_SLAVE_ADDR, I2C_DIRECTION_SEND);
    I2CTimeout = I2C_FLAG_TIMOUT;
    while (!I2C_CheckEvent(I2Cx_MASTER, I2C_EVT_MASTER_TXMODE_FLAG)) // EV6
    {
        if ((I2CTimeout--) == 0)
            return 6;
    };

    // send data
    while (len-- > 0)
    {
        I2C_SendData(I2Cx_MASTER, *sendBufferPtr++);
        I2CTimeout = I2C_FLAG_TIMOUT;
        while (!I2C_CheckEvent(I2Cx_MASTER, I2C_EVT_MASTER_DATA_SENDING)) // EV8
        {
            if ((I2CTimeout--) == 0)
                return 7;
        };
    };

    I2CTimeout = I2C_FLAG_TIMOUT;
    while (!I2C_CheckEvent(I2Cx_MASTER, I2C_EVT_MASTER_DATA_SENDED)) // EV8-2
    {
        if ((I2CTimeout--) == 0)
            return 8;
    };
    
     I2C_GenerateStop(I2Cx_MASTER, ENABLE);
    return 0;
}

int i2c_master_recv(uint8_t* data, int len)
{
    uint8_t* recvBufferPtr = data;
    I2CTimeout = I2CT_LONG_TIMEOUT;
    while (I2C_GetFlag(I2Cx_MASTER, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
            return 9;
    };

    I2C_ConfigAck(I2Cx_MASTER, ENABLE);

    // send start
     I2C_GenerateStart(I2Cx_MASTER, ENABLE);
    
    I2CTimeout = I2C_FLAG_TIMOUT;
    while (!I2C_CheckEvent(I2Cx_MASTER, I2C_EVT_MASTER_MODE_FLAG)) // EV5
    {
        if ((I2CTimeout--) == 0)
            return 10;
    };

    // send addr
    I2C_SendAddr7bit(I2Cx_MASTER, I2C_SLAVE_ADDR, I2C_DIRECTION_RECV);
    I2CTimeout = I2C_FLAG_TIMOUT;
    while (!I2C_CheckEvent(I2Cx_MASTER, I2C_EVT_MASTER_RXMODE_FLAG)) // EV6
    {
        if ((I2CTimeout--) == 0)
            return 6;
    };    
        
    // recv data
    while (len-- > 0)
    {
        I2CTimeout = I2CT_LONG_TIMEOUT;
        while (!I2C_CheckEvent(I2Cx_MASTER, I2C_EVT_MASTER_DATA_RECVD_FLAG)) // EV7
        {
           if ((I2CTimeout--) == 0)
             return 14;
        };
        
        if (len == 1)
        {
            I2C_ConfigAck(I2C1, DISABLE);
            I2C_GenerateStop(I2C1, ENABLE);
        }
        
        if (len == 0)
        {
            //I2C_GenerateStop(I2C1, ENABLE);
        }
        *recvBufferPtr++ = I2C_RecvData(I2C1);
                
    };
    return 0;
}

/**
 * @brief   Main program
 */
int main(void)
{
    uint16_t i = 0;
    
    log_init();
    log_info("\n this is a i2c master demo\r\n");
    /* Initialize the I2C EEPROM driver ----------------------------------------*/
    i2c_master_init();
    
    /* Fill the buffer to send */
    for (i = 0; i < TEST_BUFFER_SIZE; i++)
    {
        tx_buf[i] = i;
    }
    
    /* First write in the memory followed by a read of the written data --------*/
    /* Write data*/
    i2c_master_send(tx_buf, TEST_BUFFER_SIZE);
    
    /* Read data */
    i2c_master_recv(rx_buf, TEST_BUFFER_SIZE);

    /* Check if the data written to the memory is read correctly */
    test_status = Buffercmp(tx_buf, rx_buf, TEST_BUFFER_SIZE);

    if (test_status == PASSED)
    {
        log_info("write and read data the same, i2c master test pass\r\n");
    }
    else
    {
        log_info("write and read data are different, i2c master test fail\r\n");
    }
    log_buff(rx_buf, TEST_BUFFER_SIZE);
    
    /* test_status = PASSED, if the write and read data are the same  */
    /* test_status = FAILED, if the write and read data are different */
    while (1)
    {
    }
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer, pBuffer1: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer identical to pBuffer1
 *         FAILED: pBuffer differs from pBuffer1
 */
Status Buffercmp(uint8_t* pBuffer, uint8_t* pBuffer1, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer != *pBuffer1)
        {
            return FAILED;
        }

        pBuffer++;
        pBuffer1++;
    }

    return PASSED;
}

void log_init(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;
    
    GPIO_InitStruct(&GPIO_InitStructure);
    
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | LOG_PERIPH_GPIO, ENABLE);
    
    RCC_EnableAPB1PeriphClk(LOG_PERIPH, ENABLE);
    
    GPIO_InitStructure.Pin            = LOG_TX_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = LOG_GPIO_AF;
    GPIO_InitPeripheral(LOG_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.Pin             = LOG_RX_PIN;
    GPIO_InitStructure.GPIO_Alternate  = LOG_GPIO_AF;
    GPIO_InitPeripheral(LOG_GPIO, &GPIO_InitStructure);

    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    // init uart
    USART_Init(LOG_USARTx, &USART_InitStructure);

    // enable uart
    USART_Enable(LOG_USARTx, ENABLE);
}

void log_buff(uint8_t *data, int len)
{
    int i = 0;
    for(i=0; i<len; i++)
    {
        printf("0x%02x, ", data[i]);
        
        if((i != 0) && ((i+1) % 16 == 0))
        {
            printf("\n");
        }
    }
    
    printf("\n");
}

int fputc(int ch, FILE* f)
{
    USART_SendData(LOG_USARTx, (uint8_t)ch);
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(LOG_USARTx, USART_FLAG_TXC) == RESET)
    {
    }
    return ch;
}

void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    log_error("assertion failed: `%s` at %s:%d", expr, file, line);
    while (1)
    {
    }
}

/**
 * @}
 */
