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
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32wb03x.h"
#include "n32wb03x_i2c.h"
#include "stdio.h"
#include "main.h"
/** @addtogroup N32WB03X_StdPeriph_Examples
 * @{
 */

/** @addtogroup I2C_Slave_Int
 * @{
 */



//#define I2C_SLAVE_LOW_LEVEL
#define TEST_BUFFER_SIZE  100
#define I2CT_FLAG_TIMEOUT ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))
#define I2C_MASTER_ADDR   0x30
#define I2C_SLAVE_ADDR    0xA0

uint8_t data_buf[TEST_BUFFER_SIZE] = {0};
static __IO uint32_t I2CTimeout;
uint8_t flag_slave_recv_finish = 0;
uint8_t flag_slave_send_finish = 0;
static uint8_t rxDataNum = 0;
static __IO uint32_t I2CTimeout = I2CT_LONG_TIMEOUT;

Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void Memset(void* s, uint8_t c, uint32_t count);
void CommTimeOut_CallBack(ErrCode_t errcode);



/**
 * @brief  i2c slave Interrupt configuration
 * @param ch I2C channel
 */
void NVIC_ConfigurationSlave(void)
{
    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel         = I2Cx_SLAVE_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  i2c slave init
 * @return 0:init finish
 */
int i2c_slave_init(void)
{
    I2C_InitType i2c1_master;
    GPIO_InitType i2c1_gpio;
    
    RCC_EnableAPB1PeriphClk(I2Cx_SLAVE_CLK, ENABLE);
    RCC_EnableAPB2PeriphClk(I2Cx_SLAVE_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);

    GPIO_InitStruct(&i2c1_gpio);
    /*PB7 -- SCL; PB6 -- SDA*/
    i2c1_gpio.Pin                = I2Cx_SLAVE_SDA_PIN | I2Cx_SLAVE_SCL_PIN;
    i2c1_gpio.GPIO_Speed         = GPIO_SPEED_HIGH;
    i2c1_gpio.GPIO_Mode          = GPIO_MODE_AF_OD;
    i2c1_gpio.GPIO_Alternate     = I2Cx_SLAVE_GPIO_AF;    
    i2c1_gpio.GPIO_Pull          = GPIO_PULL_UP;        
    GPIO_InitPeripheral(I2Cx_SLAVE_GPIO, &i2c1_gpio);
    
    I2C_DeInit(I2Cx_SLAVE);
    i2c1_master.BusMode     = I2C_BUSMODE_I2C;
    i2c1_master.FmDutyCycle = I2C_FMDUTYCYCLE_1;
    i2c1_master.OwnAddr1    = I2C_SLAVE_ADDR;
    i2c1_master.AckEnable   = I2C_ACKEN;
    i2c1_master.AddrMode    = I2C_ADDR_MODE_7BIT;
    i2c1_master.ClkSpeed    = 100000;

    I2C_Init(I2Cx_SLAVE, &i2c1_master);
    // int enable
    I2C_ConfigInt(I2Cx_SLAVE, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, ENABLE);
    NVIC_ConfigurationSlave();

    I2C_Enable(I2Cx_SLAVE, ENABLE);
    return 0;
}

/**
 * @brief   Main program
 */
int main(void)
{
    log_init();
    log_info("this is a i2c slave int demo\r\n");

    /* Initialize the I2C slave driver ----------------------------------------*/
    i2c_slave_init();
    
    // recive data
    Memset(data_buf, (uint8_t)0, TEST_BUFFER_SIZE); // clear buf, ready to recv data
    I2CTimeout = I2CT_LONG_TIMEOUT * 1000;
    while (flag_slave_recv_finish == 0) // wait for recv data finish
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(SLAVE_UNKNOW);
        }
    }
    
    log_info("recv finish, recv len = %d\r\n", rxDataNum);
    flag_slave_recv_finish = 0;
    
    // send data
    I2CTimeout = I2CT_LONG_TIMEOUT * 1000;
    while (flag_slave_send_finish == 0) // wait for send data finish
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(SLAVE_UNKNOW);
        }
            
    }
    flag_slave_send_finish = 0;    
    log_info("tx finish,tx len = %d\r\n", rxDataNum-1);
    log_buff(data_buf, rxDataNum-1);
    
    while (1)
    {
 
    }
}

/**
 * @brief  i2c slave Interrupt service function
 */
void I2C1_IRQHandler(void)
{
    uint8_t timeout_flag = 0;
    uint32_t last_event;

    last_event = I2C_GetLastEvent(I2C1);
    if ((last_event & 0x00010000) != 0x00010000) // MSMODE = 0:I2C slave mode
    {
        switch (last_event)
        {
            case I2C_EVT_SLAVE_RECV_ADDR_MATCHED: //0x00020002.EV1 Rx addr matched
                rxDataNum = 0;
                break;
            
            case I2C_EVT_SLAVE_SEND_ADDR_MATCHED: //0x00060082.EV1 Tx addr matched
                rxDataNum = 0;
                break;
            
            case I2C_EVT_SLAVE_DATA_SENDING:  //0x00060080. EV3 Sending data
                I2C1->DAT = data_buf[rxDataNum++];
                break;
                
            case I2C_EVT_SLAVE_DATA_SENDED:
                I2C1->DAT = data_buf[rxDataNum++];
                break;
            
            // SlaveReceiver
            case I2C_EVT_SLAVE_DATA_RECVD:         //0x00020040.EV2 one byte recved
                data_buf[rxDataNum++] = I2C1->DAT;
                break;
        
            case I2C_EVT_SLAVE_STOP_RECVD: // 0x00000010 EV4
                I2C_Enable(I2C1, ENABLE);   
                if(rxDataNum != 0)
                {
                    flag_slave_recv_finish = 1;     // The STOPF bit is not set after a NACK reception
                }
                break;
            
            case I2C_EVT_SLAVE_ACK_MISS:            
                I2C_ClrFlag(I2C1, I2C_FLAG_ACKFAIL);
                if(rxDataNum != 0)                        //slave send the last data and recv NACK 
                {
                    flag_slave_send_finish = 1;
                }    
                else                                    //not the last data recv nack, send fail
                {
                
                }
                break;
            
            default:
                //printf("last_event:0x%x\r\n", last_event);
                I2C_Enable(I2C1, ENABLE);
                timeout_flag = 1;
                break;
        }
    }
    
    if (timeout_flag)
    {
        if ((I2CTimeout--) == 0)
        {
            printf("last_event:0x%x\r\n", last_event);
            CommTimeOut_CallBack(SLAVE_UNKNOW);
        }
    }
    else
    {
        I2CTimeout = I2CT_LONG_TIMEOUT;
    }
}

void Delay_us(uint32_t nCount)
{
    uint32_t tcnt;
    while (nCount--)
    {
        tcnt = 64 / 5;
        while (tcnt--){;}
    }
}

void CommTimeOut_CallBack(ErrCode_t errcode)
{
    log_info("...ErrCode:%d\r\n", errcode);
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

/**
 * @brief memery set a value
 * @param s source
 * @param c value
 * @param count number
 * @return pointer of the memery
 */
void Memset(void* s, uint8_t c, uint32_t count)
{
    char* xs = (char*)s;

    while (count--) // clear 17byte buffer
    {
        *xs++ = c;
    }

    return;
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
