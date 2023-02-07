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

/** @addtogroup I2C_Master_Int
 * @{
 */


#define TEST_BUFFER_SIZE  32
#define I2CT_FLAG_TIMEOUT ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))
#define I2C_MASTER_ADDR   0x30
#define I2C_SLAVE_ADDR    0xA0
volatile Status test_status = FAILED;

static CommCtrl_t Comm_Flag = C_READY;
uint8_t tx_buf[TEST_BUFFER_SIZE] = {0};
uint8_t rx_buf[TEST_BUFFER_SIZE] = {0};

uint8_t flag_master_recv_finish = 0;
uint8_t flag_master_send_finish = 0;
uint8_t flag_trans_direct       = 0; // write
uint8_t flag_overrun            = 0;

Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void Memset(void* s, uint8_t c, uint32_t count);
void CommTimeOut_CallBack(ErrCode_t errcode);
    
static __IO uint32_t I2CTimeout = I2CT_LONG_TIMEOUT;

/**
 * @brief  i2c Interrupt configuration
 * @param ch I2C channel
 */
void NVIC_ConfigurationMaster(void)
{
    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel             = I2Cx_MASTER_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPriority     = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd        = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  i2c Interrupt disable
 * @param ch I2C channel
 */
void NVIC_ConfigurationMasterDis(void)
{
    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel           = I2Cx_MASTER_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPriority    = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd         = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  i2c master init
 * @return 0:init finish
 *
 */
int i2c_master_init(void)
{
    I2C_InitType i2c1_master;
    GPIO_InitType i2c1_gpio;
    
    RCC_EnableAPB1PeriphClk(I2Cx_MASTER_CLK, ENABLE);
    RCC_EnableAPB2PeriphClk(I2Cx_MASTER_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);
    
    GPIO_InitStruct(&i2c1_gpio);
    /*PB6 -- SCL; PB7 -- SDA*/
    i2c1_gpio.Pin                = I2Cx_MASTER_SDA_PIN | I2Cx_MASTER_SCL_PIN;
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
    i2c1_master.ClkSpeed    = 100000; // 100K

    I2C_Init(I2Cx_MASTER, &i2c1_master);
    // int enable
    I2C_ConfigInt(I2Cx_MASTER, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, ENABLE);
    NVIC_ConfigurationMaster();
        
    I2C_Enable(I2Cx_MASTER, ENABLE);
    return 0;
}

/**
 * @brief   Main program
 */
int main(void)
{
    uint16_t i = 0;

    log_init();
    log_info("\n\rthis is a i2c master int demo\r\n");

    i2c_master_init();

    /* Fill the buffer to send */
    for (i = 0; i < TEST_BUFFER_SIZE; i++)
    {
        tx_buf[i] = i;
    }
    
    /* First write in the memory followed by a read of the written data --------*/
    /* Write data*/
    flag_trans_direct = 0;
    if (Comm_Flag == C_READY)
    {
        Comm_Flag = C_START_BIT;
        I2C_GenerateStart(I2C1, ENABLE);
    }
    
    I2CTimeout = I2CT_LONG_TIMEOUT * 1000;
    while (flag_master_send_finish == 0)
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_UNKNOW);
        }
    }    
    
    // master send finish
    log_info("I2C master send finish\r\n");
    flag_master_send_finish = 0;
    I2CTimeout = I2CT_LONG_TIMEOUT;
    while (I2C_GetFlag(I2C1, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_BUSY);
        }
    }
    Comm_Flag = C_READY;
    
    printf("send finish\r\n");
    
    /*read data*/
    flag_trans_direct = 1;
    I2C_ConfigAck(I2C1, ENABLE);
    if (Comm_Flag == C_READY)
    {
        Comm_Flag = C_START_BIT;
        I2C_GenerateStart(I2C1, ENABLE);
    }
    
    I2CTimeout = I2CT_LONG_TIMEOUT * 1000;
    while (flag_master_recv_finish == 0)
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_UNKNOW);
        }
    }
    
    log_info("I2C master recv finish\r\n");
    flag_master_recv_finish = 0;
    I2CTimeout = I2CT_LONG_TIMEOUT;
    while (I2C_GetFlag(I2C1, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_BUSY);
        }
    }
    Comm_Flag = C_READY;
    
    /* Check if the data written to the memory is read correctly */
    test_status = Buffercmp(tx_buf, rx_buf, TEST_BUFFER_SIZE);
    if (test_status == PASSED) /* test_status = PASSED, if the write and read dataare the same  */
    {
        log_info("the write and read data the same, i2c master int test pass\r\n");
    }
    else /* test_status = FAILED, if the write and read dataare different */
    {
        log_info("the write and read data are different, i2c master int test fail\r\n");
    }

    while (1)
    {
    }
}

/**
 * @brief  i2c Interrupt service function
 *
 */

/* Master mode */
#define I2C_ROLE_MASTER ((uint32_t)0x00010000) /* MSMODE */

void I2C1_IRQHandler(void)
{
    uint32_t last_event;
    static uint8_t rx_num = 0;
    static uint8_t tx_num = 0;

    last_event = I2C_GetLastEvent(I2Cx_MASTER);
    
    if ((last_event & I2C_ROLE_MASTER) == I2C_ROLE_MASTER) // master mode
    {
        switch (last_event)
        {
            case I2C_EVT_MASTER_MODE_FLAG: // 0x00030001.EV5 Send addr
                if(flag_trans_direct)     // read
                {
                    Memset(rx_buf, 0, TEST_BUFFER_SIZE); // clear recv buf, ready to recv data
                    I2C_SendAddr7bit(I2Cx_MASTER, I2C_SLAVE_ADDR, I2C_DIRECTION_RECV);
                    rx_num = 0;
                }
                else // write
                {
                    I2C_SendAddr7bit(I2Cx_MASTER, I2C_SLAVE_ADDR, I2C_DIRECTION_SEND);
                    tx_num = 0;
                }
                break;
                
             // MasterTransmitter    
            case I2C_EVT_MASTER_TXMODE_FLAG: // 0x00070082. EV6 Send first data
                Comm_Flag = C_READY;
                I2C_SendData(I2Cx_MASTER, tx_buf[tx_num++]);
                break;
            
            case I2C_EVT_MASTER_DATA_SENDING: // 0x00070080. EV8 Sending data    

                break;
                
            case I2C_EVT_MASTER_DATA_SENDED: // 0x00070084.EV8_2 Send finish
                if (tx_num == TEST_BUFFER_SIZE)
                {
                    if (Comm_Flag == C_READY)
                    {
                        Comm_Flag = C_STOP_BIT;
                        I2C_GenerateStop(I2Cx_MASTER, ENABLE);
                        flag_master_send_finish = 1;
                    }
                }
                else
                {
                    I2C_SendData(I2Cx_MASTER, tx_buf[tx_num++]);
                }
                break;
                
                // MasterReceiver
            case I2C_EVT_MASTER_RXMODE_FLAG: // 0x00030002.EV6
                break;
            
            case I2C_EVT_MASTER_DATA_RECVD_FLAG: // 0x00030040. EV7.one byte recved             
            case I2C_EVT_MASTER_DATA_RECVD_BSF_FLAG: // 0x00030044. EV7.When the I2C communication rate is too high, BSF = 1
                rx_buf[rx_num++] = I2C_RecvData(I2Cx_MASTER);
                if (rx_num == (TEST_BUFFER_SIZE - 1))
                { 
                    I2C_ConfigAck(I2C1, DISABLE);   // Disable I2C1 acknowledgement.
                    I2C_GenerateStop(I2C1, ENABLE); // Send I2C1 STOP Condition.
                }
                else if (rx_num == TEST_BUFFER_SIZE)
                {
                    flag_master_recv_finish = 1;
                }
                break;
                
            case 0x00030201: // Arbitration lost
            case 0x00030401: // Acknowledge failure
            case 0x00030501: // Acknowledge failure and Bus error
                I2C_GenerateStop(I2C1, ENABLE);
                break;
            
            default:
                log_info("I2C error status:0x%x\r\n", last_event);
                while(1); //stop
                //break;
        }
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
