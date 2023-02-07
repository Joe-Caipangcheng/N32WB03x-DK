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
 * @file software_i2c.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __SOFTWARE_I2C_H__
#define __SOFTWARE_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32wb03x.h"


 
 
//??IIC??????
typedef struct 
{
    GPIO_Module    *SDA_GPIOx;       //SDA port
    GPIO_Module    *SCL_GPIOx;       //SCL port
    uint32_t        SDA_PINx;        //SDA pin
    uint32_t        SCL_PINx;        //SCL pin
    uint32_t        DelayUS;         //delay time in US
    //port function
    void (*Start)(void *pHandle);                //SI2C start signal
    void (*Stop)(void *pHandle);                //SI2C stop signal
    bool (*SendByte)(void *pHandle, uint8_t data);    //SI2C send a byte
    uint8_t (*ReadByte)(void *pHandle,bool isAck);    //SI2C read a byte
}SI2C_HANDLE;
 
 
bool SI2C_Init(SI2C_HANDLE *pHandle, GPIO_Module *SDA_GPIOx, GPIO_Module *SCL_GPIOx,\
                uint32_t SDA_Pin, uint32_t SCL_Pin,uint8_t DelayUS);    //??IIC???
 
void SI2C_Start(SI2C_HANDLE *pHandle);                //SI2C start signal
void SI2C_Stop(SI2C_HANDLE *pHandle);                //SI2C stop signal
bool SI2C_WaitAck(SI2C_HANDLE *pHandle);            //SI2C wait ack signal
void SI2C_Ack(SI2C_HANDLE *pHandle);                //SI2C send ack signal
void SI2C_NAck(SI2C_HANDLE *pHandle);                //SI2C send nack signal
bool SI2C_SendByte(SI2C_HANDLE *pHandle, uint8_t data);    //SI2C send a byte
uint8_t SI2C_ReadByte(SI2C_HANDLE *pHandle,bool isAck);    //SI2C read a byte
bool SI2C_ReadReg(SI2C_HANDLE *pHandle, uint8_t SlaveAddr, uint16_t RegAddr, \
                  bool is8bitRegAddr, uint8_t *pDataBuff, uint16_t ReadByteNum); //SI2C read reg
bool SI2C_WriteReg(SI2C_HANDLE *pHandle, uint8_t SlaveAddr, uint16_t RegAddr, \
                   bool is8bitRegAddr, uint8_t *pDataBuff, uint16_t WriteByteNum); //SI2C write reg

bool SI2C_MasterWrite(SI2C_HANDLE *pHandle, uint8_t SlaveAddr, uint8_t *pDataBuff, uint16_t WriteByteNum);
bool SI2C_MasterRead(SI2C_HANDLE *pHandle, uint8_t SlaveAddr, uint8_t *pDataBuff, uint16_t ReadByteNum);
#ifdef __cplusplus
}
#endif

#endif /* __SOFTWARE_I2C_H__ */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
