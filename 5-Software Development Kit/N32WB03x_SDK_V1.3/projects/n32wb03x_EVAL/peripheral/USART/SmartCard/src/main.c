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
 
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "main.h"
#include "log.h"
#include "smartcard.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* Private define ------------------------------------------------------------*/


/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t gTestResult;
uint8_t looptime = 0;
uint8_t F_HotReset = 0;

uint8_t AppletAID[8] = {0xA0, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00};

/* Data: Send to card */
uint8_t TxData[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 
                    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
/* Data: Receive from card */
uint8_t RxData[16] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 
                    0xaa, 0x55, 0xff, 0x00, 0x11, 0x22, 0x33, 0x44};   

char gJsonObject[512];
/* Private function prototypes -----------------------------------------------*/
void Test_PSAM(void);
/* Private functions ---------------------------------------------------------*/



/** @addtogroup N32WB03X_StdPeriph_Examples
 * @{
 */

/** @addtogroup SmartCard
 * @{
 */



/**
 * @brief  Main program
 */
int main(void)
{
 
    log_init();
    /* Output a message on Hyperterminal using printf function */
    printf("\n\rSmartCard 7816 demo\n\r");

    while (1)
    {
        Test_PSAM();
        printf("PSAM_1 test finish\r\n");
        
        SC1_DeInit();
        Delay_ms(1000);

//        Uart_Init(115200);
        printf("System Init Finish!\r\n");
    }
}

/**
 * @brief  smart card test
 * @param   
 * @return 
 * @note   Note
 */
void Test_PSAM(void)
{
    uint16_t i;
    uint8_t timeout =0;
    SC_InitStructure SC_InitCfg;
    SC_State SCState = SC_POWER_OFF;
    SC_ADPU_Commands SC_ADPU;
    SC_ADPU_Responce SC_Responce;
    
    printf("USARTx Smart_Card_1 Test Start\r\n");
    
    SC_InitCfg.Clk_Div   =  10;    
    SC_InitCfg.GT        =  16; 
    SC_InitCfg.StopBits  =  3;  
    SC_InitCfg.Parity    =  1;        
    SC_InitCfg.NackEn    =  0;  
    looptime             =  10;
    

    SCState = SC_POWER_ON;  
    
    SC_ADPU.Header.CLA  = 0x00;  
    SC_ADPU.Header.INS  = SC_GET_A2R;  
    SC_ADPU.Header.P1   = 0x00;  
    SC_ADPU.Header.P2   = 0x00;  
    SC_ADPU.Body.LC     = 0x00;
    
    while(SCState != SC_ACTIVE_ON_T0)   
    {  
        SC_Handler(&SCState, &SC_ADPU, &SC_Responce, &SC_InitCfg);  
        
        timeout++;
        Delay_ms(10);
        if(timeout>10)
        {
             break;
        }
    }  

    /* Apply the Procedure Type Selection (PTS)  */
    SC_PTSConfig();
    //
    Delay_ms(10);
    // Select Applet
    SC_ADPU.Header.CLA  = 0x00;
    SC_ADPU.Header.INS  = 0x84;
    SC_ADPU.Header.P1   = 0x00;
    SC_ADPU.Header.P2   = 0x00;
    SC_ADPU.Body.LC     = 0x00;
    SC_ADPU.Body.LE     = 0x08;

    for(i = 0; i < SC_ADPU.Body.LC; i++)
    {
        SC_ADPU.Body.Data[i] = AppletAID[i];
    }
    while(i < LC_MAX)
    {
        SC_ADPU.Body.Data[i++] = 0;
    }
    
    SC_Handler(&SCState, &SC_ADPU, &SC_Responce, &SC_InitCfg);
    Delay_ms(10);
    if((SC_Responce.SW1 == 0x90)&&(SC_Responce.SW2 == 0x00))
    {
        printf("SmartCard Read Random success! \r\n");
        for(i = 0;i<8;i++)
        {
            printf("Random_Table[%d] = %x \r\n",i,SC_Responce.Data[i]);
        }
        
    }
    
    F_HotReset = 1;
    printf("USARTx Smart_Card Hot_Reset\r\n");

    SC1_Reset(Bit_RESET);
    Delay_ms(10);
    SC1_Reset(Bit_SET);

    SC_InitCfg.Clk_Div   =  10;    
    SC_InitCfg.GT        =  16; 
    SC_InitCfg.StopBits  =  3;  
    SC_InitCfg.Parity    =  1;        
    SC_InitCfg.NackEn    =  0;  
    looptime             =  10;
    
    SCState = SC_POWER_ON;  
    
    SC_ADPU.Header.CLA  = 0x00;  
    SC_ADPU.Header.INS  = SC_GET_A2R;  
    SC_ADPU.Header.P1   = 0x00;  
    SC_ADPU.Header.P2   = 0x00;  
    SC_ADPU.Body.LC     = 0x00;
    
    while(SCState != SC_ACTIVE_ON_T0)   
    {  
        SC_Handler(&SCState, &SC_ADPU, &SC_Responce, &SC_InitCfg);  
    }  
    printf("USARTx Smart_Card Hot_Reset Finish\r\n");
    F_HotReset = 0;
}

/**
 * @}
 */

/**
 * @}
 */
