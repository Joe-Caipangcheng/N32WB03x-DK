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
 * @file system_n32wb03x.c
 * @author Nations Firmware Team
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32wb03x.h"
#include "string.h"
/* Uncomment the line corresponding to the desired System clock (SYSCLK)
   frequency (after reset the HSI is used as SYSCLK source)

   IMPORTANT NOTE:
   ==============
   1. After each device reset the HSI is used as System clock source.

   2. Please make sure that the selected System clock doesn't exceed your
   device's maximum frequency.

   3. If none of the define below is enabled, the HSI is used as System clock
    source.

   4. The System clock configuration functions provided within this file assume
   that:
        - HSI is configer as 64M and used to driverd the system clock.
        - External 32MHz crystal use for Bluetooth RF system only.
        - If Bluetooth stack is enable, we should select the LSI or LSE when configer
          the Bluetooth stack only.
*/

#define SYSCLK_USE_HSI     1
#define SYSCLK_USE_HSE     0

#ifndef SYSCLK_FREQ
#define SYSCLK_FREQ HSI_VALUE
#endif

/*
* SYSCLK_SRC *
** SYSCLK_USE_HSI     **
** SYSCLK_USE_HSE     **
*/
#ifndef SYSCLK_SRC
#define SYSCLK_SRC SYSCLK_USE_HSI
#endif



#define TRIM_STORE_ADDR         0x1000 
#define TRIM_READ_CMD_CODE_LEN  0x140
#define TRIM_READ_CMD_CODE_CRC  0x3aa0
const unsigned char  TRIM_READ_CMD_CODE[] ={
0x40,0xba,0x70,0x47,0xc0,0xba,0x70,0x47,0x01,0x38,0xfd,0xd1,0x70,0x47,0x00,0x00,
0xf7,0xb5,0x03,0x25,0xad,0x06,0x28,0x6a,0x82,0xb0,0x16,0x46,0x0c,0x46,0x40,0x08,
0x40,0x00,0x28,0x62,0x28,0x6a,0x02,0x21,0x08,0x43,0x28,0x62,0x28,0x6a,0x80,0x07,
0xfc,0xd4,0x66,0x20,0x68,0x60,0x01,0x27,0x2f,0x61,0xa8,0x6a,0xc0,0x05,0xfc,0xd5,
0x99,0x20,0x68,0x60,0x2f,0x61,0xa8,0x6a,0xc0,0x05,0xfc,0xd5,0xff,0x20,0x91,0x30,
0xff,0xf7,0xda,0xff,0xff,0x23,0x01,0x33,0xab,0x62,0x68,0x46,0xef,0x60,0x35,0x21,
0x69,0x60,0x2f,0x61,0xa9,0x6a,0xc9,0x05,0xfc,0xd5,0xab,0x62,0xa9,0x69,0xef,0x60,
0xc9,0xb2,0x05,0x22,0x6a,0x60,0x2f,0x61,0xaa,0x6a,0xd2,0x05,0xfc,0xd5,0xab,0x62,
0xaa,0x69,0x09,0x02,0xd2,0xb2,0x11,0x43,0x01,0x80,0xc8,0x07,0x02,0xd0,0x03,0x20,
0x05,0xb0,0xf0,0xbd,0xab,0x62,0x68,0x69,0xff,0x21,0x08,0x31,0x88,0x43,0x68,0x61,
0x68,0x69,0xc9,0x1e,0x08,0x43,0x68,0x61,0x02,0x98,0x00,0x02,0x48,0x30,0x68,0x60,
0x08,0x20,0xa8,0x60,0xee,0x60,0x2f,0x61,0xa8,0x6a,0xc0,0x05,0xfc,0xd5,0xff,0x20,
0x01,0x30,0xa8,0x62,0x00,0x23,0xf6,0x1c,0xb0,0x08,0x0e,0xd0,0xb2,0x08,0x11,0x48,
0x00,0x68,0x99,0x00,0x60,0x54,0x06,0x0a,0x09,0x19,0x4e,0x70,0x06,0x0c,0x00,0x0e,
0x8e,0x70,0x5b,0x1c,0xc8,0x70,0x9a,0x42,0xf1,0xd8,0xff,0x20,0x01,0x30,0xa8,0x62,
0x68,0x69,0xff,0x21,0x08,0x31,0x88,0x43,0x68,0x61,0x68,0x69,0x38,0x43,0x68,0x61,
0x28,0x6a,0x80,0x08,0x80,0x00,0x28,0x62,0x28,0x6a,0x38,0x43,0x28,0x62,0x00,0x20,
0x05,0xb0,0xf0,0xbd,0x80,0x00,0x00,0x0c,0x03,0x20,0x80,0x06,0x41,0x69,0xff,0x22,
0x08,0x32,0x91,0x43,0x41,0x61,0x42,0x69,0x01,0x21,0x0a,0x43,0x42,0x61,0x02,0x6a,
0x92,0x08,0x92,0x00,0x02,0x62,0x02,0x6a,0x0a,0x43,0x02,0x62,0x70,0x47,0x00,0x00,
};
typedef uint32_t (*trim_read_cmd_func_t)(uint32_t,uint8_t*,uint32_t);
trim_stored_t trim_stored;

/*******************************************************************************
 *  Clock Definitions
 *******************************************************************************/
uint32_t SystemCoreClock = SYSCLK_FREQ; /*!< System Clock Frequency (Core Clock) */
static void RCC_HsiCalib(uint32_t systemfreq);
    
void SystemTrimValueRead(uint8_t* p_data,uint32_t byte_length)
{
    uint32_t ramcode[TRIM_READ_CMD_CODE_LEN/4 +1 ];
    trim_read_cmd_func_t trim_read_cmd_func = (trim_read_cmd_func_t)((uint8_t*)&ramcode[0] + 0x11);
    memcpy((void*)ramcode,(const void*)TRIM_READ_CMD_CODE,TRIM_READ_CMD_CODE_LEN);
    (*trim_read_cmd_func)(TRIM_STORE_ADDR, p_data, byte_length);
}

trim_stored_t* SystemTrimValueGet(void)
{
    //read the trim value if not in RAM yet
    if(trim_stored.stote_rc64m_trim_value == 0xFFFFFFFF || trim_stored.stote_rc64m_trim_value == 0)
    {
        SystemTrimValueRead((uint8_t*)&trim_stored,sizeof(trim_stored));
    }
    //check again if read trim value sucessful
    if(trim_stored.stote_rc64m_trim_value == 0xFFFFFFFF || trim_stored.stote_rc64m_trim_value == 0)
    {
        return NULL;
    }else{
        return &trim_stored;
    }
}

uint8_t* SystemGetUUID(void)
{
    //read the trim value if not in RAM yet
    if(trim_stored.stote_rc64m_trim_value == 0xFFFFFFFF || trim_stored.stote_rc64m_trim_value == 0)
    {
        SystemTrimValueRead((uint8_t*)&trim_stored,sizeof(trim_stored));
    }
    //check again if read trim value sucessful
    if(trim_stored.stote_rc64m_trim_value == 0xFFFFFFFF || trim_stored.stote_rc64m_trim_value == 0)
    {
        return NULL;
    }else{
        return trim_stored.flash_uuid;
    }
    
}

uint8_t* SystemGetMacAddr(void)
{
    //read the trim value if not in RAM yet
    if(trim_stored.stote_rc64m_trim_value == 0xFFFFFFFF || trim_stored.stote_rc64m_trim_value == 0)
    {
        SystemTrimValueRead((uint8_t*)&trim_stored,sizeof(trim_stored));
    }
    //check again if read trim value sucessful
    if(trim_stored.stote_rc64m_trim_value == 0xFFFFFFFF || trim_stored.stote_rc64m_trim_value == 0)
    {
        return NULL;
    }else{
        return &trim_stored.flash_uuid[5];
    }
    
}
    

/**
 * @brief  Setup the microcontroller system
 *         Initialize the Embedded Flash Interface, the PLL and update the
 *         SystemCoreClock variable.
 * @note   This function should be used only after reset.
 */
void SystemInit(void)
{
    uint32_t  tmp;
    RCC->APB1PCLKEN |= RCC_APB1_PERIPH_PWR; // PWR enable
    PWR->VTOR_REG = 0x81000000; //set irq vtor to flash address

    *(uint32_t*)0x40007014 = 0x0000080F; 
    *(uint32_t*)0x40007020 = 0x00020018; 
    *(uint32_t*)0x40011000 &= ~0xC000;

    SystemTrimValueRead((uint8_t*)&trim_stored,sizeof(trim_stored));
    /* check otp has been write */
    if(trim_stored.stote_rc64m_trim_value == 0xFFFFFFFF || trim_stored.stote_rc64m_trim_value == 0) 
    {
        RCC->CFG |= RCC_CFG_HSISRC_DIV1; // USE HSI as system clock
        RCC->CFG &= ~RCC_CFG_APB1PRES;
        RCC->CFG |=  RCC_HCLK_DIV2; //APB1 = HCLK/2, APB1 max is 32M
        /* Calib from HSE */
        RCC_HsiCalib(SYSCLK_FREQ);
    }
    else
    {
        tmp = (PWR->reserved4)&(~0X1F);
        tmp |= (trim_stored.stote_bg_vtrim_value)&0X1F;
        PWR->reserved4 = tmp;
		if(SYSCLK_FREQ == 64000000)
		{           
            RCC->CTRL &= ~0x8000;// Set HSI as 64M
            /* Configures LSI trim */
            tmp = RCC->CTRL & ~(0x7F << 8);   // TRIM 8-14 bit
            RCC->CTRL = tmp|(trim_stored.stote_rc64m_trim_value << 8);// clear and set TRIM value 
            
            RCC->CFG |= RCC_CFG_HSISRC_DIV1; // USE HSI as system clock
            
            RCC->CFG &= ~RCC_CFG_APB1PRES;
            RCC->CFG |=  RCC_HCLK_DIV2; //APB1 = HCLK/2, APB1 max is 32M
		}
//        else if(SYSCLK_FREQ == 96000000)
//        {                      
//            RCC->CTRL |= 0x8000; // Set HSI as 96M
//            /* Configures LSI trim */
//            tmp = RCC->CTRL & ~(0x7F << 8);   // TRIM 8-14 bit
//            RCC->CTRL = tmp|(trim_stored.stote_rc96m_trim_value << 8);// clear and set TRIM value 
//            
//            RCC->CFG |= RCC_CFG_HSISRC_DIV1; // USE HSI as system clock
//            tmp = RCC->CFG & ~RCC_CFG_APB1PRES;
//            RCC->CFG |=  RCC_HCLK_DIV4; //APB1 = HCLK/4, APB1 max is 32M
//            
//            tmp = RCC->CFG & ~RCC_CFG_APB2PRES;
//            RCC->CFG |=  RCC_HCLK_DIV4<<3; //APB2 = HCLK/2, APB1 max is 64M
//        }
        /* Configures LSI trim */
        RCC->LSCTRL &= ~RCC_LSCTRL_LSTTRIM;
        RCC->LSCTRL |=  trim_stored.stote_rc32768_trim_value << 8;        
        
    }
 
}

/**
 * @brief  Update SystemCoreClock variable according to Clock Register Values.
 *         The SystemCoreClock variable contains the core clock (HCLK), it can
 *         be used by the user application to setup the SysTick timer or
 * configure other parameters.
 *
 * @note   Each time the core clock (HCLK) changes, this function must be called
 *         to update SystemCoreClock variable value. Otherwise, any
 * configuration based on this variable will be incorrect.
 *
 * @note   - The system frequency computed by this function is not the real
 *           frequency in the chip. It is calculated based on the predefined
 *           constant and the selected clock source:
 */
void SystemCoreClockUpdate(void)
{
    SystemCoreClock = HSI_VALUE;
}

/**
 * @brief dealy cycles.
 */
__ASM  void system_delay_cycles(uint32_t i)
{
    SUBS r0, #1
    BNE system_delay_cycles
    BX LR
}

/**
 * @brief  Dealy 10 us
 */
void system_delay_n_10us(uint32_t value)
{
    system_delay_cycles(107*value);
}

/**
 * @brief  Enable the HSI and calibration it.
 */
static void RCC_HsiCalib(uint32_t systemfreq) 
{

    uint32_t g_hsi_accuracy = 0;
    uint32_t g_timeoutcnt = 1000;
    uint32_t g_cal_hsi_cnt_value = 1024;

    uint32_t delta =0;
    uint32_t min = 0;
    uint32_t max = 127;                                 //32M TRIM 8-14 bit
    uint32_t mid = (RCC->CTRL >> 8) & 0x7F; 
    uint16_t count_value = 0;
    uint32_t hsi_timeoutcnt = 0; 
    uint8_t tmp_trim;

    if(systemfreq == 64000000)
    {
        RCC->CTRL &= ~0x8000;
        g_cal_hsi_cnt_value = 2048;            //for 64M            
    }
    else
    {
        RCC->CTRL |= 0x8000;
        g_cal_hsi_cnt_value = 3072;            //for 96M    
    }
    
    do{
        uint32_t  tmp = RCC->CTRL & ~(0x7F << 8);   //32M TRIM 8-14 bit
        RCC->CTRL  =  tmp| (mid << 8);              // clear and set TRIM value  //and start to cnt
        system_delay_cycles(5);                                // delay scape          
        while(1)
        {
            system_delay_cycles(1);        
            if((RCC->OSCFCSR & 0x02))
            {
                break;
            }    
            if(hsi_timeoutcnt++ > g_timeoutcnt)
            {    
                break;
            }
        }
        count_value = RCC->OSCFCHSICNT;                 //ready cnt value       
		if(count_value > g_cal_hsi_cnt_value)
		{
			delta = count_value - g_cal_hsi_cnt_value; 
		}
		else
		{
			delta = g_cal_hsi_cnt_value - count_value; 
		}
        
        if(count_value >=  g_cal_hsi_cnt_value)
        {
            max = mid;
        }
        else
        {
            min = mid;
        }
        
        tmp_trim = (min + max)/2;  
        if(tmp_trim == mid )                //0 and 127 if not used        
        {
            break;
        }
        mid = tmp_trim;
    }while(delta > g_hsi_accuracy);  
    RCC->CFG &= ~1;
    while((RCC->CFG & (1<<2)));
}

