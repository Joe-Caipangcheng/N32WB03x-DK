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
 * @file n32wb03x.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __N32WB03X_H__
#define __N32WB03X_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup N32WB03X_Library_Basic
 * @{
 */

#if !defined USE_STDPERIPH_DRIVER
/*
 * Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will
   be based on direct access to peripherals registers
   */
#define USE_STDPERIPH_DRIVER
#endif

/*
 * In the following line adjust the value of External High Speed oscillator (HSE)
   used in your application

   Tip: To avoid modifying this file each time you need to use different HSE, you
        can define the HSE value in your toolchain compiler preprocessor.
  */
#if !defined HSE_VALUE
#define HSE_VALUE (32000000) /*!< Value of the External oscillator in Hz */
#endif                      /* HSE_VALUE */

/*
 * In the following line adjust the External High Speed oscillator (HSE) Startup
   Timeout value
   */
#define HSE_STARTUP_TIMEOUT ((uint16_t)0x2000) /*!< Time out for HSE start up */
#define HSI_STARTUP_TIMEOUT ((uint16_t)0x0500) /*!< Time out for HSI start up */
#define LSE_STARTUP_TIMEOUT ((uint16_t)0x1000) /*!< Time out for LSE start up */
#define LSI_STARTUP_TIMEOUT ((uint16_t)0x1000) /*!< Time out for LSI start up */

#define HSI_VALUE (64000000) /*!< Value of the Internal oscillator in Hz*/

#define LSE_VALUE (32768) /*!< Value of the External Low Speed oscillator in Hz*/
#define LSI_VALUE (32000) /*!< Value of the Internal Low Speed oscillator in Hz*/

#define ADC_AUDIOPLLCLK_VALUE   (4096000) /*!< Value of the Internal ADC clk in Hz*/

#define __N32WB03X_STDPERIPH_VERSION_MAIN (0x01) /*!< [31:24] main version */
#define __N32WB03X_STDPERIPH_VERSION_SUB1 (0x00) /*!< [23:16] sub1 version */
#define __N32WB03X_STDPERIPH_VERSION_SUB2 (0x00) /*!< [15:8]  sub2 version */
#define __N32WB03X_STDPERIPH_VERSION_RC   (0x00) /*!< [7:0]  release candidate */

/**
 * @brief N32WB03X Standard Peripheral Library version number
 */
#define __N32WB03X_STDPERIPH_VERSION                                                                                    \
    ((__N32WB03X_STDPERIPH_VERSION_MAIN << 24) | (__N32WB03X_STDPERIPH_VERSION_SUB1 << 16)                               \
     | (__N32WB03X_STDPERIPH_VERSION_SUB2 << 8) | (__N32WB03X_STDPERIPH_VERSION_RC))

/*
 * Configuration of the Cortex-M0 Processor and Core Peripherals
 */
#define __NVIC_PRIO_BITS       2 /*!< N32WB03X uses 4 Bits for the Priority Levels    */
#define __Vendor_SysTickConfig 0 /*!< Set to 1 if different SysTick Config is used */

/**
 * @brief N32WB03X Interrupt Number Definition
 */
typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
    NonMaskableInt_IRQn      = -14, /*!< 2 Non Maskable Interrupt                            */
    HardFault_IRQn           = -13, /*!< 3 Hard Fault Interrupt                              */
    SVCall_IRQn              = -5,  /*!< 11 Cortex-M0 SV Call Interrupt                      */
    PendSV_IRQn              = -2,  /*!< 14 Cortex-M0 Pend SV Interrupt                      */
    SysTick_IRQn             = -1,  /*!< 15 Cortex-M0 System Tick Interrupt                  */

    /******  N32WB03X specific Interrupt Numbers ********************************************************/
    WWDG_IRQn                = 0,  /*!< Window WatchDog Interrupt                            */
    BLE_SW_IRQn              = 1,  /*!< BLE_SW Interrupt                                     */
    RTC_IRQn                 = 2,  /*!< RTC interrupt through EXTI line 17/19/20             */
    BLE_HSLOT_IRQn           = 3,  /*!< BLE_HSLOT Interrupt                                  */
    FLASH_IRQn               = 4,  /*!< FLASH global Interrupt                               */
    RCC_IRQn                 = 5,  /*!< RCC global Interrupt                                 */
    EXTI0_1_IRQn             = 6,  /*!< EXTI Line0/1 Interrupt                               */
    EXTI2_3_IRQn             = 7,  /*!< EXTI Line2/3 Interrupt                               */
    EXTI4_12_IRQn            = 8,  /*!< EXTI Line4 ~ 12 Interrupt                            */
    BLE_FINETGT_IRQn         = 9,  /*!< BLE_FINETGT global Interrupt                         */
    BLE_FIFO_IRQn            = 10, /*!< BLE_FIFO global Interrupt                            */
    DMA_Channel1_2_3_4_IRQn  = 11, /*!< DMA Channel 1/2/3/4 global Interrupt                 */
    DMA_Channel5_IRQn        = 12, /*!< DMA Channel 5 global Interrupt                       */
    TIM1_BRK_UP_TRG_COM_IRQn = 13, /*!< TIM1 Break Update Trigger and Commutation Interrupt  */
    TIM1_CC_IRQn             = 14, /*!< TIM1 Capture Compare Interrupt                       */
    RESERVED_IRQn            = 15, /*!< RESERVED Interrupt                                   */
    TIM3_IRQn                = 16, /*!< TIM3 global Interrupt                                */
    BLE_ERROR_IRQn           = 17, /*!< BLE_ERROR Interrupt                                  */
    BLE_CRYPT_IRQn           = 18, /*!< BLE_CYPT Interrupt                                   */
    BLE_TIMESTAMP_TGT1_IRQn  = 19, /*!< BLE_TIMESTAMP_TGT1 Interrupt                         */
    TIM6_IRQn                = 20, /*!< TIM6 global Interrupts                               */
    ADC_IRQn                 = 21, /*!< ADC global Interrupts                                */
    SPI2_IRQn                = 22, /*!< SPI2 global Interrupts                               */
    I2C1_IRQn                = 23, /*!< I2C1 global Interrupts                               */
    BLE_TIMESTAMP_TGT2_IRQn  = 24, /*!< BLE_TIMESTAMP_TGT12 Interrupts                       */
    SPI1_IRQn                = 25, /*!< SPI1 global Interrupts                               */
    BLE_SLP_IRQn             = 26, /*!< BLE_SLP global Interrupts                            */
    KEYSCAN_IRQn             = 27, /*!< KEYSCAN Interrupt                                    */
    USART1_IRQn              = 28, /*!< USART1 global Interrupt                              */
    LPUART1_IRQn             = 29, /*!< LPUART global Interrupt                              */
    USART2_IRQn              = 30, /*!< USART2 global Interrupt                              */
    IRC_IRQn                 = 31, /*!< IRC global Interrupt                                 */
} IRQn_Type;

#include "core_cm0.h"
#include "system_n32wb03x.h"
#include <stdint.h>
#include <stdbool.h>

typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;

typedef const int32_t sc32; /*!< Read Only */
typedef const int16_t sc16; /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t vs32;
typedef __IO int16_t vs16;
typedef __IO int8_t vs8;

typedef __I int32_t vsc32; /*!< Read Only */
typedef __I int16_t vsc16; /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

typedef const uint32_t uc32; /*!< Read Only */
typedef const uint16_t uc16; /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t vu8;

typedef __I uint32_t vuc32; /*!< Read Only */
typedef __I uint16_t vuc16; /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */
typedef enum
{
    RESET = 0,
    SET   = !RESET
} FlagStatus,
    INTStatus;

typedef enum
{
    DISABLE = 0,
    ENABLE  = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
    ERROR   = 0,
    SUCCESS = !ERROR
} ErrorStatus;

/* N32WB03X Standard Peripheral Library old definitions (maintained for legacy purpose) */
#define HSEStartUp_TimeOut HSE_STARTUP_TIMEOUT
#define HSE_Value          HSE_VALUE
#define HSI_Value          HSI_VALUE

/**
 * @brief Analog to Digital Converter
 */
typedef struct
{
    __IO uint32_t CTRL;
    __IO uint32_t SR;
    __IO uint32_t OVR_SAMP_CNT;
    __IO uint32_t DAT;
    __IO uint32_t WDT_THRES;
    __IO uint32_t FGA_CFG;
    __IO uint32_t VOICE_DET_CR;
    __IO uint32_t VOICE_ZCR_THRES;
    __IO uint32_t VOICE_ED_THRES;
    __IO uint32_t VOICE_ED_DWN_THRES;
    __IO uint32_t VOICE_ED_UP_THRES; 
} ADC_Module;

/**
 * @brief CRC calculation unit
 */

typedef struct
{
    __IO uint32_t CRC32DAT; /*!< CRC data register */
    __IO uint8_t CRC32IDAT; /*!< CRC independent data register*/
    uint8_t RESERVED0;
    uint16_t RESERVED1;
    __IO uint32_t CRC32CTRL; /*!< CRC control register */
    __IO uint32_t CRC16CTRL;
    __IO uint8_t CRC16DAT;
    uint8_t RESERVED2;
    uint16_t RESERVED3;
    __IO uint16_t CRC16D;
    uint16_t RESERVED4;
    __IO uint8_t LRC;
    uint8_t RESERVED5;
    uint16_t RESERVED6;
} CRC_Module;


/**
 * @brief DMA Controller
 */

typedef struct
{
    __IO uint32_t CHCFG;
    __IO uint32_t TXNUM;
    __IO uint32_t PADDR;
    __IO uint32_t MADDR;
    __IO uint32_t CHSEL;

} DMA_ChannelType;

typedef struct
{
    __IO uint32_t INTSTS;
    __IO uint32_t INTCLR;
    __IO DMA_ChannelType DMA_Channel[5];
} DMA_Module;

/**
 * @brief External Interrupt/Event Controller
 */

typedef struct
{
    __IO uint32_t IMASK;    /*offset 0x00*/
    __IO uint32_t EMASK;    /*offset 0x04*/
    __IO uint32_t RT_CFG;   /*offset 0x08*/
    __IO uint32_t FT_CFG;   /*offset 0x0C*/
    __IO uint32_t SWIE;     /*offset 0x10*/
    __IO uint32_t PEND;     /*offset 0x14*/
} EXTI_Module;

/**
 * @brief General Purpose I/O
 */

typedef struct
{
    __IO uint32_t PMODE;  /*offset 0x00*/
    __IO uint32_t POTYPE; /*offset 0x04*/
    __IO uint32_t SR;  /*offset 0x08*/
    __IO uint32_t PUPD;  /*offset 0x0C*/
    __IO uint32_t PID;   /*offset 0x10*/
    __IO uint32_t POD;   /*offset 0x14*/
    __IO uint32_t PBSC;  /*offset 0x18*/
    __IO uint32_t PLOCK;     /*offset 0x1C*/
    __IO uint32_t AFL;         /*offset 0x20*/
    __IO uint32_t AFH;         /*offset 0x24*/
    __IO uint32_t PBC;   /*offset 0x28*/
    __IO uint32_t DS;    /*offset 0x2C*/
} GPIO_Module;

/**
 * @brief Alternate Function I/O
 */

typedef struct
{
    __IO uint32_t CFG;
    __IO uint32_t EXTI_CFG[2];
} AFIO_Module;
/**
 * @brief Inter Integrated Circuit Interface
 */

typedef struct
{
    __IO uint16_t CTRL1;
    uint16_t RESERVED0;
    __IO uint16_t CTRL2;
    uint16_t RESERVED1;
    __IO uint16_t OADDR1;
    uint16_t RESERVED2;
    __IO uint16_t OADDR2;
    uint16_t RESERVED3;
    __IO uint16_t DAT;
    uint16_t RESERVED4;
    __IO uint16_t STS1;
    uint16_t RESERVED5;
    __IO uint16_t STS2;
    uint16_t RESERVED6;
    __IO uint16_t CLKCTRL;
    uint16_t RESERVED7;
    __IO uint16_t TMRISE;
    uint16_t RESERVED8;
} I2C_Module;

/**
 * @brief Independent WATCHDOG
 */

typedef struct
{
    __IO uint32_t KEY;
    __IO uint32_t PREDIV; /*!< IWDG PREDIV */
    __IO uint32_t RELV;
    __IO uint32_t STS;
} IWDG_Module;

/**
 * @brief Power Control
 */

typedef struct
{
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t reserved0;
    __IO uint32_t reserved1;
    __IO uint32_t reserved2;
    __IO uint32_t reserved3;
    __IO uint32_t reserved4;
    __IO uint32_t reserved5;
    __IO uint32_t reserved6;
    __IO uint32_t reserved7;
    __IO uint32_t reserved8;
    __IO uint32_t reserved9;
    __IO uint32_t VTOR_REG;
} PWR_Module;

/**
 * @brief Reset and Clock Control
 */

typedef struct
{
    __IO uint32_t CTRL;
    __IO uint32_t CFG;
    __IO uint32_t CLKINT;
    __IO uint32_t APB2PRST;
    __IO uint32_t APB1PRST;
    __IO uint32_t AHBPCLKEN;
    __IO uint32_t APB2PCLKEN;
    __IO uint32_t APB1PCLKEN;
    __IO uint32_t LSCTRL;
    __IO uint32_t CTRLSTS;
    __IO uint32_t AHBPRST;
    __IO uint32_t CFG2;
    __IO uint32_t OSCFCCR;
    __IO uint32_t OSCFCSR;
    __IO uint32_t OSCFCLSICNT;
    __IO uint32_t OSCFCHSICNT;
    __IO uint32_t ROMCCR;
    __IO uint32_t DBGMCU_CR;
} RCC_Module;

/**
 * @brief Real-Time Clock
 */

typedef struct
{
    __IO uint32_t TSH;         /*!< RTC time register,                                         Address offset: 0x00 */
    __IO uint32_t DATE;        /*!< RTC date register,                                         Address offset: 0x04 */
    __IO uint32_t CTRL;        /*!< RTC control register,                                      Address offset: 0x08 */
    __IO uint32_t INITSTS;     /*!< RTC initialization and status register,                    Address offset: 0x0C */
    __IO uint32_t PRE;         /*!< RTC prescaler register,                                    Address offset: 0x10 */
    __IO uint32_t WKUPT;       /*!< RTC wakeup timer register,                                 Address offset: 0x14 */
    uint32_t reserved0;        /*!< Reserved                                                   Address offset: 0x18 */
    __IO uint32_t ALARMA;      /*!< RTC alarm A register,                                      Address offset: 0x1C */
    uint32_t reserved1;        /*!< Reserved                                                   Address offset: 0x20 */
    __IO uint32_t WRP;         /*!< RTC write protection register,                             Address offset: 0x24 */
    __IO uint32_t SUBS;        /*!< RTC sub second register,                                   Address offset: 0x28 */
    __IO uint32_t SCTRL;       /*!< RTC shift control register,                                Address offset: 0x2C */
    uint32_t reserved2;        /*!< Reserved                                                   Address offset: 0x30 */
    uint32_t reserved3;        /*!< Reserved                                                   Address offset: 0x34 */
    uint32_t reserved4;        /*!< Reserved                                                   Address offset: 0x38 */
    __IO uint32_t CALIB;       /*!< RTC calibration register,                                  Address offset: 0x3C */
    uint32_t reserved5;        /*!< Reserved                                                   Address offset: 0x40 */
    __IO uint32_t ALRMASS;     /*!< RTC alarm A sub second register,                           Address offset: 0x44 */
} RTC_Module;

/**
 * @brief Serial Peripheral Interface
 */

typedef struct
{
    __IO uint16_t CTRL1;
    uint16_t RESERVED0;
    __IO uint16_t CTRL2;
    uint16_t RESERVED1;
    __IO uint16_t STS;
    uint16_t RESERVED2;
    __IO uint16_t DAT;
    uint16_t RESERVED3;
    __IO uint16_t CRCPOLY;
    uint16_t RESERVED4;
    __IO uint16_t CRCRDAT;
    uint16_t RESERVED5;
    __IO uint16_t CRCTDAT;
    uint16_t RESERVED6;
    __IO uint16_t I2SCFG;
    uint16_t RESERVED7;
    __IO uint16_t I2SPREDIV;
    uint16_t RESERVED8;
} SPI_Module;

/**
 * @brief TIM
 */

typedef struct
{
    __IO uint32_t CTRL1;
    __IO uint32_t CTRL2;
    __IO uint16_t SMCTRL;
    uint16_t RESERVED1;
    __IO uint16_t DINTEN;
    uint16_t RESERVED2;
    __IO uint32_t STS;
    __IO uint16_t EVTGEN;
    uint16_t RESERVED3;
    __IO uint16_t CCMOD1;
    uint16_t RESERVED4;
    __IO uint16_t CCMOD2;
    uint16_t RESERVED5;
    __IO uint32_t CCEN;
    __IO uint16_t CNT;
    uint16_t RESERVED6;
    __IO uint16_t PSC;
    uint16_t RESERVED7;
    __IO uint16_t AR;
    uint16_t RESERVED8;
    __IO uint16_t REPCNT;
    uint16_t RESERVED9;
    __IO uint16_t CCDAT1;
    uint16_t RESERVED10;
    __IO uint16_t CCDAT2;
    uint16_t RESERVED11;
    __IO uint16_t CCDAT3;
    uint16_t RESERVED12;
    __IO uint16_t CCDAT4;
    uint16_t RESERVED13;
    __IO uint16_t BKDT;
    uint16_t RESERVED14;
    __IO uint16_t DCTRL;
    uint16_t RESERVED15;
    __IO uint16_t DADDR;
    uint16_t RESERVED16;
    uint32_t RESERVED17;
    __IO uint16_t CCMOD3;
    uint16_t RESERVED18;
    __IO uint16_t CCDAT5;
    uint16_t RESERVED19;
    __IO uint16_t CCDAT6;
    uint16_t RESERVED20;
} TIM_Module;

/**
 * @brief Universal Synchronous Asynchronous Receiver Transmitter
 */

typedef struct
{
    __IO uint16_t STS;
    uint16_t RESERVED0;
    __IO uint16_t DAT;
    uint16_t RESERVED1;
    __IO uint16_t BRCF;
    uint16_t RESERVED2;
    __IO uint16_t CTRL1;
    uint16_t RESERVED3;
    __IO uint16_t CTRL2;
    uint16_t RESERVED4;
    __IO uint16_t CTRL3;
    uint16_t RESERVED5;
    __IO uint16_t GTP;
    uint16_t RESERVED6;
} USART_Module;

/**
 * @brief Low-power Universal Asynchronous Receiver Transmitter
 */

typedef struct
{
    __IO uint16_t STS;
    uint16_t RESERVED0;
    __IO uint8_t INTEN;
    uint8_t RESERVED1;
    uint16_t RESERVED2;
    __IO uint16_t CTRL;
    uint16_t RESERVED3;
    __IO uint16_t BRCFG1;
    uint16_t RESERVED4;
    __IO uint8_t DAT;
    uint8_t RESERVED5;
    uint16_t RESERVED6;
    __IO uint8_t BRCFG2;
    uint8_t RESERVED7;
    uint16_t RESERVED8;
    __IO uint32_t WUDAT;
} LPUART_Module;

/**
 * @brief Window WATCHDOG
 */

typedef struct
{
    __IO uint32_t CTRL;
    __IO uint32_t CFG;
    __IO uint32_t STS;
} WWDG_Module;

/**
 * @brief IRC
 */
typedef struct
{
    __IO uint32_t  FREQ_CARRIER_ON;            
    __IO uint32_t  FREQ_CARRIER_OFF;           
    __IO uint32_t  LOGIC_ONE_TIME;             
    __IO uint32_t  LOGIC_ZERO_TIME;            
    __IO uint32_t  CTRL;                       
    __IO uint32_t  STATUS;                     
    __IO uint32_t  REPEAT_TIME;                
    __IO uint32_t  CODE_FIFO;                  
    __IO uint32_t  REPEAT_FIFO;                
} IRC_Module;

/**
 * @brief Keyscan Peripheral Interface
 */
typedef struct
{
    __IO uint32_t KEYCR;
    __IO uint32_t KEYDATA0;
    __IO uint32_t KEYDATA1;
    __IO uint32_t KEYDATA2;
    __IO uint32_t KEYDATA3;
    __IO uint32_t KEYDATA4;    
} KEYSCAN_Module;

#define FLASH_BASE  ((uint32_t)0x01000000) /*!< FLASH base address in the alias region */
#define SRAM_BASE   ((uint32_t)0x20000000) /*!< SRAM base address in the alias region */
#define PERIPH_BASE ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE (PERIPH_BASE)
#define APB2PERIPH_BASE (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE  (PERIPH_BASE + 0x20000)

/* APB1 */
#define TIM3_BASE          (APB1PERIPH_BASE + 0x0400)
#define TIM6_BASE          (APB1PERIPH_BASE + 0x1000)
#define RTC_BASE           (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE          (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE          (APB1PERIPH_BASE + 0x3000)
#define USART2_BASE        (APB1PERIPH_BASE + 0x4400)
#define LPUART1_BASE        (APB1PERIPH_BASE + 0x4800)
#define I2C1_BASE           (APB1PERIPH_BASE + 0x5400)
#define PWR_BASE           (APB1PERIPH_BASE + 0x7000)

/* APB2 */
#define AFIO_BASE          (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE          (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASE         (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE         (APB2PERIPH_BASE + 0x0C00)
#define AFEC_BASE          (APB2PERIPH_BASE + 0x1000)
#define KEYSCAN_BASE       (APB2PERIPH_BASE + 0x1400)
#define SPI1_BASE          (APB2PERIPH_BASE + 0x2000)
#define TIM1_BASE          (APB2PERIPH_BASE + 0x2C00)
#define USART1_BASE        (APB2PERIPH_BASE + 0x3800)
#define SPI2_BASE          (APB2PERIPH_BASE + 0x4400)

/* AHB */
#define DMA_BASE           (AHBPERIPH_BASE + 0x0000)
#define DMA_CH1_BASE       (AHBPERIPH_BASE + 0x0008)
#define DMA_CH2_BASE       (AHBPERIPH_BASE + 0x001C)
#define DMA_CH3_BASE       (AHBPERIPH_BASE + 0x0030)
#define DMA_CH4_BASE       (AHBPERIPH_BASE + 0x0044)
#define DMA_CH5_BASE       (AHBPERIPH_BASE + 0x0058)
#define ADC_BASE           (AHBPERIPH_BASE + 0x0800)
#define RCC_BASE           (AHBPERIPH_BASE + 0x1000)
#define IRC_BASE           (AHBPERIPH_BASE + 0x1C00)
#define PATCH_BASE         (AHBPERIPH_BASE + 0x2000)
#define MODEM_BASE         (AHBPERIPH_BASE + 0x2400)
#define CRC_BASE           (AHBPERIPH_BASE + 0x3000)
#define BLE_BASE           (AHBPERIPH_BASE + 0x8000)
#define BLE_EM_BASE        (AHBPERIPH_BASE + 0xC000)

#define TIM3        ((TIM_Module*)TIM3_BASE)
#define TIM6        ((TIM_Module*)TIM6_BASE)
#define RTC         ((RTC_Module*)RTC_BASE)
#define WWDG        ((WWDG_Module*)WWDG_BASE)
#define IWDG        ((IWDG_Module*)IWDG_BASE)
#define USART2      ((USART_Module*)USART2_BASE)
#define LPUART1     ((LPUART_Module*)LPUART1_BASE)
#define I2C1        ((I2C_Module*)I2C1_BASE)
#define PWR         ((PWR_Module*)PWR_BASE)
#define AFIO        ((AFIO_Module*)AFIO_BASE)
#define EXTI        ((EXTI_Module*)EXTI_BASE)
#define GPIOA       ((GPIO_Module*)GPIOA_BASE)
#define GPIOB       ((GPIO_Module*)GPIOB_BASE)
#define SPI1        ((SPI_Module*)SPI1_BASE)
#define TIM1        ((TIM_Module*)TIM1_BASE)
#define USART1      ((USART_Module*)USART1_BASE)
#define SPI2        ((SPI_Module*)SPI2_BASE)
#define DMA         ((DMA_Module*)DMA_BASE)
#define DMA_CH1     ((DMA_ChannelType*)DMA_CH1_BASE)
#define DMA_CH2     ((DMA_ChannelType*)DMA_CH2_BASE)
#define DMA_CH3     ((DMA_ChannelType*)DMA_CH3_BASE)
#define DMA_CH4     ((DMA_ChannelType*)DMA_CH4_BASE)
#define DMA_CH5     ((DMA_ChannelType*)DMA_CH5_BASE)
#define ADC         ((ADC_Module*)ADC_BASE)
#define RCC         ((RCC_Module*)RCC_BASE)
#define CRC         ((CRC_Module*)CRC_BASE)
#define IRC         ((IRC_Module*)IRC_BASE)
#define KEYSCAN     ((KEYSCAN_Module*)KEYSCAN_BASE)


//#define MODEM_BASE          (0x40022400)
#define EXCHANGE_MEM_BASE   (0x4002C000) /// start at 0x50800 8000 offset of the memory span             //TODO TRANSPLANT
#define BASEBAND_REG_BASE   (0x40028000)

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for CRC_CRC32DAT register  *********************/
#define CRC32_DAT_DAT ((uint32_t)0xFFFFFFFF) /*!< Data register bits */

/*******************  Bit definition for CRC_CRC32IDAT register  ********************/
#define CRC32_IDAT_IDAT ((uint8_t)0xFF) /*!< General-purpose 8-bit data register bits */

/********************  Bit definition for CRC_CRC32CTRL register  ********************/
#define CRC32_CTRL_RESET ((uint8_t)0x01) /*!< RESET bit */

/********************  Bit definition for CRC16_CR register  ********************/
#define CRC16_CTRL_LITTLE ((uint8_t)0x02)
#define CRC16_CTRL_BIG    ((uint8_t)0xFD)

#define CRC16_CTRL_RESET    ((uint8_t)0x04)
#define CRC16_CTRL_NO_RESET ((uint8_t)0xFB)

/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for PWR_CR1 register  ********************/
#define PWR_CR1_MODE_SEL                     ((uint16_t)0x0007)     /*!<  low power mode entered select*/
#define PWR_CR1_MODE_ACTIVE                  ((uint16_t)0x0000)    
#define PWR_CR1_MODE_STANDBY                 ((uint16_t)0x0001) 
#define PWR_CR1_MODE_SLEEP                   ((uint16_t)0x0002) 
#define PWR_CR1_MODE_PD                		 ((uint16_t)0x0004) 

#define PWR_CR1_MODE_EN                      ((uint16_t)0x0008)     

#define PWR_CR1_OSC_EN                       ((uint16_t)0x0020)   

/********************  Bit definition for PWR_CR2 register  ********************/
#define PWR_CR2_CORE_32KMEM_LP              ((uint16_t)0x0001)   
#define PWR_CR2_CORE_16KMEM_LP              ((uint16_t)0x0002)   
#define PWR_CR2_PAD_STA                     ((uint16_t)0x0008) 

#define PWR_CR2_BLE_STATE                   ((uint16_t)0xE000) 
#define PWR_CR2_BLE_STATE_POWERON           ((uint16_t)0x0000) 
#define PWR_CR2_BLE_STATE_ACTIVE            ((uint16_t)0x2000) 
#define PWR_CR2_BLE_STATE_SLEEP             ((uint16_t)0x8000) 

/********************  Bit definition for PWR_VTOR register  ********************/
#define PWR_VTOR_EN                         ((uint32_t)0x80000000) 
#define PWR_VTOR_ADDR                       ((uint32_t)0x7FFFFFFF)  

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for RCC_CTRL register  ********************/
#define RCC_CTRL_HSIEN          ((uint32_t)0x00000001) /*!< Internal High Speed clock enable */
#define RCC_CTRL_HSIRDF         ((uint32_t)0x00000002) /*!< Internal High Speed clock ready flag */
#define RCC_CTRL_HSITRIM        ((uint32_t)0x00007F00) /*!< Internal High Speed clock trimming */
#define RCC_CTRL_HSEEN          ((uint32_t)0x00010000) /*!< External High Speed clock enable */
#define RCC_CTRL_HSERDF         ((uint32_t)0x00020000) /*!< External High Speed clock ready flag */
#define RCC_CTRL_AUDIOPLLEN     ((uint32_t)0x01000000) /*!< AUDIOPLL enable */

/*******************  Bit definition for RCC_CFG register  *******************/
/*!< SCLKSW configuration */
#define RCC_CFG_SCLKSW     ((uint32_t)0x00000001) /*!< SCLKSW[0] bits (System clock Switch) */
#define RCC_CFG_SCLKSW_HSI ((uint32_t)0x00000000) /*!< HSI selected as system clock */
#define RCC_CFG_SCLKSW_HSE ((uint32_t)0x00000001) /*!< HSE selected as system clock */

/*!< SCLKSTS configuration */
#define RCC_CFG_SCLKSTS     ((uint32_t)0x00000004) /*!< SCLKSTS[0] bits (System Clock Switch Status) */
#define RCC_CFG_SCLKSTS_HSI ((uint32_t)0x00000000) /*!< HSI oscillator used as system clock */
#define RCC_CFG_SCLKSTS_HSE ((uint32_t)0x00000004) /*!< HSE oscillator used as system clock */


/*!< AHBPRES configuration */
#define RCC_CFG_AHBPRES   ((uint32_t)0x00000030) /*!< AHBPRES[1:0] bits (AHB prescaler) */

#define RCC_CFG_AHBPRES_DIV1   ((uint32_t)0x00000000) /*!< SYSCLK not divided */
#define RCC_CFG_AHBPRES_DIV2   ((uint32_t)0x00000020) /*!< SYSCLK divided by 2 */
#define RCC_CFG_AHBPRES_DIV4   ((uint32_t)0x00000030) /*!< SYSCLK divided by 4 */


/*!< APB1PRES configuration */
#define RCC_CFG_APB1PRES   ((uint32_t)0x00000700) /*!< APB1PRES[2:0] bits (APB1 prescaler) */


#define RCC_CFG_APB1PRES_DIV1  ((uint32_t)0x00000000) /*!< HCLK not divided */
#define RCC_CFG_APB1PRES_DIV2  ((uint32_t)0x00000400) /*!< HCLK divided by 2 */
#define RCC_CFG_APB1PRES_DIV4  ((uint32_t)0x00000500) /*!< HCLK divided by 4 */
#define RCC_CFG_APB1PRES_DIV8  ((uint32_t)0x00000600) /*!< HCLK divided by 8 */
#define RCC_CFG_APB1PRES_DIV16 ((uint32_t)0x00000700) /*!< HCLK divided by 16 */

/*!< APB2PRES configuration */
#define RCC_CFG_APB2PRES   ((uint32_t)0x00003800) /*!< APB2PRES[2:0] bits (APB2 prescaler) */
#define RCC_CFG_APB2PRES_0 ((uint32_t)0x00000800) /*!< Bit 0 */
#define RCC_CFG_APB2PRES_1 ((uint32_t)0x00001000) /*!< Bit 1 */
#define RCC_CFG_APB2PRES_2 ((uint32_t)0x00002000) /*!< Bit 2 */

#define RCC_CFG_APB2PRES_DIV1  ((uint32_t)0x00000000) /*!< HCLK not divided */
#define RCC_CFG_APB2PRES_DIV2  ((uint32_t)0x00002000) /*!< HCLK divided by 2 */
#define RCC_CFG_APB2PRES_DIV4  ((uint32_t)0x00002800) /*!< HCLK divided by 4 */
#define RCC_CFG_APB2PRES_DIV8  ((uint32_t)0x00003000) /*!< HCLK divided by 8 */
#define RCC_CFG_APB2PRES_DIV16 ((uint32_t)0x00003800) /*!< HCLK divided by 16 */

/*!< HSISRC configuration */
#define RCC_CFG_HSISRC         ((uint32_t)0x00010000) /*!< HSISRC[0] bit */
#define RCC_CFG_HSISRC_DIV1    ((uint32_t)0x00010000) /*!< HSI frequency divided by 1 */
#define RCC_CFG_HSISRC_DIV2    ((uint32_t)0x00000000) /*!< HSI frequency divided by 2 */


/*!< MCO configuration */
#define RCC_CFG_MCO   ((uint32_t)0x1E000000) /*!< MCO[3:0] bits (Microcontroller Clock Output) */
#define RCC_CFG_MCO_0 ((uint32_t)0x02000000) /*!< Bit 0 */
#define RCC_CFG_MCO_1 ((uint32_t)0x04000000) /*!< Bit 1 */
#define RCC_CFG_MCO_2 ((uint32_t)0x08000000) /*!< Bit 2 */
#define RCC_CFG_MCO_3 ((uint32_t)0x10000000) /*!< Bit 3 */

#define RCC_CFG_MCO_NOCLK       ((uint32_t)0x00000000) /*!< No clock */
#define RCC_CFG_MCO_LSI         ((uint32_t)0x02000000) /*!< LSI clock selected as MCO source */
#define RCC_CFG_MCO_LSE         ((uint32_t)0x04000000) /*!< LSE clock selected as MCO source */
#define RCC_CFG_MCO_SYSCLK      ((uint32_t)0x06000000) /*!< System clock selected as MCO source */
#define RCC_CFG_MCO_HSI         ((uint32_t)0x08000000) /*!< HSI clock selected as MCO source */
#define RCC_CFG_MCO_HSE         ((uint32_t)0x0A000000) /*!< HSE clock selected as MCO source */
#define RCC_CFG_MCO_HCLK        ((uint32_t)0x0C000000) /*!< HCLK clock selected as MCO source */
#define RCC_CFG_MCO_AUDIOPLL    ((uint32_t)0x0E000000) /*!< AUDIOPLL clock selected as MCO source */

/*!<******************  Bit definition for RCC_CLKINT register  ********************/
#define RCC_CLKINT_LSIRDIF     ((uint32_t)0x00000001) /*!< LSI Ready Interrupt flag */
#define RCC_CLKINT_LSERDIF     ((uint32_t)0x00000002) /*!< LSE Ready Interrupt flag */
#define RCC_CLKINT_HSIRDIF     ((uint32_t)0x00000004) /*!< HSI Ready Interrupt flag */
#define RCC_CLKINT_HSERDIF     ((uint32_t)0x00000008) /*!< HSE Ready Interrupt flag */
#define RCC_CLKINT_LSIRDIEN    ((uint32_t)0x00000100) /*!< LSI Ready Interrupt Enable */
#define RCC_CLKINT_LSERDIEN    ((uint32_t)0x00000200) /*!< LSE Ready Interrupt Enable */
#define RCC_CLKINT_HSIRDIEN    ((uint32_t)0x00000400) /*!< HSI Ready Interrupt Enable */
#define RCC_CLKINT_HSERDIEN    ((uint32_t)0x00000800) /*!< HSE Ready Interrupt Enable */
#define RCC_CLKINT_LSIRDICLR   ((uint32_t)0x00010000) /*!< LSI Ready Interrupt Clear */
#define RCC_CLKINT_LSERDICLR   ((uint32_t)0x00020000) /*!< LSE Ready Interrupt Clear */
#define RCC_CLKINT_HSIRDICLR   ((uint32_t)0x00040000) /*!< HSI Ready Interrupt Clear */
#define RCC_CLKINT_HSERDICLR   ((uint32_t)0x00080000) /*!< HSE Ready Interrupt Clear */

/*****************  Bit definition for RCC_APB2PRST register  *****************/
#define RCC_APB2PRST_AFIORST   ((uint32_t)0x00000001) /*!< Alternate Function I/O reset */
#define RCC_APB2PRST_IOPARST   ((uint32_t)0x00000004) /*!< I/O port A reset */
#define RCC_APB2PRST_IOPBRST   ((uint32_t)0x00000008) /*!< I/O port B reset */
#define RCC_APB2PRST_SPI1RST   ((uint32_t)0x00000200) /*!< SPI1 reset */
#define RCC_APB2PRST_SPI2RST   ((uint32_t)0x00000400) /*!< SPI2 reset */
#define RCC_APB2PRST_TIM1RST   ((uint32_t)0x00001000) /*!< TIM1 Timer reset */
#define RCC_APB2PRST_USART1RST ((uint32_t)0x00004000) /*!< USART1 reset */

/*****************  Bit definition for RCC_APB1PRST register  *****************/
#define RCC_APB1PRST_TIM3RST    ((uint32_t)0x00000002) /*!< Timer 3 reset */
#define RCC_APB1PRST_TIM6RST    ((uint32_t)0x00000010) /*!< Timer 6 reset */
#define RCC_APB1PRST_WWDGRST    ((uint32_t)0x00000800) /*!< Window Watchdog reset */
#define RCC_APB1PRST_USART2RST  ((uint32_t)0x00020000) /*!< USART 2 reset */
#define RCC_APB1PRST_I2C1RST    ((uint32_t)0x00200000) /*!< I2C 1 reset */
#define RCC_APB1PRST_PWRRST     ((uint32_t)0x10000000) /*!< Power interface reset */

/******************  Bit definition for RCC_AHBPCLKEN register  ******************/
#define RCC_AHBPCLKEN_DMAEN    ((uint32_t)0x00000001) /*!< DMA clock enable */
#define RCC_AHBPCLKEN_SRAMEN   ((uint32_t)0x00000004) /*!< SRAM interface clock enable */
#define RCC_AHBPCLKEN_FLITFEN  ((uint32_t)0x00000010) /*!< FLITF clock enable */
#define RCC_AHBPCLKEN_PATCHEN  ((uint32_t)0x00000020) /*!< PATCH clock enable */
#define RCC_AHBPCLKEN_CRCEN    ((uint32_t)0x00000040) /*!< CRC clock enable */
#define RCC_AHBPCLKEN_ADCEN    ((uint32_t)0x00001000) /*!< ADC clock enable */
#define RCC_AHBPCLKEN_IRCEN    ((uint32_t)0x00004000) /*!< IRC clock enable */

/******************  Bit definition for RCC_APB2PCLKEN register  *****************/
#define RCC_APB2PCLKEN_AFIOEN   ((uint32_t)0x00000001) /*!< Alternate Function I/O clock enable */
#define RCC_APB2PCLKEN_IOPAEN   ((uint32_t)0x00000004) /*!< I/O port A clock enable */
#define RCC_APB2PCLKEN_IOPBEN   ((uint32_t)0x00000008) /*!< I/O port B clock enable */
#define RCC_APB2PCLKEN_SPI1EN   ((uint32_t)0x00000200) /*!< SPI1 clock enable */
#define RCC_APB2PCLKEN_SPI2EN   ((uint32_t)0x00000400) /*!< SPI2 clock enable */
#define RCC_APB2PCLKEN_TIM1EN   ((uint32_t)0x00001000) /*!< TIM1 Timer clock enable */
#define RCC_APB2PCLKEN_USART1EN ((uint32_t)0x00004000) /*!< USART1 clock enable */

/*****************  Bit definition for RCC_APB1PCLKEN register  ******************/
#define RCC_APB1PCLKEN_TIM3EN     ((uint32_t)0x00000002) /*!< Timer 3 clock enable */
#define RCC_APB1PCLKEN_TIM6EN     ((uint32_t)0x00000010) /*!< Timer 6 clock enable */
#define RCC_APB1PCLKEN_WWDGEN     ((uint32_t)0x00000800) /*!< Window Watchdog clock enable */
#define RCC_APB1PCLKEN_USART2EN   ((uint32_t)0x00020000) /*!< USART 2 clock enable */
#define RCC_APB1PCLKEN_I2C1EN     ((uint32_t)0x00200000) /*!< I2C 1 clock enable */
#define RCC_APB1PCLKEN_PWREN      ((uint32_t)0x10000000) /*!< Power interface clock enable */

/*******************  Bit definition for RCC_LSCTRL register  *******************/
#define RCC_LSCTRL_LSIEN ((uint32_t)0x00000001) /*!< Internal Low Speed oscillator enable */
#define RCC_LSCTRL_LSIRD ((uint32_t)0x00000002) /*!< Internal Low Speed oscillator Ready */
#define RCC_LSCTRL_LSEEN ((uint32_t)0x00000004) /*!< External Low Speed oscillator enable */
#define RCC_LSCTRL_LSERD ((uint32_t)0x00000008) /*!< External Low Speed oscillator Ready */
#define RCC_LSCTRL_LSEBP ((uint32_t)0x00000010) /*!< External Low Speed oscillator Bypass */

#define RCC_LSCTRL_LSXSEL ((uint32_t)0x00000020) /*!< LSC clock source selection */
#define RCC_LSCTRL_LSXSEL_LSI ((uint32_t)0x00000000) /*!< LSI oscillator clock used as LSC clock */
#define RCC_LSCTRL_LSXSEL_LSE ((uint32_t)0x00000020) /*!< LSE oscillator clock used as LSC clock */

#define RCC_LSCTRL_LSTTRIM        ((uint32_t)0x0007FF00) /*!< LSI triming value */

#define RCC_LSCTRL_RTCEN          ((uint32_t)0x01000000) /*!< RTC clock enable */
#define RCC_LSCTRL_RTCRST         ((uint32_t)0x02000000) /*!< RTC software reset  */

#define RCC_LSCTRL_LPUARTSEL        ((uint32_t)0x04000000) /*!< LPUART clock selection */
#define RCC_LSCTRL_LPUART_APB1      ((uint32_t)0x00000000) /*!< APB1 clock selection */

#define RCC_LSCTRL_LPUARTSEL_LSI_LSE  ((uint32_t)0x04000020) /*!< LPUART clock selection */
#define RCC_LSCTRL_LPUART_LSI   ((uint32_t)0x04000000) /*!< LSI_LSE clock selection */
#define RCC_LSCTRL_LPUART_LSE   ((uint32_t)0x04000020) /*!< LSI_LSE clock selection */


#define RCC_LSCTRL_LPUARTEN       ((uint32_t)0x08000000) /*!< LPUART clock enable */
#define RCC_LSCTRL_LPUARTRST      ((uint32_t)0x10000000) /*!< LPUART reset */
#define RCC_LSCTRL_KEYSCANEN      ((uint32_t)0x40000000) /*!< KEYSCAN clock enable */

/*******************  Bit definition for RCC_CTRLSTS register  ********************/
#define RCC_CTRLSTS_RMRSTF     ((uint32_t)0x00000001) /*!< Remove reset flag */
#define RCC_CTRLSTS_PINRSTF    ((uint32_t)0x00000008) /*!< PIN reset flag */
#define RCC_CTRLSTS_PORRSTF    ((uint32_t)0x00000010) /*!< POR reset flag */
#define RCC_CTRLSTS_SFTRSTF    ((uint32_t)0x00000020) /*!< Software Reset flag */
#define RCC_CTRLSTS_IWDGRSTF   ((uint32_t)0x00000040) /*!< Independent Watchdog reset flag */
#define RCC_CTRLSTS_WWDGRSTF   ((uint32_t)0x00000080) /*!< Window watchdog reset flag */

/*******************  Bit definition for RCC_AHBPRST register  ****************/
#define RCC_AHBRST_IRCRST    ((uint32_t)0x00004000) /*!< IRC reset */
#define RCC_AHBRST_ADCRST    ((uint32_t)0x00001000) /*!< ADC reset */

/*******************  Bit definition for RCC_CFG2 register  ******************/
/*!< ADCHPRE configuration */
#define RCC_CFG2_ADCHPRES   ((uint32_t)0x00000010) /*!< ADCHPRE[3:0] bits */
#define RCC_CFG2_ADCHPRES_DIV8   ((uint32_t)0x00000005) /*!< HCLK clock divided by 8 */
#define RCC_CFG2_ADCHPRES_DIV10  ((uint32_t)0x00000006) /*!< HCLK clock divided by 10 */
#define RCC_CFG2_ADCHPRES_DIV12  ((uint32_t)0x00000007) /*!< HCLK clock divided by 12 */
#define RCC_CFG2_ADCHPRES_DIV16  ((uint32_t)0x00000008) /*!< HCLK clock divided by 16 */
#define RCC_CFG2_ADCHPRES_DIV32  ((uint32_t)0x00000009) /*!< HCLK clock divided by 32 */
#define RCC_CFG2_ADCHPRES_OTHERS ((uint32_t)0x00000009) /*!< HCLK clock divided by 32 */

/*!< ADC1MSEL configuration */
#define RCC_CFG2_ADCSEL          ((uint32_t)0x00000010) /*!< ADC clock source select */
#define RCC_CFG2_ADCSEL_AUDIOPLL ((uint32_t)0x00000000) /*!< AUDIOPLL clock selected as ADC input clock */
#define RCC_CFG2_ADCSEL_HSE_DIV8 ((uint32_t)0x00000010) /*!< HSE_DIV8 clock selected as ADC input clock */

/*!< TIMCLK_SEL configuration */
#define RCC_CFG2_TIMCLKSEL ((uint32_t)0x80000000) /*!< Timer1 clock source select */
#define RCC_CFG2_TIMCLKSEL_TIM1CLK  ((uint32_t)0x00000000) /*!< Timer1 clock selected as tim1_clk input clock */
#define RCC_CFG2_TIMCLKSEL_SYSCLK   ((uint32_t)0x80000000) /*!< Timer1 clock selected as sysclk input clock */

/*!< ROMCCTRL configuration */
#define RCC_ROMCCTRL_READCYCLESEL   ((uint32_t)0x00000020) /*!< ROMCReadcycle select */
#define RCC_ROMCCTRL_READCYCLE_ONE  ((uint32_t)0x00000000) /*!< one */
#define RCC_ROMCCTRL_READCYCLE_TWO  ((uint32_t)0x00000020) /*!< two */

/********************  Bit definition for RCC_DBGCTRL register  ********************/
#define RCC_DBGCTRL_IWDGSTP                           ((uint32_t)0x00000001)     /*!< when enter debug mode,IWDG work mode selection*/
#define RCC_DBGCTRL_WWDGSTP                           ((uint32_t)0x00000002)     /*!< when enter debug mode,WWDG work mode selection*/
#define RCC_DBGCTRL_TIM1STP                           ((uint32_t)0x00000003)     /*!< when enter debug mode,TIM1 work mode selection*/
#define RCC_DBGCTRL_TIM3STP                           ((uint32_t)0x00000004)     /*!< when enter debug mode,TIM3 work mode selection*/
#define RCC_DBGCTRL_TIM6STP                           ((uint32_t)0x00000010)     /*!< when enter debug mode,TIM6 work mode selection*/
#define RCC_DBGCTRL_I2C1TIMOUT                        ((uint32_t)0x00000020)     /*!< when enter debug mode,I2C1 timeout work mode selection*/

/******************************************************************************/
/*                                                                            */
/*                               SystemTick                                   */
/*                                                                            */
/******************************************************************************/

/*****************  Bit definition for SysTick_CTRL register  *****************/
#define SysTick_CTRL_ENABLE    ((uint32_t)0x00000001) /*!< Counter enable */
#define SysTick_CTRL_TICKINT   ((uint32_t)0x00000002) /*!< Counting down to 0 pends the SysTick handler */
#define SysTick_CTRL_CLKSOURCE ((uint32_t)0x00000004) /*!< Clock source */
#define SysTick_CTRL_COUNTFLAG ((uint32_t)0x00010000) /*!< Count Flag */

/*****************  Bit definition for SysTick_LOAD register  *****************/
#define SysTick_LOAD_RELOAD                                                                                            \
    ((uint32_t)0x00FFFFFF) /*!< Value to load into the SysTick Current Value Register when the counter reaches 0 */

/*****************  Bit definition for SysTick_VAL register  ******************/
#define SysTick_VAL_CURRENT ((uint32_t)0x00FFFFFF) /*!< Current value at the time the register is accessed */

/*****************  Bit definition for SysTick_CALIB register  ****************/
#define SysTick_CALIB_TENMS ((uint32_t)0x00FFFFFF) /*!< Reload value to use for 10ms timing */
#define SysTick_CALIB_SKEW  ((uint32_t)0x40000000) /*!< Calibration value is not exactly 10 ms */
#define SysTick_CALIB_NOREF ((uint32_t)0x80000000) /*!< The reference clock is not provided */

/******************************************************************************/
/*                                                                            */
/*                  Nested Vectored Interrupt Controller                      */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for NVIC_ISER register  *******************/
#define NVIC_ISER_SETENA    ((uint32_t)0xFFFFFFFF) /*!< Interrupt set enable bits */
#define NVIC_ISER_SETENA_0  ((uint32_t)0x00000001) /*!< bit 0 */
#define NVIC_ISER_SETENA_1  ((uint32_t)0x00000002) /*!< bit 1 */
#define NVIC_ISER_SETENA_2  ((uint32_t)0x00000004) /*!< bit 2 */
#define NVIC_ISER_SETENA_3  ((uint32_t)0x00000008) /*!< bit 3 */
#define NVIC_ISER_SETENA_4  ((uint32_t)0x00000010) /*!< bit 4 */
#define NVIC_ISER_SETENA_5  ((uint32_t)0x00000020) /*!< bit 5 */
#define NVIC_ISER_SETENA_6  ((uint32_t)0x00000040) /*!< bit 6 */
#define NVIC_ISER_SETENA_7  ((uint32_t)0x00000080) /*!< bit 7 */
#define NVIC_ISER_SETENA_8  ((uint32_t)0x00000100) /*!< bit 8 */
#define NVIC_ISER_SETENA_9  ((uint32_t)0x00000200) /*!< bit 9 */
#define NVIC_ISER_SETENA_10 ((uint32_t)0x00000400) /*!< bit 10 */
#define NVIC_ISER_SETENA_11 ((uint32_t)0x00000800) /*!< bit 11 */
#define NVIC_ISER_SETENA_12 ((uint32_t)0x00001000) /*!< bit 12 */
#define NVIC_ISER_SETENA_13 ((uint32_t)0x00002000) /*!< bit 13 */
#define NVIC_ISER_SETENA_14 ((uint32_t)0x00004000) /*!< bit 14 */
#define NVIC_ISER_SETENA_15 ((uint32_t)0x00008000) /*!< bit 15 */
#define NVIC_ISER_SETENA_16 ((uint32_t)0x00010000) /*!< bit 16 */
#define NVIC_ISER_SETENA_17 ((uint32_t)0x00020000) /*!< bit 17 */
#define NVIC_ISER_SETENA_18 ((uint32_t)0x00040000) /*!< bit 18 */
#define NVIC_ISER_SETENA_19 ((uint32_t)0x00080000) /*!< bit 19 */
#define NVIC_ISER_SETENA_20 ((uint32_t)0x00100000) /*!< bit 20 */
#define NVIC_ISER_SETENA_21 ((uint32_t)0x00200000) /*!< bit 21 */
#define NVIC_ISER_SETENA_22 ((uint32_t)0x00400000) /*!< bit 22 */
#define NVIC_ISER_SETENA_23 ((uint32_t)0x00800000) /*!< bit 23 */
#define NVIC_ISER_SETENA_24 ((uint32_t)0x01000000) /*!< bit 24 */
#define NVIC_ISER_SETENA_25 ((uint32_t)0x02000000) /*!< bit 25 */
#define NVIC_ISER_SETENA_26 ((uint32_t)0x04000000) /*!< bit 26 */
#define NVIC_ISER_SETENA_27 ((uint32_t)0x08000000) /*!< bit 27 */
#define NVIC_ISER_SETENA_28 ((uint32_t)0x10000000) /*!< bit 28 */
#define NVIC_ISER_SETENA_29 ((uint32_t)0x20000000) /*!< bit 29 */
#define NVIC_ISER_SETENA_30 ((uint32_t)0x40000000) /*!< bit 30 */
#define NVIC_ISER_SETENA_31 ((uint32_t)0x80000000) /*!< bit 31 */

/******************  Bit definition for NVIC_ICER register  *******************/
#define NVIC_ICER_CLRENA    ((uint32_t)0xFFFFFFFF) /*!< Interrupt clear-enable bits */
#define NVIC_ICER_CLRENA_0  ((uint32_t)0x00000001) /*!< bit 0 */
#define NVIC_ICER_CLRENA_1  ((uint32_t)0x00000002) /*!< bit 1 */
#define NVIC_ICER_CLRENA_2  ((uint32_t)0x00000004) /*!< bit 2 */
#define NVIC_ICER_CLRENA_3  ((uint32_t)0x00000008) /*!< bit 3 */
#define NVIC_ICER_CLRENA_4  ((uint32_t)0x00000010) /*!< bit 4 */
#define NVIC_ICER_CLRENA_5  ((uint32_t)0x00000020) /*!< bit 5 */
#define NVIC_ICER_CLRENA_6  ((uint32_t)0x00000040) /*!< bit 6 */
#define NVIC_ICER_CLRENA_7  ((uint32_t)0x00000080) /*!< bit 7 */
#define NVIC_ICER_CLRENA_8  ((uint32_t)0x00000100) /*!< bit 8 */
#define NVIC_ICER_CLRENA_9  ((uint32_t)0x00000200) /*!< bit 9 */
#define NVIC_ICER_CLRENA_10 ((uint32_t)0x00000400) /*!< bit 10 */
#define NVIC_ICER_CLRENA_11 ((uint32_t)0x00000800) /*!< bit 11 */
#define NVIC_ICER_CLRENA_12 ((uint32_t)0x00001000) /*!< bit 12 */
#define NVIC_ICER_CLRENA_13 ((uint32_t)0x00002000) /*!< bit 13 */
#define NVIC_ICER_CLRENA_14 ((uint32_t)0x00004000) /*!< bit 14 */
#define NVIC_ICER_CLRENA_15 ((uint32_t)0x00008000) /*!< bit 15 */
#define NVIC_ICER_CLRENA_16 ((uint32_t)0x00010000) /*!< bit 16 */
#define NVIC_ICER_CLRENA_17 ((uint32_t)0x00020000) /*!< bit 17 */
#define NVIC_ICER_CLRENA_18 ((uint32_t)0x00040000) /*!< bit 18 */
#define NVIC_ICER_CLRENA_19 ((uint32_t)0x00080000) /*!< bit 19 */
#define NVIC_ICER_CLRENA_20 ((uint32_t)0x00100000) /*!< bit 20 */
#define NVIC_ICER_CLRENA_21 ((uint32_t)0x00200000) /*!< bit 21 */
#define NVIC_ICER_CLRENA_22 ((uint32_t)0x00400000) /*!< bit 22 */
#define NVIC_ICER_CLRENA_23 ((uint32_t)0x00800000) /*!< bit 23 */
#define NVIC_ICER_CLRENA_24 ((uint32_t)0x01000000) /*!< bit 24 */
#define NVIC_ICER_CLRENA_25 ((uint32_t)0x02000000) /*!< bit 25 */
#define NVIC_ICER_CLRENA_26 ((uint32_t)0x04000000) /*!< bit 26 */
#define NVIC_ICER_CLRENA_27 ((uint32_t)0x08000000) /*!< bit 27 */
#define NVIC_ICER_CLRENA_28 ((uint32_t)0x10000000) /*!< bit 28 */
#define NVIC_ICER_CLRENA_29 ((uint32_t)0x20000000) /*!< bit 29 */
#define NVIC_ICER_CLRENA_30 ((uint32_t)0x40000000) /*!< bit 30 */
#define NVIC_ICER_CLRENA_31 ((uint32_t)0x80000000) /*!< bit 31 */

/******************  Bit definition for NVIC_ISPR register  *******************/
#define NVIC_ISPR_SETPEND    ((uint32_t)0xFFFFFFFF) /*!< Interrupt set-pending bits */
#define NVIC_ISPR_SETPEND_0  ((uint32_t)0x00000001) /*!< bit 0 */
#define NVIC_ISPR_SETPEND_1  ((uint32_t)0x00000002) /*!< bit 1 */
#define NVIC_ISPR_SETPEND_2  ((uint32_t)0x00000004) /*!< bit 2 */
#define NVIC_ISPR_SETPEND_3  ((uint32_t)0x00000008) /*!< bit 3 */
#define NVIC_ISPR_SETPEND_4  ((uint32_t)0x00000010) /*!< bit 4 */
#define NVIC_ISPR_SETPEND_5  ((uint32_t)0x00000020) /*!< bit 5 */
#define NVIC_ISPR_SETPEND_6  ((uint32_t)0x00000040) /*!< bit 6 */
#define NVIC_ISPR_SETPEND_7  ((uint32_t)0x00000080) /*!< bit 7 */
#define NVIC_ISPR_SETPEND_8  ((uint32_t)0x00000100) /*!< bit 8 */
#define NVIC_ISPR_SETPEND_9  ((uint32_t)0x00000200) /*!< bit 9 */
#define NVIC_ISPR_SETPEND_10 ((uint32_t)0x00000400) /*!< bit 10 */
#define NVIC_ISPR_SETPEND_11 ((uint32_t)0x00000800) /*!< bit 11 */
#define NVIC_ISPR_SETPEND_12 ((uint32_t)0x00001000) /*!< bit 12 */
#define NVIC_ISPR_SETPEND_13 ((uint32_t)0x00002000) /*!< bit 13 */
#define NVIC_ISPR_SETPEND_14 ((uint32_t)0x00004000) /*!< bit 14 */
#define NVIC_ISPR_SETPEND_15 ((uint32_t)0x00008000) /*!< bit 15 */
#define NVIC_ISPR_SETPEND_16 ((uint32_t)0x00010000) /*!< bit 16 */
#define NVIC_ISPR_SETPEND_17 ((uint32_t)0x00020000) /*!< bit 17 */
#define NVIC_ISPR_SETPEND_18 ((uint32_t)0x00040000) /*!< bit 18 */
#define NVIC_ISPR_SETPEND_19 ((uint32_t)0x00080000) /*!< bit 19 */
#define NVIC_ISPR_SETPEND_20 ((uint32_t)0x00100000) /*!< bit 20 */
#define NVIC_ISPR_SETPEND_21 ((uint32_t)0x00200000) /*!< bit 21 */
#define NVIC_ISPR_SETPEND_22 ((uint32_t)0x00400000) /*!< bit 22 */
#define NVIC_ISPR_SETPEND_23 ((uint32_t)0x00800000) /*!< bit 23 */
#define NVIC_ISPR_SETPEND_24 ((uint32_t)0x01000000) /*!< bit 24 */
#define NVIC_ISPR_SETPEND_25 ((uint32_t)0x02000000) /*!< bit 25 */
#define NVIC_ISPR_SETPEND_26 ((uint32_t)0x04000000) /*!< bit 26 */
#define NVIC_ISPR_SETPEND_27 ((uint32_t)0x08000000) /*!< bit 27 */
#define NVIC_ISPR_SETPEND_28 ((uint32_t)0x10000000) /*!< bit 28 */
#define NVIC_ISPR_SETPEND_29 ((uint32_t)0x20000000) /*!< bit 29 */
#define NVIC_ISPR_SETPEND_30 ((uint32_t)0x40000000) /*!< bit 30 */
#define NVIC_ISPR_SETPEND_31 ((uint32_t)0x80000000) /*!< bit 31 */

/******************  Bit definition for NVIC_ICPR register  *******************/
#define NVIC_ICPR_CLRPEND    ((uint32_t)0xFFFFFFFF) /*!< Interrupt clear-pending bits */
#define NVIC_ICPR_CLRPEND_0  ((uint32_t)0x00000001) /*!< bit 0 */
#define NVIC_ICPR_CLRPEND_1  ((uint32_t)0x00000002) /*!< bit 1 */
#define NVIC_ICPR_CLRPEND_2  ((uint32_t)0x00000004) /*!< bit 2 */
#define NVIC_ICPR_CLRPEND_3  ((uint32_t)0x00000008) /*!< bit 3 */
#define NVIC_ICPR_CLRPEND_4  ((uint32_t)0x00000010) /*!< bit 4 */
#define NVIC_ICPR_CLRPEND_5  ((uint32_t)0x00000020) /*!< bit 5 */
#define NVIC_ICPR_CLRPEND_6  ((uint32_t)0x00000040) /*!< bit 6 */
#define NVIC_ICPR_CLRPEND_7  ((uint32_t)0x00000080) /*!< bit 7 */
#define NVIC_ICPR_CLRPEND_8  ((uint32_t)0x00000100) /*!< bit 8 */
#define NVIC_ICPR_CLRPEND_9  ((uint32_t)0x00000200) /*!< bit 9 */
#define NVIC_ICPR_CLRPEND_10 ((uint32_t)0x00000400) /*!< bit 10 */
#define NVIC_ICPR_CLRPEND_11 ((uint32_t)0x00000800) /*!< bit 11 */
#define NVIC_ICPR_CLRPEND_12 ((uint32_t)0x00001000) /*!< bit 12 */
#define NVIC_ICPR_CLRPEND_13 ((uint32_t)0x00002000) /*!< bit 13 */
#define NVIC_ICPR_CLRPEND_14 ((uint32_t)0x00004000) /*!< bit 14 */
#define NVIC_ICPR_CLRPEND_15 ((uint32_t)0x00008000) /*!< bit 15 */
#define NVIC_ICPR_CLRPEND_16 ((uint32_t)0x00010000) /*!< bit 16 */
#define NVIC_ICPR_CLRPEND_17 ((uint32_t)0x00020000) /*!< bit 17 */
#define NVIC_ICPR_CLRPEND_18 ((uint32_t)0x00040000) /*!< bit 18 */
#define NVIC_ICPR_CLRPEND_19 ((uint32_t)0x00080000) /*!< bit 19 */
#define NVIC_ICPR_CLRPEND_20 ((uint32_t)0x00100000) /*!< bit 20 */
#define NVIC_ICPR_CLRPEND_21 ((uint32_t)0x00200000) /*!< bit 21 */
#define NVIC_ICPR_CLRPEND_22 ((uint32_t)0x00400000) /*!< bit 22 */
#define NVIC_ICPR_CLRPEND_23 ((uint32_t)0x00800000) /*!< bit 23 */
#define NVIC_ICPR_CLRPEND_24 ((uint32_t)0x01000000) /*!< bit 24 */
#define NVIC_ICPR_CLRPEND_25 ((uint32_t)0x02000000) /*!< bit 25 */
#define NVIC_ICPR_CLRPEND_26 ((uint32_t)0x04000000) /*!< bit 26 */
#define NVIC_ICPR_CLRPEND_27 ((uint32_t)0x08000000) /*!< bit 27 */
#define NVIC_ICPR_CLRPEND_28 ((uint32_t)0x10000000) /*!< bit 28 */
#define NVIC_ICPR_CLRPEND_29 ((uint32_t)0x20000000) /*!< bit 29 */
#define NVIC_ICPR_CLRPEND_30 ((uint32_t)0x40000000) /*!< bit 30 */
#define NVIC_ICPR_CLRPEND_31 ((uint32_t)0x80000000) /*!< bit 31 */

/******************  Bit definition for NVIC_IABR register  *******************/
#define NVIC_IABR_ACTIVE    ((uint32_t)0xFFFFFFFF) /*!< Interrupt active flags */
#define NVIC_IABR_ACTIVE_0  ((uint32_t)0x00000001) /*!< bit 0 */
#define NVIC_IABR_ACTIVE_1  ((uint32_t)0x00000002) /*!< bit 1 */
#define NVIC_IABR_ACTIVE_2  ((uint32_t)0x00000004) /*!< bit 2 */
#define NVIC_IABR_ACTIVE_3  ((uint32_t)0x00000008) /*!< bit 3 */
#define NVIC_IABR_ACTIVE_4  ((uint32_t)0x00000010) /*!< bit 4 */
#define NVIC_IABR_ACTIVE_5  ((uint32_t)0x00000020) /*!< bit 5 */
#define NVIC_IABR_ACTIVE_6  ((uint32_t)0x00000040) /*!< bit 6 */
#define NVIC_IABR_ACTIVE_7  ((uint32_t)0x00000080) /*!< bit 7 */
#define NVIC_IABR_ACTIVE_8  ((uint32_t)0x00000100) /*!< bit 8 */
#define NVIC_IABR_ACTIVE_9  ((uint32_t)0x00000200) /*!< bit 9 */
#define NVIC_IABR_ACTIVE_10 ((uint32_t)0x00000400) /*!< bit 10 */
#define NVIC_IABR_ACTIVE_11 ((uint32_t)0x00000800) /*!< bit 11 */
#define NVIC_IABR_ACTIVE_12 ((uint32_t)0x00001000) /*!< bit 12 */
#define NVIC_IABR_ACTIVE_13 ((uint32_t)0x00002000) /*!< bit 13 */
#define NVIC_IABR_ACTIVE_14 ((uint32_t)0x00004000) /*!< bit 14 */
#define NVIC_IABR_ACTIVE_15 ((uint32_t)0x00008000) /*!< bit 15 */
#define NVIC_IABR_ACTIVE_16 ((uint32_t)0x00010000) /*!< bit 16 */
#define NVIC_IABR_ACTIVE_17 ((uint32_t)0x00020000) /*!< bit 17 */
#define NVIC_IABR_ACTIVE_18 ((uint32_t)0x00040000) /*!< bit 18 */
#define NVIC_IABR_ACTIVE_19 ((uint32_t)0x00080000) /*!< bit 19 */
#define NVIC_IABR_ACTIVE_20 ((uint32_t)0x00100000) /*!< bit 20 */
#define NVIC_IABR_ACTIVE_21 ((uint32_t)0x00200000) /*!< bit 21 */
#define NVIC_IABR_ACTIVE_22 ((uint32_t)0x00400000) /*!< bit 22 */
#define NVIC_IABR_ACTIVE_23 ((uint32_t)0x00800000) /*!< bit 23 */
#define NVIC_IABR_ACTIVE_24 ((uint32_t)0x01000000) /*!< bit 24 */
#define NVIC_IABR_ACTIVE_25 ((uint32_t)0x02000000) /*!< bit 25 */
#define NVIC_IABR_ACTIVE_26 ((uint32_t)0x04000000) /*!< bit 26 */
#define NVIC_IABR_ACTIVE_27 ((uint32_t)0x08000000) /*!< bit 27 */
#define NVIC_IABR_ACTIVE_28 ((uint32_t)0x10000000) /*!< bit 28 */
#define NVIC_IABR_ACTIVE_29 ((uint32_t)0x20000000) /*!< bit 29 */
#define NVIC_IABR_ACTIVE_30 ((uint32_t)0x40000000) /*!< bit 30 */
#define NVIC_IABR_ACTIVE_31 ((uint32_t)0x80000000) /*!< bit 31 */

/******************  Bit definition for NVIC_PRI0 register  *******************/
#define NVIC_IPR0_PRI_0 ((uint32_t)0x000000FF) /*!< Priority of interrupt 0 */
#define NVIC_IPR0_PRI_1 ((uint32_t)0x0000FF00) /*!< Priority of interrupt 1 */
#define NVIC_IPR0_PRI_2 ((uint32_t)0x00FF0000) /*!< Priority of interrupt 2 */
#define NVIC_IPR0_PRI_3 ((uint32_t)0xFF000000) /*!< Priority of interrupt 3 */

/******************  Bit definition for NVIC_PRI1 register  *******************/
#define NVIC_IPR1_PRI_4 ((uint32_t)0x000000FF) /*!< Priority of interrupt 4 */
#define NVIC_IPR1_PRI_5 ((uint32_t)0x0000FF00) /*!< Priority of interrupt 5 */
#define NVIC_IPR1_PRI_6 ((uint32_t)0x00FF0000) /*!< Priority of interrupt 6 */
#define NVIC_IPR1_PRI_7 ((uint32_t)0xFF000000) /*!< Priority of interrupt 7 */

/******************  Bit definition for NVIC_PRI2 register  *******************/
#define NVIC_IPR2_PRI_8  ((uint32_t)0x000000FF) /*!< Priority of interrupt 8 */
#define NVIC_IPR2_PRI_9  ((uint32_t)0x0000FF00) /*!< Priority of interrupt 9 */
#define NVIC_IPR2_PRI_10 ((uint32_t)0x00FF0000) /*!< Priority of interrupt 10 */
#define NVIC_IPR2_PRI_11 ((uint32_t)0xFF000000) /*!< Priority of interrupt 11 */

/******************  Bit definition for NVIC_PRI3 register  *******************/
#define NVIC_IPR3_PRI_12 ((uint32_t)0x000000FF) /*!< Priority of interrupt 12 */
#define NVIC_IPR3_PRI_13 ((uint32_t)0x0000FF00) /*!< Priority of interrupt 13 */
#define NVIC_IPR3_PRI_14 ((uint32_t)0x00FF0000) /*!< Priority of interrupt 14 */
#define NVIC_IPR3_PRI_15 ((uint32_t)0xFF000000) /*!< Priority of interrupt 15 */

/******************  Bit definition for NVIC_PRI4 register  *******************/
#define NVIC_IPR4_PRI_16 ((uint32_t)0x000000FF) /*!< Priority of interrupt 16 */
#define NVIC_IPR4_PRI_17 ((uint32_t)0x0000FF00) /*!< Priority of interrupt 17 */
#define NVIC_IPR4_PRI_18 ((uint32_t)0x00FF0000) /*!< Priority of interrupt 18 */
#define NVIC_IPR4_PRI_19 ((uint32_t)0xFF000000) /*!< Priority of interrupt 19 */

/******************  Bit definition for NVIC_PRI5 register  *******************/
#define NVIC_IPR5_PRI_20 ((uint32_t)0x000000FF) /*!< Priority of interrupt 20 */
#define NVIC_IPR5_PRI_21 ((uint32_t)0x0000FF00) /*!< Priority of interrupt 21 */
#define NVIC_IPR5_PRI_22 ((uint32_t)0x00FF0000) /*!< Priority of interrupt 22 */
#define NVIC_IPR5_PRI_23 ((uint32_t)0xFF000000) /*!< Priority of interrupt 23 */

/******************  Bit definition for NVIC_PRI6 register  *******************/
#define NVIC_IPR6_PRI_24 ((uint32_t)0x000000FF) /*!< Priority of interrupt 24 */
#define NVIC_IPR6_PRI_25 ((uint32_t)0x0000FF00) /*!< Priority of interrupt 25 */
#define NVIC_IPR6_PRI_26 ((uint32_t)0x00FF0000) /*!< Priority of interrupt 26 */
#define NVIC_IPR6_PRI_27 ((uint32_t)0xFF000000) /*!< Priority of interrupt 27 */

/******************  Bit definition for NVIC_PRI7 register  *******************/
#define NVIC_IPR7_PRI_28 ((uint32_t)0x000000FF) /*!< Priority of interrupt 28 */
#define NVIC_IPR7_PRI_29 ((uint32_t)0x0000FF00) /*!< Priority of interrupt 29 */
#define NVIC_IPR7_PRI_30 ((uint32_t)0x00FF0000) /*!< Priority of interrupt 30 */
#define NVIC_IPR7_PRI_31 ((uint32_t)0xFF000000) /*!< Priority of interrupt 31 */

/******************  Bit definition for SCB_CPUID register  *******************/
#define SCB_CPUID_REVISION    ((uint32_t)0x0000000F) /*!< Implementation defined revision number */
#define SCB_CPUID_PARTNO      ((uint32_t)0x0000FFF0) /*!< Number of processor within family */
#define SCB_CPUID_Constant    ((uint32_t)0x000F0000) /*!< Reads as 0x0F */
#define SCB_CPUID_VARIANT     ((uint32_t)0x00F00000) /*!< Implementation defined variant number */
#define SCB_CPUID_IMPLEMENTER ((uint32_t)0xFF000000) /*!< Implementer code. ARM is 0x41 */

/*******************  Bit definition for SCB_ICSR register  *******************/
#define SCB_ICSR_VECTACTIVE ((uint32_t)0x000001FF) /*!< Active INTSTS number field */
#define SCB_ICSR_RETTOBASE                                                                                             \
    ((uint32_t)0x00000800) /*!< All active exceptions minus the IPSR_current_exception yields the empty set */
#define SCB_ICSR_VECTPENDING ((uint32_t)0x003FF000) /*!< Pending INTSTS number field */
#define SCB_ICSR_ISRPENDING  ((uint32_t)0x00400000) /*!< Interrupt pending flag */
#define SCB_ICSR_ISRPREEMPT                                                                                            \
    ((uint32_t)0x00800000) /*!< It indicates that a pending interrupt becomes active in the next running cycle */
#define SCB_ICSR_PENDSTCLR  ((uint32_t)0x02000000) /*!< Clear pending SysTick bit */
#define SCB_ICSR_PENDSTSET  ((uint32_t)0x04000000) /*!< Set pending SysTick bit */
#define SCB_ICSR_PENDSVCLR  ((uint32_t)0x08000000) /*!< Clear pending pendSV bit */
#define SCB_ICSR_PENDSVSET  ((uint32_t)0x10000000) /*!< Set pending pendSV bit */
#define SCB_ICSR_NMIPENDSET ((uint32_t)0x80000000) /*!< Set pending NMI bit */

/*!<*****************  Bit definition for SCB_AIRCR register  *******************/
#define SCB_AIRCR_VECTRESET     ((uint32_t)0x00000001) /*!< System Reset bit */
#define SCB_AIRCR_VECTCLRACTIVE ((uint32_t)0x00000002) /*!< Clear active vector bit */
#define SCB_AIRCR_SYSRESETREQ   ((uint32_t)0x00000004) /*!< Requests chip control logic to generate a reset */

#define SCB_AIRCR_PRIGROUP   ((uint32_t)0x00000700) /*!< PRIGROUP[2:0] bits (Priority group) */
#define SCB_AIRCR_PRIGROUP_0 ((uint32_t)0x00000100) /*!< Bit 0 */
#define SCB_AIRCR_PRIGROUP_1 ((uint32_t)0x00000200) /*!< Bit 1 */
#define SCB_AIRCR_PRIGROUP_2 ((uint32_t)0x00000400) /*!< Bit 2  */

/* prority group configuration */
#define SCB_AIRCR_PRIGROUP0                                                                                            \
    ((uint32_t)0x00000000) /*!< Priority group=0 (7 bits of pre-emption priority, 1 bit of subpriority) */
#define SCB_AIRCR_PRIGROUP1                                                                                            \
    ((uint32_t)0x00000100) /*!< Priority group=1 (6 bits of pre-emption priority, 2 bits of subpriority) */
#define SCB_AIRCR_PRIGROUP2                                                                                            \
    ((uint32_t)0x00000200) /*!< Priority group=2 (5 bits of pre-emption priority, 3 bits of subpriority) */
#define SCB_AIRCR_PRIGROUP3                                                                                            \
    ((uint32_t)0x00000300) /*!< Priority group=3 (4 bits of pre-emption priority, 4 bits of subpriority) */
#define SCB_AIRCR_PRIGROUP4                                                                                            \
    ((uint32_t)0x00000400) /*!< Priority group=4 (3 bits of pre-emption priority, 5 bits of subpriority) */
#define SCB_AIRCR_PRIGROUP5                                                                                            \
    ((uint32_t)0x00000500) /*!< Priority group=5 (2 bits of pre-emption priority, 6 bits of subpriority) */
#define SCB_AIRCR_PRIGROUP6                                                                                            \
    ((uint32_t)0x00000600) /*!< Priority group=6 (1 bit of pre-emption priority, 7 bits of subpriority) */
#define SCB_AIRCR_PRIGROUP7                                                                                            \
    ((uint32_t)0x00000700) /*!< Priority group=7 (no pre-emption priority, 8 bits of subpriority) */

#define SCB_AIRCR_ENDIANESS ((uint32_t)0x00008000) /*!< Data endianness bit */
#define SCB_AIRCR_VECTKEY   ((uint32_t)0xFFFF0000) /*!< Register key (VECTKEY) - Reads as 0xFA05 (VECTKEYSTAT) */

/*******************  Bit definition for SCB_SCR register  ********************/
#define SCB_SCR_SLEEPONEXIT ((uint8_t)0x02) /*!< Sleep on exit bit */
#define SCB_SCR_SLEEPDEEP   ((uint8_t)0x04) /*!< Sleep deep bit */
#define SCB_SCR_SEVONPEND   ((uint8_t)0x10) /*!< Wake up from WFE */

/********************  Bit definition for SCB_CCR register  *******************/
#define SCB_CCR_NONBASETHRDENA                                                                                         \
    ((uint16_t)0x0001) /*!< Thread mode can be entered from any level in Handler mode by controlled return value */
#define SCB_CCR_USERSETMPEND                                                                                           \
    ((uint16_t)0x0002) /*!< Enables user code to write the Software Trigger Interrupt register to trigger (pend) a     \
                          Main exception */
#define SCB_CCR_UNALIGN_TRP ((uint16_t)0x0008) /*!< Trap for unaligned access */
#define SCB_CCR_DIV_0_TRP   ((uint16_t)0x0010) /*!< Trap on Divide by 0 */
#define SCB_CCR_BFHFNMIGN   ((uint16_t)0x0100) /*!< Handlers running at priority -1 and -2 */
#define SCB_CCR_STKALIGN                                                                                               \
    ((uint16_t)0x0200) /*!< On exception entry, the SP used prior to the exception is adjusted to be 8-byte aligned */

/*******************  Bit definition for SCB_SHPR register ********************/
#define SCB_SHPR_PRI_N                                                                                                 \
    ((uint32_t)0x000000FF) /*!< Priority of system handler 4,8, and 12. Mem Manage, reserved and Debug Monitor */
#define SCB_SHPR_PRI_N1                                                                                                \
    ((uint32_t)0x0000FF00) /*!< Priority of system handler 5,9, and 13. Bus Fault, reserved and reserved */
#define SCB_SHPR_PRI_N2                                                                                                \
    ((uint32_t)0x00FF0000) /*!< Priority of system handler 6,10, and 14. Usage Fault, reserved and PendSV */
#define SCB_SHPR_PRI_N3                                                                                                \
    ((uint32_t)0xFF000000) /*!< Priority of system handler 7,11, and 15. Reserved, SVCall and SysTick */

/******************  Bit definition for SCB_SHCSR register  *******************/
#define SCB_SHCSR_MEMFAULTACT    ((uint32_t)0x00000001) /*!< MemManage is active */
#define SCB_SHCSR_BUSFAULTACT    ((uint32_t)0x00000002) /*!< BusFault is active */
#define SCB_SHCSR_USGFAULTACT    ((uint32_t)0x00000008) /*!< UsageFault is active */
#define SCB_SHCSR_SVCALLACT      ((uint32_t)0x00000080) /*!< SVCall is active */
#define SCB_SHCSR_MONITORACT     ((uint32_t)0x00000100) /*!< Monitor is active */
#define SCB_SHCSR_PENDSVACT      ((uint32_t)0x00000400) /*!< PendSV is active */
#define SCB_SHCSR_SYSTICKACT     ((uint32_t)0x00000800) /*!< SysTick is active */
#define SCB_SHCSR_USGFAULTPENDED ((uint32_t)0x00001000) /*!< Usage Fault is pended */
#define SCB_SHCSR_MEMFAULTPENDED ((uint32_t)0x00002000) /*!< MemManage is pended */
#define SCB_SHCSR_BUSFAULTPENDED ((uint32_t)0x00004000) /*!< Bus Fault is pended */
#define SCB_SHCSR_SVCALLPENDED   ((uint32_t)0x00008000) /*!< SVCall is pended */
#define SCB_SHCSR_MEMFAULTENA    ((uint32_t)0x00010000) /*!< MemManage enable */
#define SCB_SHCSR_BUSFAULTENA    ((uint32_t)0x00020000) /*!< Bus Fault enable */
#define SCB_SHCSR_USGFAULTENA    ((uint32_t)0x00040000) /*!< UsageFault enable */

/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for DMA_INTSTS register  ********************/
#define DMA_INTSTS_GLBF1 ((uint32_t)0x00000001) /*!< Channel 1 Global interrupt flag */
#define DMA_INTSTS_TXCF1 ((uint32_t)0x00000002) /*!< Channel 1 Transfer Complete flag */
#define DMA_INTSTS_HTXF1 ((uint32_t)0x00000004) /*!< Channel 1 Half Transfer flag */
#define DMA_INTSTS_ERRF1 ((uint32_t)0x00000008) /*!< Channel 1 Transfer Error flag */
#define DMA_INTSTS_GLBF2 ((uint32_t)0x00000010) /*!< Channel 2 Global interrupt flag */
#define DMA_INTSTS_TXCF2 ((uint32_t)0x00000020) /*!< Channel 2 Transfer Complete flag */
#define DMA_INTSTS_HTXF2 ((uint32_t)0x00000040) /*!< Channel 2 Half Transfer flag */
#define DMA_INTSTS_ERRF2 ((uint32_t)0x00000080) /*!< Channel 2 Transfer Error flag */
#define DMA_INTSTS_GLBF3 ((uint32_t)0x00000100) /*!< Channel 3 Global interrupt flag */
#define DMA_INTSTS_TXCF3 ((uint32_t)0x00000200) /*!< Channel 3 Transfer Complete flag */
#define DMA_INTSTS_HTXF3 ((uint32_t)0x00000400) /*!< Channel 3 Half Transfer flag */
#define DMA_INTSTS_ERRF3 ((uint32_t)0x00000800) /*!< Channel 3 Transfer Error flag */
#define DMA_INTSTS_GLBF4 ((uint32_t)0x00001000) /*!< Channel 4 Global interrupt flag */
#define DMA_INTSTS_TXCF4 ((uint32_t)0x00002000) /*!< Channel 4 Transfer Complete flag */
#define DMA_INTSTS_HTXF4 ((uint32_t)0x00004000) /*!< Channel 4 Half Transfer flag */
#define DMA_INTSTS_ERRF4 ((uint32_t)0x00008000) /*!< Channel 4 Transfer Error flag */
#define DMA_INTSTS_GLBF5 ((uint32_t)0x00010000) /*!< Channel 5 Global interrupt flag */
#define DMA_INTSTS_TXCF5 ((uint32_t)0x00020000) /*!< Channel 5 Transfer Complete flag */
#define DMA_INTSTS_HTXF5 ((uint32_t)0x00040000) /*!< Channel 5 Half Transfer flag */
#define DMA_INTSTS_ERRF5 ((uint32_t)0x00080000) /*!< Channel 5 Transfer Error flag */

/*******************  Bit definition for DMA_INTCLR register  *******************/
#define DMA_INTCLR_CGLBF1 ((uint32_t)0x00000001) /*!< Channel 1 Global interrupt clear */
#define DMA_INTCLR_CTXCF1 ((uint32_t)0x00000002) /*!< Channel 1 Transfer Complete clear */
#define DMA_INTCLR_CHTXF1 ((uint32_t)0x00000004) /*!< Channel 1 Half Transfer clear */
#define DMA_INTCLR_CERRF1 ((uint32_t)0x00000008) /*!< Channel 1 Transfer Error clear */
#define DMA_INTCLR_CGLBF2 ((uint32_t)0x00000010) /*!< Channel 2 Global interrupt clear */
#define DMA_INTCLR_CTXCF2 ((uint32_t)0x00000020) /*!< Channel 2 Transfer Complete clear */
#define DMA_INTCLR_CHTXF2 ((uint32_t)0x00000040) /*!< Channel 2 Half Transfer clear */
#define DMA_INTCLR_CERRF2 ((uint32_t)0x00000080) /*!< Channel 2 Transfer Error clear */
#define DMA_INTCLR_CGLBF3 ((uint32_t)0x00000100) /*!< Channel 3 Global interrupt clear */
#define DMA_INTCLR_CTXCF3 ((uint32_t)0x00000200) /*!< Channel 3 Transfer Complete clear */
#define DMA_INTCLR_CHTXF3 ((uint32_t)0x00000400) /*!< Channel 3 Half Transfer clear */
#define DMA_INTCLR_CERRF3 ((uint32_t)0x00000800) /*!< Channel 3 Transfer Error clear */
#define DMA_INTCLR_CGLBF4 ((uint32_t)0x00001000) /*!< Channel 4 Global interrupt clear */
#define DMA_INTCLR_CTXCF4 ((uint32_t)0x00002000) /*!< Channel 4 Transfer Complete clear */
#define DMA_INTCLR_CHTXF4 ((uint32_t)0x00004000) /*!< Channel 4 Half Transfer clear */
#define DMA_INTCLR_CERRF4 ((uint32_t)0x00008000) /*!< Channel 4 Transfer Error clear */
#define DMA_INTCLR_CGLBF5 ((uint32_t)0x00010000) /*!< Channel 5 Global interrupt clear */
#define DMA_INTCLR_CTXCF5 ((uint32_t)0x00020000) /*!< Channel 5 Transfer Complete clear */
#define DMA_INTCLR_CHTXF5 ((uint32_t)0x00040000) /*!< Channel 5 Half Transfer clear */
#define DMA_INTCLR_CERRF5 ((uint32_t)0x00080000) /*!< Channel 5 Transfer Error clear */

/*******************  Bit definition for DMA_CHCFG1 register  *******************/
#define DMA_CHCFG1_CHEN  ((uint16_t)0x0001) /*!< Channel enable*/
#define DMA_CHCFG1_TXCIE ((uint16_t)0x0002) /*!< Transfer complete interrupt enable */
#define DMA_CHCFG1_HTXIE ((uint16_t)0x0004) /*!< Half Transfer interrupt enable */
#define DMA_CHCFG1_ERRIE ((uint16_t)0x0008) /*!< Transfer error interrupt enable */
#define DMA_CHCFG1_DIR   ((uint16_t)0x0010) /*!< Data transfer direction */
#define DMA_CHCFG1_CIRC  ((uint16_t)0x0020) /*!< Circular mode */
#define DMA_CHCFG1_PINC  ((uint16_t)0x0040) /*!< Peripheral increment mode */
#define DMA_CHCFG1_MINC  ((uint16_t)0x0080) /*!< Memory increment mode */

#define DMA_CHCFG1_PSIZE   ((uint16_t)0x0300) /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CHCFG1_PSIZE_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define DMA_CHCFG1_PSIZE_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define DMA_CHCFG1_MSIZE   ((uint16_t)0x0C00) /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CHCFG1_MSIZE_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define DMA_CHCFG1_MSIZE_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define DMA_CHCFG1_PRIOLVL   ((uint16_t)0x3000) /*!< PL[1:0] bits(Channel Priority level) */
#define DMA_CHCFG1_PRIOLVL_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define DMA_CHCFG1_PRIOLVL_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define DMA_CHCFG1_MEM2MEM ((uint16_t)0x4000) /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CHCFG2 register  *******************/
#define DMA_CHCFG2_CHEN  ((uint16_t)0x0001) /*!< Channel enable */
#define DMA_CHCFG2_TXCIE ((uint16_t)0x0002) /*!< Transfer complete interrupt enable */
#define DMA_CHCFG2_HTXIE ((uint16_t)0x0004) /*!< Half Transfer interrupt enable */
#define DMA_CHCFG2_ERRIE ((uint16_t)0x0008) /*!< Transfer error interrupt enable */
#define DMA_CHCFG2_DIR   ((uint16_t)0x0010) /*!< Data transfer direction */
#define DMA_CHCFG2_CIRC  ((uint16_t)0x0020) /*!< Circular mode */
#define DMA_CHCFG2_PINC  ((uint16_t)0x0040) /*!< Peripheral increment mode */
#define DMA_CHCFG2_MINC  ((uint16_t)0x0080) /*!< Memory increment mode */

#define DMA_CHCFG2_PSIZE   ((uint16_t)0x0300) /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CHCFG2_PSIZE_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define DMA_CHCFG2_PSIZE_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define DMA_CHCFG2_MSIZE   ((uint16_t)0x0C00) /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CHCFG2_MSIZE_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define DMA_CHCFG2_MSIZE_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define DMA_CHCFG2_PRIOLVL   ((uint16_t)0x3000) /*!< PL[1:0] bits (Channel Priority level) */
#define DMA_CHCFG2_PRIOLVL_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define DMA_CHCFG2_PRIOLVL_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define DMA_CHCFG2_MEM2MEM ((uint16_t)0x4000) /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CHCFG3 register  *******************/
#define DMA_CHCFG3_CHEN  ((uint16_t)0x0001) /*!< Channel enable */
#define DMA_CHCFG3_TXCIE ((uint16_t)0x0002) /*!< Transfer complete interrupt enable */
#define DMA_CHCFG3_HTXIE ((uint16_t)0x0004) /*!< Half Transfer interrupt enable */
#define DMA_CHCFG3_ERRIE ((uint16_t)0x0008) /*!< Transfer error interrupt enable */
#define DMA_CHCFG3_DIR   ((uint16_t)0x0010) /*!< Data transfer direction */
#define DMA_CHCFG3_CIRC  ((uint16_t)0x0020) /*!< Circular mode */
#define DMA_CHCFG3_PINC  ((uint16_t)0x0040) /*!< Peripheral increment mode */
#define DMA_CHCFG3_MINC  ((uint16_t)0x0080) /*!< Memory increment mode */

#define DMA_CHCFG3_PSIZE   ((uint16_t)0x0300) /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CHCFG3_PSIZE_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define DMA_CHCFG3_PSIZE_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define DMA_CHCFG3_MSIZE   ((uint16_t)0x0C00) /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CHCFG3_MSIZE_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define DMA_CHCFG3_MSIZE_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define DMA_CHCFG3_PRIOLVL   ((uint16_t)0x3000) /*!< PL[1:0] bits (Channel Priority level) */
#define DMA_CHCFG3_PRIOLVL_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define DMA_CHCFG3_PRIOLVL_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define DMA_CHCFG3_MEM2MEM ((uint16_t)0x4000) /*!< Memory to memory mode */

/*!<******************  Bit definition for DMA_CHCFG4 register  *******************/
#define DMA_CHCFG4_CHEN  ((uint16_t)0x0001) /*!< Channel enable */
#define DMA_CHCFG4_TXCIE ((uint16_t)0x0002) /*!< Transfer complete interrupt enable */
#define DMA_CHCFG4_HTXIE ((uint16_t)0x0004) /*!< Half Transfer interrupt enable */
#define DMA_CHCFG4_ERRIE ((uint16_t)0x0008) /*!< Transfer error interrupt enable */
#define DMA_CHCFG4_DIR   ((uint16_t)0x0010) /*!< Data transfer direction */
#define DMA_CHCFG4_CIRC  ((uint16_t)0x0020) /*!< Circular mode */
#define DMA_CHCFG4_PINC  ((uint16_t)0x0040) /*!< Peripheral increment mode */
#define DMA_CHCFG4_MINC  ((uint16_t)0x0080) /*!< Memory increment mode */

#define DMA_CHCFG4_PSIZE   ((uint16_t)0x0300) /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CHCFG4_PSIZE_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define DMA_CHCFG4_PSIZE_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define DMA_CHCFG4_MSIZE   ((uint16_t)0x0C00) /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CHCFG4_MSIZE_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define DMA_CHCFG4_MSIZE_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define DMA_CHCFG4_PRIOLVL   ((uint16_t)0x3000) /*!< PL[1:0] bits (Channel Priority level) */
#define DMA_CHCFG4_PRIOLVL_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define DMA_CHCFG4_PRIOLVL_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define DMA_CHCFG4_MEM2MEM ((uint16_t)0x4000) /*!< Memory to memory mode */

/******************  Bit definition for DMA_CHCFG5 register  *******************/
#define DMA_CHCFG5_CHEN  ((uint16_t)0x0001) /*!< Channel enable */
#define DMA_CHCFG5_TXCIE ((uint16_t)0x0002) /*!< Transfer complete interrupt enable */
#define DMA_CHCFG5_HTXIE ((uint16_t)0x0004) /*!< Half Transfer interrupt enable */
#define DMA_CHCFG5_ERRIE ((uint16_t)0x0008) /*!< Transfer error interrupt enable */
#define DMA_CHCFG5_DIR   ((uint16_t)0x0010) /*!< Data transfer direction */
#define DMA_CHCFG5_CIRC  ((uint16_t)0x0020) /*!< Circular mode */
#define DMA_CHCFG5_PINC  ((uint16_t)0x0040) /*!< Peripheral increment mode */
#define DMA_CHCFG5_MINC  ((uint16_t)0x0080) /*!< Memory increment mode */

#define DMA_CHCFG5_PSIZE   ((uint16_t)0x0300) /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CHCFG5_PSIZE_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define DMA_CHCFG5_PSIZE_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define DMA_CHCFG5_MSIZE   ((uint16_t)0x0C00) /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CHCFG5_MSIZE_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define DMA_CHCFG5_MSIZE_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define DMA_CHCFG5_PRIOLVL   ((uint16_t)0x3000) /*!< PL[1:0] bits (Channel Priority level) */
#define DMA_CHCFG5_PRIOLVL_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define DMA_CHCFG5_PRIOLVL_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define DMA_CHCFG5_MEM2MEM ((uint16_t)0x4000) /*!< Memory to memory mode enable */

/******************  Bit definition for DMA_TXNUM1 register  ******************/
#define DMA_TXNUM1_NDTX ((uint16_t)0xFFFF) /*!< Number of data to Transfer */

/******************  Bit definition for DMA_TXNUM2 register  ******************/
#define DMA_TXNUM2_NDTX ((uint16_t)0xFFFF) /*!< Number of data to Transfer */

/******************  Bit definition for DMA_TXNUM3 register  ******************/
#define DMA_TXNUM3_NDTX ((uint16_t)0xFFFF) /*!< Number of data to Transfer */

/******************  Bit definition for DMA_TXNUM4 register  ******************/
#define DMA_TXNUM4_NDTX ((uint16_t)0xFFFF) /*!< Number of data to Transfer */

/******************  Bit definition for DMA_TXNUM5 register  ******************/
#define DMA_TXNUM5_NDTX ((uint16_t)0xFFFF) /*!< Number of data to Transfer */

/******************  Bit definition for DMA_PADDR1 register  *******************/
#define DMA_PADDR1_ADDR ((uint32_t)0xFFFFFFFF) /*!< Peripheral Address */

/******************  Bit definition for DMA_PADDR2 register  *******************/
#define DMA_PADDR2_ADDR ((uint32_t)0xFFFFFFFF) /*!< Peripheral Address */

/******************  Bit definition for DMA_PADDR3 register  *******************/
#define DMA_PADDR3_ADDR ((uint32_t)0xFFFFFFFF) /*!< Peripheral Address */

/******************  Bit definition for DMA_PADDR4 register  *******************/
#define DMA_PADDR4_ADDR ((uint32_t)0xFFFFFFFF) /*!< Peripheral Address */

/******************  Bit definition for DMA_PADDR5 register  *******************/
#define DMA_PADDR5_ADDR ((uint32_t)0xFFFFFFFF) /*!< Peripheral Address */

/******************  Bit definition for DMA_MADDR1 register  *******************/
#define DMA_MADDR1_ADDR ((uint32_t)0xFFFFFFFF) /*!< Memory Address */

/******************  Bit definition for DMA_MADDR2 register  *******************/
#define DMA_MADDR2_ADDR ((uint32_t)0xFFFFFFFF) /*!< Memory Address */

/******************  Bit definition for DMA_MADDR3 register  *******************/
#define DMA_MADDR3_ADDR ((uint32_t)0xFFFFFFFF) /*!< Memory Address */

/******************  Bit definition for DMA_MADDR4 register  *******************/
#define DMA_MADDR4_ADDR ((uint32_t)0xFFFFFFFF) /*!< Memory Address */

/******************  Bit definition for DMA_MADDR5 register  *******************/
#define DMA_MADDR5_ADDR ((uint32_t)0xFFFFFFFF) /*!< Memory Address */

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for ADC_CTRL register  ********************/
#define ADC_CTRL_EN             ((uint32_t)0x00000001) /*!< Bit 0 */
#define ADC_CTRL_TS_EN             ((uint32_t)0x00000002) /*!< Bit 1 */
#define ADC_CTRL_AWD_EN         ((uint32_t)0x00000004) /*!< Bit 2 */
#define ADC_CTRL_DMA_MODE_EN     ((uint32_t)0x00000008) /*!< Bit 3 */

#define ADC_CTRL_CH_SEL    ((uint32_t)0x00000070) /*!< ADC_CH[6:4] bits (channel select bits) */
#define ADC_CTRL_CH_0     ((uint32_t)0x00000000) /*!< Bit 0 */
#define ADC_CTRL_CH_1     ((uint32_t)0x00000010) /*!< Bit 1 */
#define ADC_CTRL_CH_2     ((uint32_t)0x00000020) /*!< Bit 2 */
#define ADC_CTRL_CH_3     ((uint32_t)0x00000030) /*!< Bit 3 */
#define ADC_CTRL_CH_4     ((uint32_t)0x00000040) /*!< Bit 4 */
#define ADC_CTRL_CH_5     ((uint32_t)0x00000050) /*!< Bit 5 */
#define ADC_CTRL_CH_6     ((uint32_t)0x00000060) /*!< Bit 6 */
#define ADC_CTRL_CH_7     ((uint32_t)0x00000070) /*!< Bit 7 */

#define ADC_CTRL_MODE        ((uint32_t)0x00000080) 
#define ADC_CTRL_DONE_IE   ((uint32_t)0x00000100) /*!< interrupt enable */
#define ADC_CTRL_AWD_IE    ((uint32_t)0x00000200) 
#define ADC_CTRL_PGARDY_IE ((uint32_t)0x00000400) 

/********************  Bit definition for ADC_SR register  ********************/
#define ADC_SR_DONE_F   ((uint8_t)0x01) /*!< DONE flag */
#define ADC_SR_AWD_F       ((uint8_t)0x02) /*!< Analog watchdog flag */
#define ADC_SR_PGARDY_F ((uint8_t)0x04) /*!< PGA_READY flag */

/********************  Bit definition for ADC_OVR_SAMP_CNT register  ********************/
#define ADC_OS_CNT_LD_CNT   ((uint8_t)0x1F) 

/********************  Bit definition for ADC_DAT register  ********************/
#define ADC_DAT_DATA     ((uint32_t)0x0000FFFF) /*!< Regular data */

/********************  Bit definition for ADC_WDT_THRES register  ********************/
#define ADC_WDT_THRES_LT     ((uint32_t)0x000003FF) /*!< Regular data */
#define ADC_WDT_THRES_HT     ((uint32_t)0x0FFC0000) /*!< Regular data */

/********************  Bit definition for ADC_PGA_CFG register  ********************/
#define ADC_PGA_CFG_MICBIAS_EN  ((uint32_t)0x00000001) /*!< Regular data */

#define ADC_PGA_CFG_MICBIAS     ((uint32_t)0x0000000E) /*!< Regular data */
#define ADC_PGA_CFG_MICBIAS_0   ((uint32_t)0x00000000) 
#define ADC_PGA_CFG_MICBIAS_1   ((uint32_t)0x00000002) 
#define ADC_PGA_CFG_MICBIAS_2   ((uint32_t)0x00000004) 
#define ADC_PGA_CFG_MICBIAS_3   ((uint32_t)0x00000006) 
#define ADC_PGA_CFG_MICBIAS_4   ((uint32_t)0x00000008) 
#define ADC_PGA_CFG_MICBIAS_5   ((uint32_t)0x0000000A) 
#define ADC_PGA_CFG_MICBIAS_6   ((uint32_t)0x0000000C) 
#define ADC_PGA_CFG_MICBIAS_7   ((uint32_t)0x0000000E) 

#define ADC_PGA_CFG_INIT_ENA    ((uint32_t)0x00000010) 

#define ADC_PGA_CFG_PEAK        ((uint32_t)0x00000600)
#define ADC_PGA_CFG_PEAK_0         ((uint32_t)0x00000000)
#define ADC_PGA_CFG_PEAK_1        ((uint32_t)0x00000200)
#define ADC_PGA_CFG_PEAK_2         ((uint32_t)0x00000400)
#define ADC_PGA_CFG_PEAK_3         ((uint32_t)0x00000600)

#define ADC_PGA_CFG_DRIVE         ((uint32_t)0x00001800) 
#define ADC_PGA_CFG_DRIVE_0     ((uint32_t)0x00000000)
#define ADC_PGA_CFG_DRIVE_1     ((uint32_t)0x00000800)
#define ADC_PGA_CFG_DRIVE_2     ((uint32_t)0x00001000) 
#define ADC_PGA_CFG_DRIVE_3     ((uint32_t)0x00001800) 

#define ADC_PGA_CFG_EN             ((uint32_t)0x00002000) 

#define ADC_PGA_CFG_GAIN         ((uint32_t)0x0001C000) 
#define ADC_PGA_CFG_GAIN_0dB     ((uint32_t)0x00000000)
#define ADC_PGA_CFG_GAIN_6dB     ((uint32_t)0x00004000)
#define ADC_PGA_CFG_GAIN_12dB     ((uint32_t)0x00008000)
#define ADC_PGA_CFG_GAIN_18dB    ((uint32_t)0x0000C000)
#define ADC_PGA_CFG_GAIN_24dB    ((uint32_t)0x00010000)
#define ADC_PGA_CFG_GAIN_30dB    ((uint32_t)0x00014000)
#define ADC_PGA_CFG_GAIN_36dB     ((uint32_t)0x00018000)
#define ADC_PGA_CFG_GAIN_42dB     ((uint32_t)0x0001C000)

/*******************  Bit definition for ADC_VOICE_DET_CR register  ********************/
#define ADC_VOICE_DET_CR_ZCRD_EN        ((uint32_t)0x00000001) 
#define ADC_VOICE_DET_CR_ED_EN           ((uint32_t)0x00000002) 
#define ADC_VOICE_DET_CR_FILTTER_BYP     ((uint32_t)0x00000004) 

/*******************  Bit definition for ADC_VOICE_ZCR_THRES register  ********************/
#define ADC_VAD_ZCRD_LOW_THRES       ((uint32_t)0x000000FF) 
#define ADC_VAD_ZCRD_HIGH_THRES     ((uint32_t)0x0000FF00) 

/*******************  Bit definition for ADC_VOICE_ED_THRES register  ********************/
#define ADC_VOICE_ED_THRES_MOFFSET        ((uint32_t)0x007FFFFF) 

/*******************  Bit definition for ADC_VOICE_ED_DWN_THRES register  ********************/
#define ADC_VOICE_ED_DWN_THRES_GATE_MIN ((uint32_t)0x007FFFFF) 

/*******************  Bit definition for ADC_VOICE_ED_UP_THRES register  ********************/
#define ADC_VOICE_ED_UP_THRES_GATE_MAX ((uint32_t)0x007FFFFF) 

/******************************************************************************/
/*                                                                            */
/*                                    IRC                                     */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for FREQ_CARRI_ON register  *******************/
#define IR_FREQ_CARR_ON_Msk       ((uint16_t)0x03FF)      /*!< IR_FREQ_CARR_ON (Bitfield-Mask: 0x3ff) */

/********************  Bits definition for FREQ_CARR_OFF register  *******************/
#define IR_FREQ_CARR_OFF_Msk      ((uint16_t)0x03FF)     /*!< IR_FREQ_CARR_OFF (Bitfield-Mask: 0x3ff) */

/********************  Bits definition for LOGIC_ONE_TIME register  *******************/
#define IR_LOGIC_ONE_TIME_LOGIC_ONE_SPACE_Pos   (0)                     /*!< IR_LOGIC_ONE_TIME_LOGIC_ONE_SPACE (Bit 0)        */
#define IR_LOGIC_ONE_TIME_LOGIC_ONE_SPACE_Msk   ((uint16_t)0x00FF)      /*!< IR_LOGIC_ONE_TIME_LOGIC_ONE_SPACE (Bitfield-Mask: 0xff) */
#define IR_LOGIC_ONE_TIME_LOGIC_ONE_MARK_Pos    (8)                     /*!< IR_LOGIC_ONE_TIME_LOGIC_ONE_MARK (Bit 0)        */
#define IR_LOGIC_ONE_TIME_LOGIC_ONE_MARK_Msk    ((uint16_t)0xFF00)      /*!< IR_LOGIC_ONE_TIME_LOGIC_ONE_MARK (Bitfield-Mask: 0xff) */

/********************  Bits definition for LOGIC_ZERO_TIME register  *******************/
#define IR_LOGIC_ZERO_TIME_LOGIC_ZERO_SPACE_Pos (0)                     /*!< IR_LOGIC_ZERO_TIME_LOGIC_ZERO_SPACE (Bit 0)      */
#define IR_LOGIC_ZERO_TIME_LOGIC_ZERO_SPACE_Msk ((uint16_t)0x00FF)      /*!< IR_LOGIC_ZERO_TIME_LOGIC_ZERO_SPACE (Bitfield-Mask: 0xff) */
#define IR_LOGIC_ZERO_TIME_LOGIC_ZERO_MARK_Pos  (8)                     /*!< IR_LOGIC_ZERO_TIME_LOGIC_ZERO_MARK (Bit 8)       */
#define IR_LOGIC_ZERO_TIME_LOGIC_ZERO_MARK_Msk  ((uint16_t)0xFF00)      /*!< IR_LOGIC_ZERO_TIME_LOGIC_ZERO_MARK (Bitfield-Mask: 0xff) */

/********************  Bits definition for IR_CTRL register  *******************/
#define IR_CTRL_CODE_FIFO_RESET_Pos     (0)                     /*!< IR_CTRL_REG: IR_CODE_FIFO_RESET (Bit 0)                  */
#define IR_CTRL_CODE_FIFO_RESET_Msk     ((uint16_t)0x0001)      /*!< IR_CTRL_REG: IR_CODE_FIFO_RESET (Bitfield-Mask: 0x01)    */
#define IR_CTRL_REP_FIFO_RESET_Pos      (1)                     /*!< IR_CTRL_REG: IR_REP_FIFO_RESET (Bit 1)                   */
#define IR_CTRL_REP_FIFO_RESET_Msk      ((uint16_t)0x0002)      /*!< IR_CTRL_REG: IR_REP_FIFO_RESET (Bitfield-Mask: 0x01)     */
#define IR_CTRL_ENABLE_Pos              (2)                     /*!< IR_CTRL_REG: IR_ENABLE (Bit 2)                           */
#define IR_CTRL_ENABLE_Msk              ((uint16_t)0x0004)      /*!< IR_CTRL_REG: IR_ENABLE (Bitfield-Mask: 0x01)             */
#define IR_CTRL_TX_START_Pos            (3)                     /*!< IR_CTRL_REG: IR_TX_START (Bit 3)                         */
#define IR_CTRL_TX_START_Msk            ((uint16_t)0x0008)      /*!< IR_CTRL_REG: IR_TX_START (Bitfield-Mask: 0x01)           */

#define IR_CTRL_INVERT_OUTPUT_Pos       (5)                     /*!< IR_CTRL_INVERT_OUTPUT (Bit 5)                    */
#define IR_CTRL_INVERT_OUTPUT_Msk       ((uint16_t)0x0020)      /*!< IR_CTRL_INVERT_OUTPUT (Bitfield-Mask: 0x01)      */
#define IR_CTRL_LOGIC_ZERO_FORMAT_Pos   (6)                     /*!< IR_CTRL_LOGIC_ZERO_FORMAT (Bit 6)                */
#define IR_CTRL_LOGIC_ZERO_FORMAT_Msk   ((uint16_t)0x0040)      /*!< IR_CTRL_LOGIC_ZERO_FORMAT (Bitfield-Mask: 0x01)  */
#define IR_CTRL_LOGIC_ONE_FORMAT_Pos    (7)                     /*!< IR_CTRL_LOGIC_ONE_FORMAT (Bit 7)                 */
#define IR_CTRL_LOGIC_ONE_FORMAT_Msk    ((uint16_t)0x0080)      /*!< IR_CTRL_LOGIC_ONE_FORMAT (Bitfield-Mask: 0x01)   */
#define IR_CTRL_IRQ_EN_Pos              (8)                     /*!< IR_CTRL_IRQ_EN (Bit 8)                           */
#define IR_CTRL_IRQ_EN_Msk              ((uint16_t)0x0100)      /*!< IR_CTRL_IRQ_EN (Bitfield-Mask: 0x01)             */

/********************  Bits definition for IR_STATUS register  *******************/
#define IR_STATUS_CODE_FIFO_WORDS_Pos   (0)                    /*!< IR IR_STATUS_REG: IR_CODE_FIFO_WRDS (Bit 0) */
#define IR_STATUS_CODE_FIFO_WORDS_Msk   ((uint16_t)0x003F)     /*!< IR_STATUS_CODE_FIFO_WORDS_MASK (Bitfield-Mask: 0x3f) */
#define IR_STATUS_REP_FIFO_WORDS_Pos    (6)                    /*!< IR IR_STATUS_REG: IR_REP_FIFO_WRDS (Bit 6) */
#define IR_STATUS_REP_FIFO_WORDS_Msk    ((uint16_t)0x03C0)     /*!< IR_STATUS_REP_FIFO_WORDS_MASK (Bitfield-Mask: 0x0f) */
#define IR_STATUS_BUSY                  ((uint16_t)0x0400)     /*!< IR_STATUS_BUSY (Bitfield-Mask: 0x01) */
#define IR_IRQ_FLAG                     ((uint16_t)0x0800)     /*!< IR_IRQ_FLAG (Bitfield-Mask: 0x01) */

/********************  Bits definition for IR_REPEAT_TIME register  *******************/
#define IR_REPEAT_TIME_Msk     ((uint32_t)0x0000FFFF)         /*!< IR_REPEAT_TIME (Bitfield-Mask: 0xffff) */

/********************  Bits definition for IR_CODE_FIFO register  *******************/
#define IR_CODE_FIFO_Msk       ((uint32_t)0x001FFFFF)         /*!< IR_CODE_FIFO (Bitfield-Mask: 0xffff) */

/********************  Bits definition for IR_REPEAT_FIFO register  *******************/
#define IR_REPEAT_FIFO_Msk     ((uint32_t)0x001FFFFF)        /*!< IR_REPEAT_FIFO (Bitfield-Mask: 0xffff) */

/******************************************************************************/
/*                                                                            */
/*                                   KEYSCAN                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for KEYSCAN_CTRL register  *******************/
#define KEYSCAN_CTRL_EN             ((uint32_t)0x00000001) 

#define KEYSCAN_CTRL_MODE_SEL       ((uint32_t)0x0000000C)
#define KEYSCAN_CTRL_MODE_AUTO      ((uint32_t)0x00000000)
#define KEYSCAN_CTRL_MODE_SOFT      ((uint32_t)0x00000004)
#define KEYSCAN_CTRL_MODE_LP        ((uint32_t)0x00000008)

#define KEYSCAN_CTRL_DTS            ((uint32_t)0x00000070)
#define KEYSCAN_CTRL_DTS_10MS       ((uint32_t)0x00000000)
#define KEYSCAN_CTRL_DTS_20MS       ((uint32_t)0x00000010)
#define KEYSCAN_CTRL_DTS_40MS       ((uint32_t)0x00000020)
#define KEYSCAN_CTRL_DTS_80MS       ((uint32_t)0x00000030)
#define KEYSCAN_CTRL_DTS_160MS      ((uint32_t)0x00000040)
#define KEYSCAN_CTRL_DTS_320MS      ((uint32_t)0x00000050)
#define KEYSCAN_CTRL_DTS_640MS      ((uint32_t)0x00000060)

#define KEYSCAN_CTRL_WTS            ((uint32_t)0x00000380)
#define KEYSCAN_CTRL_WTS_0MS        ((uint32_t)0x00000000)
#define KEYSCAN_CTRL_WTS_32MS       ((uint32_t)0x00000080)
#define KEYSCAN_CTRL_WTS_64MS       ((uint32_t)0x00000100)
#define KEYSCAN_CTRL_WTS_96MS       ((uint32_t)0x00000180)
#define KEYSCAN_CTRL_WTS_128MS      ((uint32_t)0x00000200)
#define KEYSCAN_CTRL_WTS_160MS      ((uint32_t)0x00000280)
#define KEYSCAN_CTRL_WTS_192MS      ((uint32_t)0x00000300)
#define KEYSCAN_CTRL_WTS_224MS      ((uint32_t)0x00000380)

#define KEYSCAN_CTRL_MASK           ((uint32_t)0x00000C00) 
#define KEYSCAN_CTRL_MASK_13        ((uint32_t)0x00000000) 
#define KEYSCAN_CTRL_MASK_8         ((uint32_t)0x00000400) 
#define KEYSCAN_CTRL_MASK_10        ((uint32_t)0x00000800)

#define KEYSCAN_CTRL_SOFT_START     ((uint32_t)0x00001000) 
#define KEYSCAN_CTRL_INFO_CLR       ((uint32_t)0x00002000) 
#define KEYSCAN_CTRL_INTEN          ((uint32_t)0x00400000) 
#define KEYSCAN_CTRL_IRP            ((uint32_t)0x00800000) 

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for TIM_CTRL1 register  ********************/
#define TIM_CTRL1_CNTEN ((uint32_t)0x00000001) /*!< Counter enable */
#define TIM_CTRL1_UPDIS ((uint32_t)0x00000002) /*!< Update disable */
#define TIM_CTRL1_UPRS  ((uint32_t)0x00000004) /*!< Update request source */
#define TIM_CTRL1_ONEPM ((uint32_t)0x00000008) /*!< One pulse mode */
#define TIM_CTRL1_DIR   ((uint32_t)0x00000010) /*!< Direction */

#define TIM_CTRL1_CAMSEL   ((uint32_t)0x00000060) /*!< CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CTRL1_CAMSEL_0 ((uint32_t)0x00000020) /*!< Bit 0 */
#define TIM_CTRL1_CAMSEL_1 ((uint32_t)0x00000040) /*!< Bit 1 */

#define TIM_CTRL1_ARPEN ((uint32_t)0x00000080) /*!< Auto-reload preload enable */

#define TIM_CTRL1_CLKD   ((uint32_t)0x00000300) /*!< CKD[1:0] bits (clock division) */
#define TIM_CTRL1_CLKD_0 ((uint32_t)0x00000100) /*!< Bit 0 */
#define TIM_CTRL1_CLKD_1 ((uint32_t)0x00000200) /*!< Bit 1 */

#define TIM_CTRL1_IOMBKPEN ((uint32_t)0x00000400) /*!< Break_in selection from IOM/COMP */
#define TIM_CTRL1_LBKPEN ((uint32_t)0x00010000) /*!< LOCKUP as bkp Enable*/

/*******************  Bit definition for TIM_CTRL2 register  ********************/
#define TIM_CTRL2_CCPCTL ((uint32_t)0x00000001) /*!< Capture/Compare Preloaded Control */
#define TIM_CTRL2_CCUSEL ((uint32_t)0x00000004) /*!< Capture/Compare Control Update Selection */
#define TIM_CTRL2_CCDSEL ((uint32_t)0x00000008) /*!< Capture/Compare DMA Selection */

#define TIM_CTRL2_MMSEL   ((uint32_t)0x00000070) /*!< MMS[2:0] bits (Master Mode Selection) */
#define TIM_CTRL2_MMSEL_0 ((uint32_t)0x00000010) /*!< Bit 0 */
#define TIM_CTRL2_MMSEL_1 ((uint32_t)0x00000020) /*!< Bit 1 */
#define TIM_CTRL2_MMSEL_2 ((uint32_t)0x00000040) /*!< Bit 2 */

#define TIM_CTRL2_TI1SEL ((uint32_t)0x00000080) /*!< TI1 Selection */
#define TIM_CTRL2_OI1    ((uint32_t)0x00000100) /*!< Output Idle state 1 (OC1 output) */
#define TIM_CTRL2_OI1N   ((uint32_t)0x00000200) /*!< Output Idle state 1 (OC1N output) */
#define TIM_CTRL2_OI2    ((uint32_t)0x00000400) /*!< Output Idle state 2 (OC2 output) */
#define TIM_CTRL2_OI2N   ((uint32_t)0x00000800) /*!< Output Idle state 2 (OC2N output) */
#define TIM_CTRL2_OI3    ((uint32_t)0x00001000) /*!< Output Idle state 3 (OC3 output) */
#define TIM_CTRL2_OI3N   ((uint32_t)0x00002000) /*!< Output Idle state 3 (OC3N output) */
#define TIM_CTRL2_OI4    ((uint32_t)0x00004000) /*!< Output Idle state 4 (OC4 output) */

#define TIM_CTRL2_OI5 ((uint32_t)0x00010000) /*!< Output Idle state 5 (OC5 output) */
#define TIM_CTRL2_OI6 ((uint32_t)0x00040000) /*!< Output Idle state 6 (OC6 output) */

/*******************  Bit definition for TIM_SMCTRL register  *******************/
#define TIM_SMCTRL_SMSEL   ((uint16_t)0x0007) /*!< SMS[2:0] bits (Slave mode selection) */
#define TIM_SMCTRL_SMSEL_0 ((uint16_t)0x0001) /*!< Bit 0 */
#define TIM_SMCTRL_SMSEL_1 ((uint16_t)0x0002) /*!< Bit 1 */
#define TIM_SMCTRL_SMSEL_2 ((uint16_t)0x0004) /*!< Bit 2 */

#define TIM_SMCTRL_TSEL   ((uint16_t)0x0070) /*!< TS[2:0] bits (Trigger selection) */
#define TIM_SMCTRL_TSEL_0 ((uint16_t)0x0010) /*!< Bit 0 */
#define TIM_SMCTRL_TSEL_1 ((uint16_t)0x0020) /*!< Bit 1 */
#define TIM_SMCTRL_TSEL_2 ((uint16_t)0x0040) /*!< Bit 2 */

#define TIM_SMCTRL_MSMD ((uint16_t)0x0080) /*!< Master/slave mode */

#define TIM_SMCTRL_EXTF   ((uint16_t)0x0F00) /*!< ETF[3:0] bits (External trigger filter) */
#define TIM_SMCTRL_EXTF_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define TIM_SMCTRL_EXTF_1 ((uint16_t)0x0200) /*!< Bit 1 */
#define TIM_SMCTRL_EXTF_2 ((uint16_t)0x0400) /*!< Bit 2 */
#define TIM_SMCTRL_EXTF_3 ((uint16_t)0x0800) /*!< Bit 3 */

#define TIM_SMCTRL_EXTPS   ((uint16_t)0x3000) /*!< ETPS[1:0] bits (External trigger prescaler) */
#define TIM_SMCTRL_EXTPS_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define TIM_SMCTRL_EXTPS_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define TIM_SMCTRL_EXCEN ((uint16_t)0x4000) /*!< External clock enable */
#define TIM_SMCTRL_EXTP  ((uint16_t)0x8000) /*!< External trigger polarity */

/*******************  Bit definition for TIM_DINTEN register  *******************/
#define TIM_DINTEN_UIEN   ((uint16_t)0x0001) /*!< Update interrupt enable */
#define TIM_DINTEN_CC1IEN ((uint16_t)0x0002) /*!< Capture/Compare 1 interrupt enable */
#define TIM_DINTEN_CC2IEN ((uint16_t)0x0004) /*!< Capture/Compare 2 interrupt enable */
#define TIM_DINTEN_CC3IEN ((uint16_t)0x0008) /*!< Capture/Compare 3 interrupt enable */
#define TIM_DINTEN_CC4IEN ((uint16_t)0x0010) /*!< Capture/Compare 4 interrupt enable */
#define TIM_DINTEN_COMIEN ((uint16_t)0x0020) /*!< COM interrupt enable */
#define TIM_DINTEN_TIEN   ((uint16_t)0x0040) /*!< Trigger interrupt enable */
#define TIM_DINTEN_BIEN   ((uint16_t)0x0080) /*!< Break interrupt enable */
#define TIM_DINTEN_UDEN   ((uint16_t)0x0100) /*!< Update DMA request enable */
#define TIM_DINTEN_CC1DEN ((uint16_t)0x0200) /*!< Capture/Compare 1 DMA request enable */
#define TIM_DINTEN_CC2DEN ((uint16_t)0x0400) /*!< Capture/Compare 2 DMA request enable */
#define TIM_DINTEN_CC3DEN ((uint16_t)0x0800) /*!< Capture/Compare 3 DMA request enable */
#define TIM_DINTEN_CC4DEN ((uint16_t)0x1000) /*!< Capture/Compare 4 DMA request enable */
#define TIM_DINTEN_COMDEN ((uint16_t)0x2000) /*!< COM DMA request enable */
#define TIM_DINTEN_TDEN   ((uint16_t)0x4000) /*!< Trigger DMA request enable */

/********************  Bit definition for TIM_STS register  ********************/
#define TIM_STS_UDITF  ((uint32_t)0x00000001) /*!< Update interrupt Flag */
#define TIM_STS_CC1ITF ((uint32_t)0x00000002) /*!< Capture/Compare 1 interrupt Flag */
#define TIM_STS_CC2ITF ((uint32_t)0x00000004) /*!< Capture/Compare 2 interrupt Flag */
#define TIM_STS_CC3ITF ((uint32_t)0x00000008) /*!< Capture/Compare 3 interrupt Flag */
#define TIM_STS_CC4ITF ((uint32_t)0x00000010) /*!< Capture/Compare 4 interrupt Flag */
#define TIM_STS_COMITF ((uint32_t)0x00000020) /*!< COM interrupt Flag */
#define TIM_STS_TITF   ((uint32_t)0x00000040) /*!< Trigger interrupt Flag */
#define TIM_STS_BITF   ((uint32_t)0x00000080) /*!< Break interrupt Flag */
#define TIM_STS_CC1OCF ((uint32_t)0x00000200) /*!< Capture/Compare 1 Overcapture Flag */
#define TIM_STS_CC2OCF ((uint32_t)0x00000400) /*!< Capture/Compare 2 Overcapture Flag */
#define TIM_STS_CC3OCF ((uint32_t)0x00000800) /*!< Capture/Compare 3 Overcapture Flag */
#define TIM_STS_CC4OCF ((uint32_t)0x00001000) /*!< Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EVTGEN register  ********************/
#define TIM_EVTGEN_UDGN   ((uint8_t)0x01) /*!< Update Generation */
#define TIM_EVTGEN_CC1GN  ((uint8_t)0x02) /*!< Capture/Compare 1 Generation */
#define TIM_EVTGEN_CC2GN  ((uint8_t)0x04) /*!< Capture/Compare 2 Generation */
#define TIM_EVTGEN_CC3GN  ((uint8_t)0x08) /*!< Capture/Compare 3 Generation */
#define TIM_EVTGEN_CC4GN  ((uint8_t)0x10) /*!< Capture/Compare 4 Generation */
#define TIM_EVTGEN_CCUDGN ((uint8_t)0x20) /*!< Capture/Compare Control Update Generation */
#define TIM_EVTGEN_TGN    ((uint8_t)0x40) /*!< Trigger Generation */
#define TIM_EVTGEN_BGN    ((uint8_t)0x80) /*!< Break Generation */

/******************  Bit definition for TIM_CCMOD1 register  *******************/
#define TIM_CCMOD1_CC1SEL   ((uint16_t)0x0003) /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMOD1_CC1SEL_0 ((uint16_t)0x0001) /*!< Bit 0 */
#define TIM_CCMOD1_CC1SEL_1 ((uint16_t)0x0002) /*!< Bit 1 */

#define TIM_CCMOD1_OC1FEN ((uint16_t)0x0004) /*!< Output Compare 1 Fast enable */
#define TIM_CCMOD1_OC1PEN ((uint16_t)0x0008) /*!< Output Compare 1 Preload enable */

#define TIM_CCMOD1_OC1M   ((uint16_t)0x0070) /*!< OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_CCMOD1_OC1M_0 ((uint16_t)0x0010) /*!< Bit 0 */
#define TIM_CCMOD1_OC1M_1 ((uint16_t)0x0020) /*!< Bit 1 */
#define TIM_CCMOD1_OC1M_2 ((uint16_t)0x0040) /*!< Bit 2 */

#define TIM_CCMOD1_OC1CEN ((uint16_t)0x0080) /*!< Output Compare 1Clear Enable */

#define TIM_CCMOD1_CC2SEL   ((uint16_t)0x0300) /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMOD1_CC2SEL_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define TIM_CCMOD1_CC2SEL_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define TIM_CCMOD1_OC2FEN ((uint16_t)0x0400) /*!< Output Compare 2 Fast enable */
#define TIM_CCMOD1_OC2PEN ((uint16_t)0x0800) /*!< Output Compare 2 Preload enable */

#define TIM_CCMOD1_OC2M   ((uint16_t)0x7000) /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMOD1_OC2M_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define TIM_CCMOD1_OC2M_1 ((uint16_t)0x2000) /*!< Bit 1 */
#define TIM_CCMOD1_OC2M_2 ((uint16_t)0x4000) /*!< Bit 2 */

#define TIM_CCMOD1_OC2CEN ((uint16_t)0x8000) /*!< Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define TIM_CCMOD1_IC1PSC   ((uint16_t)0x000C) /*!< IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMOD1_IC1PSC_0 ((uint16_t)0x0004) /*!< Bit 0 */
#define TIM_CCMOD1_IC1PSC_1 ((uint16_t)0x0008) /*!< Bit 1 */

#define TIM_CCMOD1_IC1F   ((uint16_t)0x00F0) /*!< IC1F[3:0] bits (Input Capture 1 Filter) */
#define TIM_CCMOD1_IC1F_0 ((uint16_t)0x0010) /*!< Bit 0 */
#define TIM_CCMOD1_IC1F_1 ((uint16_t)0x0020) /*!< Bit 1 */
#define TIM_CCMOD1_IC1F_2 ((uint16_t)0x0040) /*!< Bit 2 */
#define TIM_CCMOD1_IC1F_3 ((uint16_t)0x0080) /*!< Bit 3 */

#define TIM_CCMOD1_IC2PSC   ((uint16_t)0x0C00) /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMOD1_IC2PSC_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define TIM_CCMOD1_IC2PSC_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define TIM_CCMOD1_IC2F   ((uint16_t)0xF000) /*!< IC2F[3:0] bits (Input Capture 2 Filter) */
#define TIM_CCMOD1_IC2F_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define TIM_CCMOD1_IC2F_1 ((uint16_t)0x2000) /*!< Bit 1 */
#define TIM_CCMOD1_IC2F_2 ((uint16_t)0x4000) /*!< Bit 2 */
#define TIM_CCMOD1_IC2F_3 ((uint16_t)0x8000) /*!< Bit 3 */

/******************  Bit definition for TIM_CCMOD2 register  *******************/
#define TIM_CCMOD2_CC3SEL   ((uint16_t)0x0003) /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define TIM_CCMOD2_CC3SEL_0 ((uint16_t)0x0001) /*!< Bit 0 */
#define TIM_CCMOD2_CC3SEL_1 ((uint16_t)0x0002) /*!< Bit 1 */

#define TIM_CCMOD2_OC3FEN ((uint16_t)0x0004) /*!< Output Compare 3 Fast enable */
#define TIM_CCMOD2_OC3PEN ((uint16_t)0x0008) /*!< Output Compare 3 Preload enable */

#define TIM_CCMOD2_OC3MD   ((uint16_t)0x0070) /*!< OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMOD2_OC3MD_0 ((uint16_t)0x0010) /*!< Bit 0 */
#define TIM_CCMOD2_OC3MD_1 ((uint16_t)0x0020) /*!< Bit 1 */
#define TIM_CCMOD2_OC3MD_2 ((uint16_t)0x0040) /*!< Bit 2 */

#define TIM_CCMOD2_OC3CEN ((uint16_t)0x0080) /*!< Output Compare 3 Clear Enable */

#define TIM_CCMOD2_CC4SEL   ((uint16_t)0x0300) /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMOD2_CC4SEL_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define TIM_CCMOD2_CC4SEL_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define TIM_CCMOD2_OC4FEN ((uint16_t)0x0400) /*!< Output Compare 4 Fast enable */
#define TIM_CCMOD2_OC4PEN ((uint16_t)0x0800) /*!< Output Compare 4 Preload enable */

#define TIM_CCMOD2_OC4MD   ((uint16_t)0x7000) /*!< OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMOD2_OC4MD_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define TIM_CCMOD2_OC4MD_1 ((uint16_t)0x2000) /*!< Bit 1 */
#define TIM_CCMOD2_OC4MD_2 ((uint16_t)0x4000) /*!< Bit 2 */

#define TIM_CCMOD2_OC4CEN ((uint16_t)0x8000) /*!< Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define TIM_CCMOD2_IC3PSC   ((uint16_t)0x000C) /*!< IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMOD2_IC3PSC_0 ((uint16_t)0x0004) /*!< Bit 0 */
#define TIM_CCMOD2_IC3PSC_1 ((uint16_t)0x0008) /*!< Bit 1 */

#define TIM_CCMOD2_IC3F   ((uint16_t)0x00F0) /*!< IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMOD2_IC3F_0 ((uint16_t)0x0010) /*!< Bit 0 */
#define TIM_CCMOD2_IC3F_1 ((uint16_t)0x0020) /*!< Bit 1 */
#define TIM_CCMOD2_IC3F_2 ((uint16_t)0x0040) /*!< Bit 2 */
#define TIM_CCMOD2_IC3F_3 ((uint16_t)0x0080) /*!< Bit 3 */

#define TIM_CCMOD2_IC4PSC   ((uint16_t)0x0C00) /*!< IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_CCMOD2_IC4PSC_0 ((uint16_t)0x0400) /*!< Bit 0 */
#define TIM_CCMOD2_IC4PSC_1 ((uint16_t)0x0800) /*!< Bit 1 */

#define TIM_CCMOD2_IC4F   ((uint16_t)0xF000) /*!< IC4F[3:0] bits (Input Capture 4 Filter) */
#define TIM_CCMOD2_IC4F_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define TIM_CCMOD2_IC4F_1 ((uint16_t)0x2000) /*!< Bit 1 */
#define TIM_CCMOD2_IC4F_2 ((uint16_t)0x4000) /*!< Bit 2 */
#define TIM_CCMOD2_IC4F_3 ((uint16_t)0x8000) /*!< Bit 3 */

/*----------------------------------------------------------------------------*/

/*******************  Bit definition for TIM_CCEN register  *******************/
#define TIM_CCEN_CC1EN  ((uint32_t)0x00000001) /*!< Capture/Compare 1 output enable */
#define TIM_CCEN_CC1P   ((uint32_t)0x00000002) /*!< Capture/Compare 1 output Polarity */
#define TIM_CCEN_CC1NEN ((uint32_t)0x00000004) /*!< Capture/Compare 1 Complementary output enable */
#define TIM_CCEN_CC1NP  ((uint32_t)0x00000008) /*!< Capture/Compare 1 Complementary output Polarity */
#define TIM_CCEN_CC2EN  ((uint32_t)0x00000010) /*!< Capture/Compare 2 output enable */
#define TIM_CCEN_CC2P   ((uint32_t)0x00000020) /*!< Capture/Compare 2 output Polarity */
#define TIM_CCEN_CC2NEN ((uint32_t)0x00000040) /*!< Capture/Compare 2 Complementary output enable */
#define TIM_CCEN_CC2NP  ((uint32_t)0x00000080) /*!< Capture/Compare 2 Complementary output Polarity */
#define TIM_CCEN_CC3EN  ((uint32_t)0x00000100) /*!< Capture/Compare 3 output enable */
#define TIM_CCEN_CC3P   ((uint32_t)0x00000200) /*!< Capture/Compare 3 output Polarity */
#define TIM_CCEN_CC3NEN ((uint32_t)0x00000400) /*!< Capture/Compare 3 Complementary output enable */
#define TIM_CCEN_CC3NP  ((uint32_t)0x00000800) /*!< Capture/Compare 3 Complementary output Polarity */
#define TIM_CCEN_CC4EN  ((uint32_t)0x00001000) /*!< Capture/Compare 4 output enable */
#define TIM_CCEN_CC4P   ((uint32_t)0x00002000) /*!< Capture/Compare 4 output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define TIM_CNT_CNT ((uint16_t)0xFFFF) /*!< Counter Value */

/*******************  Bit definition for TIM_PSC register  ********************/
#define TIM_PSC_PSC ((uint16_t)0xFFFF) /*!< Prescaler Value */

/*******************  Bit definition for TIM_AR register  ********************/
#define TIM_AR_AR ((uint16_t)0xFFFF) /*!< actual auto-reload Value */

/*******************  Bit definition for TIM_REPCNT register  ********************/
#define TIM_REPCNT_REPCNT ((uint8_t)0xFF) /*!< Repetition Counter Value */

/*******************  Bit definition for TIM_CCDAT1 register  *******************/
#define TIM_CCDAT1_CCDAT1 ((uint16_t)0xFFFF) /*!< Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCDAT2 register  *******************/
#define TIM_CCDAT2_CCDAT2 ((uint16_t)0xFFFF) /*!< Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCDAT3 register  *******************/
#define TIM_CCDAT3_CCDAT3 ((uint16_t)0xFFFF) /*!< Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCDAT4 register  *******************/
#define TIM_CCDAT4_CCDAT4 ((uint16_t)0xFFFF) /*!< Capture/Compare 4 Value */

/*******************  Bit definition for TIM_BKDT register  *******************/
#define TIM_BKDT_DTGN   ((uint16_t)0x00FF) /*!< DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_BKDT_DTGN_0 ((uint16_t)0x0001) /*!< Bit 0 */
#define TIM_BKDT_DTGN_1 ((uint16_t)0x0002) /*!< Bit 1 */
#define TIM_BKDT_DTGN_2 ((uint16_t)0x0004) /*!< Bit 2 */
#define TIM_BKDT_DTGN_3 ((uint16_t)0x0008) /*!< Bit 3 */
#define TIM_BKDT_DTGN_4 ((uint16_t)0x0010) /*!< Bit 4 */
#define TIM_BKDT_DTGN_5 ((uint16_t)0x0020) /*!< Bit 5 */
#define TIM_BKDT_DTGN_6 ((uint16_t)0x0040) /*!< Bit 6 */
#define TIM_BKDT_DTGN_7 ((uint16_t)0x0080) /*!< Bit 7 */

#define TIM_BKDT_LCKCFG   ((uint16_t)0x0300) /*!< LOCK[1:0] bits (Lock Configuration) */
#define TIM_BKDT_LCKCFG_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define TIM_BKDT_LCKCFG_1 ((uint16_t)0x0200) /*!< Bit 1 */

#define TIM_BKDT_OSSI ((uint16_t)0x0400) /*!< Off-State Selection for Idle mode */
#define TIM_BKDT_OSSR ((uint16_t)0x0800) /*!< Off-State Selection for Run mode */
#define TIM_BKDT_BKEN ((uint16_t)0x1000) /*!< Break enable */
#define TIM_BKDT_BKP  ((uint16_t)0x2000) /*!< Break Polarity */
#define TIM_BKDT_AOEN ((uint16_t)0x4000) /*!< Automatic Output enable */
#define TIM_BKDT_MOEN ((uint16_t)0x8000) /*!< Main Output enable */

/*******************  Bit definition for TIM_DCTRL register  ********************/
#define TIM_DCTRL_DBADDR   ((uint16_t)0x001F) /*!< DBA[4:0] bits (DMA Base Address) */
#define TIM_DCTRL_DBADDR_0 ((uint16_t)0x0001) /*!< Bit 0 */
#define TIM_DCTRL_DBADDR_1 ((uint16_t)0x0002) /*!< Bit 1 */
#define TIM_DCTRL_DBADDR_2 ((uint16_t)0x0004) /*!< Bit 2 */
#define TIM_DCTRL_DBADDR_3 ((uint16_t)0x0008) /*!< Bit 3 */
#define TIM_DCTRL_DBADDR_4 ((uint16_t)0x0010) /*!< Bit 4 */

#define TIM_DCTRL_DBLEN   ((uint16_t)0x1F00) /*!< DBL[4:0] bits (DMA Burst Length) */
#define TIM_DCTRL_DBLEN_0 ((uint16_t)0x0100) /*!< Bit 0 */
#define TIM_DCTRL_DBLEN_1 ((uint16_t)0x0200) /*!< Bit 1 */
#define TIM_DCTRL_DBLEN_2 ((uint16_t)0x0400) /*!< Bit 2 */
#define TIM_DCTRL_DBLEN_3 ((uint16_t)0x0800) /*!< Bit 3 */
#define TIM_DCTRL_DBLEN_4 ((uint16_t)0x1000) /*!< Bit 4 */

/*******************  Bit definition for TIM_DADDR register  *******************/
#define TIM_DADDR_BURST ((uint16_t)0xFFFF) /*!< DMA register for burst accesses */

/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock (RTC)                            */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RTC_TSH register  *******************/
#define RTC_TSH_APM   ((uint32_t)0x00400000)
#define RTC_TSH_HOT   ((uint32_t)0x00300000)
#define RTC_TSH_HOT_0 ((uint32_t)0x00100000)
#define RTC_TSH_HOT_1 ((uint32_t)0x00200000)
#define RTC_TSH_HOU   ((uint32_t)0x000F0000)
#define RTC_TSH_HOU_0 ((uint32_t)0x00010000)
#define RTC_TSH_HOU_1 ((uint32_t)0x00020000)
#define RTC_TSH_HOU_2 ((uint32_t)0x00040000)
#define RTC_TSH_HOU_3 ((uint32_t)0x00080000)
#define RTC_TSH_MIT   ((uint32_t)0x00007000)
#define RTC_TSH_MIT_0 ((uint32_t)0x00001000)
#define RTC_TSH_MIT_1 ((uint32_t)0x00002000)
#define RTC_TSH_MIT_2 ((uint32_t)0x00004000)
#define RTC_TSH_MIU   ((uint32_t)0x00000F00)
#define RTC_TSH_MIU_0 ((uint32_t)0x00000100)
#define RTC_TSH_MIU_1 ((uint32_t)0x00000200)
#define RTC_TSH_MIU_2 ((uint32_t)0x00000400)
#define RTC_TSH_MIU_3 ((uint32_t)0x00000800)
#define RTC_TSH_SCT   ((uint32_t)0x00000070)
#define RTC_TSH_SCT_0 ((uint32_t)0x00000010)
#define RTC_TSH_SCT_1 ((uint32_t)0x00000020)
#define RTC_TSH_SCT_2 ((uint32_t)0x00000040)
#define RTC_TSH_SCU   ((uint32_t)0x0000000F)
#define RTC_TSH_SCU_0 ((uint32_t)0x00000001)
#define RTC_TSH_SCU_1 ((uint32_t)0x00000002)
#define RTC_TSH_SCU_2 ((uint32_t)0x00000004)
#define RTC_TSH_SCU_3 ((uint32_t)0x00000008)

/********************  Bits definition for RTC_DATE register  *******************/
#define RTC_DATE_YRT   ((uint32_t)0x00F00000)
#define RTC_DATE_YRT_0 ((uint32_t)0x00100000)
#define RTC_DATE_YRT_1 ((uint32_t)0x00200000)
#define RTC_DATE_YRT_2 ((uint32_t)0x00400000)
#define RTC_DATE_YRT_3 ((uint32_t)0x00800000)
#define RTC_DATE_YRU   ((uint32_t)0x000F0000)
#define RTC_DATE_YRU_0 ((uint32_t)0x00010000)
#define RTC_DATE_YRU_1 ((uint32_t)0x00020000)
#define RTC_DATE_YRU_2 ((uint32_t)0x00040000)
#define RTC_DATE_YRU_3 ((uint32_t)0x00080000)
#define RTC_DATE_WDU   ((uint32_t)0x0000E000)
#define RTC_DATE_WDU_0 ((uint32_t)0x00002000)
#define RTC_DATE_WDU_1 ((uint32_t)0x00004000)
#define RTC_DATE_WDU_2 ((uint32_t)0x00008000)
#define RTC_DATE_MOT   ((uint32_t)0x00001000)
#define RTC_DATE_MOU   ((uint32_t)0x00000F00)
#define RTC_DATE_MOU_0 ((uint32_t)0x00000100)
#define RTC_DATE_MOU_1 ((uint32_t)0x00000200)
#define RTC_DATE_MOU_2 ((uint32_t)0x00000400)
#define RTC_DATE_MOU_3 ((uint32_t)0x00000800)
#define RTC_DATE_DAT   ((uint32_t)0x00000030)
#define RTC_DATE_DAT_0 ((uint32_t)0x00000010)
#define RTC_DATE_DAT_1 ((uint32_t)0x00000020)
#define RTC_DATE_DAU   ((uint32_t)0x0000000F)
#define RTC_DATE_DAU_0 ((uint32_t)0x00000001)
#define RTC_DATE_DAU_1 ((uint32_t)0x00000002)
#define RTC_DATE_DAU_2 ((uint32_t)0x00000004)
#define RTC_DATE_DAU_3 ((uint32_t)0x00000008)

/********************  Bits definition for RTC_CTRL register  *******************/
#define RTC_CTRL_BAKP     ((uint32_t)0x00040000)
#define RTC_CTRL_SU1H     ((uint32_t)0x00020000)
#define RTC_CTRL_AD1H     ((uint32_t)0x00010000)
#define RTC_CTRL_WTIEN    ((uint32_t)0x00004000)
#define RTC_CTRL_ALAIEN   ((uint32_t)0x00001000)
#define RTC_CTRL_WTEN     ((uint32_t)0x00000400)
#define RTC_CTRL_ALAEN    ((uint32_t)0x00000100)
#define RTC_CTRL_HFMT      ((uint32_t)0x00000040)
#define RTC_CTRL_BYPS      ((uint32_t)0x00000020)
#define RTC_CTRL_WKUPSEL   ((uint32_t)0x00000007)
#define RTC_CTRL_WKUPSEL_0 ((uint32_t)0x00000001)
#define RTC_CTRL_WKUPSEL_1 ((uint32_t)0x00000002)
#define RTC_CTRL_WKUPSEL_2 ((uint32_t)0x00000004)

/********************  Bits definition for RTC_INITSTS register  ******************/
#define RTC_INITSTS_RECPF  ((uint32_t)0x00010000)
#define RTC_INITSTS_WTF    ((uint32_t)0x00000400)
#define RTC_INITSTS_ALAF   ((uint32_t)0x00000100)
#define RTC_INITSTS_INITM  ((uint32_t)0x00000080)
#define RTC_INITSTS_INITF  ((uint32_t)0x00000040)
#define RTC_INITSTS_RSYF   ((uint32_t)0x00000020)
#define RTC_INITSTS_INITSF ((uint32_t)0x00000010)
#define RTC_INITSTS_SHOPF  ((uint32_t)0x00000008)
#define RTC_INITSTS_WTWF   ((uint32_t)0x00000004)
#define RTC_INITSTS_ALAWF  ((uint32_t)0x00000001)

/********************  Bits definition for RTC_PRE register  *****************/
#define RTC_PRE_DIVA ((uint32_t)0x007F0000)
#define RTC_PRE_DIVS ((uint32_t)0x00007FFF)

/********************  Bits definition for RTC_WKUPT register  *****************/
#define RTC_WKUPT_WKUPT ((uint32_t)0x0000FFFF)

/********************  Bits definition for RTC_ALARMA register  ***************/
#define RTC_ALARMA_MASK4  ((uint32_t)0x80000000)
#define RTC_ALARMA_WKDSEL ((uint32_t)0x40000000)
#define RTC_ALARMA_DTT    ((uint32_t)0x30000000)
#define RTC_ALARMA_DTT_0  ((uint32_t)0x10000000)
#define RTC_ALARMA_DTT_1  ((uint32_t)0x20000000)
#define RTC_ALARMA_DTU    ((uint32_t)0x0F000000)
#define RTC_ALARMA_DTU_0  ((uint32_t)0x01000000)
#define RTC_ALARMA_DTU_1  ((uint32_t)0x02000000)
#define RTC_ALARMA_DTU_2  ((uint32_t)0x04000000)
#define RTC_ALARMA_DTU_3  ((uint32_t)0x08000000)
#define RTC_ALARMA_MASK3  ((uint32_t)0x00800000)
#define RTC_ALARMA_APM    ((uint32_t)0x00400000)
#define RTC_ALARMA_HOT    ((uint32_t)0x00300000)
#define RTC_ALARMA_HOT_0  ((uint32_t)0x00100000)
#define RTC_ALARMA_HOT_1  ((uint32_t)0x00200000)
#define RTC_ALARMA_HOU    ((uint32_t)0x000F0000)
#define RTC_ALARMA_HOU_0  ((uint32_t)0x00010000)
#define RTC_ALARMA_HOU_1  ((uint32_t)0x00020000)
#define RTC_ALARMA_HOU_2  ((uint32_t)0x00040000)
#define RTC_ALARMA_HOU_3  ((uint32_t)0x00080000)
#define RTC_ALARMA_MASK2  ((uint32_t)0x00008000)
#define RTC_ALARMA_MIT    ((uint32_t)0x00007000)
#define RTC_ALARMA_MIT_0  ((uint32_t)0x00001000)
#define RTC_ALARMA_MIT_1  ((uint32_t)0x00002000)
#define RTC_ALARMA_MIT_2  ((uint32_t)0x00004000)
#define RTC_ALARMA_MIU    ((uint32_t)0x00000F00)
#define RTC_ALARMA_MIU_0  ((uint32_t)0x00000100)
#define RTC_ALARMA_MIU_1  ((uint32_t)0x00000200)
#define RTC_ALARMA_MIU_2  ((uint32_t)0x00000400)
#define RTC_ALARMA_MIU_3  ((uint32_t)0x00000800)
#define RTC_ALARMA_MASK1  ((uint32_t)0x00000080)
#define RTC_ALARMA_SET    ((uint32_t)0x00000070)
#define RTC_ALARMA_SET_0  ((uint32_t)0x00000010)
#define RTC_ALARMA_SET_1  ((uint32_t)0x00000020)
#define RTC_ALARMA_SET_2  ((uint32_t)0x00000040)
#define RTC_ALARMA_SEU    ((uint32_t)0x0000000F)
#define RTC_ALARMA_SEU_0  ((uint32_t)0x00000001)
#define RTC_ALARMA_SEU_1  ((uint32_t)0x00000002)
#define RTC_ALARMA_SEU_2  ((uint32_t)0x00000004)
#define RTC_ALARMA_SEU_3  ((uint32_t)0x00000008)

/********************  Bits definition for RTC_WRP register  ******************/
#define RTC_WRP_PKEY ((uint32_t)0x000000FF)

/********************  Bits definition for RTC_SUBS register  ******************/
#define RTC_SUBS_SS ((uint32_t)0x0000FFFF)

/********************  Bits definition for RTC_SCTRL register  ***************/
#define RTC_SCTRL_SUBF   ((uint32_t)0x00007FFF)
#define RTC_SCTRL_ADD1S  ((uint32_t)0x80000000)

/********************  Bits definition for RTC_CALIB register  *****************/
#define RTC_CALIB_CP   ((uint32_t)0x00008000)
#define RTC_CALIB_CW8  ((uint32_t)0x00004000)
#define RTC_CALIB_CW16 ((uint32_t)0x00002000)
#define RTC_CALIB_CM   ((uint32_t)0x000001FF)
#define RTC_CALIB_CM_0 ((uint32_t)0x00000001)
#define RTC_CALIB_CM_1 ((uint32_t)0x00000002)
#define RTC_CALIB_CM_2 ((uint32_t)0x00000004)
#define RTC_CALIB_CM_3 ((uint32_t)0x00000008)
#define RTC_CALIB_CM_4 ((uint32_t)0x00000010)
#define RTC_CALIB_CM_5 ((uint32_t)0x00000020)
#define RTC_CALIB_CM_6 ((uint32_t)0x00000040)
#define RTC_CALIB_CM_7 ((uint32_t)0x00000080)
#define RTC_CALIB_CM_8 ((uint32_t)0x00000100)

/********************  Bits definition for RTC_ALRMASS register  *************/
#define RTC_ALRMASS_MASKSSB    ((uint32_t)0x0F000000)
#define RTC_ALRMASS_MASKSSB_0  ((uint32_t)0x00000000)
#define RTC_ALRMASS_MASKSSB_1  ((uint32_t)0x01000000)
#define RTC_ALRMASS_MASKSSB_2  ((uint32_t)0x02000000)
#define RTC_ALRMASS_MASKSSB_3  ((uint32_t)0x03000000)
#define RTC_ALRMASS_MASKSSB_4  ((uint32_t)0x04000000)
#define RTC_ALRMASS_MASKSSB_5  ((uint32_t)0x05000000)
#define RTC_ALRMASS_MASKSSB_6  ((uint32_t)0x06000000)
#define RTC_ALRMASS_MASKSSB_7  ((uint32_t)0x07000000)
#define RTC_ALRMASS_MASKSSB_8  ((uint32_t)0x08000000)
#define RTC_ALRMASS_MASKSSB_9  ((uint32_t)0x09000000)
#define RTC_ALRMASS_MASKSSB_10 ((uint32_t)0x0A000000)
#define RTC_ALRMASS_MASKSSB_11 ((uint32_t)0x0B000000)
#define RTC_ALRMASS_MASKSSB_12 ((uint32_t)0x0C000000)
#define RTC_ALRMASS_MASKSSB_13 ((uint32_t)0x0D000000)
#define RTC_ALRMASS_MASKSSB_14 ((uint32_t)0x0E000000)
#define RTC_ALRMASS_MASKSSB_15 ((uint32_t)0x0F000000)
#define RTC_ALRMASS_SSV        ((uint32_t)0x00007FFF) 

/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for IWDG_KEY register  ********************/
#define IWDG_KEY_KEYV ((uint16_t)0xFFFF) /*!< Key value (write only, read 0000h) */

/*******************  Bit definition for IWDG_PREDIV register  ********************/
#define IWDG_PREDIV_PD ((uint8_t)0x07) /*!< PD[2:0] (Prescaler divider) */
#define IWDG_PR_PR_0   ((uint8_t)0x01) /*!< Bit 0 */
#define IWDG_PR_PR_1   ((uint8_t)0x02) /*!< Bit 1 */
#define IWDG_PR_PR_2   ((uint8_t)0x04) /*!< Bit 2 */

/*******************  Bit definition for IWDG_RELV register  *******************/
#define IWDG_RELV_REL ((uint16_t)0x0FFF) /*!< Watchdog counter reload value */

/*******************  Bit definition for IWDG_STS register  ********************/
#define IWDG_STS_PVU  ((uint8_t)0x01) /*!< Watchdog prescaler value update */
#define IWDG_STS_CRVU ((uint8_t)0x02) /*!< Watchdog counter reload value update */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for WWDG_CTRL register  ********************/
#define WWDG_CTRL_T  ((uint8_t)0x7F) /*!< T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define WWDG_CTRL_T0 ((uint8_t)0x01) /*!< Bit 0 */
#define WWDG_CTRL_T1 ((uint8_t)0x02) /*!< Bit 1 */
#define WWDG_CTRL_T2 ((uint8_t)0x04) /*!< Bit 2 */
#define WWDG_CTRL_T3 ((uint8_t)0x08) /*!< Bit 3 */
#define WWDG_CTRL_T4 ((uint8_t)0x10) /*!< Bit 4 */
#define WWDG_CTRL_T5 ((uint8_t)0x20) /*!< Bit 5 */
#define WWDG_CTRL_T6 ((uint8_t)0x40) /*!< Bit 6 */

#define WWDG_CTRL_ACTB ((uint8_t)0x80) /*!< Activation bit */

/*******************  Bit definition for WWDG_CFG register  *******************/
#define WWDG_CFG_W  ((uint16_t)0x007F) /*!< W[6:0] bits (7-bit window value) */
#define WWDG_CFG_W0 ((uint16_t)0x0001) /*!< Bit 0 */
#define WWDG_CFG_W1 ((uint16_t)0x0002) /*!< Bit 1 */
#define WWDG_CFG_W2 ((uint16_t)0x0004) /*!< Bit 2 */
#define WWDG_CFG_W3 ((uint16_t)0x0008) /*!< Bit 3 */
#define WWDG_CFG_W4 ((uint16_t)0x0010) /*!< Bit 4 */
#define WWDG_CFG_W5 ((uint16_t)0x0020) /*!< Bit 5 */
#define WWDG_CFG_W6 ((uint16_t)0x0040) /*!< Bit 6 */

#define WWDG_CFG_TIMERB  ((uint16_t)0x0180) /*!< TIMERB[1:0] bits (Timer Base) */
#define WWDG_CFG_TIMERB0 ((uint16_t)0x0080) /*!< Bit 0 */
#define WWDG_CFG_TIMERB1 ((uint16_t)0x0100) /*!< Bit 1 */

#define WWDG_CFG_EWINT ((uint16_t)0x0200) /*!< Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_STS register  ********************/
#define WWDG_STS_EWINTF ((uint8_t)0x01) /*!< Early Wakeup Interrupt Flag */

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for SPI_CTRL1 register  ********************/
#define SPI_CTRL1_CLKPHA ((uint16_t)0x0001) /*!< Clock Phase */
#define SPI_CTRL1_CLKPOL ((uint16_t)0x0002) /*!< Clock Polarity */
#define SPI_CTRL1_MSEL   ((uint16_t)0x0004) /*!< Master Selection */

#define SPI_CTRL1_BR  ((uint16_t)0x0038) /*!< BR[2:0] bits (Baud Rate Control) */
#define SPI_CTRL1_BR0 ((uint16_t)0x0008) /*!< Bit 0 */
#define SPI_CTRL1_BR1 ((uint16_t)0x0010) /*!< Bit 1 */
#define SPI_CTRL1_BR2 ((uint16_t)0x0020) /*!< Bit 2 */

#define SPI_CTRL1_SPIEN     ((uint16_t)0x0040) /*!< SPI Enable */
#define SPI_CTRL1_LSBFF     ((uint16_t)0x0080) /*!< Frame Format */
#define SPI_CTRL1_SSEL      ((uint16_t)0x0100) /*!< Internal slave select */
#define SPI_CTRL1_SSMEN     ((uint16_t)0x0200) /*!< Software slave management */
#define SPI_CTRL1_RONLY     ((uint16_t)0x0400) /*!< Receive only */
#define SPI_CTRL1_DATFF     ((uint16_t)0x0800) /*!< Data Frame Format */
#define SPI_CTRL1_CRCNEXT   ((uint16_t)0x1000) /*!< Transmit CRC next */
#define SPI_CTRL1_CRCEN     ((uint16_t)0x2000) /*!< Hardware CRC calculation enable */
#define SPI_CTRL1_BIDIROEN  ((uint16_t)0x4000) /*!< Output enable in bidirectional mode */
#define SPI_CTRL1_BIDIRMODE ((uint16_t)0x8000) /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CTRL2 register  ********************/
#define SPI_CTRL2_RDMAEN   ((uint8_t)0x01) /*!< Rx Buffer DMA Enable */
#define SPI_CTRL2_TDMAEN   ((uint8_t)0x02) /*!< Tx Buffer DMA Enable */
#define SPI_CTRL2_SSOEN    ((uint8_t)0x04) /*!< SS Output Enable */
#define SPI_CTRL2_ERRINTEN ((uint8_t)0x20) /*!< Error Interrupt Enable */
#define SPI_CTRL2_RNEINTEN ((uint8_t)0x40) /*!< RX buffer Not Empty Interrupt Enable */
#define SPI_CTRL2_TEINTEN  ((uint8_t)0x80) /*!< Tx buffer Empty Interrupt Enable */

/********************  Bit definition for SPI_STS register  ********************/
#define SPI_STS_RNE    ((uint8_t)0x01) /*!< Receive buffer Not Empty */
#define SPI_STS_TE     ((uint8_t)0x02) /*!< Transmit buffer Empty */
#define SPI_STS_CRCERR ((uint8_t)0x10) /*!< CRC Error flag */
#define SPI_STS_MODERR ((uint8_t)0x20) /*!< Mode fault */
#define SPI_STS_OVER   ((uint8_t)0x40) /*!< Overrun flag */
#define SPI_STS_BUSY   ((uint8_t)0x80) /*!< Busy flag */

/********************  Bit definition for SPI_DAT register  ********************/
#define SPI_DAT_DAT ((uint16_t)0xFFFF) /*!< Data Register */


/*******************  Bit definition for SPI_CRCPOLY register  ******************/
#define SPI_CRCPOLY_CRCPOLY ((uint16_t)0xFFFF) /*!< CRC polynomial register */

/******************  Bit definition for SPI_CRCRDAT register  ******************/
#define SPI_CRCRDAT_CRCRDAT ((uint16_t)0xFFFF) /*!< Rx CRC Register */

/******************  Bit definition for SPI_CRCTDAT register  ******************/
#define SPI_CRCTDAT_CRCTDAT ((uint16_t)0xFFFF) /*!< Tx CRC Register */

/******************  Bit definition for SPI_I2SCFG register  *****************/
#define SPI_I2SCFG_CHBITS ((uint16_t)0x0001) /*!< Channel length (number of bits per audio channel) */

#define SPI_I2SCFG_TDATLEN  ((uint16_t)0x0006) /*!< TDATLEN[1:0] bits (Data length to be transferred) */
#define SPI_I2SCFG_TDATLEN0 ((uint16_t)0x0002) /*!< Bit 0 */
#define SPI_I2SCFG_TDATLEN1 ((uint16_t)0x0004) /*!< Bit 1 */

#define SPI_I2SCFG_CLKPOL ((uint16_t)0x0008) /*!< steady state clock polarity */

#define SPI_I2SCFG_STDSEL  ((uint16_t)0x0030) /*!< STDSEL[1:0] bits (I2S standard selection) */
#define SPI_I2SCFG_STDSEL0 ((uint16_t)0x0010) /*!< Bit 0 */
#define SPI_I2SCFG_STDSEL1 ((uint16_t)0x0020) /*!< Bit 1 */

#define SPI_I2SCFG_PCMFSYNC ((uint16_t)0x0080) /*!< PCM frame synchronization */

#define SPI_I2SCFG_MODCFG  ((uint16_t)0x0300) /*!< MODCFG[1:0] bits (I2S configuration mode) */
#define SPI_I2SCFG_MODCFG0 ((uint16_t)0x0100) /*!< Bit 0 */
#define SPI_I2SCFG_MODCFG1 ((uint16_t)0x0200) /*!< Bit 1 */

#define SPI_I2SCFG_I2SEN  ((uint16_t)0x0400) /*!< I2S Enable */
#define SPI_I2SCFG_MODSEL ((uint16_t)0x0800) /*!< I2S mode selection */

/******************  Bit definition for SPI_I2SPREDIV register  *******************/
#define SPI_I2SPREDIV_LDIV     ((uint16_t)0x00FF) /*!< I2S Linear prescaler */
#define SPI_I2SPREDIV_ODD_EVEN ((uint16_t)0x0100) /*!< Odd factor for the prescaler */
#define SPI_I2SPREDIV_MCLKOEN  ((uint16_t)0x0200) /*!< Master Clock Output Enable */

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for I2C_CTRL1 register  ********************/
#define I2C_CTRL1_EN       ((uint16_t)0x0001) /*!< Peripheral Enable */
#define I2C_CTRL1_SMBMODE  ((uint16_t)0x0002) /*!< SMBus Mode */
#define I2C_CTRL1_SMBTYPE  ((uint16_t)0x0008) /*!< SMBus Type */
#define I2C_CTRL1_ARPEN    ((uint16_t)0x0010) /*!< ARP Enable */
#define I2C_CTRL1_PECEN    ((uint16_t)0x0020) /*!< PEC Enable */
#define I2C_CTRL1_GCEN     ((uint16_t)0x0040) /*!< General Call Enable */
#define I2C_CTRL1_NOEXTEND ((uint16_t)0x0080) /*!< Clock Stretching Disable (Slave mode) */
#define I2C_CTRL1_STARTGEN ((uint16_t)0x0100) /*!< Start Generation */
#define I2C_CTRL1_STOPGEN  ((uint16_t)0x0200) /*!< Stop Generation */
#define I2C_CTRL1_ACKEN    ((uint16_t)0x0400) /*!< Acknowledge Enable */
#define I2C_CTRL1_ACKPOS   ((uint16_t)0x0800) /*!< Acknowledge/PEC Position (for data reception) */
#define I2C_CTRL1_PEC      ((uint16_t)0x1000) /*!< Packet Error Checking */
#define I2C_CTRL1_SMBALERT ((uint16_t)0x2000) /*!< SMBus Alert */
#define I2C_CTRL1_SWRESET  ((uint16_t)0x8000) /*!< Software Reset */

/*******************  Bit definition for I2C_CTRL2 register  ********************/
#define I2C_CTRL2_CLKFREQ   ((uint16_t)0x003F) /*!< FREQ[5:0] bits (Peripheral Clock Frequency) */
#define I2C_CTRL2_CLKFREQ_0 ((uint16_t)0x0001) /*!< Bit 0 */
#define I2C_CTRL2_CLKFREQ_1 ((uint16_t)0x0002) /*!< Bit 1 */
#define I2C_CTRL2_CLKFREQ_2 ((uint16_t)0x0004) /*!< Bit 2 */
#define I2C_CTRL2_CLKFREQ_3 ((uint16_t)0x0008) /*!< Bit 3 */
#define I2C_CTRL2_CLKFREQ_4 ((uint16_t)0x0010) /*!< Bit 4 */
#define I2C_CTRL2_CLKFREQ_5 ((uint16_t)0x0020) /*!< Bit 5 */

#define I2C_CTRL2_ERRINTEN ((uint16_t)0x0100) /*!< Error Interrupt Enable */
#define I2C_CTRL2_EVTINTEN ((uint16_t)0x0200) /*!< Event Interrupt Enable */
#define I2C_CTRL2_BUFINTEN ((uint16_t)0x0400) /*!< Buffer Interrupt Enable */
#define I2C_CTRL2_DMAEN    ((uint16_t)0x0800) /*!< DMA Requests Enable */
#define I2C_CTRL2_DMALAST  ((uint16_t)0x1000) /*!< DMA Last Transfer */

/*******************  Bit definition for I2C_OADDR1 register  *******************/
#define I2C_OADDR1_ADDR1_7 ((uint16_t)0x00FE) /*!< Interface Address */
#define I2C_OADDR1_ADDR8_9 ((uint16_t)0x0300) /*!< Interface Address */

#define I2C_OADDR1_ADDR0 ((uint16_t)0x0001) /*!< Bit 0 */
#define I2C_OADDR1_ADDR1 ((uint16_t)0x0002) /*!< Bit 1 */
#define I2C_OADDR1_ADDR2 ((uint16_t)0x0004) /*!< Bit 2 */
#define I2C_OADDR1_ADDR3 ((uint16_t)0x0008) /*!< Bit 3 */
#define I2C_OADDR1_ADDR4 ((uint16_t)0x0010) /*!< Bit 4 */
#define I2C_OADDR1_ADDR5 ((uint16_t)0x0020) /*!< Bit 5 */
#define I2C_OADDR1_ADDR6 ((uint16_t)0x0040) /*!< Bit 6 */
#define I2C_OADDR1_ADDR7 ((uint16_t)0x0080) /*!< Bit 7 */
#define I2C_OADDR1_ADDR8 ((uint16_t)0x0100) /*!< Bit 8 */
#define I2C_OADDR1_ADDR9 ((uint16_t)0x0200) /*!< Bit 9 */

#define I2C_OADDR1_ADDRMODE ((uint16_t)0x8000) /*!< Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OADDR2 register  *******************/
#define I2C_OADDR2_DUALEN ((uint8_t)0x01) /*!< Dual addressing mode enable */
#define I2C_OADDR2_ADDR2  ((uint8_t)0xFE) /*!< Interface address */

/********************  Bit definition for I2C_DAT register  ********************/
#define I2C_DAT_DATA ((uint8_t)0xFF) /*!< 8-bit Data Register */

/*******************  Bit definition for I2C_STS1 register  ********************/
#define I2C_STS1_STARTBF  ((uint16_t)0x0001) /*!< Start Bit (Master mode) */
#define I2C_STS1_ADDRF    ((uint16_t)0x0002) /*!< Address sent (master mode)/matched (slave mode) */
#define I2C_STS1_BSF      ((uint16_t)0x0004) /*!< Byte Transfer Finished */
#define I2C_STS1_ADDR10F  ((uint16_t)0x0008) /*!< 10-bit header sent (Master mode) */
#define I2C_STS1_STOPF    ((uint16_t)0x0010) /*!< Stop detection (Slave mode) */
#define I2C_STS1_RXDATNE  ((uint16_t)0x0040) /*!< Data Register not Empty (receivers) */
#define I2C_STS1_TXDATE   ((uint16_t)0x0080) /*!< Data Register Empty (transmitters) */
#define I2C_STS1_BUSERR   ((uint16_t)0x0100) /*!< Bus Error */
#define I2C_STS1_ARLOST   ((uint16_t)0x0200) /*!< Arbitration Lost (master mode) */
#define I2C_STS1_ACKFAIL  ((uint16_t)0x0400) /*!< Acknowledge Failure */
#define I2C_STS1_OVERRUN  ((uint16_t)0x0800) /*!< Overrun/Underrun */
#define I2C_STS1_PECERR   ((uint16_t)0x1000) /*!< PEC Error in reception */
#define I2C_STS1_TIMOUT   ((uint16_t)0x4000) /*!< Timeout or Tlow Error */
#define I2C_STS1_SMBALERT ((uint16_t)0x8000) /*!< SMBus Alert */

/*******************  Bit definition for I2C_STS2 register  ********************/
#define I2C_STS2_MSMODE    ((uint16_t)0x0001) /*!< Master/Slave */
#define I2C_STS2_BUSY      ((uint16_t)0x0002) /*!< Bus Busy */
#define I2C_STS2_TRF       ((uint16_t)0x0004) /*!< Transmitter/Receiver */
#define I2C_STS2_GCALLADDR ((uint16_t)0x0010) /*!< General Call Address (Slave mode) */
#define I2C_STS2_SMBDADDR  ((uint16_t)0x0020) /*!< SMBus Device Default Address (Slave mode) */
#define I2C_STS2_SMBHADDR  ((uint16_t)0x0040) /*!< SMBus Host Header (Slave mode) */
#define I2C_STS2_DUALFLAG  ((uint16_t)0x0080) /*!< Dual Flag (Slave mode) */
#define I2C_STS2_PECVAL    ((uint16_t)0xFF00) /*!< Packet Error Checking Register */

/*******************  Bit definition for I2C_CLKCTRL register  ********************/
#define I2C_CLKCTRL_CLKCTRL ((uint16_t)0x0FFF) /*!< Clock Control Register in Fast/Standard mode (Master mode) */
#define I2C_CLKCTRL_DUTY    ((uint16_t)0x4000) /*!< Fast Mode Duty Cycle */
#define I2C_CLKCTRL_FSMODE  ((uint16_t)0x8000) /*!< I2C Master Mode Selection */

/******************  Bit definition for I2C_TRISE register  *******************/
#define I2C_TMRISE_TMRISE ((uint8_t)0x3F) /*!< Maximum Rise Time in Fast/Standard mode (Master mode) */

/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for USART_STS register  *******************/
#define USART_STS_PEF    ((uint16_t)0x0001) /*!< Parity Error */
#define USART_STS_FEF    ((uint16_t)0x0002) /*!< Framing Error */
#define USART_STS_NEF    ((uint16_t)0x0004) /*!< Noise Error Flag */
#define USART_STS_OREF   ((uint16_t)0x0008) /*!< OverRun Error */
#define USART_STS_IDLEF  ((uint16_t)0x0010) /*!< IDLE line detected */
#define USART_STS_RXDNE  ((uint16_t)0x0020) /*!< Read Data Register Not Empty */
#define USART_STS_TXC    ((uint16_t)0x0040) /*!< Transmission Complete */
#define USART_STS_TXDE   ((uint16_t)0x0080) /*!< Transmit Data Register Empty */
#define USART_STS_LINBDF ((uint16_t)0x0100) /*!< LIN Break Detection Flag */
#define USART_STS_CTSF   ((uint16_t)0x0200) /*!< CTS Flag */

/*******************  Bit definition for USART_DAT register  *******************/
#define USART_DAT_DATV ((uint16_t)0x01FF) /*!< Data value */

/******************  Bit definition for USART_BRCF register  *******************/
#define USART_BRCF_DIV_Decimal ((uint16_t)0x000F) /*!< Fraction of USARTDIV */
#define USART_BRCF_DIV_Integer ((uint16_t)0xFFF0) /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_CTRL1 register  *******************/
#define USART_CTRL1_SDBRK    ((uint16_t)0x0001) /*!< Send Break */
#define USART_CTRL1_RCVWU    ((uint16_t)0x0002) /*!< Receiver wakeup */
#define USART_CTRL1_RXEN     ((uint16_t)0x0004) /*!< Receiver Enable */
#define USART_CTRL1_TXEN     ((uint16_t)0x0008) /*!< Transmitter Enable */
#define USART_CTRL1_IDLEIEN  ((uint16_t)0x0010) /*!< IDLE Interrupt Enable */
#define USART_CTRL1_RXDNEIEN ((uint16_t)0x0020) /*!< RXNE Interrupt Enable */
#define USART_CTRL1_TXCIEN   ((uint16_t)0x0040) /*!< Transmission Complete Interrupt Enable */
#define USART_CTRL1_TXDEIEN  ((uint16_t)0x0080) /*!< PE Interrupt Enable */
#define USART_CTRL1_PEIEN    ((uint16_t)0x0100) /*!< PE Interrupt Enable */
#define USART_CTRL1_PSEL     ((uint16_t)0x0200) /*!< Parity Selection */
#define USART_CTRL1_PCEN     ((uint16_t)0x0400) /*!< Parity Control Enable */
#define USART_CTRL1_WUM      ((uint16_t)0x0800) /*!< Wakeup method */
#define USART_CTRL1_WL       ((uint16_t)0x1000) /*!< Word length */
#define USART_CTRL1_UEN      ((uint16_t)0x2000) /*!< USART Enable */

/******************  Bit definition for USART_CTRL2 register  *******************/
#define USART_CTRL2_ADDR        ((uint16_t)0x000F) /*!< Address of the USART node */
#define USART_CTRL2_LINBDL      ((uint16_t)0x0020) /*!< LIN Break Detection Length */
#define USART_CTRL2_LINBDIEN    ((uint16_t)0x0040) /*!< LIN Break Detection Interrupt Enable */
#define USART_CTRL2_LBCLK       ((uint16_t)0x0100) /*!< Last Bit Clock pulse */
#define USART_CTRL2_CLKPHA      ((uint16_t)0x0200) /*!< Clock Phase */
#define USART_CTRL2_CLKPOL      ((uint16_t)0x0400) /*!< Clock Polarity */
#define USART_CTRL2_CLKEN       ((uint16_t)0x0800) /*!< Clock Enable */

#define USART_CTRL2_STPB   ((uint16_t)0x3000) /*!< STOP[1:0] bits (STOP bits) */
#define USART_CTRL2_STPB_0 ((uint16_t)0x1000) /*!< Bit 0 */
#define USART_CTRL2_STPB_1 ((uint16_t)0x2000) /*!< Bit 1 */

#define USART_CTRL2_LINMEN ((uint16_t)0x4000) /*!< LIN mode enable */

/******************  Bit definition for USART_CTRL3 register  *******************/
#define USART_CTRL3_ERRIEN  ((uint16_t)0x0001) /*!< Error Interrupt Enable */
#define USART_CTRL3_IRDAMEN ((uint16_t)0x0002) /*!< IrDA mode Enable */
#define USART_CTRL3_IRDALP  ((uint16_t)0x0004) /*!< IrDA Low-Power */
#define USART_CTRL3_HDMEN   ((uint16_t)0x0008) /*!< Half-Duplex Selection */
#define USART_CTRL3_SCNACK  ((uint16_t)0x0010) /*!< Smartcard NACK enable */
#define USART_CTRL3_SCMEN   ((uint16_t)0x0020) /*!< Smartcard mode enable */
#define USART_CTRL3_DMARXEN ((uint16_t)0x0040) /*!< DMA Enable Receiver */
#define USART_CTRL3_DMATXEN ((uint16_t)0x0080) /*!< DMA Enable Transmitter */
#define USART_CTRL3_RTSEN   ((uint16_t)0x0100) /*!< RTS Enable */
#define USART_CTRL3_CTSEN   ((uint16_t)0x0200) /*!< CTS Enable */
#define USART_CTRL3_CTSIEN  ((uint16_t)0x0400) /*!< CTS Interrupt Enable */

/******************  Bit definition for USART_GTP register  ******************/
#define USART_GTP_PSCV   ((uint16_t)0x00FF) /*!< PSC[7:0] bits (Prescaler value) */
#define USART_GTP_PSCV_0 ((uint16_t)0x0001) /*!< Bit 0 */
#define USART_GTP_PSCV_1 ((uint16_t)0x0002) /*!< Bit 1 */
#define USART_GTP_PSCV_2 ((uint16_t)0x0004) /*!< Bit 2 */
#define USART_GTP_PSCV_3 ((uint16_t)0x0008) /*!< Bit 3 */
#define USART_GTP_PSCV_4 ((uint16_t)0x0010) /*!< Bit 4 */
#define USART_GTP_PSCV_5 ((uint16_t)0x0020) /*!< Bit 5 */
#define USART_GTP_PSCV_6 ((uint16_t)0x0040) /*!< Bit 6 */
#define USART_GTP_PSCV_7 ((uint16_t)0x0080) /*!< Bit 7 */

#define USART_GTP_GTV ((uint16_t)0xFF00) /*!< Guard time value */

/******************************************************************************/
/*                                                                            */
/*          Low-power Universal Asynchronous Receiver Transmitter             */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for LPUART_STS register  ******************/
#define LPUART_STS_PEF      ((uint16_t)0x0001) /*!< Parity Check Error Flag */
#define LPUART_STS_TXC      ((uint16_t)0x0002) /*!< TX Complete Flag */
#define LPUART_STS_FIFO_OV  ((uint16_t)0x0004) /*!< FIFO Overflow Flag */
#define LPUART_STS_FIFO_FU  ((uint16_t)0x0008) /*!< FIFO Full Flag */
#define LPUART_STS_FIFO_HF  ((uint16_t)0x0010) /*!< FIFO Half Full Flag */
#define LPUART_STS_FIFO_NE  ((uint16_t)0x0020) /*!< FIFO Non-Empty Flag */
#define LPUART_STS_CTS      ((uint16_t)0x0040) /*!< Clear to Send (Hardware Flow Control) Flag */
#define LPUART_STS_WUF      ((uint16_t)0x0080) /*!< Wakeup from Stop mode Flag */
#define LPUART_STS_NF       ((uint16_t)0x0100) /*!< Noise Detected Flag */

/******************  Bit definition for LPUART_INTEN register  ******************/
#define LPUART_INTEN_PEIE       ((uint8_t)0x01) /*!< Parity Check Error Interrupt Enable */
#define LPUART_INTEN_TXCIE      ((uint8_t)0x02) /*!< TX Complete Interrupt Enable */
#define LPUART_INTEN_FIFO_OVIE  ((uint8_t)0x04) /*!< FIFO Overflow Interrupt Enable */
#define LPUART_INTEN_FIFO_FUIE  ((uint8_t)0x08) /*!< FIFO Full Interrupt Enable*/
#define LPUART_INTEN_FIFO_HFIE  ((uint8_t)0x10) /*!< FIFO Half Full Interrupt Enable */
#define LPUART_INTEN_FIFO_NEIE  ((uint8_t)0x20) /*!< FIFO Non-Empty Interrupt Enable */
#define LPUART_INTEN_WUFIE      ((uint8_t)0x40) /*!< Wakeup Interrupt Enable */

/******************  Bit definition for LPUART_CTRL register  ******************/
#define LPUART_CTRL_PSEL      ((uint16_t)0x0001) /*!< Odd Parity Bit Enable */
#define LPUART_CTRL_TXEN      ((uint16_t)0x0002) /*!< TX Enable */
#define LPUART_CTRL_FLUSH     ((uint16_t)0x0004) /*!< Flush Receiver FIFO Enable */
#define LPUART_CTRL_PCDIS     ((uint16_t)0x0008) /*!< Parity Control Disable */
#define LPUART_CTRL_LOOPBACK  ((uint16_t)0x0010) /*!< Loop Back Self-Test */
#define LPUART_CTRL_DMA_TXEN  ((uint16_t)0x0020) /*!< DMA TX Request Enable */
#define LPUART_CTRL_DMA_RXEN  ((uint16_t)0x0040) /*!< DMA RX Request Enable */
#define LPUART_CTRL_WUSTP     ((uint16_t)0x0080) /*!< LPUART Wakeup Enable in Stop mode */
#define LPUART_CTRL_RTS_THSEL ((uint16_t)0x0300) /*!< RTS Threshold Selection */
#define LPUART_CTRL_CTSEN     ((uint16_t)0x0400) /*!< Hardware Flow Control TX Enable */
#define LPUART_CTRL_RTSEN     ((uint16_t)0x0800) /*!< Hardware Flow Control RX Enable */
#define LPUART_CTRL_WUSEL     ((uint16_t)0x3000) /*!< Wakeup Event Selection */
#define LPUART_CTRL_SMPCNT    ((uint16_t)0x4000) /*!< Specify the Sampling Method */

/******************  Bit definition for LPUART_BRCFG1 register  ******************/
#define LPUART_BRCFG1_INTEGER  ((uint16_t)0xFFFF) /*!< Baud Rate Parameter Configeration Register1: Fraction */

/******************  Bit definition for LPUART_DAT register  ******************/
#define LPUART_DAT_DAT  ((uint8_t)0xFF) /*!< Data Register */

/******************  Bit definition for LPUART_BRCFG2 register  ******************/
#define LPUART_BRCFG2_DECIMAL  ((uint8_t)0xFF) /*!< Baud Rate Parameter Configeration Register2: Mantissa */

/******************  Bit definition for LPUART_WUDAT register  ******************/
#define LPUART_WUDAT_WUDAT  ((uint32_t)0xFFFFFFFF) /*!< Data Register */

/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                General Purpose and Alternate Function I/O                  */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for GPIO_PMODE register  *******************/
#define GPIO_PMODE0_Pos            (0)                                   
#define GPIO_PMODE0_Msk            (0x3UL << GPIO_PMODE0_Pos)         /*!< 0x00000003 */ 
#define GPIO_PMODE0                GPIO_PMODE0_Msk
#define GPIO_PMODE0_0              (0x0UL << GPIO_PMODE0_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE0_1              (0x1UL << GPIO_PMODE0_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE0_2              (0x2UL << GPIO_PMODE0_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE0_3              (0x3UL << GPIO_PMODE0_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE1_Pos            (2)                                   
#define GPIO_PMODE1_Msk            (0x3UL << GPIO_PMODE1_Pos)         /*!< 0x00000003 */
#define GPIO_PMODE1                GPIO_PMODE1_Msk
#define GPIO_PMODE1_0              (0x0UL << GPIO_PMODE1_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE1_1              (0x1UL << GPIO_PMODE1_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE1_2              (0x2UL << GPIO_PMODE1_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE1_3              (0x3UL << GPIO_PMODE1_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE2_Pos            (4)                                   
#define GPIO_PMODE2_Msk            (0x3UL << GPIO_PMODE2_Pos)         /*!< 0x00000003 */                 
#define GPIO_PMODE2                GPIO_PMODE2_Msk
#define GPIO_PMODE2_0              (0x0UL << GPIO_PMODE2_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE2_1              (0x1UL << GPIO_PMODE2_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE2_2              (0x2UL << GPIO_PMODE2_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE2_3              (0x3UL << GPIO_PMODE2_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE3_Pos            (6)                                   
#define GPIO_PMODE3_Msk            (0x3UL << GPIO_PMODE3_Pos)         /*!< 0x00000003 */ 
#define GPIO_PMODE3                GPIO_PMODE3_Msk
#define GPIO_PMODE3_0              (0x0UL << GPIO_PMODE3_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE3_1              (0x1UL << GPIO_PMODE3_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE3_2              (0x2UL << GPIO_PMODE3_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE3_3              (0x3UL << GPIO_PMODE3_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE4_Pos            (8)                                   
#define GPIO_PMODE4_Msk            (0x3UL << GPIO_PMODE4_Pos)         /*!< 0x00000003 */ 
#define GPIO_PMODE4                GPIO_PMODE4_Msk
#define GPIO_PMODE4_0              (0x0UL << GPIO_PMODE4_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE4_1              (0x1UL << GPIO_PMODE4_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE4_2              (0x2UL << GPIO_PMODE4_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE4_3              (0x3UL << GPIO_PMODE4_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE5_Pos            (10)                                   
#define GPIO_PMODE5_Msk            (0x3UL << GPIO_PMODE5_Pos)         /*!< 0x00000003 */
#define GPIO_PMODE5                GPIO_PMODE5_Msk
#define GPIO_PMODE5_0              (0x0UL << GPIO_PMODE5_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE5_1              (0x1UL << GPIO_PMODE5_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE5_2              (0x2UL << GPIO_PMODE5_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE5_3              (0x3UL << GPIO_PMODE5_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE6_Pos            (12)                                   
#define GPIO_PMODE6_Msk            (0x3UL << GPIO_PMODE6_Pos)         /*!< 0x00000003 */      
#define GPIO_PMODE6                GPIO_PMODE6_Msk
#define GPIO_PMODE6_0              (0x0UL << GPIO_PMODE6_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE6_1              (0x1UL << GPIO_PMODE6_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE6_2              (0x2UL << GPIO_PMODE6_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE6_3              (0x3UL << GPIO_PMODE6_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE7_Pos            (14)                                   
#define GPIO_PMODE7_Msk            (0x3UL << GPIO_PMODE7_Pos)         /*!< 0x00000003 */
#define GPIO_PMODE7                GPIO_PMODE7_Msk
#define GPIO_PMODE7_0              (0x0UL << GPIO_PMODE7_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE7_1              (0x1UL << GPIO_PMODE7_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE7_2              (0x2UL << GPIO_PMODE7_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE7_3              (0x3UL << GPIO_PMODE7_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE8_Pos            (16)                                   
#define GPIO_PMODE8_Msk            (0x3UL << GPIO_PMODE8_Pos)         /*!< 0x00000003 */
#define GPIO_PMODE8                GPIO_PMODE8_Msk
#define GPIO_PMODE8_0              (0x0UL << GPIO_PMODE8_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE8_1              (0x1UL << GPIO_PMODE8_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE8_2              (0x2UL << GPIO_PMODE8_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE8_3              (0x3UL << GPIO_PMODE8_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE9_Pos            (18)                                   
#define GPIO_PMODE9_Msk            (0x3UL << GPIO_PMODE9_Pos)         /*!< 0x00000003 */
#define GPIO_PMODE9                GPIO_PMODE9_Msk
#define GPIO_PMODE9_0              (0x0UL << GPIO_PMODE9_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE9_1              (0x1UL << GPIO_PMODE9_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE9_2              (0x2UL << GPIO_PMODE9_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE9_3              (0x3UL << GPIO_PMODE9_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE10_Pos            (20)                                   
#define GPIO_PMODE10_Msk            (0x3UL << GPIO_PMODE10_Pos)         /*!< 0x00000003 */ 
#define GPIO_PMODE10                GPIO_PMODE10_Msk
#define GPIO_PMODE10_0              (0x0UL << GPIO_PMODE10_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE10_1              (0x1UL << GPIO_PMODE10_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE10_2              (0x2UL << GPIO_PMODE10_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE10_3              (0x3UL << GPIO_PMODE10_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE11_Pos            (22)                                   
#define GPIO_PMODE11_Msk            (0x3UL << GPIO_PMODE11_Pos)         /*!< 0x00000003 */ 
#define GPIO_PMODE11                GPIO_PMODE11_Msk
#define GPIO_PMODE11_0              (0x0UL << GPIO_PMODE11_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE11_1              (0x1UL << GPIO_PMODE11_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE11_2              (0x2UL << GPIO_PMODE11_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE11_3              (0x3UL << GPIO_PMODE11_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE12_Pos            (24)                                   
#define GPIO_PMODE12_Msk            (0x3UL << GPIO_PMODE12_Pos)         /*!< 0x00000003 */ 
#define GPIO_PMODE12                GPIO_PMODE12_Msk
#define GPIO_PMODE12_0              (0x0UL << GPIO_PMODE12_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE12_1              (0x1UL << GPIO_PMODE12_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE12_2              (0x2UL << GPIO_PMODE12_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE12_3              (0x3UL << GPIO_PMODE12_Pos)         /*!< 0x00000003 */

#define GPIO_PMODE13_Pos            (26)                                   
#define GPIO_PMODE13_Msk            (0x3UL << GPIO_PMODE13_Pos)         /*!< 0x00000003 */ 
#define GPIO_PMODE13                GPIO_PMODE13_Msk
#define GPIO_PMODE13_0              (0x0UL << GPIO_PMODE13_Pos)         /*!< 0x00000000 */
#define GPIO_PMODE13_1              (0x1UL << GPIO_PMODE13_Pos)         /*!< 0x00000001 */
#define GPIO_PMODE13_2              (0x2UL << GPIO_PMODE13_Pos)         /*!< 0x00000002 */
#define GPIO_PMODE13_3              (0x3UL << GPIO_PMODE13_Pos)         /*!< 0x00000003 */

/******************  Bit definition for GPIO_POTYPE register  *****************/
#define GPIO_POTYPE_POT_0                (0x00000001)                          
#define GPIO_POTYPE_POT_1                (0x00000002)                          
#define GPIO_POTYPE_POT_2                (0x00000004)                          
#define GPIO_POTYPE_POT_3                (0x00000008)                          
#define GPIO_POTYPE_POT_4                (0x00000010)                          
#define GPIO_POTYPE_POT_5                (0x00000020)                          
#define GPIO_POTYPE_POT_6                (0x00000040)                          
#define GPIO_POTYPE_POT_7                (0x00000080)                          
#define GPIO_POTYPE_POT_8                (0x00000100)                          
#define GPIO_POTYPE_POT_9                (0x00000200)                          
#define GPIO_POTYPE_POT_10               (0x00000400)                          
#define GPIO_POTYPE_POT_11               (0x00000800)                          
#define GPIO_POTYPE_POT_12               (0x00001000)                          
#define GPIO_POTYPE_POT_13               (0x00002000)                           

/*******************  Bit definition for GPIO_SR register  *******************/
#define GPIO_SR_SR0  ((uint16_t)0x0001) /*!<  Slew rate bit 0 */
#define GPIO_SR_SR1  ((uint16_t)0x0002) /*!<  Slew rate bit 1 */
#define GPIO_SR_SR2  ((uint16_t)0x0004) /*!<  Slew rate bit 2 */
#define GPIO_SR_SR3  ((uint16_t)0x0008) /*!<  Slew rate bit 3 */
#define GPIO_SR_SR4  ((uint16_t)0x0010) /*!<  Slew rate bit 4 */
#define GPIO_SR_SR5  ((uint16_t)0x0020) /*!<  Slew rate bit 5 */
#define GPIO_SR_SR6  ((uint16_t)0x0040) /*!<  Slew rate bit 6 */
#define GPIO_SR_SR7  ((uint16_t)0x0080) /*!<  Slew rate bit 7 */
#define GPIO_SR_SR8  ((uint16_t)0x0100) /*!<  Slew rate bit 8 */
#define GPIO_SR_SR9  ((uint16_t)0x0200) /*!<  Slew rate bit 9 */
#define GPIO_SR_SR10 ((uint16_t)0x0400) /*!<  Slew rate bit 10 */
#define GPIO_SR_SR11 ((uint16_t)0x0800) /*!<  Slew rate bit 11 */
#define GPIO_SR_SR12 ((uint16_t)0x1000) /*!<  Slew rate bit 12 */
#define GPIO_SR_SR13 ((uint16_t)0x2000) /*!<  Slew rate bit 13 */

/*******************  Bit definition for GPIO_PUPD register ******************/
#define GPIO_PUPD0_Pos            (0)                                   
#define GPIO_PUPD0_Msk            (0x3UL << GPIO_PUPD0_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD0                GPIO_PUPD0_Msk                   
#define GPIO_PUPD0_0              (0x0UL << GPIO_PUPD0_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD0_1              (0x1UL << GPIO_PUPD0_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD0_2              (0x2UL << GPIO_PUPD0_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD1_Pos            (2)                                   
#define GPIO_PUPD1_Msk            (0x3UL << GPIO_PUPD1_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD1                GPIO_PUPD1_Msk                   
#define GPIO_PUPD1_0              (0x0UL << GPIO_PUPD1_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD1_1              (0x1UL << GPIO_PUPD1_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD1_2              (0x2UL << GPIO_PUPD1_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD2_Pos             (4)                                   
#define GPIO_PUPD2_Msk            (0x3UL << GPIO_PUPD2_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD2                GPIO_PUPD2_Msk                   
#define GPIO_PUPD2_0              (0x0UL << GPIO_PUPD2_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD2_1              (0x1UL << GPIO_PUPD2_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD2_2              (0x2UL << GPIO_PUPD2_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD3_Pos            (6)                                   
#define GPIO_PUPD3_Msk            (0x3UL << GPIO_PUPD3_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD3                GPIO_PUPD3_Msk                   
#define GPIO_PUPD3_0              (0x0UL << GPIO_PUPD3_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD3_1              (0x1UL << GPIO_PUPD3_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD3_2              (0x2UL << GPIO_PUPD3_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD4_Pos            (8)                                   
#define GPIO_PUPD4_Msk            (0x3UL << GPIO_PUPD4_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD4                GPIO_PUPD4_Msk                   
#define GPIO_PUPD4_0              (0x0UL << GPIO_PUPD4_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD4_1              (0x1UL << GPIO_PUPD4_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD4_2              (0x2UL << GPIO_PUPD4_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD5_Pos            (10)                                   
#define GPIO_PUPD5_Msk            (0x3UL << GPIO_PUPD5_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD5                GPIO_PUPD5_Msk                   
#define GPIO_PUPD5_0              (0x0UL << GPIO_PUPD5_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD5_1              (0x1UL << GPIO_PUPD5_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD5_2              (0x2UL << GPIO_PUPD5_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD6_Pos            (12)                                   
#define GPIO_PUPD6_Msk            (0x3UL << GPIO_PUPD6_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD6                GPIO_PUPD6_Msk                   
#define GPIO_PUPD6_0              (0x0UL << GPIO_PUPD6_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD6_1              (0x1UL << GPIO_PUPD6_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD6_2              (0x2UL << GPIO_PUPD6_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD7_Pos            (14)                                   
#define GPIO_PUPD7_Msk            (0x3UL << GPIO_PUPD7_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD7                GPIO_PUPD7_Msk                   
#define GPIO_PUPD7_0              (0x0UL << GPIO_PUPD7_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD7_1              (0x1UL << GPIO_PUPD7_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD7_2              (0x2UL << GPIO_PUPD7_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD8_Pos            (16)                                   
#define GPIO_PUPD8_Msk            (0x3UL << GPIO_PUPD8_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD8                GPIO_PUPD8_Msk                   
#define GPIO_PUPD8_0              (0x0UL << GPIO_PUPD8_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD8_1              (0x1UL << GPIO_PUPD8_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD8_2              (0x2UL << GPIO_PUPD8_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD9_Pos            (18)                                   
#define GPIO_PUPD9_Msk            (0x3UL << GPIO_PUPD9_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD9                GPIO_PUPD9_Msk                   
#define GPIO_PUPD9_0              (0x0UL << GPIO_PUPD9_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD9_1              (0x1UL << GPIO_PUPD9_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD9_2              (0x2UL << GPIO_PUPD9_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD10_Pos             (20)                                   
#define GPIO_PUPD10_Msk            (0x3UL << GPIO_PUPD10_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD10                GPIO_PUPD10_Msk                   
#define GPIO_PUPD10_0              (0x0UL << GPIO_PUPD10_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD10_1              (0x1UL << GPIO_PUPD10_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD10_2              (0x2UL << GPIO_PUPD10_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD11_Pos            (22)                                   
#define GPIO_PUPD11_Msk            (0x3UL << GPIO_PUPD11_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD11                GPIO_PUPD11_Msk                   
#define GPIO_PUPD11_0              (0x0UL << GPIO_PUPD11_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD11_1              (0x1UL << GPIO_PUPD11_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD11_2              (0x2UL << GPIO_PUPD11_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD12_Pos            (24)                                   
#define GPIO_PUPD12_Msk            (0x3UL << GPIO_PUPD12_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD12                GPIO_PUPD12_Msk                   
#define GPIO_PUPD12_0              (0x0UL << GPIO_PUPD12_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD12_1              (0x1UL << GPIO_PUPD12_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD12_2              (0x2UL << GPIO_PUPD12_Pos)         /*!< 0x00000002 */

#define GPIO_PUPD13_Pos            (26)                                   
#define GPIO_PUPD13_Msk            (0x3UL << GPIO_PUPD13_Pos)         /*!< 0x00000003 */
#define GPIO_PUPD13                GPIO_PUPD13_Msk                   
#define GPIO_PUPD13_0              (0x0UL << GPIO_PUPD13_Pos)         /*!< 0x00000000 */
#define GPIO_PUPD13_1              (0x1UL << GPIO_PUPD13_Pos)         /*!< 0x00000001 */
#define GPIO_PUPD13_2              (0x2UL << GPIO_PUPD13_Pos)         /*!< 0x00000002 */

/*!<******************  Bit definition for GPIO_PID register  *******************/
#define GPIO_PID_PID0  ((uint16_t)0x0001) /*!< Port input data, bit 0 */
#define GPIO_PID_PID1  ((uint16_t)0x0002) /*!< Port input data, bit 1 */
#define GPIO_PID_PID2  ((uint16_t)0x0004) /*!< Port input data, bit 2 */
#define GPIO_PID_PID3  ((uint16_t)0x0008) /*!< Port input data, bit 3 */
#define GPIO_PID_PID4  ((uint16_t)0x0010) /*!< Port input data, bit 4 */
#define GPIO_PID_PID5  ((uint16_t)0x0020) /*!< Port input data, bit 5 */
#define GPIO_PID_PID6  ((uint16_t)0x0040) /*!< Port input data, bit 6 */
#define GPIO_PID_PID7  ((uint16_t)0x0080) /*!< Port input data, bit 7 */
#define GPIO_PID_PID8  ((uint16_t)0x0100) /*!< Port input data, bit 8 */
#define GPIO_PID_PID9  ((uint16_t)0x0200) /*!< Port input data, bit 9 */
#define GPIO_PID_PID10 ((uint16_t)0x0400) /*!< Port input data, bit 10 */
#define GPIO_PID_PID11 ((uint16_t)0x0800) /*!< Port input data, bit 11 */
#define GPIO_PID_PID12 ((uint16_t)0x1000) /*!< Port input data, bit 12 */
#define GPIO_PID_PID13 ((uint16_t)0x2000) /*!< Port input data, bit 13 */

/*******************  Bit definition for GPIO_POD register  *******************/
#define GPIO_POD_POD0  ((uint16_t)0x0001) /*!< Port output data, bit 0 */
#define GPIO_POD_POD1  ((uint16_t)0x0002) /*!< Port output data, bit 1 */
#define GPIO_POD_POD2  ((uint16_t)0x0004) /*!< Port output data, bit 2 */
#define GPIO_POD_POD3  ((uint16_t)0x0008) /*!< Port output data, bit 3 */
#define GPIO_POD_POD4  ((uint16_t)0x0010) /*!< Port output data, bit 4 */
#define GPIO_POD_POD5  ((uint16_t)0x0020) /*!< Port output data, bit 5 */
#define GPIO_POD_POD6  ((uint16_t)0x0040) /*!< Port output data, bit 6 */
#define GPIO_POD_POD7  ((uint16_t)0x0080) /*!< Port output data, bit 7 */
#define GPIO_POD_POD8  ((uint16_t)0x0100) /*!< Port output data, bit 8 */
#define GPIO_POD_POD9  ((uint16_t)0x0200) /*!< Port output data, bit 9 */
#define GPIO_POD_POD10 ((uint16_t)0x0400) /*!< Port output data, bit 10 */
#define GPIO_POD_POD11 ((uint16_t)0x0800) /*!< Port output data, bit 11 */
#define GPIO_POD_POD12 ((uint16_t)0x1000) /*!< Port output data, bit 12 */
#define GPIO_POD_POD13 ((uint16_t)0x2000) /*!< Port output data, bit 13 */

/******************  Bit definition for GPIO_PBSC register  *******************/
#define GPIO_PBSC_PBS0  ((uint32_t)0x00000001) /*!< Port x Set bit 0 */
#define GPIO_PBSC_PBS1  ((uint32_t)0x00000002) /*!< Port x Set bit 1 */
#define GPIO_PBSC_PBS2  ((uint32_t)0x00000004) /*!< Port x Set bit 2 */
#define GPIO_PBSC_PBS3  ((uint32_t)0x00000008) /*!< Port x Set bit 3 */
#define GPIO_PBSC_PBS4  ((uint32_t)0x00000010) /*!< Port x Set bit 4 */
#define GPIO_PBSC_PBS5  ((uint32_t)0x00000020) /*!< Port x Set bit 5 */
#define GPIO_PBSC_PBS6  ((uint32_t)0x00000040) /*!< Port x Set bit 6 */
#define GPIO_PBSC_PBS7  ((uint32_t)0x00000080) /*!< Port x Set bit 7 */
#define GPIO_PBSC_PBS8  ((uint32_t)0x00000100) /*!< Port x Set bit 8 */
#define GPIO_PBSC_PBS9  ((uint32_t)0x00000200) /*!< Port x Set bit 9 */
#define GPIO_PBSC_PBS10 ((uint32_t)0x00000400) /*!< Port x Set bit 10 */
#define GPIO_PBSC_PBS11 ((uint32_t)0x00000800) /*!< Port x Set bit 11 */
#define GPIO_PBSC_PBS12 ((uint32_t)0x00001000) /*!< Port x Set bit 12 */
#define GPIO_PBSC_PBS13 ((uint32_t)0x00002000) /*!< Port x Set bit 13 */

#define GPIO_PBSC_PBC0  ((uint32_t)0x00010000) /*!< Port x Reset bit 0 */
#define GPIO_PBSC_PBC1  ((uint32_t)0x00020000) /*!< Port x Reset bit 1 */
#define GPIO_PBSC_PBC2  ((uint32_t)0x00040000) /*!< Port x Reset bit 2 */
#define GPIO_PBSC_PBC3  ((uint32_t)0x00080000) /*!< Port x Reset bit 3 */
#define GPIO_PBSC_PBC4  ((uint32_t)0x00100000) /*!< Port x Reset bit 4 */
#define GPIO_PBSC_PBC5  ((uint32_t)0x00200000) /*!< Port x Reset bit 5 */
#define GPIO_PBSC_PBC6  ((uint32_t)0x00400000) /*!< Port x Reset bit 6 */
#define GPIO_PBSC_PBC7  ((uint32_t)0x00800000) /*!< Port x Reset bit 7 */
#define GPIO_PBSC_PBC8  ((uint32_t)0x01000000) /*!< Port x Reset bit 8 */
#define GPIO_PBSC_PBC9  ((uint32_t)0x02000000) /*!< Port x Reset bit 9 */
#define GPIO_PBSC_PBC10 ((uint32_t)0x04000000) /*!< Port x Reset bit 10 */
#define GPIO_PBSC_PBC11 ((uint32_t)0x08000000) /*!< Port x Reset bit 11 */
#define GPIO_PBSC_PBC12 ((uint32_t)0x10000000) /*!< Port x Reset bit 12 */
#define GPIO_PBSC_PBC13 ((uint32_t)0x20000000) /*!< Port x Reset bit 13 */

/******************  Bit definition for GPIO_PLOCK register  *******************/
#define GPIO_PLOCK_PLOCK0  ((uint32_t)0x00000001) /*!< Port x Lock bit 0 */
#define GPIO_PLOCK_PLOCK1  ((uint32_t)0x00000002) /*!< Port x Lock bit 1 */
#define GPIO_PLOCK_PLOCK2  ((uint32_t)0x00000004) /*!< Port x Lock bit 2 */
#define GPIO_PLOCK_PLOCK3  ((uint32_t)0x00000008) /*!< Port x Lock bit 3 */
#define GPIO_PLOCK_PLOCK4  ((uint32_t)0x00000010) /*!< Port x Lock bit 4 */
#define GPIO_PLOCK_PLOCK5  ((uint32_t)0x00000020) /*!< Port x Lock bit 5 */
#define GPIO_PLOCK_PLOCK6  ((uint32_t)0x00000040) /*!< Port x Lock bit 6 */
#define GPIO_PLOCK_PLOCK7  ((uint32_t)0x00000080) /*!< Port x Lock bit 7 */
#define GPIO_PLOCK_PLOCK8  ((uint32_t)0x00000100) /*!< Port x Lock bit 8 */
#define GPIO_PLOCK_PLOCK9  ((uint32_t)0x00000200) /*!< Port x Lock bit 9 */
#define GPIO_PLOCK_PLOCK10 ((uint32_t)0x00000400) /*!< Port x Lock bit 10 */
#define GPIO_PLOCK_PLOCK11 ((uint32_t)0x00000800) /*!< Port x Lock bit 11 */
#define GPIO_PLOCK_PLOCK12 ((uint32_t)0x00001000) /*!< Port x Lock bit 12 */
#define GPIO_PLOCK_PLOCK13 ((uint32_t)0x00002000) /*!< Port x Lock bit 13 */
#define GPIO_PLOCK_PLOCKK  ((uint32_t)0x00010000) /*!< Lock key */

/******************  Bit definition for GPIO_AFL register  *******************/
#define GPIO_AFL_AFSEL0  ((uint32_t)0x0000000F) /*!< Port x AFL bit (0..3) */
#define GPIO_AFL_AFSEL1  ((uint32_t)0x000000F0) /*!< Port x AFL bit (4..7) */
#define GPIO_AFL_AFSEL2  ((uint32_t)0x00000F00) /*!< Port x AFL bit (8..11) */
#define GPIO_AFL_AFSEL3  ((uint32_t)0x0000F000) /*!< Port x AFL bit (12..15) */
#define GPIO_AFL_AFSEL4  ((uint32_t)0x000F0000) /*!< Port x AFL bit (16..19) */
#define GPIO_AFL_AFSEL5  ((uint32_t)0x00F00000) /*!< Port x AFL bit (20..23) */
#define GPIO_AFL_AFSEL6  ((uint32_t)0x0F000000) /*!< Port x AFL bit (24..27) */
#define GPIO_AFL_AFSEL7  ((uint32_t)0xF0000000) /*!< Port x AFL bit (27..31) */

/******************  Bit definition for GPIO_AFH register  *******************/
#define GPIO_AFH_AFSEL8  ((uint32_t)0x0000000F) /*!< Port x AFH bit (0..3) */
#define GPIO_AFH_AFSEL9  ((uint32_t)0x000000F0) /*!< Port x AFH bit (4..7) */
#define GPIO_AFH_AFSEL10 ((uint32_t)0x00000F00) /*!< Port x AFH bit (8..11) */
#define GPIO_AFH_AFSEL11 ((uint32_t)0x0000F000) /*!< Port x AFH bit (12..15) */
#define GPIO_AFH_AFSEL12 ((uint32_t)0x000F0000) /*!< Port x AFH bit (16..19) */
#define GPIO_AFH_AFSEL13 ((uint32_t)0x00F00000) /*!< Port x AFH bit (20..23) */

/*******************  Bit definition for GPIO_PBC register  *******************/
#define GPIO_PBC_PBC0  ((uint16_t)0x0001) /*!< Port x Reset bit 0 */
#define GPIO_PBC_PBC1  ((uint16_t)0x0002) /*!< Port x Reset bit 1 */
#define GPIO_PBC_PBC2  ((uint16_t)0x0004) /*!< Port x Reset bit 2 */
#define GPIO_PBC_PBC3  ((uint16_t)0x0008) /*!< Port x Reset bit 3 */
#define GPIO_PBC_PBC4  ((uint16_t)0x0010) /*!< Port x Reset bit 4 */
#define GPIO_PBC_PBC5  ((uint16_t)0x0020) /*!< Port x Reset bit 5 */
#define GPIO_PBC_PBC6  ((uint16_t)0x0040) /*!< Port x Reset bit 6 */
#define GPIO_PBC_PBC7  ((uint16_t)0x0080) /*!< Port x Reset bit 7 */
#define GPIO_PBC_PBC8  ((uint16_t)0x0100) /*!< Port x Reset bit 8 */
#define GPIO_PBC_PBC9  ((uint16_t)0x0200) /*!< Port x Reset bit 9 */
#define GPIO_PBC_PBC10 ((uint16_t)0x0400) /*!< Port x Reset bit 10 */
#define GPIO_PBC_PBC11 ((uint16_t)0x0800) /*!< Port x Reset bit 11 */
#define GPIO_PBC_PBC12 ((uint16_t)0x1000) /*!< Port x Reset bit 12 */
#define GPIO_PBC_PBC13 ((uint16_t)0x2000) /*!< Port x Reset bit 13 */

/*******************  Bit definition for GPIO_DS register ******************/
#define GPIO_DS_DS0  ((uint16_t)0x0001) /*!<  Port x driver strength bit 0 */
#define GPIO_DS_DS1  ((uint16_t)0x0002) /*!<  Port x driver strength bit 1 */
#define GPIO_DS_DS2  ((uint16_t)0x0004) /*!<  Port x driver strength bit 2 */
#define GPIO_DS_DS3  ((uint16_t)0x0008) /*!<  Port x driver strength bit 3 */
#define GPIO_DS_DS4  ((uint16_t)0x0010) /*!<  Port x driver strength bit 4 */
#define GPIO_DS_DS5  ((uint16_t)0x0020) /*!<  Port x driver strength bit 5 */
#define GPIO_DS_DS6  ((uint16_t)0x0040) /*!<  Port x driver strength bit 6 */
#define GPIO_DS_DS7  ((uint16_t)0x0080) /*!<  Port x driver strength bit 7 */
#define GPIO_DS_DS8  ((uint16_t)0x0100) /*!<  Port x driver strength bit 8 */
#define GPIO_DS_DS9  ((uint16_t)0x0200) /*!<  Port x driver strength bit 9 */
#define GPIO_DS_DS10 ((uint16_t)0x0400) /*!<  Port x driver strength bit 10 */
#define GPIO_DS_DS11 ((uint16_t)0x0800) /*!<  Port x driver strength bit 11 */
#define GPIO_DS_DS12 ((uint16_t)0x1000) /*!<  Port x driver strength bit 12 */
#define GPIO_DS_DS13 ((uint16_t)0x2000) /*!<  Port x driver strength bit 13 */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for AFIO_CFG register  *****************/
#define AFIO_CFG_SPI2_NSS    ((uint32_t)0x00000800) /*!< AFIO_CFG bit 11 */
#define AFIO_CFG_SPI1_NSS    ((uint32_t)0x00000400) /*!< AFIO_CFG bit 10 */
#define AFIO_CFG_EXTI_ETRI   ((uint32_t)0x00000070) /*!< AFIO_CFG bit (6-4) */
#define AFIO_CFG_EXTI_ETRR   ((uint32_t)0x00000007) /*!< AFIO_CFG bit (2-0) */

/*****************  Bit definition for AFIO_EXTI_CFG1 register  *****************/
#define AFIO_EXTI_CFG1_EXTI0 ((uint16_t)0x0003) /*!< EXTI 0 configuration */
#define AFIO_EXTI_CFG1_EXTI1 ((uint16_t)0x0030) /*!< EXTI 1 configuration */
#define AFIO_EXTI_CFG1_EXTI2 ((uint16_t)0x0300) /*!< EXTI 2 configuration */
#define AFIO_EXTI_CFG1_EXTI3 ((uint16_t)0x3000) /*!< EXTI 3 configuration */

/*!< EXTI0 configuration */
#define AFIO_EXTI_CFG1_EXTI0_PA ((uint16_t)0x0000) /*!< PA[0] pin */
#define AFIO_EXTI_CFG1_EXTI0_PB ((uint16_t)0x0001) /*!< PB[0] pin */

/*!< EXTI1 configuration */
#define AFIO_EXTI_CFG1_EXTI1_PA ((uint16_t)0x0000) /*!< PA[1] pin */ 
#define AFIO_EXTI_CFG1_EXTI1_PB ((uint16_t)0x0010) /*!< PB[1] pin */

/*!< EXTI2 configuration */
#define AFIO_EXTI_CFG1_EXTI2_PA ((uint16_t)0x0000) /*!< PA[2] pin */
#define AFIO_EXTI_CFG1_EXTI2_PB ((uint16_t)0x0100) /*!< PB[2] pin */

/*!< EXTI3 configuration */
#define AFIO_EXTI_CFG1_EXTI3_PA ((uint16_t)0x0000) /*!< PA[3] pin */
#define AFIO_EXTI_CFG1_EXTI3_PB ((uint16_t)0x1000) /*!< PB[3] pin */

/*****************  Bit definition for AFIO_EXTI_CFG2 register  *****************/
#define AFIO_EXTI_CFG2_EXTI4 ((uint16_t)0x0003) /*!< EXTI 4 configuration */
#define AFIO_EXTI_CFG2_EXTI5 ((uint16_t)0x0030) /*!< EXTI 5 configuration */
#define AFIO_EXTI_CFG2_EXTI6 ((uint16_t)0x0300) /*!< EXTI 6 configuration */
#define AFIO_EXTI_CFG2_EXTI7 ((uint16_t)0x3000) /*!< EXTI 7 configuration */

/*!< EXTI4 configuration */
#define AFIO_EXTI_CFG2_EXTI4_PA ((uint16_t)0x0000) /*!< PA[4] pin */
#define AFIO_EXTI_CFG2_EXTI4_PB ((uint16_t)0x0001) /*!< PB[4] pin */

/*!< EXTI5 configuration */
#define AFIO_EXTI_CFG2_EXTI5_PA ((uint16_t)0x0000) /*!< PA[5] pin */
#define AFIO_EXTI_CFG2_EXTI5_PB ((uint16_t)0x0010) /*!< PB[5] pin */

/*!< EXTI6 configuration */
#define AFIO_EXTI_CFG2_EXTI6_PA ((uint16_t)0x0000) /*!< PA[6] pin */
#define AFIO_EXTI_CFG2_EXTI6_PB ((uint16_t)0x0100) /*!< PB[6] pin */

/*!< EXTI7 configuration */
#define AFIO_EXTI_CFG2_EXTI7_PA ((uint16_t)0x0000) /*!< PA[7] pin */
#define AFIO_EXTI_CFG2_EXTI7_PB ((uint16_t)0x1000) /*!< PB[7] pin */

/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for EXTI_IMR register  *******************/
#define EXTI_IMASK_IMASK0  ((uint32_t)0x00000001) /*!< Interrupt Mask on line 0 */
#define EXTI_IMASK_IMASK1  ((uint32_t)0x00000002) /*!< Interrupt Mask on line 1 */
#define EXTI_IMASK_IMASK2  ((uint32_t)0x00000004) /*!< Interrupt Mask on line 2 */
#define EXTI_IMASK_IMASK3  ((uint32_t)0x00000008) /*!< Interrupt Mask on line 3 */
#define EXTI_IMASK_IMASK4  ((uint32_t)0x00000010) /*!< Interrupt Mask on line 4 */
#define EXTI_IMASK_IMASK5  ((uint32_t)0x00000020) /*!< Interrupt Mask on line 5 */
#define EXTI_IMASK_IMASK6  ((uint32_t)0x00000040) /*!< Interrupt Mask on line 6 */
#define EXTI_IMASK_IMASK7  ((uint32_t)0x00000080) /*!< Interrupt Mask on line 7 */
#define EXTI_IMASK_IMASK8  ((uint32_t)0x00000100) /*!< Interrupt Mask on line 8 */
#define EXTI_IMASK_IMASK9  ((uint32_t)0x00000200) /*!< Interrupt Mask on line 9 */
#define EXTI_IMASK_IMASK10 ((uint32_t)0x00000400) /*!< Interrupt Mask on line 10 */
#define EXTI_IMASK_IMASK11 ((uint32_t)0x00000800) /*!< Interrupt Mask on line 11 */
#define EXTI_IMASK_IMASK12 ((uint32_t)0x00001000) /*!< Interrupt Mask on line 12 */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define EXTI_EMASK_EMASK0  ((uint32_t)0x00000001) /*!< Event Mask on line 0 */
#define EXTI_EMASK_EMASK1  ((uint32_t)0x00000002) /*!< Event Mask on line 1 */
#define EXTI_EMASK_EMASK2  ((uint32_t)0x00000004) /*!< Event Mask on line 2 */
#define EXTI_EMASK_EMASK3  ((uint32_t)0x00000008) /*!< Event Mask on line 3 */
#define EXTI_EMASK_EMASK4  ((uint32_t)0x00000010) /*!< Event Mask on line 4 */
#define EXTI_EMASK_EMASK5  ((uint32_t)0x00000020) /*!< Event Mask on line 5 */
#define EXTI_EMASK_EMASK6  ((uint32_t)0x00000040) /*!< Event Mask on line 6 */
#define EXTI_EMASK_EMASK7  ((uint32_t)0x00000080) /*!< Event Mask on line 7 */
#define EXTI_EMASK_EMASK8  ((uint32_t)0x00000100) /*!< Event Mask on line 8 */
#define EXTI_EMASK_EMASK9  ((uint32_t)0x00000200) /*!< Event Mask on line 9 */
#define EXTI_EMASK_EMASK10 ((uint32_t)0x00000400) /*!< Event Mask on line 10 */
#define EXTI_EMASK_EMASK11 ((uint32_t)0x00000800) /*!< Event Mask on line 11 */
#define EXTI_EMASK_EMASK12 ((uint32_t)0x00001000) /*!< Event Mask on line 12 */

/******************  Bit definition for EXTI_RT_CFG register  *******************/
#define EXTI_EMASK_RT_CFG_RT_CFG0  ((uint32_t)0x00000001) /*!< Rising trigger event configuration bit of line 0 */
#define EXTI_EMASK_RT_CFG_RT_CFG1  ((uint32_t)0x00000002) /*!< Rising trigger event configuration bit of line 1 */
#define EXTI_EMASK_RT_CFG_RT_CFG2  ((uint32_t)0x00000004) /*!< Rising trigger event configuration bit of line 2 */
#define EXTI_EMASK_RT_CFG_RT_CFG3  ((uint32_t)0x00000008) /*!< Rising trigger event configuration bit of line 3 */
#define EXTI_EMASK_RT_CFG_RT_CFG4  ((uint32_t)0x00000010) /*!< Rising trigger event configuration bit of line 4 */
#define EXTI_EMASK_RT_CFG_RT_CFG5  ((uint32_t)0x00000020) /*!< Rising trigger event configuration bit of line 5 */
#define EXTI_EMASK_RT_CFG_RT_CFG6  ((uint32_t)0x00000040) /*!< Rising trigger event configuration bit of line 6 */
#define EXTI_EMASK_RT_CFG_RT_CFG7  ((uint32_t)0x00000080) /*!< Rising trigger event configuration bit of line 7 */
#define EXTI_EMASK_RT_CFG_RT_CFG8  ((uint32_t)0x00000100) /*!< Rising trigger event configuration bit of line 8 */
#define EXTI_EMASK_RT_CFG_RT_CFG9  ((uint32_t)0x00000200) /*!< Rising trigger event configuration bit of line 9 */
#define EXTI_EMASK_RT_CFG_RT_CFG10 ((uint32_t)0x00000400) /*!< Rising trigger event configuration bit of line 10 */
#define EXTI_EMASK_RT_CFG_RT_CFG11 ((uint32_t)0x00000800) /*!< Rising trigger event configuration bit of line 11 */
#define EXTI_EMASK_RT_CFG_RT_CFG12 ((uint32_t)0x00001000) /*!< Rising trigger event configuration bit of line 12 */

/******************  Bit definition for EXTI_FT_CFG register  *******************/
#define EXTI_EMASK_FT_CFG_FT_CFG0  ((uint32_t)0x00000001) /*!< Falling trigger event configuration bit of line 0 */
#define EXTI_EMASK_FT_CFG_FT_CFG1  ((uint32_t)0x00000002) /*!< Falling trigger event configuration bit of line 1 */
#define EXTI_EMASK_FT_CFG_FT_CFG2  ((uint32_t)0x00000004) /*!< Falling trigger event configuration bit of line 2 */
#define EXTI_EMASK_FT_CFG_FT_CFG3  ((uint32_t)0x00000008) /*!< Falling trigger event configuration bit of line 3 */
#define EXTI_EMASK_FT_CFG_FT_CFG4  ((uint32_t)0x00000010) /*!< Falling trigger event configuration bit of line 4 */
#define EXTI_EMASK_FT_CFG_FT_CFG5  ((uint32_t)0x00000020) /*!< Falling trigger event configuration bit of line 5 */
#define EXTI_EMASK_FT_CFG_FT_CFG6  ((uint32_t)0x00000040) /*!< Falling trigger event configuration bit of line 6 */
#define EXTI_EMASK_FT_CFG_FT_CFG7  ((uint32_t)0x00000080) /*!< Falling trigger event configuration bit of line 7 */
#define EXTI_EMASK_FT_CFG_FT_CFG8  ((uint32_t)0x00000100) /*!< Falling trigger event configuration bit of line 8 */
#define EXTI_EMASK_FT_CFG_FT_CFG9  ((uint32_t)0x00000200) /*!< Falling trigger event configuration bit of line 9 */
#define EXTI_EMASK_FT_CFG_FT_CFG10 ((uint32_t)0x00000400) /*!< Falling trigger event configuration bit of line 10 */
#define EXTI_EMASK_FT_CFG_FT_CFG11 ((uint32_t)0x00000800) /*!< Falling trigger event configuration bit of line 11 */
#define EXTI_EMASK_FT_CFG_FT_CFG12 ((uint32_t)0x00001000) /*!< Falling trigger event configuration bit of line 12 */

/******************  Bit definition for EXTI_SWIE register  ******************/
#define EXTI_SWIE_SWIE0  ((uint32_t)0x00000001) /*!< Software Interrupt on line 0 */
#define EXTI_SWIE_SWIE1  ((uint32_t)0x00000002) /*!< Software Interrupt on line 1 */
#define EXTI_SWIE_SWIE2  ((uint32_t)0x00000004) /*!< Software Interrupt on line 2 */
#define EXTI_SWIE_SWIE3  ((uint32_t)0x00000008) /*!< Software Interrupt on line 3 */
#define EXTI_SWIE_SWIE4  ((uint32_t)0x00000010) /*!< Software Interrupt on line 4 */
#define EXTI_SWIE_SWIE5  ((uint32_t)0x00000020) /*!< Software Interrupt on line 5 */
#define EXTI_SWIE_SWIE6  ((uint32_t)0x00000040) /*!< Software Interrupt on line 6 */
#define EXTI_SWIE_SWIE7  ((uint32_t)0x00000080) /*!< Software Interrupt on line 7 */
#define EXTI_SWIE_SWIE8  ((uint32_t)0x00000100) /*!< Software Interrupt on line 8 */
#define EXTI_SWIE_SWIE9  ((uint32_t)0x00000200) /*!< Software Interrupt on line 9 */
#define EXTI_SWIE_SWIE10 ((uint32_t)0x00000400) /*!< Software Interrupt on line 10 */
#define EXTI_SWIE_SWIE11 ((uint32_t)0x00000800) /*!< Software Interrupt on line 11 */
#define EXTI_SWIE_SWIE12 ((uint32_t)0x00001000) /*!< Software Interrupt on line 12 */

/*******************  Bit definition for EXTI_PEND register  ********************/
#define EXTI_PEND_PEND0  ((uint32_t)0x00000001) /*!< Pending bit for line 0 */
#define EXTI_PEND_PEND1  ((uint32_t)0x00000002) /*!< Pending bit for line 1 */
#define EXTI_PEND_PEND2  ((uint32_t)0x00000004) /*!< Pending bit for line 2 */
#define EXTI_PEND_PEND3  ((uint32_t)0x00000008) /*!< Pending bit for line 3 */
#define EXTI_PEND_PEND4  ((uint32_t)0x00000010) /*!< Pending bit for line 4 */
#define EXTI_PEND_PEND5  ((uint32_t)0x00000020) /*!< Pending bit for line 5 */
#define EXTI_PEND_PEND6  ((uint32_t)0x00000040) /*!< Pending bit for line 6 */
#define EXTI_PEND_PEND7  ((uint32_t)0x00000080) /*!< Pending bit for line 7 */
#define EXTI_PEND_PEND8  ((uint32_t)0x00000100) /*!< Pending bit for line 8 */
#define EXTI_PEND_PEND9  ((uint32_t)0x00000200) /*!< Pending bit for line 9 */
#define EXTI_PEND_PEND10 ((uint32_t)0x00000400) /*!< Pending bit for line 10 */
#define EXTI_PEND_PEND11 ((uint32_t)0x00000800) /*!< Pending bit for line 11 */
#define EXTI_PEND_PEND12 ((uint32_t)0x00001000) /*!< Pending bit for line 12 */

/***********************  Common macro fuction define       *******************/

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))


/***********************  OTP1 data struct   *******************/
typedef struct{
    uint32_t stote_bg_vtrim_value;  
    uint32_t stote_rc28800_trim_value;     
    uint32_t stote_rc32000_trim_value; 
    uint32_t stote_rc32768_trim_value; 
    uint32_t stote_rc64m_trim_value; 
    uint32_t stote_rc96m_trim_value; 
    uint32_t rc_adc_ts_25c; 
    uint32_t rc_gpadc_value_3400mv;
    uint32_t rc_gpadc_value_600mv; 
    uint8_t  flash_uuid[16];
    uint32_t re_array[32];
}otp1_stored_t;

/**
 * @}
 */

#ifdef USE_STDPERIPH_DRIVER
#include "n32wb03x_conf.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __N32WB03X_H__ */
