/**
  ******************************************************************************
  * @file    Smartcard/inc/smartcard.h
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    07/27/2009
  * @brief   This file contains all the functions prototypes for the Smartcard
  *          firmware library.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMARTCARD_H
#define __SMARTCARD_H

/* Includes ------------------------------------------------------------------*/
#include "n32wb03x.h"


/* Exported constants --------------------------------------------------------*/
#define T0_PROTOCOL        0x00  /* T0 protocol */
#define DIRECT             0x3B  /* Direct bit convention */
#define INDIRECT           0x3F  /* Indirect bit convention */
#define SETUP_LENGTH       20
#define HIST_LENGTH        20
#define LC_MAX             256//20
#define SC_RECEIVE_TIMEOUT 0x4000  /* Direction to reader */

/* SC Tree Structure -----------------------------------------------------------
                              MasterFile
                           ________|___________
                          |        |           |
                        System   UserData     Note
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------//
#define SC1_USART                 USART2
#define SC1_USART_GPIO            GPIOB
#define SC1_USART_CLK             RCC_APB1_PERIPH_USART2
#define SC1_USART_GPIO_CLK        RCC_APB2_PERIPH_GPIOB
#define SC1_USART_PIN_TX          GPIO_PIN_4
#define SC1_USART_PIN_CK          GPIO_PIN_13
#define SC1_USART_PIN_AF          GPIO_AF3_USART2
#define SC1_USART_IRQn            USART2_IRQn
#define SC1_USART_IRQHandler      USART2_IRQHandler
//------------------------------------------------------------------------//

#define SC1_PIN_RESET              GPIO_PIN_0
#define SC1_PIN_RESET_GPIO         GPIOA
#define SC1_PIN_RESET_GPIO_CLK     RCC_APB2_PERIPH_GPIOA
//------------------------------------------------------------------------//

//#define SC_DETECT_EXTI            EXTI_LINE7
//#define SC_DETECT_PIN             GPIOC_PORT_SOURCE//GPIO_PortSourceGPIOC
//#define SC_DETECT_GPIO            GPIO_PIN_SOURCE7
//#define SC_DETECT_IRQ             EXTI9_5_IRQn
//#define SC_DETECT_IRQHandler      EXTI9_5_IRQHandler   //xzy9_4->4_15



/* SC ADPU Command: Operation Code -------------------------------------------*/
#define SC_CLA_GSM11       0xA0

/*------------------------ Data Area Management Commands ---------------------*/
#define SC_SELECT_FILE     0xA4
#define SC_GET_RESPONCE    0xC0
#define SC_STATUS          0xF2
#define SC_UPDATE_BINARY   0xD6
#define SC_READ_BINARY     0xB0
#define SC_WRITE_BINARY    0xD0
#define SC_UPDATE_RECORD   0xDC
#define SC_READ_RECORD     0xB2

/*-------------------------- Administrative Commands -------------------------*/
#define SC_CREATE_FILE     0xE0

/*-------------------------- Safety Management Commands ----------------------*/
#define SC_VERIFY          0x20
#define SC_CHANGE          0x24
#define SC_DISABLE         0x26
#define SC_ENABLE          0x28
#define SC_UNBLOCK         0x2C
#define SC_EXTERNAL_AUTH   0x82
#define SC_GET_CHALLENGE   0x84

/*-------------------------- Answer to reset Commands ------------------------*/
#define SC_GET_A2R         0x00

/* SC STATUS: Status Code ----------------------------------------------------*/
#define SC_EF_SELECTED     0x9F
#define SC_DF_SELECTED     0x9F
#define SC_OP_TERMINATED   0x9000

/* Smartcard Voltage */
#define SC_VOLTAGE_5V      0
#define SC_VOLTAGE_3V      1

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  SC_POWER_ON = 0x00,
  SC_RESET_LOW = 0x01,
  SC_RESET_HIGH = 0x02,
  SC_ACTIVE = 0x03,
  SC_ACTIVE_ON_T0 = 0x04,
  SC_POWER_OFF = 0x05
} SC_State;
///* BitAction types ------------------------------------------------------------*/
//typedef enum
//{
//	Bit_RESET = 0,
//	Bit_SET
//}BitAction;


/* ATR structure - Answer To Reset -------------------------------------------*/
typedef struct
{
  uint8_t TS;               /* Bit Convention */
  uint8_t T0;               /* High nibble = Number of setup byte; low nibble = Number of historical byte */
  uint8_t T[SETUP_LENGTH];  /* Setup array */
  uint8_t H[HIST_LENGTH];   /* Historical array */
  uint8_t Tlength;          /* Setup array dimension */
  uint8_t Hlength;          /* Historical array dimension */
} SC_ATR;

/* ADPU-Header command structure ---------------------------------------------*/
typedef struct
{
  uint8_t CLA;  /* Command class */
  uint8_t INS;  /* Operation code */
  uint8_t P1;   /* Selection Mode */
  uint8_t P2;   /* Selection Option */
} SC_Header;

/* ADPU-Body command structure -----------------------------------------------*/
typedef struct
{
  uint8_t LC;           /* Data field length */
  uint8_t Data[LC_MAX];  /* Command parameters */
  uint8_t LE;           /* Expected length of data to be returned */
} SC_Body;

/* ADPU Command structure ----------------------------------------------------*/
typedef struct
{
  SC_Header Header;
  SC_Body Body;
} SC_ADPU_Commands;

/* SC response structure -----------------------------------------------------*/
typedef struct
{
  uint8_t Data[LC_MAX];  /* Data returned from the card */
  uint8_t SW1;          /* Command Processing status */
  uint8_t SW2;          /* Command Processing qualification */
} SC_ADPU_Responce;

/* SC Initial structure -----------------------------------------------------*/
typedef struct
{
  uint8_t Clk_Div;     /* USART Clk Div Value */
  uint8_t GT;          /* Guard Time*/
  uint8_t StopBits;  
  uint8_t Parity;
  uint8_t NackEn;
} SC_InitStructure;




/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* APPLICATION LAYER ---------------------------------------------------------*/
void SC_Handler(SC_State *SCState, SC_ADPU_Commands *SC_ADPU, SC_ADPU_Responce *SC_Response, SC_InitStructure *SC_InitCfg);
void SC_PowerCmd(FunctionalState NewState);
void SC1_Reset(Bit_OperateType ResetState);
void SC_DetectPinConfig(void);
void SC_ParityErrorHandler(void);
void SC_PTSConfig(void);

void SC1_DeInit(void);
#endif /* __SMARTCARD_H */

