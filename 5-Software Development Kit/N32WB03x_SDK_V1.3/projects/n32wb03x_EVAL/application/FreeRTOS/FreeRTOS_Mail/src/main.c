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
#include "main.h"
#include <stdio.h>
#include "cmsis_os.h"

/**
 *  FreeRTOS Mail
 */

/* Private typedef -----------------------------------------------------------*/
typedef struct 
{ /* Mail object structure */
  uint32_t var1; /* var1 is a uint32_t */
  uint32_t var2; /* var2 is a uint32_t */
  uint8_t var3; /* var3 is a uint8_t */
} Amail_TypeDef;
/* Private define ------------------------------------------------------------*/
#define blckqSTACK_SIZE       configMINIMAL_STACK_SIZE
#define MAIL_SIZE             (uint32_t) 1
#define SYSTICK_1MS           ((uint32_t)1000)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Tick_num = 0;
//osThreadId LED1_ThreadId, LED2_ThreadId;
osMailQId mailId;

uint32_t ProducerValue1 = 0, ProducerValue2 = 0;
uint8_t ProducerValue3 = 0;
uint32_t ConsumerValue1 = 0, ConsumerValue2 = 0;
uint8_t ConsumerValue3 = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program.
 */
int main(void)
{
    /*Configure the SysTick IRQ priority */
    N32_NVIC_SetPriority(SysTick_IRQn, TICK_INT_PRIORITY, 0);
    /* Get SystemCoreClock */
    SystemCoreClockUpdate();

    /* Config 1s SysTick 1ms  */
    SysTick_Config(SystemCoreClock/SYSTICK_1MS);
  
    /* Initialize Led1~Led2 as output pushpull mode*/
    LedInit(LED1_PORT, LED1_PIN);
    LedInit(LED2_PORT, LED2_PIN);

    /*Turn off Led1~Led2*/
    LedOff(LED1_PORT, LED1_PIN);
    LedOff(LED2_PORT, LED2_PIN);
  
    /* Create the mail queue used by the two tasks to pass the struct Amail_TypeDef */
    osMailQDef(mail, MAIL_SIZE, Amail_TypeDef); /* Define mail queue */
    mailId = osMailCreate(osMailQ(mail), NULL); /* create mail queue */

    /* Note the producer has a lower priority than the consumer when the tasks are
       spawned. */
    osThreadDef(QCons, MailQueueConsumer, osPriorityBelowNormal, 0, blckqSTACK_SIZE);
    osThreadCreate(osThread(QCons), NULL);
  
    osThreadDef(QProd, MailQueueProducer, osPriorityBelowNormal, 0, blckqSTACK_SIZE);
    osThreadCreate(osThread(QProd), NULL);
  
    /* Start scheduler */
    osKernelStart ();
  
    /* We should never get here as control is now taken by the scheduler */
    for(;;);
}

/**
  * @brief  Mail Producer Thread.
  * @param  argument: Not used
  * @retval None
  */
static void MailQueueProducer(const void *argument)
{
  Amail_TypeDef *pTMail;
  
  for(;;)
  {		

    pTMail = osMailAlloc(mailId, osWaitForever); /* Allocate memory */
    pTMail->var1 = ProducerValue1; /* Set the mail content */
    pTMail->var2 = ProducerValue2;
    pTMail->var3 = ProducerValue3;
    
    if(osMailPut(mailId, pTMail) != osOK) /* Send Mail */  
    {      
      /* Turn on LED2 to indicate error */
      LedOn(LED2_PORT,LED2_PIN);
    }
    else
    {
      /* Increment the variables we are going to post next time round.  The
      consumer will expect the numbers to follow in numerical order. */
      ++ProducerValue1;
      ProducerValue2 += 2;
      ProducerValue3 += 3;
      
      /* Toggle LED1 to indicate a correct number received  */
      LedBlink(LED1_PORT,LED1_PIN);

      osDelay(250);
    }
  }
}

/**
  * @brief  Mail Consumer Thread.
  * @param  argument: Not used
  * @retval None
  */
static void MailQueueConsumer (const void *argument)
{
  osEvent event;
  Amail_TypeDef *pRMail;
  
  for(;;)
  {
    /* Get the message from the queue */
    event = osMailGet(mailId, osWaitForever); /* wait for mail */
    
    if(event.status == osEventMail)
    {
      pRMail = event.value.p;
      
      if((pRMail->var1 != ConsumerValue1) || (pRMail->var2 != ConsumerValue2) || (pRMail->var3 != ConsumerValue3))
      {
        /* Catch-up. */
        ConsumerValue1 = pRMail->var1;
        ConsumerValue2 = pRMail->var2;
        ConsumerValue3 = pRMail->var3;
        
        /* Turn on LED2 to indicate error */
        LedOn(LED2_PORT,LED2_PIN);
      }
      else
      {  
        /* Calculate values we expect to remove from the mail queue next time
        round. */
        ++ConsumerValue1;
        ConsumerValue2 += 2;
        ConsumerValue3 += 3;        
      }

      osMailFree(mailId, pRMail); /* free memory allocated for mail */       
    }		
  }
}

/**
  * @brief  Sets the priority of an interrupt.
  * @param  IRQn External interrupt number .
  *         This parameter can be an enumerator of  IRQn_Type enumeration
  * @param  PreemptPriority The pre-emption priority for the IRQn channel.
  *         This parameter can be a value between 0 and 3.
  *         A lower priority value indicates a higher priority 
  * @param  SubPriority the subpriority level for the IRQ channel.
  *         this parameter is a dummy value and it is ignored, because 
  *         no subpriority supported in Cortex M0 based products.   
  * @retval None
  */
void N32_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{ 
    /* Check the parameters */
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority));
  NVIC_SetPriority(IRQn,PreemptPriority);
}

/**
 * @brief  Configures LED GPIO.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    /* Enable the GPIO Clock */
    if (GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    else
    {
        return;
    }

    /* Configure the GPIO pin */
    if (Pin <= GPIO_PIN_ALL)
    {
        GPIO_InitStruct(&GPIO_InitStructure);
        GPIO_InitStructure.Pin = Pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
    }
}

/**
 * @brief  Turns selected Led on.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedOn(GPIO_Module *GPIOx, uint16_t Pin)
{
    GPIO_SetBits(GPIOx, Pin);
}

/**
 * @brief  Turns selected Led Off.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedOff(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_ResetBits(GPIOx, Pin);
}

/**
 * @brief  Toggles the selected Led.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_TogglePin(GPIOx, Pin);
}

/**
 * @}
 */

/**
 * @}
 */
