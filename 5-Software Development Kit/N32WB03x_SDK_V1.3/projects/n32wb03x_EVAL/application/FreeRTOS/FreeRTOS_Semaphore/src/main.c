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
 *  FreeRTOS Semaphore
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_1MS           ((uint32_t)1000)
/* Private macro -------------------------------------------------------------*/
#define semtstSTACK_SIZE      configMINIMAL_STACK_SIZE
/* Private variables ---------------------------------------------------------*/
uint32_t Tick_num = 0;
osThreadId SemThread1Handle, SemThread2Handle;
osSemaphoreId osSemaphore;
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

    /*Turn on Led1~Led2*/
    LedOn(LED1_PORT, LED1_PIN);
    LedOn(LED2_PORT, LED2_PIN);

    /* Define used semaphore */
    osSemaphoreDef(SEM);

    /* Create the semaphore used by the two threads */
    osSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 1);

    /* Create the first Thread */
    osThreadDef(SEM_Thread1, SemaphoreThread1, osPriorityLow, 0, semtstSTACK_SIZE);
    SemThread1Handle = osThreadCreate(osThread(SEM_Thread1), (void *) osSemaphore);

    /* Create the second Thread */
    osThreadDef(SEM_Thread2, SemaphoreThread2, osPriorityIdle, 0, semtstSTACK_SIZE);
    SemThread2Handle = osThreadCreate(osThread(SEM_Thread2), (void *) osSemaphore);

    /* Start scheduler */
    osKernelStart();
  
    /* We should never get here as control is now taken by the scheduler */
    for(;;);
}

/**
  * @brief  Semaphore Thread 1 function
  * @param  argument: shared semaphore
  * @retval None
  */
static void SemaphoreThread1(void const *argument)
{
    uint32_t count = 0;
    osSemaphoreId semaphore = (osSemaphoreId) argument;

    for (;;)
    {
        if (semaphore != NULL)
        {
            /* Try to obtain the semaphore */
            if (osSemaphoreWait(semaphore , 100) == osOK)
            {
                count = osKernelSysTick() + 5000;

                /* Toggle LED2 every 200 ms for 5 seconds */
                while (count >= osKernelSysTick())
                {
                    /* Toggle LED1 */
                    LedBlink(LED1_PORT, LED1_PIN);

                    /* Delay 200 ms */
                    osDelay(200);
                }

                /* Turn off LED1*/
                LedOff(LED1_PORT, LED1_PIN);;
                /* Release the semaphore */
                osSemaphoreRelease(semaphore);

                /* Suspend ourseleves to execute thread 2 (lower priority)  */
                osThreadSuspend(NULL);
            }
        }
    }
}

/**
  * @brief  Semaphore Thread 2 function
  * @param  argument: shared semaphore
  * @retval None
  */
static void SemaphoreThread2(void const *argument)
{
    uint32_t count = 0;
    osSemaphoreId semaphore = (osSemaphoreId) argument;

    for (;;)
    {
        if (semaphore != NULL)
        {
            /* Try to obtain the semaphore */
            if (osSemaphoreWait(semaphore , 0) == osOK)
            {
                /* Resume Thread 1 (higher priority)*/
                osThreadResume(SemThread1Handle);

                count = osKernelSysTick() + 5000;

                /* Toggle LED2 every 200 ms for 5 seconds*/
                while (count >= osKernelSysTick())
                {
                    LedBlink(LED2_PORT, LED2_PIN);

                    osDelay(200);
                }

                /* Turn off LED2 */
                LedOff(LED2_PORT, LED2_PIN);

                /* Release the semaphore to unblock Thread 1 (higher priority)  */
                osSemaphoreRelease(semaphore);
            }
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
