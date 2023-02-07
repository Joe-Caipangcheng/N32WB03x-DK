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
 *  FreeRTOS Mutexes
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_1MS           ((uint32_t)1000)
/* Private macro -------------------------------------------------------------*/
#define mutexSHORT_DELAY     ((uint32_t) 20)
#define mutexNO_DELAY        ((uint32_t) 0)
#define mutexTWO_TICK_DELAY  ((uint32_t) 2)
/* Private variables ---------------------------------------------------------*/
uint32_t Tick_num = 0;
static osMutexId osMutex;

/* Variables used to detect and latch errors */
__IO uint32_t HighPriorityThreadCycles = 0, MediumPriorityThreadCycles = 0, LowPriorityThreadCycles = 0;

/* Handles of the two higher priority tasks, required so they can be resumed(unsuspended) */
static osThreadId osHighPriorityThreadHandle, osMediumPriorityThreadHandle;

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

    /* Create the mutex  */
    osMutexDef(osMutex);
    osMutex = osMutexCreate(osMutex(osMutex));

    if (osMutex != NULL)
    {
        /* Define and create the high priority thread */
        osThreadDef(MutHigh, MutexHighPriorityThread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE);
        osHighPriorityThreadHandle = osThreadCreate(osThread(MutHigh), NULL);

        /* Define and create the medium priority thread */
        osThreadDef(MutMedium, MutexMediumPriorityThread, osPriorityLow, 0, configMINIMAL_STACK_SIZE);
        osMediumPriorityThreadHandle = osThreadCreate(osThread(MutMedium), NULL);

        /* Define and create the low priority thread */
        osThreadDef(MutLow, MutexLowPriorityThread, osPriorityIdle, 0, configMINIMAL_STACK_SIZE);
        osThreadCreate(osThread(MutLow), NULL);
    }

    /* Start scheduler */
    osKernelStart();
  
    /* We should never get here as control is now taken by the scheduler */
    for(;;);
}

/**
  * @brief  Mutex High Priority Thread.
  * @param  argument: Not used
  * @retval None
  */
static void MutexHighPriorityThread(void const *argument)
{
  /* Just to remove compiler warning */
  (void) argument;

  for (;;)
  {
    /* The first time through the mutex will be immediately available, on
    subsequent times through the mutex will be held by the low priority thread
    at this point and this Take will cause the low priority thread to inherit
    the priority of this tadhr.  In this case the block time must be
    long enough to ensure the low priority thread will execute again before the
    block time expires.  If the block time does expire then the error
    flag will be set here */
    if (osMutexWait(osMutex, mutexTWO_TICK_DELAY) != osOK)
    {
      /* Toggle LED2 to indicate error */
      LedBlink(LED2_PORT,LED2_PIN);
    }

    /* Ensure the other thread attempting to access the mutex
    are able to execute to ensure they either block (where a block
    time is specified) or return an error (where no block time is
    specified) as the mutex is held by this task */
    osDelay(mutexSHORT_DELAY);

    /* We should now be able to release the mutex .
    When the mutex is available again the medium priority thread
    should be unblocked but not run because it has a lower priority
    than this thread.  The low priority thread should also not run
    at this point as it too has a lower priority than this thread */
    if (osMutexRelease(osMutex) != osOK)
    {
      /* Toggle LED2 to indicate error */
      LedBlink(LED2_PORT,LED2_PIN);
    }

    /* Keep count of the number of cycles this thread has performed */
    HighPriorityThreadCycles++;
    LedBlink(LED1_PORT,LED1_PIN);

    /* Suspend ourselves to the medium priority thread can execute */
    osThreadSuspend(NULL);
  }
}

/**
  * @brief  Mutex Medium Priority Thread.
  * @param  argument: Not used
  * @retval None
  */
static void MutexMediumPriorityThread(void const *argument)
{
  /* Just to remove compiler warning */
  (void) argument;

  for (;;)
  {
    /* This thread will run while the high-priority thread is blocked, and the
    high-priority thread will block only once it has the mutex - therefore
    this call should block until the high-priority thread has given up the
    mutex, and not actually execute past this call until the high-priority
    thread is suspended */
    if (osMutexWait(osMutex, osWaitForever) == osOK)
    {
      if (osThreadGetState(osHighPriorityThreadHandle) != osThreadSuspended)
      {
        /* Did not expect to execute until the high priority thread was
        suspended.
        Toggle LED2 to indicate error */
        LedBlink(LED2_PORT,LED2_PIN);
      }
      else
      {
        /* Give the mutex back before suspending ourselves to allow
        the low priority thread to obtain the mutex */
        if (osMutexRelease(osMutex) != osOK)
        {
          /* Toggle LED2 to indicate error */
          LedBlink(LED2_PORT,LED2_PIN);
        }
        osThreadSuspend(NULL);
      }
    }
    else
    {
      /* We should not leave the osMutexWait() function
      until the mutex was obtained.
      Toggle LED2 to indicate error */
      LedBlink(LED2_PORT,LED2_PIN);
    }

    /* The High and Medium priority threads should be in lock step */
    if (HighPriorityThreadCycles != (MediumPriorityThreadCycles + 1))
    {
      /* Toggle LED2 to indicate error */
      LedBlink(LED2_PORT,LED2_PIN);
    }

    /* Keep count of the number of cycles this task has performed so a
    stall can be detected */
    MediumPriorityThreadCycles++;
    LedBlink(LED1_PORT,LED1_PIN);
  }
}

/**
  * @brief  Mutex Low Priority Thread.
  * @param  argument: Not used
  * @retval None
  */
static void MutexLowPriorityThread(void const *argument)
{
  /* Just to remove compiler warning */
  (void) argument;

  for (;;)
  {
    /* Keep attempting to obtain the mutex.  We should only obtain it when
    the medium-priority thread has suspended itself, which in turn should only
    happen when the high-priority thread is also suspended */
    if (osMutexWait(osMutex, mutexNO_DELAY) == osOK)
    {
      /* Is the haigh and medium-priority threads suspended? */
      if ((osThreadGetState(osHighPriorityThreadHandle) != osThreadSuspended) || (osThreadGetState(osMediumPriorityThreadHandle) != osThreadSuspended))
      {
        /* Toggle LED2 to indicate error */
        LedBlink(LED2_PORT,LED2_PIN);
      }
      else
      {
        /* Keep count of the number of cycles this task has performed
        so a stall can be detected */
        LowPriorityThreadCycles++;
        LedBlink(LED1_PORT,LED1_PIN);

        /* We can resume the other tasks here even though they have a
        higher priority than the this thread. When they execute they
        will attempt to obtain the mutex but fail because the low-priority 
        thread is still the mutex holder.  this thread will then inherit 
        the higher priority.  The medium-priority thread will block indefinitely
        when it attempts to obtain the mutex, the high-priority thread will only
        block for a fixed period and an error will be latched if the
        high-priority thread has not returned the mutex by the time this
        fixed period has expired */
        osThreadResume(osMediumPriorityThreadHandle);
        osThreadResume(osHighPriorityThreadHandle);

        /* The other two tasks should now have executed and no longer
        be suspended */
        if ((osThreadGetState(osHighPriorityThreadHandle) == osThreadSuspended) || (osThreadGetState(osMediumPriorityThreadHandle) == osThreadSuspended))
        {
          /* Toggle LED2 to indicate error */
          LedBlink(LED2_PORT,LED2_PIN);
        }

        /* Release the mutex, disinheriting the higher priority again */
        if (osMutexRelease(osMutex) != osOK)
        {
          /* Toggle LED2 to indicate error */
          LedBlink(LED2_PORT,LED2_PIN);
        }
      }
    }

#if configUSE_PREEMPTION == 0
    {
      taskYIELD();
    }
#endif
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
