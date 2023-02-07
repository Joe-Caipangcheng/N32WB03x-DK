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
#include "log.h"

/**
 *  Cortex-M0 ProcessStack
 */

#define DEMO_USART_BAUDRATE ((uint32_t)115200)

#define SP_PROCESS_SIZE          0x200 /* Process stack size */
#define SP_PROCESS               0x02  /* Process stack */
#define SP_MAIN                  0x00  /* Main stack */

/* clang-format off */
#if defined ( __CC_ARM   )
  __ASM void __SVC(void) 
  { 
    SVC 0x01 
    BX R14
  }
#elif defined ( __ICCARM__ )
  static __INLINE  void __SVC()                     { __ASM ("svc 0x01");}
#elif defined   (  __GNUC__  )
  static __INLINE void __SVC()                      { __ASM volatile ("svc 0x01");}
#endif
/* clang-format on */

__IO uint8_t PSPMemAlloc[SP_PROCESS_SIZE];
__IO uint32_t Index = 0, PSPValue = 0, CurrentStack = 0, IsrStack = 0xFF;

/**
 * @brief  Main program.
 */
int main(void)
{
    /* log Init */
    log_init();
    printf("Cortex-M0 ProcessStack \r\n");

    /* Switch Thread mode Stack from Main to Process */
    /* Initialize memory reserved for Process Stack */
    for (Index = 0; Index < SP_PROCESS_SIZE; Index++)
    {
        PSPMemAlloc[Index] = 0x00;
    }

    /* Set Process stack value */
    __set_PSP((uint32_t)PSPMemAlloc + SP_PROCESS_SIZE);

    /* Select Process Stack as Thread mode Stack */
    __set_CONTROL(SP_PROCESS);

    /* Execute ISB instruction to flush pipeline as recommended by Arm */
    __ISB();

    /* Get the Thread mode stack used */
    CurrentStack = (__get_CONTROL() & 0x02);

    /* Get process stack pointer value */
    PSPValue = __get_PSP();
  
    /* Check is mode has been well applied */
    if(CurrentStack != SP_PROCESS)
    {
        printf("Cortex-M0 ProcessStack Test Error 0 \r\n");
        while(1);
    }

    /* Generate a system call exception: Main Stack pointer should be automaticcaly 
    when entering in ISR context */
    __SVC();

    /* Check is Main stack was used under ISR*/
    if(IsrStack != SP_MAIN)
    {
        printf("Cortex-M0 ProcessStack Test Error 1 \r\n");
        while(1);
    }

    /* Get the Thread mode stack used to verify we have switched back automatically 
    to Process Stack */
    
    CurrentStack = (__get_CONTROL() & 0x02);
    
    /* Check is mode has been well applied */
    if(CurrentStack != SP_PROCESS)
    {
        printf("Cortex-M0 ProcessStack Test Error 2 \r\n");
    }
    else
    {
        printf("Cortex-M0 ProcessStack Test Success \r\n");
    }

    while (1)
    {
    }
}

/**
 * @}
 */

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
*          line: assert_param error line source number
 * @return None
 */
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {}
}

/**
 * @}
 */
#endif
