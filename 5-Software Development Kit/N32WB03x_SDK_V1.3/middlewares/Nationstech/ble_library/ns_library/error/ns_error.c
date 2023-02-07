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
 * @file ns_error.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "ns_error.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
 * @brief Save the error state.
 * @param[in] error information
 * @return none
 */
static void error_save_and_stop(uint32_t info)
{
    /* static error variables - in order to prevent removal by optimizers */
    static volatile struct
    {
        uint32_t      err_code;
        uint32_t        line_num;
        const uint8_t * p_file_name;
    } m_error_data = {0,0,NULL};

    // The following variable helps Keil keep the call stack visible, in addition, it can be set to
    // 0 in the debugger to continue executing code after the error check.
    volatile bool loop = true;

        m_error_data.err_code     = ((error_info_t *)(info))->err_code;
        m_error_data.line_num     = (unsigned int) ((error_info_t *)(info))->line_num;
        m_error_data.p_file_name  = ((error_info_t *)(info))->p_file_name;


        
        
    while (loop);

}




/**
 * @brief Handle the error state.
 * @param[in] error_code error code.
 * @param[in] line_num error file line number.
 * @param[in] p_file_name error file name.
 * @return none
 */
void error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    error_info_t error_info =
    {
        .line_num    = line_num,
        .p_file_name = p_file_name,
        .err_code    = error_code,
    };
    error_save_and_stop((uint32_t)(&error_info));
}



















