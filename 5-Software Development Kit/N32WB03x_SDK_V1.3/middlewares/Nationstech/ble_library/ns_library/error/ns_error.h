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
 * @file ns_error.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

 /** @addtogroup 
 * @{
 */
#ifndef __NS_LIB_ERROR_H__
#define __NS_LIB_ERROR_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>  
#include <stdbool.h>     
#include <stddef.h>      
/* Public define ------------------------------------------------------------*/
#define ERROR_BASE_NUM                      (0x0)       
 
#define ERROR_SUCCESS                     (ERROR_BASE_NUM + 0)   
#define ERROR_INTERNAL                    (ERROR_BASE_NUM + 1)  
#define ERROR_NO_MEM                      (ERROR_BASE_NUM + 2)  
#define ERROR_NOT_FOUND                   (ERROR_BASE_NUM + 3)  
#define ERROR_NOT_SUPPORTED               (ERROR_BASE_NUM + 4)  
#define ERROR_INVALID_PARAM               (ERROR_BASE_NUM + 5)  
#define ERROR_INVALID_STATE               (ERROR_BASE_NUM + 6)  
#define ERROR_INVALID_LENGTH              (ERROR_BASE_NUM + 7)  
#define ERROR_INVALID_FLAGS               (ERROR_BASE_NUM + 8) 
#define ERROR_INVALID_DATA                (ERROR_BASE_NUM + 9) 
#define ERROR_DATA_SIZE                   (ERROR_BASE_NUM + 10) 
#define ERROR_TIMEOUT                     (ERROR_BASE_NUM + 11) 
#define ERROR_NULL                        (ERROR_BASE_NUM + 12) 
#define ERROR_FORBIDDEN                   (ERROR_BASE_NUM + 13) 
#define ERROR_INVALID_ADDR                (ERROR_BASE_NUM + 14) 
#define ERROR_BUSY                        (ERROR_BASE_NUM + 15) 
#define ERROR_CRC                         (ERROR_BASE_NUM + 16) 
#define ERROR_HARD_FAULT                  (ERROR_BASE_NUM + 17)     
/* Public typedef -----------------------------------------------------------*/
typedef struct
{
    uint16_t        line_num;    
    uint8_t const * p_file_name; 
    uint32_t        err_code;    
} error_info_t;
/* Public constants ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name);


#define ERROR_HANDLER(ERR_CODE)                                    \
    do                                                                 \
    {                                                                  \
        error_handler((ERR_CODE), __LINE__, (uint8_t*) __FILE__);  \
    } while (0)



#define ERROR_CHECK(ERR_CODE)                           \
    do                                                      \
    {                                                       \
        const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
        if (LOCAL_ERR_CODE != ERROR_SUCCESS)                  \
        {                                                   \
            ERROR_HANDLER(LOCAL_ERR_CODE);              \
        }                                                   \
    } while (0)
    







#ifdef __cplusplus
}
#endif








#endif //__NS_LIB_ERROR_H__
/**
 * @}
 */


