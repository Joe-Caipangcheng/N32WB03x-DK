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
 * @file TypeDefine.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __TYPES_H__
#define __TYPES_H__

//------------------------------------------------------------------
//                        Headers
//------------------------------------------------------------------
#include "core_cm0.h"
//------------------------------------------------------------------
//                        Definitions
//------------------------------------------------------------------
#define    FALSE       0
#define    TRUE        1

//#define NULL        0

#define REG8(addr)          (*(volatile UINT8 *) (addr))
#define REG16(addr)          (*(volatile UINT16 *)(addr))
#define REG32(addr)          (*(volatile UINT32 *)(addr))

//------------------------------------------------------------------
//                        TypeDefs
//------------------------------------------------------------------
typedef    unsigned char     UINT8;    ///<unsigned char
typedef    signed char       INT8;    ///< char

typedef    unsigned short    UINT16;    ///<unsigned char
typedef    signed short      INT16;    ///<short

typedef unsigned int      UINT32;    ///<unsigned int
typedef    signed int        INT32;    ///<int

typedef unsigned char     BOOL;    ///<BOOL

typedef unsigned int      U32;
typedef unsigned short    U16;
typedef unsigned char     U8;
typedef signed short      S16;
typedef signed int        S32;
typedef signed char       S8;
typedef unsigned long long    U64;

//typedef unsigned char     uint8_t;
//typedef unsigned short    uint16_t;
//typedef unsigned int      uint32_t;

//------------------------------------------------------------------
//                        Exported variables
//------------------------------------------------------------------

//------------------------------------------------------------------
//                        Exported functions
//------------------------------------------------------------------

#endif

