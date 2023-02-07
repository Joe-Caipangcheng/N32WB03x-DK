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
 * @file uECC.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

 /** @addtogroup 
 * @{
 */
#ifndef __UECC_H__
#define __UECC_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
typedef uint32_t uECC_word_t;
typedef uint64_t uECC_dword_t;
typedef int8_t wordcount_t;
typedef int16_t bitcount_t;
typedef int8_t cmpresult_t;
/* Public define ------------------------------------------------------------*/  
#define HIGH_BIT_SET 0x80000000
#define uECC_WORD_BITS 32
#define uECC_MAX_WORDS 8
#define uECC_WORD_BITS_SHIFT 5
#define uECC_WORD_BITS_MASK 0x01F


#define num_bytes_secp256r1 32
#define num_words_secp256r1 8




#define BYTES_TO_WORDS_8(a, b, c, d, e, f, g, h) 0x##d##c##b##a, 0x##h##g##f##e
#define BYTES_TO_WORDS_4(a, b, c, d) 0x##d##c##b##a


#define uECC_WORD_SIZE 4
#define BITS_TO_WORDS(num_bits) ((num_bits + ((uECC_WORD_SIZE * 8) - 1)) / (uECC_WORD_SIZE * 8))
#define BITS_TO_BYTES(num_bits) ((num_bits + 7) / 8)

typedef const struct uECC_Curve_t * uECC_Curve;
struct uECC_Curve_t {
    wordcount_t num_words;
    wordcount_t num_bytes;
    bitcount_t num_n_bits;
    uECC_word_t p[uECC_MAX_WORDS];
    uECC_word_t n[uECC_MAX_WORDS];
    uECC_word_t G[uECC_MAX_WORDS * 2];
    uECC_word_t b[uECC_MAX_WORDS];
    void (*double_jacobian)(uECC_word_t * X1,
                            uECC_word_t * Y1,
                            uECC_word_t * Z1,
                            uECC_Curve curve);
    void (*x_side)(uECC_word_t *result, const uECC_word_t *x, uECC_Curve curve);
    void (*mmod_fast)(uECC_word_t *result, uECC_word_t *product);
};



extern const struct uECC_Curve_t curve_secp256r1;

/* Public constants ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
int uECC_verify(const uint8_t *public_key,
                const uint8_t *message_hash,
                unsigned hash_size,
                const uint8_t *signature,
                uECC_Curve curve);


#ifdef __cplusplus
}
#endif

#endif //__UECC_H__
/**
 * @}
 */

