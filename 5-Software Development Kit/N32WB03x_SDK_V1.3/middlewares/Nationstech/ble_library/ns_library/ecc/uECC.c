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
 * @file uECC.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "uECC.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define uECC_VLI_API static

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/






static void bcopy(uint8_t *dst,
                  const uint8_t *src,
                  unsigned num_bytes) {
    while (0 != num_bytes) {
        num_bytes--;
        dst[num_bytes] = src[num_bytes];
    }
}




uECC_VLI_API uECC_word_t uECC_vli_add(uECC_word_t *result,
                                      const uECC_word_t *left,
                                      const uECC_word_t *right,
                                      wordcount_t num_words) {
    uECC_word_t carry = 0;
    wordcount_t i;
    for (i = 0; i < num_words; ++i) {
        uECC_word_t sum = left[i] + right[i] + carry;
        if (sum != left[i]) {
            carry = (sum < left[i]);
        }
        result[i] = sum;
    }
    return carry;
}

uECC_VLI_API uECC_word_t uECC_vli_sub(uECC_word_t *result,
                                      const uECC_word_t *left,
                                      const uECC_word_t *right,
                                      wordcount_t num_words) {
    uECC_word_t borrow = 0;
    wordcount_t i;
    for (i = 0; i < num_words; ++i) {
        uECC_word_t diff = left[i] - right[i] - borrow;
        if (diff != left[i]) {
            borrow = (diff > left[i]);
        }
        result[i] = diff;
    }
    return borrow;
}


static void muladd(uECC_word_t a,
                   uECC_word_t b,
                   uECC_word_t *r0,
                   uECC_word_t *r1,
                   uECC_word_t *r2) {
    uECC_dword_t p = (uECC_dword_t)a * b;
    uECC_dword_t r01 = ((uECC_dword_t)(*r1) << uECC_WORD_BITS) | *r0;
    r01 += p;
    *r2 += (r01 < p);
    *r1 = r01 >> uECC_WORD_BITS;
    *r0 = (uECC_word_t)r01;
}

uECC_VLI_API void uECC_vli_mult(uECC_word_t *result,
                                const uECC_word_t *left,
                                const uECC_word_t *right,
                                wordcount_t num_words) {
    uECC_word_t r0 = 0;
    uECC_word_t r1 = 0;
    uECC_word_t r2 = 0;
    wordcount_t i, k;

    /* Compute each digit of result in sequence, maintaining the carries. */
    for (k = 0; k < num_words; ++k) {
        for (i = 0; i <= k; ++i) {
            muladd(left[i], right[k - i], &r0, &r1, &r2);
        }
        result[k] = r0;
        r0 = r1;
        r1 = r2;
        r2 = 0;
    }
    for (k = num_words; k < num_words * 2 - 1; ++k) {
        for (i = (k + 1) - num_words; i < num_words; ++i) {
            muladd(left[i], right[k - i], &r0, &r1, &r2);
        }
        result[k] = r0;
        r0 = r1;
        r1 = r2;
        r2 = 0;
    }
    result[num_words * 2 - 1] = r0;
}

uECC_VLI_API void uECC_vli_set(uECC_word_t *dest, const uECC_word_t *src, wordcount_t num_words) {
    wordcount_t i;
    for (i = 0; i < num_words; ++i) {
        dest[i] = src[i];
    }
}

/* Constant-time comparison to zero - secure way to compare long integers */
/* Returns 1 if vli == 0, 0 otherwise. */
uECC_VLI_API uECC_word_t uECC_vli_isZero(const uECC_word_t *vli, wordcount_t num_words) {
    uECC_word_t bits = 0;
    wordcount_t i;
    for (i = 0; i < num_words; ++i) {
        bits |= vli[i];
    }
    return (bits == 0);
}
uECC_VLI_API void uECC_vli_modMult_fast(uECC_word_t *result,
                                        const uECC_word_t *left,
                                        const uECC_word_t *right,
                                        uECC_Curve curve) {
    uECC_word_t product[2 * uECC_MAX_WORDS];
    uECC_vli_mult(product, left, right, curve->num_words);
    curve->mmod_fast(result, product);
}


/* Returns sign of left - right. */
static cmpresult_t uECC_vli_cmp_unsafe(const uECC_word_t *left,
                                       const uECC_word_t *right,
                                       wordcount_t num_words) {
    wordcount_t i;
    for (i = num_words - 1; i >= 0; --i) {
        if (left[i] > right[i]) {
            return 1;
        } else if (left[i] < right[i]) {
            return -1;
        }
    }
    return 0;
}

/* Computes result = (left + right) % mod.
   Assumes that left < mod and right < mod, and that result does not overlap mod. */
uECC_VLI_API void uECC_vli_modAdd(uECC_word_t *result,
                                  const uECC_word_t *left,
                                  const uECC_word_t *right,
                                  const uECC_word_t *mod,
                                  wordcount_t num_words) {
    uECC_word_t carry = uECC_vli_add(result, left, right, num_words);
    if (carry || uECC_vli_cmp_unsafe(mod, result, num_words) != 1) {
        /* result > mod (result = mod + remainder), so subtract mod to get remainder. */
        uECC_vli_sub(result, result, mod, num_words);
    }
}



uECC_VLI_API void uECC_vli_modSquare_fast(uECC_word_t *result,
                                          const uECC_word_t *left,
                                          uECC_Curve curve) {
    uECC_vli_modMult_fast(result, left, left, curve);
}


/* Computes result = (left - right) % mod.
   Assumes that left < mod and right < mod, and that result does not overlap mod. */
uECC_VLI_API void uECC_vli_modSub(uECC_word_t *result,
                                  const uECC_word_t *left,
                                  const uECC_word_t *right,
                                  const uECC_word_t *mod,
                                  wordcount_t num_words) {
    uECC_word_t l_borrow = uECC_vli_sub(result, left, right, num_words);
    if (l_borrow) {
        /* In this case, result == -diff == (max int) - diff. Since -x % d == d - x,
           we can get the correct result from result + mod (with overflow). */
        uECC_vli_add(result, result, mod, num_words);
    }
}
/* Returns nonzero if bit 'bit' of vli is set. */
uECC_VLI_API uECC_word_t uECC_vli_testBit(const uECC_word_t *vli, bitcount_t bit) {
    return (vli[bit >> uECC_WORD_BITS_SHIFT] & ((uECC_word_t)1 << (bit & uECC_WORD_BITS_MASK)));
}
uECC_VLI_API void uECC_vli_rshift1(uECC_word_t *vli, wordcount_t num_words) {
    uECC_word_t *end = vli;
    uECC_word_t carry = 0;

    vli += num_words;
    while (vli-- > end) {
        uECC_word_t temp = *vli;
        *vli = (temp >> 1) | carry;
        carry = temp << (uECC_WORD_BITS - 1);
    }
}

static void double_jacobian_default(uECC_word_t * X1,
                                    uECC_word_t * Y1,
                                    uECC_word_t * Z1,
                                    uECC_Curve curve) {
    /* t1 = X, t2 = Y, t3 = Z */
    uECC_word_t t4[uECC_MAX_WORDS];
    uECC_word_t t5[uECC_MAX_WORDS];
    wordcount_t num_words = curve->num_words;

    if (uECC_vli_isZero(Z1, num_words)) {
        return;
    }

    uECC_vli_modSquare_fast(t4, Y1, curve);   /* t4 = y1^2 */
    uECC_vli_modMult_fast(t5, X1, t4, curve); /* t5 = x1*y1^2 = A */
    uECC_vli_modSquare_fast(t4, t4, curve);   /* t4 = y1^4 */
    uECC_vli_modMult_fast(Y1, Y1, Z1, curve); /* t2 = y1*z1 = z3 */
    uECC_vli_modSquare_fast(Z1, Z1, curve);   /* t3 = z1^2 */

    uECC_vli_modAdd(X1, X1, Z1, curve->p, num_words); /* t1 = x1 + z1^2 */
    uECC_vli_modAdd(Z1, Z1, Z1, curve->p, num_words); /* t3 = 2*z1^2 */
    uECC_vli_modSub(Z1, X1, Z1, curve->p, num_words); /* t3 = x1 - z1^2 */
    uECC_vli_modMult_fast(X1, X1, Z1, curve);                /* t1 = x1^2 - z1^4 */

    uECC_vli_modAdd(Z1, X1, X1, curve->p, num_words); /* t3 = 2*(x1^2 - z1^4) */
    uECC_vli_modAdd(X1, X1, Z1, curve->p, num_words); /* t1 = 3*(x1^2 - z1^4) */
    if (uECC_vli_testBit(X1, 0)) {
        uECC_word_t l_carry = uECC_vli_add(X1, X1, curve->p, num_words);
        uECC_vli_rshift1(X1, num_words);
        X1[num_words - 1] |= l_carry << (uECC_WORD_BITS - 1);
    } else {
        uECC_vli_rshift1(X1, num_words);
    }
    /* t1 = 3/2*(x1^2 - z1^4) = B */

    uECC_vli_modSquare_fast(Z1, X1, curve);                  /* t3 = B^2 */
    uECC_vli_modSub(Z1, Z1, t5, curve->p, num_words); /* t3 = B^2 - A */
    uECC_vli_modSub(Z1, Z1, t5, curve->p, num_words); /* t3 = B^2 - 2A = x3 */
    uECC_vli_modSub(t5, t5, Z1, curve->p, num_words); /* t5 = A - x3 */
    uECC_vli_modMult_fast(X1, X1, t5, curve);                /* t1 = B * (A - x3) */
    uECC_vli_modSub(t4, X1, t4, curve->p, num_words); /* t4 = B * (A - x3) - y1^4 = y3 */

    uECC_vli_set(X1, Z1, num_words);
    uECC_vli_set(Z1, Y1, num_words);
    uECC_vli_set(Y1, t4, num_words);
}




const struct uECC_Curve_t curve_secp256r1;



void vli_mmod_fast_secp256r1(uint32_t *result, uint32_t *product) {
    uint32_t tmp[num_words_secp256r1];
    int carry;
    
    /* t */
    uECC_vli_set(result, product, num_words_secp256r1);
    
    /* s1 */
    tmp[0] = tmp[1] = tmp[2] = 0;
    tmp[3] = product[11];
    tmp[4] = product[12];
    tmp[5] = product[13];
    tmp[6] = product[14];
    tmp[7] = product[15];
    carry = uECC_vli_add(tmp, tmp, tmp, num_words_secp256r1);
    carry += uECC_vli_add(result, result, tmp, num_words_secp256r1);
    
    /* s2 */
    tmp[3] = product[12];
    tmp[4] = product[13];
    tmp[5] = product[14];
    tmp[6] = product[15];
    tmp[7] = 0;
    carry += uECC_vli_add(tmp, tmp, tmp, num_words_secp256r1);
    carry += uECC_vli_add(result, result, tmp, num_words_secp256r1);
    
    /* s3 */
    tmp[0] = product[8];
    tmp[1] = product[9];
    tmp[2] = product[10];
    tmp[3] = tmp[4] = tmp[5] = 0;
    tmp[6] = product[14];
    tmp[7] = product[15];
    carry += uECC_vli_add(result, result, tmp, num_words_secp256r1);
    
    /* s4 */
    tmp[0] = product[9];
    tmp[1] = product[10];
    tmp[2] = product[11];
    tmp[3] = product[13];
    tmp[4] = product[14];
    tmp[5] = product[15];
    tmp[6] = product[13];
    tmp[7] = product[8];
    carry += uECC_vli_add(result, result, tmp, num_words_secp256r1);
    
    /* d1 */
    tmp[0] = product[11];
    tmp[1] = product[12];
    tmp[2] = product[13];
    tmp[3] = tmp[4] = tmp[5] = 0;
    tmp[6] = product[8];
    tmp[7] = product[10];
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp256r1);
    
    /* d2 */
    tmp[0] = product[12];
    tmp[1] = product[13];
    tmp[2] = product[14];
    tmp[3] = product[15];
    tmp[4] = tmp[5] = 0;
    tmp[6] = product[9];
    tmp[7] = product[11];
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp256r1);
    
    /* d3 */
    tmp[0] = product[13];
    tmp[1] = product[14];
    tmp[2] = product[15];
    tmp[3] = product[8];
    tmp[4] = product[9];
    tmp[5] = product[10];
    tmp[6] = 0;
    tmp[7] = product[12];
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp256r1);
    
    /* d4 */
    tmp[0] = product[14];
    tmp[1] = product[15];
    tmp[2] = 0;
    tmp[3] = product[9];
    tmp[4] = product[10];
    tmp[5] = product[11];
    tmp[6] = 0;
    tmp[7] = product[13];
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp256r1);
    
    if (carry < 0) {
        do {
            carry += uECC_vli_add(result, result, curve_secp256r1.p, num_words_secp256r1);
        } while (carry < 0);
    } else {
        while (carry || uECC_vli_cmp_unsafe(curve_secp256r1.p, result, num_words_secp256r1) != 1) {
            carry -= uECC_vli_sub(result, result, curve_secp256r1.p, num_words_secp256r1);
        }
    }
}

/* Computes result = x^3 + ax + b. result must not overlap x. */
static void x_side_default(uECC_word_t *result, const uECC_word_t *x, uECC_Curve curve) {
    uECC_word_t _3[uECC_MAX_WORDS] = {3}; /* -a = 3 */
    wordcount_t num_words = curve->num_words;

    uECC_vli_modSquare_fast(result, x, curve);                             /* r = x^2 */
    uECC_vli_modSub(result, result, _3, curve->p, num_words);       /* r = x^2 - 3 */
    uECC_vli_modMult_fast(result, result, x, curve);                       /* r = x^3 - 3x */
    uECC_vli_modAdd(result, result, curve->b, curve->p, num_words); /* r = x^3 - 3x + b */
}


const struct uECC_Curve_t curve_secp256r1 = {
    num_words_secp256r1,
    num_bytes_secp256r1,
    256, /* num_n_bits */
    { BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(01, 00, 00, 00, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(51, 25, 63, FC, C2, CA, B9, F3),
        BYTES_TO_WORDS_8(84, 9E, 17, A7, AD, FA, E6, BC),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(00, 00, 00, 00, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(96, C2, 98, D8, 45, 39, A1, F4),
        BYTES_TO_WORDS_8(A0, 33, EB, 2D, 81, 7D, 03, 77),
        BYTES_TO_WORDS_8(F2, 40, A4, 63, E5, E6, BC, F8),
        BYTES_TO_WORDS_8(47, 42, 2C, E1, F2, D1, 17, 6B),

        BYTES_TO_WORDS_8(F5, 51, BF, 37, 68, 40, B6, CB),
        BYTES_TO_WORDS_8(CE, 5E, 31, 6B, 57, 33, CE, 2B),
        BYTES_TO_WORDS_8(16, 9E, 0F, 7C, 4A, EB, E7, 8E),
        BYTES_TO_WORDS_8(9B, 7F, 1A, FE, E2, 42, E3, 4F) },
    { BYTES_TO_WORDS_8(4B, 60, D2, 27, 3E, 3C, CE, 3B),
        BYTES_TO_WORDS_8(F6, B0, 53, CC, B0, 06, 1D, 65),
        BYTES_TO_WORDS_8(BC, 86, 98, 76, 55, BD, EB, B3),
        BYTES_TO_WORDS_8(E7, 93, 3A, AA, D8, 35, C6, 5A) },
    &double_jacobian_default,
    &x_side_default,
    &vli_mmod_fast_secp256r1
};

uECC_VLI_API void uECC_vli_clear(uECC_word_t *vli, wordcount_t num_words) {
    wordcount_t i;
    for (i = 0; i < num_words; ++i) {
        vli[i] = 0;
    }
}
#define EVEN(vli) (!(vli[0] & 1))
static void vli_modInv_update(uECC_word_t *uv,
                              const uECC_word_t *mod,
                              wordcount_t num_words) {
    uECC_word_t carry = 0;
    if (!EVEN(uv)) {
        carry = uECC_vli_add(uv, uv, mod, num_words);
    }
    uECC_vli_rshift1(uv, num_words);
    if (carry) {
        uv[num_words - 1] |= HIGH_BIT_SET;
    }
}
/* Computes result = (1 / input) % mod. All VLIs are the same size.
   See "From Euclid's GCD to Montgomery Multiplication to the Great Divide" */
uECC_VLI_API void uECC_vli_modInv(uECC_word_t *result,
                                  const uECC_word_t *input,
                                  const uECC_word_t *mod,
                                  wordcount_t num_words) {
    uECC_word_t a[uECC_MAX_WORDS], b[uECC_MAX_WORDS], u[uECC_MAX_WORDS], v[uECC_MAX_WORDS];
    cmpresult_t cmpResult;

    if (uECC_vli_isZero(input, num_words)) {
        uECC_vli_clear(result, num_words);
        return;
    }

    uECC_vli_set(a, input, num_words);
    uECC_vli_set(b, mod, num_words);
    uECC_vli_clear(u, num_words);
    u[0] = 1;
    uECC_vli_clear(v, num_words);
    while ((cmpResult = uECC_vli_cmp_unsafe(a, b, num_words)) != 0) {
        if (EVEN(a)) {
            uECC_vli_rshift1(a, num_words);
            vli_modInv_update(u, mod, num_words);
        } else if (EVEN(b)) {
            uECC_vli_rshift1(b, num_words);
            vli_modInv_update(v, mod, num_words);
        } else if (cmpResult > 0) {
            uECC_vli_sub(a, a, b, num_words);
            uECC_vli_rshift1(a, num_words);
            if (uECC_vli_cmp_unsafe(u, v, num_words) < 0) {
                uECC_vli_add(u, u, mod, num_words);
            }
            uECC_vli_sub(u, u, v, num_words);
            vli_modInv_update(u, mod, num_words);
        } else {
            uECC_vli_sub(b, b, a, num_words);
            uECC_vli_rshift1(b, num_words);
            if (uECC_vli_cmp_unsafe(v, u, num_words) < 0) {
                uECC_vli_add(v, v, mod, num_words);
            }
            uECC_vli_sub(v, v, u, num_words);
            vli_modInv_update(v, mod, num_words);
        }
    }
    uECC_vli_set(result, u, num_words);
}
static void bits2int(uECC_word_t *native,
                     const uint8_t *bits,
                     unsigned bits_size,
                     uECC_Curve curve) {
    unsigned num_n_bytes = BITS_TO_BYTES(curve->num_n_bits);
    unsigned num_n_words = BITS_TO_WORDS(curve->num_n_bits);
    int shift;
    uECC_word_t carry;
    uECC_word_t *ptr;

    if (bits_size > num_n_bytes) {
        bits_size = num_n_bytes;
    }

    uECC_vli_clear(native, num_n_words);
    bcopy((uint8_t *) native, bits, bits_size);
    if (bits_size * 8 <= (unsigned)curve->num_n_bits) {
        return;
    }
    shift = bits_size * 8 - curve->num_n_bits;
    carry = 0;
    ptr = native + num_n_words;
    while (ptr-- > native) {
        uECC_word_t temp = *ptr;
        *ptr = (temp >> shift) | carry;
        carry = temp << (uECC_WORD_BITS - shift);
    }

    /* Reduce mod curve_n */
    if (uECC_vli_cmp_unsafe(curve->n, native, num_n_words) != 1) {
        uECC_vli_sub(native, native, curve->n, num_n_words);
    }
}
                        
/* Counts the number of words in vli. */
static wordcount_t vli_numDigits(const uECC_word_t *vli, const wordcount_t max_words) {
    wordcount_t i;
    /* Search from the end until we find a non-zero digit.
       We do it in reverse because we expect that most digits will be nonzero. */
    for (i = max_words - 1; i >= 0 && vli[i] == 0; --i) {
    }

    return (i + 1);
}
/* Counts the number of bits required to represent vli. */
uECC_VLI_API bitcount_t uECC_vli_numBits(const uECC_word_t *vli, const wordcount_t max_words) {
    uECC_word_t i;
    uECC_word_t digit;

    wordcount_t num_digits = vli_numDigits(vli, max_words);
    if (num_digits == 0) {
        return 0;
    }

    digit = vli[num_digits - 1];
    for (i = 0; digit; ++i) {
        digit >>= 1;
    }

    return (((bitcount_t)(num_digits - 1) << uECC_WORD_BITS_SHIFT) + i);
}

/* Computes result = product % mod, where product is 2N words long. */
/* Currently only designed to work for curve_p or curve_n. */
uECC_VLI_API void uECC_vli_mmod(uECC_word_t *result,
                                uECC_word_t *product,
                                const uECC_word_t *mod,
                                wordcount_t num_words) {
    uECC_word_t mod_multiple[2 * uECC_MAX_WORDS];
    uECC_word_t tmp[2 * uECC_MAX_WORDS];
    uECC_word_t *v[2] = {tmp, product};
    uECC_word_t index;

    /* Shift mod so its highest set bit is at the maximum position. */
    bitcount_t shift = (num_words * 2 * uECC_WORD_BITS) - uECC_vli_numBits(mod, num_words);
    wordcount_t word_shift = shift / uECC_WORD_BITS;
    wordcount_t bit_shift = shift % uECC_WORD_BITS;
    uECC_word_t carry = 0;
    uECC_vli_clear(mod_multiple, word_shift);
    if (bit_shift > 0) {
        for(index = 0; index < (uECC_word_t)num_words; ++index) {
            mod_multiple[word_shift + index] = (mod[index] << bit_shift) | carry;
            carry = mod[index] >> (uECC_WORD_BITS - bit_shift);
        }
    } else {
        uECC_vli_set(mod_multiple + word_shift, mod, num_words);
    }

    for (index = 1; shift >= 0; --shift) {
        uECC_word_t borrow = 0;
        wordcount_t i;
        for (i = 0; i < num_words * 2; ++i) {
            uECC_word_t diff = v[index][i] - mod_multiple[i] - borrow;
            if (diff != v[index][i]) {
                borrow = (diff > v[index][i]);
            }
            v[1 - index][i] = diff;
        }
        index = !(index ^ borrow); /* Swap the index if there was no borrow */
        uECC_vli_rshift1(mod_multiple, num_words);
        mod_multiple[num_words - 1] |= mod_multiple[num_words] << (uECC_WORD_BITS - 1);
        uECC_vli_rshift1(mod_multiple + num_words, num_words);
    }
    uECC_vli_set(result, v[index], num_words);
}
/* Computes result = (left * right) % mod. */
uECC_VLI_API void uECC_vli_modMult(uECC_word_t *result,
                                   const uECC_word_t *left,
                                   const uECC_word_t *right,
                                   const uECC_word_t *mod,
                                   wordcount_t num_words) {
    uECC_word_t product[2 * uECC_MAX_WORDS];
    uECC_vli_mult(product, left, right, num_words);
    uECC_vli_mmod(result, product, mod, num_words);
}
                                


/* Input P = (x1, y1, Z), Q = (x2, y2, Z)
   Output P' = (x1', y1', Z3), P + Q = (x3, y3, Z3)
   or P => P', Q => P + Q
*/
static void XYcZ_add(uECC_word_t * X1,
                     uECC_word_t * Y1,
                     uECC_word_t * X2,
                     uECC_word_t * Y2,
                     uECC_Curve curve) {
    /* t1 = X1, t2 = Y1, t3 = X2, t4 = Y2 */
    uECC_word_t t5[uECC_MAX_WORDS];
    wordcount_t num_words = curve->num_words;

    uECC_vli_modSub(t5, X2, X1, curve->p, num_words); /* t5 = x2 - x1 */
    uECC_vli_modSquare_fast(t5, t5, curve);                  /* t5 = (x2 - x1)^2 = A */
    uECC_vli_modMult_fast(X1, X1, t5, curve);                /* t1 = x1*A = B */
    uECC_vli_modMult_fast(X2, X2, t5, curve);                /* t3 = x2*A = C */
    uECC_vli_modSub(Y2, Y2, Y1, curve->p, num_words); /* t4 = y2 - y1 */
    uECC_vli_modSquare_fast(t5, Y2, curve);                  /* t5 = (y2 - y1)^2 = D */

    uECC_vli_modSub(t5, t5, X1, curve->p, num_words); /* t5 = D - B */
    uECC_vli_modSub(t5, t5, X2, curve->p, num_words); /* t5 = D - B - C = x3 */
    uECC_vli_modSub(X2, X2, X1, curve->p, num_words); /* t3 = C - B */
    uECC_vli_modMult_fast(Y1, Y1, X2, curve);                /* t2 = y1*(C - B) */
    uECC_vli_modSub(X2, X1, t5, curve->p, num_words); /* t3 = B - x3 */
    uECC_vli_modMult_fast(Y2, Y2, X2, curve);                /* t4 = (y2 - y1)*(B - x3) */
    uECC_vli_modSub(Y2, Y2, Y1, curve->p, num_words); /* t4 = y3 */

    uECC_vli_set(X2, t5, num_words);
}
/* Point multiplication algorithm using Montgomery's ladder with co-Z coordinates.
From http://eprint.iacr.org/2011/338.pdf
*/

/* Modify (x1, y1) => (x1 * z^2, y1 * z^3) */
static void apply_z(uECC_word_t * X1,
                    uECC_word_t * Y1,
                    const uECC_word_t * const Z,
                    uECC_Curve curve) {
    uECC_word_t t1[uECC_MAX_WORDS];

    uECC_vli_modSquare_fast(t1, Z, curve);    /* z^2 */
    uECC_vli_modMult_fast(X1, X1, t1, curve); /* x1 * z^2 */
    uECC_vli_modMult_fast(t1, t1, Z, curve);  /* z^3 */
    uECC_vli_modMult_fast(Y1, Y1, t1, curve); /* y1 * z^3 */
}
                                        
static bitcount_t smax(bitcount_t a, bitcount_t b) {
    return (a > b ? a : b);
}
/* Constant-time comparison function - secure way to compare long integers */
/* Returns one if left == right, zero otherwise. */
uECC_VLI_API uECC_word_t uECC_vli_equal(const uECC_word_t *left,
                                        const uECC_word_t *right,
                                        wordcount_t num_words) {
    uECC_word_t diff = 0;
    wordcount_t i;
    for (i = num_words - 1; i >= 0; --i) {
        diff |= (left[i] ^ right[i]);
    }
    return (diff == 0);
}
                                                                                


int uECC_verify(const uint8_t *public_key,
                const uint8_t *message_hash,
                unsigned hash_size,
                const uint8_t *signature,
                uECC_Curve curve) {
    uECC_word_t u1[uECC_MAX_WORDS], u2[uECC_MAX_WORDS];
    uECC_word_t z[uECC_MAX_WORDS];
    uECC_word_t sum[uECC_MAX_WORDS * 2];
    uECC_word_t rx[uECC_MAX_WORDS];
    uECC_word_t ry[uECC_MAX_WORDS];
    uECC_word_t tx[uECC_MAX_WORDS];
    uECC_word_t ty[uECC_MAX_WORDS];
    uECC_word_t tz[uECC_MAX_WORDS];
    const uECC_word_t *points[4];
    const uECC_word_t *point;
    bitcount_t num_bits;
    bitcount_t i;
    uECC_word_t *_public = (uECC_word_t *)public_key;
    uECC_word_t r[uECC_MAX_WORDS], s[uECC_MAX_WORDS];
    wordcount_t num_words = curve->num_words;
    wordcount_t num_n_words = BITS_TO_WORDS(curve->num_n_bits);

    rx[num_n_words - 1] = 0;
    r[num_n_words - 1] = 0;
    s[num_n_words - 1] = 0;

    bcopy((uint8_t *) r, signature, curve->num_bytes);
    bcopy((uint8_t *) s, signature + curve->num_bytes, curve->num_bytes);


    /* r, s must not be 0. */
    if (uECC_vli_isZero(r, num_words) || uECC_vli_isZero(s, num_words)) {
        return 0;
    }

    /* r, s must be < n. */
    if (uECC_vli_cmp_unsafe(curve->n, r, num_n_words) != 1 ||
            uECC_vli_cmp_unsafe(curve->n, s, num_n_words) != 1) {
        return 0;
    }

    /* Calculate u1 and u2. */
    uECC_vli_modInv(z, s, curve->n, num_n_words); /* z = 1/s */
    u1[num_n_words - 1] = 0;
    bits2int(u1, message_hash, hash_size, curve);
    uECC_vli_modMult(u1, u1, z, curve->n, num_n_words); /* u1 = e/s */
    uECC_vli_modMult(u2, r, z, curve->n, num_n_words); /* u2 = r/s */

    /* Calculate sum = G + Q. */
    uECC_vli_set(sum, _public, num_words);
    uECC_vli_set(sum + num_words, _public + num_words, num_words);
    uECC_vli_set(tx, curve->G, num_words);
    uECC_vli_set(ty, curve->G + num_words, num_words);
    uECC_vli_modSub(z, sum, tx, curve->p, num_words); /* z = x2 - x1 */
    XYcZ_add(tx, ty, sum, sum + num_words, curve);
    uECC_vli_modInv(z, z, curve->p, num_words); /* z = 1/z */
    apply_z(sum, sum + num_words, z, curve);

    /* Use Shamir's trick to calculate u1*G + u2*Q */
    points[0] = 0;
    points[1] = curve->G;
    points[2] = _public;
    points[3] = sum;
    num_bits = smax(uECC_vli_numBits(u1, num_n_words),
                    uECC_vli_numBits(u2, num_n_words));

    point = points[(!!uECC_vli_testBit(u1, num_bits - 1)) |
                   ((!!uECC_vli_testBit(u2, num_bits - 1)) << 1)];
    uECC_vli_set(rx, point, num_words);
    uECC_vli_set(ry, point + num_words, num_words);
    uECC_vli_clear(z, num_words);
    z[0] = 1;

    for (i = num_bits - 2; i >= 0; --i) {
        uECC_word_t index;
        curve->double_jacobian(rx, ry, z, curve);

        index = (!!uECC_vli_testBit(u1, i)) | ((!!uECC_vli_testBit(u2, i)) << 1);
        point = points[index];
        if (point) {
            uECC_vli_set(tx, point, num_words);
            uECC_vli_set(ty, point + num_words, num_words);
            apply_z(tx, ty, z, curve);
            uECC_vli_modSub(tz, rx, tx, curve->p, num_words); /* Z = x2 - x1 */
            XYcZ_add(tx, ty, rx, ry, curve);
            uECC_vli_modMult_fast(z, z, tz, curve);
        }
    }

    uECC_vli_modInv(z, z, curve->p, num_words); /* Z = 1/Z */
    apply_z(rx, ry, z, curve);

    /* v = x1 (mod n) */
    if (uECC_vli_cmp_unsafe(curve->n, rx, num_n_words) != 1) {
        uECC_vli_sub(rx, rx, curve->n, num_n_words);
    }

    /* Accept only if v == r. */
    return (int)(uECC_vli_equal(rx, r, num_words));
}



