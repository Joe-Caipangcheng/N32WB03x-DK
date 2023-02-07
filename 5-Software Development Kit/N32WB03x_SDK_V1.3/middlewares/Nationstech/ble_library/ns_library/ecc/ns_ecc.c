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
 * @file ns_ecc.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "ns_ecc.h"
#include <stdint.h>
#include <stdio.h>
#include "uECC.h"
#include "sha256.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/**
 * @brief Calculate hash degest.
 * @param[in] p_data raw data to process.
 * @param[in] data_len raw data length.
 * @param[out] p_hash hash degest. 
 * @return error code
 */
uint32_t ns_lib_ecc_hash_sha256(uint8_t *p_data, uint32_t data_len, uint8_t *p_hash)
{
    sha256_context_t   hash_context;
    sha256_init(&hash_context);
    if(!sha256_update(&hash_context, p_data, data_len)){
        sha256_final(&hash_context, p_hash, 0);
        return 0;
    }
    return 1;
}


/**
 * @brief big to little endian swap function.
 * @param[out] p_out output little endian data.
 * @param[in] p_in big endian raw data.
 * @param[in] size data length. 
 * @return none
 */
static void swap_endian(uint8_t * p_out, uint8_t const * p_in, size_t size)
{
    uint8_t const * p_first = p_in;
    uint8_t * p_last = p_out + size - 1;
    while (p_last >= p_out)
    {
        *p_last = *p_first;
        p_first++;
        p_last--;
    }
}

/**
 * @brief big to little endian swap function with two sets of data.
 * @param[out] p_out output little endian data.
 * @param[in] p_in big endian raw data.
 * @param[in] size half data length. 
 * @return none
 */
static void double_swap_endian(uint8_t * p_out, uint8_t const * p_in, size_t part_size)
{
    swap_endian(p_out, p_in, part_size);
    swap_endian(&p_out[part_size], &p_in[part_size], part_size);
}



/**
 * @brief ECDSA SHA256 digital signature verification function.
 * @param[in] p_public_key public key data.
 * @param[in] p_hash sha256 hash degest data.
 * @param[in] hash_size hash degest length. 
 * @param[in] p_signature digital signature. 
 * @return error code
 */
uint32_t ns_lib_ecc_ecdsa_verify(uint8_t *p_public_key, uint8_t *p_hash, uint32_t hash_size, uint8_t *p_signature)
{
    uint32_t error = 0;    
    uint8_t hash_le     [32];
    uint8_t signature_le[64];
    uint8_t public_key_le[64];    
    swap_endian(hash_le, p_hash, sizeof(hash_le));
    double_swap_endian(signature_le,p_signature,32);    
    double_swap_endian(public_key_le,p_public_key,32);
    int result = uECC_verify(public_key_le, hash_le, hash_size, signature_le, &curve_secp256r1);
    if(result != 1){
        error = 1;
    }
    return error;
}

















