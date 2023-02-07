#ifndef AES_CMAC_H__
#define AES_CMAC_H__

#include <stdint.h>


void mesh_aes_cmac(const uint8_t * const p_key, const uint8_t * const p_msg, uint16_t msg_len, uint8_t * const p_out);

#endif
