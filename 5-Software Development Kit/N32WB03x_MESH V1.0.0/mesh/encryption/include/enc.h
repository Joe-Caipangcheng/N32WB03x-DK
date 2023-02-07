#ifndef MESH_ENC_H__
#define MESH_ENC_H__

#include <stdbool.h>
#include <stdint.h>

#include "ccm_soft.h"
#include "mesh.h"
#include "net_packet.h"


#define ENC_K2_P_VALUE_MINLEN   1
#define ENC_K2_P_VALUE_MAXLEN   16

typedef enum
{
    ENC_NONCE_NET   = 0x00,              
    ENC_NONCE_APP   = 0x01,              
    ENC_NONCE_DEV   = 0x02,              
    ENC_NONCE_PROXY = 0x03               
} enc_nonce_t;
typedef struct __attribute((packed))
{
    uint8_t  type;              
    uint32_t ttl   :  7;        
    uint32_t ctl   :  1;        
    uint32_t seq   : 24;        
    uint16_t src;               
    uint16_t padding;           
    uint32_t iv_index;          
} enc_nonce_net_t;
typedef struct __attribute((packed))
{
    uint8_t  type;              
    uint32_t padding : 7;       
    uint32_t aszmic  : 1;       
    uint32_t seq     : 24;      
    uint16_t src;               
    uint16_t dst;               
    uint32_t iv_index;          
} enc_nonce_app_t;
typedef enc_nonce_app_t enc_nonce_dev_t;
typedef struct __attribute((packed))
{
    uint8_t type;      
    uint8_t pad;       
    uint32_t seq : 24; 
    uint16_t src;      
    uint16_t pad2;     
    uint32_t iv_index; 
} enc_nonce_proxy_t;

void enc_key_generate(uint8_t * p_key);
void enc_aes_encrypt(const uint8_t * p_key, const uint8_t * p_plaintext, uint8_t * p_result);
void enc_aes_cmac(const uint8_t * p_key, const uint8_t * p_data, uint16_t data_len, uint8_t * p_result);
void enc_aes_ccm_encrypt(ccm_soft_data_t * const p_ccm_data);
void enc_aes_ccm_decrypt(ccm_soft_data_t * const p_ccm_data, bool * const p_mic_passed);
void enc_nonce_generate(const network_packet_metadata_t * p_net_metadata,enc_nonce_t type,uint8_t aszmic,uint8_t * p_nonce);
void enc_s1(const uint8_t * p_in, uint16_t in_length, uint8_t * p_out);
void enc_k1(const uint8_t * p_ikm, const uint8_t ikm_length, const uint8_t * p_salt, const uint8_t * p_info, const uint8_t info_length, uint8_t * const p_out);
void enc_k2(const uint8_t * p_netkey, const uint8_t * p_p, uint16_t length_p, mesh_network_secmat_t * p_output);
void enc_k3(const uint8_t * p_in, uint8_t * p_out);
void enc_k4(const uint8_t * p_in, uint8_t * p_out);


#endif

