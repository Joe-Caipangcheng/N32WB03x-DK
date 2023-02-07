#ifndef CCM_SOFT_H__
#define CCM_SOFT_H__

#include <stdbool.h>
#include <stdint.h>


#define CCM_LENGTH_FIELD_LENGTH (2)
#define CCM_KEY_LENGTH (16)
#define CCM_NONCE_LENGTH (13)
#define CCM_MIC_LENGTH_MAX  (16)

typedef struct
{
    const uint8_t * p_key;                  
    const uint8_t * p_nonce;                
    const uint8_t * p_m;                    
                                            
    uint16_t  m_len;                        
    const uint8_t * p_a;                    
    uint16_t a_len;                         
    uint8_t * p_out;                        
    uint8_t * p_mic;                        
    uint8_t   mic_len;                      
} ccm_soft_data_t;
void ccm_soft_encrypt(ccm_soft_data_t * p_data);
void ccm_soft_decrypt(ccm_soft_data_t * p_data, bool * p_mic_passed);


#endif
