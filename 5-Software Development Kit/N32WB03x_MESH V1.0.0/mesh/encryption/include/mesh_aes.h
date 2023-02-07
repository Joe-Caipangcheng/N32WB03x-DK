#ifndef MESH_AES_H__
#define MESH_AES_H__


#include <stdint.h>


#define SOC_ECB_KEY_LENGTH                (16)                       
#define SOC_ECB_CLEARTEXT_LENGTH          (16)                       
#define SOC_ECB_CIPHERTEXT_LENGTH         (SOC_ECB_CLEARTEXT_LENGTH) 
typedef uint8_t soc_ecb_key_t[SOC_ECB_KEY_LENGTH];                
typedef uint8_t soc_ecb_cleartext_t[SOC_ECB_CLEARTEXT_LENGTH];    
typedef uint8_t soc_ecb_ciphertext_t[SOC_ECB_CIPHERTEXT_LENGTH];  
typedef struct
{
  soc_ecb_key_t        key;            
  soc_ecb_cleartext_t  cleartext;      
  soc_ecb_ciphertext_t ciphertext;     
} mesh_ecb_hal_data_t;

typedef mesh_ecb_hal_data_t aes_data_t;




void mesh_aes_encrypt(aes_data_t * p_aes_data);
void mesh_aes_init(void);




#endif
