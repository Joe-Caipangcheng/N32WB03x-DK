#ifndef MESH_PROV_TYPES_H_
#define MESH_PROV_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

#include "mesh_defines.h"


#define MESH_PROV_PUBKEY_SIZE      64
#define MESH_PROV_PRIVKEY_SIZE     32
#define MESH_PROV_ECDHSECRET_SIZE  32
#define MESH_PROV_DATANONCE_SIZE   13

#define MESH_PROV_OOB_SIZE_MAX      8

#define PROV_RANDOM_LEN               (16)
#define PROV_CONFIRMATION_LEN         (MESH_KEY_SIZE)
#define PROV_AUTH_LEN                 (MESH_KEY_SIZE)
#define PROV_SALT_LEN                 (MESH_KEY_SIZE)
#define PROV_NONCE_LEN                (13)
#define PROV_CONFIRMATION_INPUT_LEN   (17)

#define MESH_PROV_ALGORITHM_FIPS_P256EC          (1u << 0)

#define MESH_PROV_OOB_PUBKEY_TYPE_INBAND         (0)
#define MESH_PROV_OOB_PUBKEY_TYPE_OOB            (1u << 0)

#define MESH_PROV_OOB_STATIC_TYPE_SUPPORTED      (1u << 0)

#define MESH_PROV_OOB_OUTPUT_ACTION_BLINK        (1u << 0)
#define MESH_PROV_OOB_OUTPUT_ACTION_BEEP         (1u << 1)
#define MESH_PROV_OOB_OUTPUT_ACTION_VIBRATE      (1u << 2)
#define MESH_PROV_OOB_OUTPUT_ACTION_NUMERIC      (1u << 3)
#define MESH_PROV_OOB_OUTPUT_ACTION_ALPHANUMERIC (1u << 4)

#define MESH_PROV_OOB_INPUT_ACTION_PUSH          (1u << 0)
#define MESH_PROV_OOB_INPUT_ACTION_TWIST         (1u << 1)
#define MESH_PROV_OOB_INPUT_ACTION_ENTER_NUMBER  (1u << 2)
#define MESH_PROV_OOB_INPUT_ACTION_ENTER_STRING  (1u << 3)

#define MESH_PROV_OOB_INFO_SOURCE_OTHER                    (1u << 0)   
#define MESH_PROV_OOB_INFO_SOURCE_ELECTRONIC_OR_URI        (1u << 1)   
#define MESH_PROV_OOB_INFO_SOURCE_2D_MACHINE_READABLE_CODE (1u << 2)   
#define MESH_PROV_OOB_INFO_SOURCE_BAR_CODE                 (1u << 3)   
#define MESH_PROV_OOB_INFO_SOURCE_NFC                      (1u << 4)   
#define MESH_PROV_OOB_INFO_SOURCE_NUMBER                   (1u << 5)   
#define MESH_PROV_OOB_INFO_SOURCE_STRING                   (1u << 6)   
#define MESH_PROV_OOB_INFO_SOURCE_ON_BOX                   (1u << 11)  
#define MESH_PROV_OOB_INFO_SOURCE_INSIDE_BOX               (1u << 12)  
#define MESH_PROV_OOB_INFO_SOURCE_ON_PIECE_OF_PAPER        (1u << 13)  
#define MESH_PROV_OOB_INFO_SOURCE_INSIDE_MANUAL            (1u << 14)  
#define MESH_PROV_OOB_INFO_SOURCE_ON_DEVICE                (1u << 15)  

typedef enum
{
    MESH_PROV_ALGORITHM_FIPS_P256,   
    MESH_PROV_ALGORITHM_RFU          
} mesh_prov_algorithm_t;

typedef enum
{
    MESH_PROV_PUBLIC_KEY_NO_OOB = 0x00, 
    MESH_PROV_PUBLIC_KEY_OOB    = 0x01, 
    MESH_PROV_PUBLIC_KEY_PROHIBITED     
} mesh_prov_public_key_usage_t;

 typedef enum
 {
     MESH_PROV_OOB_METHOD_NONE   = 0x00, 
     MESH_PROV_OOB_METHOD_STATIC = 0x01, 
     MESH_PROV_OOB_METHOD_OUTPUT = 0x02, 
     MESH_PROV_OOB_METHOD_INPUT  = 0x03, 
     MESH_PROV_OOB_METHOD_PROHIBITED     
 } mesh_prov_oob_method_t;


typedef enum
{
 MESH_PROV_INPUT_ACTION_PUSH         = 0x00, 
 MESH_PROV_INPUT_ACTION_TWIST        = 0x01, 
 MESH_PROV_INPUT_ACTION_ENTER_NUMBER = 0x02, 
 MESH_PROV_INPUT_ACTION_ENTER_STRING = 0x03, 
 MESH_PROV_INPUT_ACTION_RFU                  
} mesh_prov_input_action_t;

typedef enum
{
 MESH_PROV_OUTPUT_ACTION_BLINK           = 0x00, 
 MESH_PROV_OUTPUT_ACTION_BEEP            = 0x01, 
 MESH_PROV_OUTPUT_ACTION_VIBRATE         = 0x02, 
 MESH_PROV_OUTPUT_ACTION_DISPLAY_NUMERIC = 0x03, 
 MESH_PROV_OUTPUT_ACTION_ALPHANUMERIC    = 0x04, 
 MESH_PROV_OUTPUT_ACTION_RFU                     
} mesh_prov_output_action_t;


typedef enum
{
 MESH_PROV_FAILURE_CODE_INVALID_PDU         = 0x01, 
 MESH_PROV_FAILURE_CODE_INVALID_FORMAT      = 0x02, 
 MESH_PROV_FAILURE_CODE_UNEXPECTED_PDU      = 0x03, 
 MESH_PROV_FAILURE_CODE_CONFIRMATION_FAILED = 0x04, 
 MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES    = 0x05, 
 MESH_PROV_FAILURE_CODE_DECRYPTION_FAILED   = 0x06, 
 MESH_PROV_FAILURE_CODE_UNEXPECTED_ERROR    = 0x07, 
 MESH_PROV_FAILURE_CODE_CANNOT_ASSIGN_ADDR  = 0x08, 
} mesh_prov_failure_code_t;


typedef struct __attribute((packed))
{
    uint8_t  num_elements;       
    uint16_t algorithms;         
    uint8_t  pubkey_type;        
    uint8_t  oob_static_types;   
    uint8_t  oob_output_size;    
    uint16_t oob_output_actions; 
    uint8_t  oob_input_size;     
    uint16_t oob_input_actions;  
} mesh_prov_oob_caps_t;

typedef enum
{
    MESH_PROV_STATE_IDLE,                   
    MESH_PROV_STATE_WAIT_LINK,              
    MESH_PROV_STATE_INVITE,                 
    MESH_PROV_STATE_WAIT_CAPS,              
    MESH_PROV_STATE_WAIT_CAPS_CONFIRM,      
    MESH_PROV_STATE_WAIT_START,             
    MESH_PROV_STATE_WAIT_START_ACK,         
    MESH_PROV_STATE_WAIT_PUB_KEY_ACK,       
    MESH_PROV_STATE_WAIT_PUB_KEY,           
    MESH_PROV_STATE_WAIT_OOB_PUB_KEY,       
    MESH_PROV_STATE_WAIT_EXTERNAL_ECDH,     
    MESH_PROV_STATE_WAIT_OOB_INPUT,         
    MESH_PROV_STATE_WAIT_OOB_STATIC,        
    MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD, 
    MESH_PROV_STATE_WAIT_CONFIRMATION_ACK,  
    MESH_PROV_STATE_WAIT_CONFIRMATION,      
    MESH_PROV_STATE_WAIT_INPUT_COMPLETE,    
    MESH_PROV_STATE_WAIT_RANDOM,            
    MESH_PROV_STATE_WAIT_DATA,              
    MESH_PROV_STATE_WAIT_COMPLETE,          
    MESH_PROV_STATE_COMPLETE,               
    MESH_PROV_STATE_FAILED                  
} mesh_prov_state_t;


typedef struct __attribute((packed))
{
    uint8_t  netkey[MESH_KEY_SIZE];
    uint16_t netkey_index;
    uint32_t iv_index;
    uint16_t address;
    struct __attribute((packed)) {
        uint8_t iv_update   : 1;
        uint8_t key_refresh : 1;
    } flags;
} mesh_prov_provisioning_data_t;

typedef enum
{
    MESH_PROV_ROLE_PROVISIONER, 
    MESH_PROV_ROLE_PROVISIONEE  
} mesh_prov_role_t;

typedef struct mesh_prov_ctx mesh_prov_ctx_t;


#endif /* MESH_PROV_TYPES_H__ */
