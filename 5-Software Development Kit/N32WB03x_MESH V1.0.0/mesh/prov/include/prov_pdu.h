#ifndef PROV_PDU_H__
#define PROV_PDU_H__

#include <stdint.h>
#include "mesh_defines.h"
#include "mesh_prov_types.h"


#define PROV_PDU_MAX_LENGTH (sizeof(prov_pdu_pubkey_t))


#define PROV_PDU_DATA_MIC_LENGTH    (8)

typedef enum
{
    PROV_PDU_TYPE_INVITE,
    PROV_PDU_TYPE_CAPABILITIES,
    PROV_PDU_TYPE_START,
    PROV_PDU_TYPE_PUBLIC_KEY,
    PROV_PDU_TYPE_INPUT_COMPLETE,
    PROV_PDU_TYPE_CONFIRMATION,
    PROV_PDU_TYPE_RANDOM,
    PROV_PDU_TYPE_DATA,
    PROV_PDU_TYPE_COMPLETE,
    PROV_PDU_TYPE_FAILED,
    PROV_PDU_TYPE_INVALID,
    PROV_PDU_TYPE_COUNT = PROV_PDU_TYPE_INVALID
} prov_pdu_type_t;

#define PROV_PDU_START_ALGO_FIPS_P256   0x00
typedef struct __attribute((packed))
{
    uint8_t pdu_type;               
    uint8_t attention_duration_s;   
} prov_pdu_invite_t;

typedef struct __attribute((packed))
{
    uint8_t  pdu_type;           
    uint8_t  num_elements;       
    uint16_t algorithms;         
    uint8_t  pubkey_type;        
    uint8_t  oob_static_types;   
    uint8_t  oob_output_size;    
    uint16_t oob_output_actions; 
    uint8_t  oob_input_size;     
    uint16_t oob_input_actions;  
} prov_pdu_caps_t;

typedef struct __attribute((packed))
{
    uint8_t pdu_type;           
    uint8_t algorithm;          
    uint8_t public_key;         
    uint8_t auth_method;        
    uint8_t auth_action;        
    uint8_t auth_size;          
} prov_pdu_prov_start_t;

typedef struct __attribute((packed))
{
    uint8_t pdu_type;       
    uint8_t public_key[MESH_ECDH_PUBLIC_KEY_SIZE]; 
} prov_pdu_pubkey_t;

typedef struct __attribute((packed))
{
    uint8_t pdu_type;           
    uint8_t confirmation[16];   
} prov_pdu_confirm_t;

typedef struct __attribute((packed))
{
    uint8_t pdu_type;           
} prov_pdu_input_complete_t;

typedef struct __attribute((packed))
{
    uint8_t  netkey[MESH_KEY_SIZE]; 
    uint16_t netkey_index;             
    struct __attribute((packed))
    {
        uint8_t key_refresh : 1; 
        uint8_t iv_update   : 1; 
        uint8_t _rfu        : 6; 
    } flags;                     
    uint32_t iv_index;           
    uint16_t address;            
} prov_pdu_data_block_t;

typedef struct __attribute((packed))
{
    uint8_t  pdu_type;          
    prov_pdu_data_block_t data; 
    uint8_t  mic[PROV_PDU_DATA_MIC_LENGTH]; 
} prov_pdu_data_t;

typedef struct __attribute((packed))
{
    uint8_t pdu_type;   
    uint8_t random[16]; 
} prov_pdu_random_t;


typedef struct __attribute((packed))
{
    uint8_t pdu_type;   
} prov_pdu_complete_t;

typedef struct __attribute((packed))
{
    uint8_t pdu_type;     
    uint8_t failure_code; 
} prov_pdu_failed_t;


#endif

