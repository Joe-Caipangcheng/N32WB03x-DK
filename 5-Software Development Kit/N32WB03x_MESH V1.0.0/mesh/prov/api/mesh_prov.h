#ifndef MESH_PROV_H__
#define MESH_PROV_H__


#include <stdbool.h>
#include <stdint.h>


#include "mesh_prov_types.h"
#include "mesh_prov_events.h"
#include "mesh_list.h"


struct mesh_prov_ctx
{
    list_node_t * p_bearers;    
    uint32_t supported_bearers; 
    prov_bearer_t * p_active_bearer; 
    mesh_prov_evt_handler_cb_t event_handler; 

    const uint8_t * p_public_key;  
    const uint8_t * p_private_key;

    uint8_t peer_public_key[MESH_PROV_PUBKEY_SIZE];  
    uint8_t shared_secret[MESH_PROV_ECDHSECRET_SIZE]; 

    uint8_t device_key[MESH_KEY_SIZE]; 
    uint8_t session_key[MESH_KEY_SIZE]; 
    uint8_t data_nonce[PROV_NONCE_LEN];  

    uint8_t node_random[PROV_RANDOM_LEN]; 
    uint8_t peer_random[PROV_RANDOM_LEN];
    uint8_t auth_value[PROV_AUTH_LEN];   

    uint8_t confirmation_salt[PROV_SALT_LEN];   
    uint8_t peer_confirmation[PROV_CONFIRMATION_LEN];   
    uint8_t confirmation_inputs[PROV_CONFIRMATION_INPUT_LEN]; 

    uint8_t oob_size;  
    uint8_t oob_action;
    bool pubkey_oob;   

    mesh_prov_role_t role;                 
    mesh_prov_failure_code_t failure_code; 
    mesh_prov_state_t state;               
    mesh_prov_oob_method_t oob_method;     
    mesh_prov_oob_caps_t capabilities;     
    mesh_prov_provisioning_data_t data;    

    uint8_t attention_duration_s;  
};




uint32_t mesh_prov_init(mesh_prov_ctx_t *            p_ctx,
                            const uint8_t *                  p_public_key,
                            const uint8_t *                  p_private_key,
                            const mesh_prov_oob_caps_t * p_caps,
                            mesh_prov_evt_handler_cb_t   event_handler);

uint32_t mesh_prov_bearer_add(mesh_prov_ctx_t * p_ctx, prov_bearer_t * p_prov_bearer);
uint32_t mesh_prov_generate_keys(uint8_t * p_public, uint8_t * p_private);                
              


uint32_t mesh_prov_listen(mesh_prov_ctx_t *       p_ctx,
                              const char *                URI,
                              uint16_t                    oob_info_sources,
                              uint32_t                    bearer_types);

uint32_t mesh_prov_listen_stop(mesh_prov_ctx_t * p_ctx);


                    
#endif
