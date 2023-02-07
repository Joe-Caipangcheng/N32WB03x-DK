#ifndef MESH_KEYGEN__
#define MESH_KEYGEN__

#include <stdint.h>
#include "mesh.h"


typedef struct
{
    uint16_t lpn_address;
    uint16_t friend_address;
    uint16_t lpn_counter;
    uint16_t friend_counter;
} mesh_keygen_friendship_secmat_params_t;

uint32_t mesh_keygen_aid(const uint8_t * p_appkey, uint8_t * p_aid);
uint32_t mesh_keygen_network_secmat(const uint8_t * p_netkey, mesh_network_secmat_t * p_secmat);
uint32_t mesh_keygen_friendship_secmat(const uint8_t * p_netkey, const mesh_keygen_friendship_secmat_params_t * p_params,mesh_network_secmat_t * p_secmat);
uint32_t mesh_keygen_beacon_secmat(const uint8_t * p_netkey, mesh_beacon_secmat_t * p_secmat);
uint32_t mesh_keygen_identitykey(const uint8_t * p_netkey, uint8_t * p_key);
uint32_t mesh_keygen_virtual_address(const uint8_t * p_virtual_uuid, uint16_t * p_address);


#endif  /* MESH_KEYGEN__ */
