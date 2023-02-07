#ifndef MESH_UTILS_H__
#define MESH_UTILS_H__

#include <stdint.h>
#include <stddef.h>
#include "mesh_defines.h"
#include "mesh.h"


#define BLE_GAP_ADDR_TYPE_RANDOM_INVALID 0x04 
uint32_t mesh_rand_get(uint8_t * p_output, uint8_t size);
uint8_t mesh_gap_address_type_get(const uint8_t * p_address, uint8_t txadd_bit);
static const mesh_beacon_secmat_t * mesh_beacon_secmat_from_info(const mesh_beacon_info_t * p_beacon_info, mesh_key_refresh_phase_t kr_phase)
{
    if (p_beacon_info)
    {
        return ((kr_phase == MESH_KEY_REFRESH_PHASE_0) ? &p_beacon_info->secmat : &p_beacon_info->secmat_updated);
    }
    else
    {
        return NULL;
    }
}
static inline mesh_address_type_t mesh_address_type_get(uint16_t address)
{
    if (address == MESH_ADDR_UNASSIGNED)
    {
        return MESH_ADDRESS_TYPE_INVALID;
    }
    else
    {
        static const mesh_address_type_t types_lookup[] =
        {
            MESH_ADDRESS_TYPE_UNICAST, 
            MESH_ADDRESS_TYPE_UNICAST, 
            MESH_ADDRESS_TYPE_VIRTUAL, 
            MESH_ADDRESS_TYPE_GROUP,   
        };
        return types_lookup[(address & MESH_ADDR_TYPE_BITS_MASK) >> MESH_ADDR_TYPE_BITS_OFFSET];
    }
}

#endif /* MESH_UTILS_H__*/
