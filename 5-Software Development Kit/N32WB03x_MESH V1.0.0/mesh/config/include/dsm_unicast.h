#ifndef __DSM_UNICAST_H__
#define __DSM_UNICAST_H__


#include "dsm_common.h"





extern dsm_local_unicast_address_t dsm_local_unicast_addr;
extern uint32_t dsm_addr_unicast_allocated[BITFIELD_BLOCK_COUNT(1)];






bool rx_unicast_address_get(uint16_t address, mesh_address_t * p_address);
uint32_t dsm_local_unicast_addresses_set(const dsm_local_unicast_address_t * p_address);



static inline bool is_own_unicast_addr(uint16_t addr)
{
    return (bitfield_get(dsm_addr_unicast_allocated, 0) &&
            addr >= dsm_local_unicast_addr.address_start &&
            addr < dsm_local_unicast_addr.address_start + dsm_local_unicast_addr.count);
}


static inline bool rx_unicast_address_get(uint16_t address, mesh_address_t * p_address)
{
    if (is_own_unicast_addr(address))
    {
        p_address->value = address;
        p_address->type = MESH_ADDRESS_TYPE_UNICAST;
        p_address->p_virtual_uuid = NULL;
        return true;
    }
    return false;
}







#endif //__DSM_UNICAST_H__
