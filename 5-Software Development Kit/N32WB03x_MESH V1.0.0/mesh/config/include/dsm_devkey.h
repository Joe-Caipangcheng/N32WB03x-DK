#ifndef __DSM_DEVKEY_H__
#define __DSM_DEVKEY_H__




#include "dsm_common.h"




extern devkey_t dsm_devkeys[DSM_DEVICE_MAX];
extern uint32_t dsm_devkey_allocated[BITFIELD_BLOCK_COUNT(DSM_DEVICE_MAX)];








void devkey_set(uint16_t key_owner, dsm_handle_t subnet_handle, const uint8_t * p_key, dsm_handle_t handle);
const mesh_application_secmat_t * get_devkey_secmat(uint16_t key_address);
bool dev_key_handle_get(uint16_t owner_addr, dsm_handle_t * p_handle);
uint32_t dsm_devkey_add(uint16_t raw_unicast_addr, dsm_handle_t subnet_handle, const uint8_t * p_key, dsm_handle_t * p_devkey_handle);





static inline const mesh_application_secmat_t * get_devkey_secmat(uint16_t key_address)
{
    if (key_address == MESH_ADDR_UNASSIGNED)
    {
        return NULL;
    }
    for (uint32_t i = 0; i < DSM_DEVICE_MAX; ++i)
    {
        if (dsm_devkeys[i].key_owner == key_address)
        {
            return &dsm_devkeys[i].secmat;
        }
    }
    return NULL;
}



static inline void mesh_devkey_secmat_get(uint16_t owner_addr, const mesh_application_secmat_t ** pp_app_secmat)
{
    *pp_app_secmat = get_devkey_secmat(owner_addr);
}





#endif //__DSM_DEVKEY_H__
