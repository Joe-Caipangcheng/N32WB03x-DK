#ifndef __DSM_NONVIRTUAL_ADDR_H__
#define __DSM_NONVIRTUAL_ADDR_H__




#include "dsm_common.h"



extern uint32_t dsm_addr_nonvirtual_allocated[BITFIELD_BLOCK_COUNT(DSM_NONVIRTUAL_ADDR_MAX)];
extern regular_address_t dsm_addresses[DSM_NONVIRTUAL_ADDR_MAX];









void nonvirtual_address_set(uint16_t raw_address, dsm_handle_t handle);
bool address_handle_nonvirtual_valid(dsm_handle_t address_handle);
bool non_virtual_address_handle_get(uint16_t address, dsm_handle_t * p_handle);
bool address_nonvirtual_subscription_exists(uint16_t address, dsm_handle_t * p_handle);







static inline bool address_nonvirtual_subscription_exists(uint16_t address, dsm_handle_t * p_handle)
{
    *p_handle = DSM_HANDLE_INVALID;
    for (uint32_t i = 0; i < DSM_NONVIRTUAL_ADDR_MAX; ++i)
    {
        if (*p_handle == DSM_HANDLE_INVALID && !bitfield_get(dsm_addr_nonvirtual_allocated, i))
        {
            *p_handle = i;
        }
        else if (dsm_addresses[i].address == address && dsm_addresses[i].subscription_count > 0)
        {
            *p_handle = i;
            return true;
        }
    }
    return 0;
}





#endif //__DSM_NONVIRTUAL_ADDR_H__
