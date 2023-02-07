#ifndef __DSM_SUBNET_H__
#define __DSM_SUBNET_H__




#include "dsm_common.h"



extern subnet_t dsm_subnets[DSM_SUBNET_MAX];
extern uint32_t dsm_subnet_allocated[BITFIELD_BLOCK_COUNT(DSM_SUBNET_MAX)];




void subnet_set(mesh_key_index_t net_key_index, const uint8_t * p_key, dsm_handle_t handle);
bool get_all_subnets(mesh_key_index_t * p_key_list, uint32_t * p_count);
dsm_handle_t get_subnet_handle_by_beacon_info(const mesh_beacon_info_t * p_beacon_info);
bool net_key_handle_get(mesh_key_index_t net_key_index, dsm_handle_t * p_handle);
dsm_handle_t get_subnet_handle(const mesh_network_secmat_t * p_secmat);
uint32_t dsm_subnet_add(mesh_key_index_t net_key_index, const uint8_t * p_key, dsm_handle_t * p_subnet_handle);
uint32_t dsm_subnet_get_all(mesh_key_index_t * p_key_list, uint32_t * p_count);
dsm_handle_t dsm_net_key_index_to_subnet_handle(mesh_key_index_t net_key_index);
uint32_t dsm_subnet_kr_phase_get(dsm_handle_t subnet_handle, mesh_key_refresh_phase_t * p_phase);









static inline dsm_handle_t get_subnet_handle(const mesh_network_secmat_t * p_secmat)
{
    dsm_handle_t retval = DSM_HANDLE_INVALID;
    for(uint16_t i = 0; i < DSM_SUBNET_MAX; i++)
    {
        if (dsm_subnets[i].key_refresh_phase != MESH_KEY_REFRESH_PHASE_0)
        {
            if (p_secmat == &dsm_subnets[i].secmat || p_secmat == &dsm_subnets[i].secmat_updated)
            {
                retval = i;
                break;
            }
        }
        else if (p_secmat == &dsm_subnets[i].secmat)
        {
            retval = i;
            break;
        }
    }

    return retval;
}

static inline dsm_handle_t dsm_subnet_handle_get(const mesh_network_secmat_t * p_secmat)
{
    if (p_secmat == NULL)
    {
        return DSM_HANDLE_INVALID;
    }
    return get_subnet_handle(p_secmat);
}



#endif //__DSM_SUBNET_H__
