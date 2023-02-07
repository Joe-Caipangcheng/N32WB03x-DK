#ifndef __DSM_VIRTUAL_ADDR_H__
#define __DSM_VIRTUAL_ADDR_H__




#include "dsm_common.h"



extern virtual_address_t dsm_virtual_addresses[DSM_VIRTUAL_ADDR_MAX];
extern uint32_t dsm_addr_virtual_allocated[BITFIELD_BLOCK_COUNT(DSM_VIRTUAL_ADDR_MAX)];










void virtual_address_set(const uint8_t * p_label_uuid, dsm_handle_t handle);
bool address_handle_virtual_valid(dsm_handle_t address_handle);
bool virtual_address_uuid_index_get(const uint8_t * p_uuid, uint16_t * p_index);
uint32_t add_address_virtual(const uint8_t * p_label_uuid, dsm_handle_t * p_address_handle, dsm_address_role_t role);

#endif //__DSM_VIRTUAL_ADDR_H__
