#ifndef DEVICE_STATE_MANAGER_H__
#define DEVICE_STATE_MANAGER_H__

#include <stdbool.h>
#include <stdint.h>
#include "toolchain.h"
#include "mesh.h"
#include "mesh_config_app.h"
#include "mesh_opt_dsm.h"
#include "dsm_unicast.h"
#include "dsm_metadata.h"
#include "dsm_subnet.h"
#include "dsm_devkey.h"
#include "dsm_appkey.h"
#include "dsm_nonvirtual_addr.h"
#include "dsm_virtual_addr.h"









extern bool dsm_has_primary_subnet;













void dsm_init(void);
uint32_t dsm_load_config_apply(void);
void dsm_clear(void);




uint32_t dsm_address_subscription_add_handle(dsm_handle_t address_handle);
uint32_t dsm_address_subscription_add(uint16_t raw_address, dsm_handle_t * p_address_handle);
uint32_t dsm_address_subscription_remove(dsm_handle_t address_handle);

uint32_t dsm_address_publish_virtual_add(const uint8_t * p_label_uuid, dsm_handle_t * p_address_handle);
uint32_t dsm_address_publish_add(uint16_t raw_address, dsm_handle_t * p_address_handle);
uint32_t dsm_address_publish_add_handle(dsm_handle_t address_handle);
uint32_t dsm_address_publish_remove(dsm_handle_t address_handle);


uint32_t dsm_appkey_handle_to_subnet_handle(dsm_handle_t appkey_handle, dsm_handle_t *p_netkey_handle);
uint32_t dsm_address_get(dsm_handle_t address_handle, mesh_address_t * p_address);
uint32_t dsm_address_handle_get(const mesh_address_t * p_address, dsm_handle_t * p_address_handle);
uint32_t dsm_tx_secmat_get(dsm_handle_t subnet_handle, dsm_handle_t app_handle, mesh_secmat_t * p_secmat);




#endif /* DEVICE_STATE_MANAGER_H__ */

