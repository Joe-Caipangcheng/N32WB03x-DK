#ifndef __DSM_COMMON_H__
#define __DSM_COMMON_H__




#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "mesh.h"
#include "mesh_defines.h"
#include "mesh_utils.h"
#include "bitfield.h"
#include "utils.h"
#include "bearer_event.h"
#include "mesh_events.h"
#include "mesh_config_entry.h"
#include "mesh_opt_dsm.h"
#include "mesh_keygen.h"
#include "mesh_opt_core.h"
#include "packet_mesh.h"
#include "mesh_config_app.h"



#define PRIMARY_SUBNET_INDEX        (0)
#define DSM_DEVKEY_HANDLE_START     DSM_APP_MAX
#define DSM_VIRTUAL_HANDLE_START     DSM_NONVIRTUAL_ADDR_MAX
#define DSM_ADDR_MAX (DSM_VIRTUAL_ADDR_MAX + DSM_NONVIRTUAL_ADDR_MAX)
#define DSM_HANDLE_INVALID  (0xFFFF)
#define DSM_KEY_INDEX_MAX   (MESH_GLOBAL_KEY_INDEX_MAX)
#define DSM_APP_MAX_LIMIT   (247)
#define DSM_SUBNET_MAX_LIMIT (252)
#define DSM_ADDR_MAX_LIMIT  (185)



typedef struct
{
    uint8_t is_metadata_stored : 1;
    uint8_t is_load_failed : 1;
    uint8_t is_legacy_found : 1;
} local_dsm_status_t;


typedef struct
{
    mesh_key_index_t net_key_index;
    uint8_t root_key[MESH_KEY_SIZE];
    uint8_t root_key_updated[MESH_KEY_SIZE];
    mesh_network_secmat_t secmat;
    mesh_network_secmat_t secmat_updated;
    mesh_key_refresh_phase_t key_refresh_phase;
    struct
    {
        mesh_beacon_tx_info_t tx_info;
        mesh_beacon_info_t info;
    } beacon;   
} subnet_t;


typedef struct
{
    uint16_t key_owner; 
    dsm_handle_t subnet_handle; 
    mesh_application_secmat_t secmat; 
} devkey_t;

typedef struct
{
    mesh_key_index_t app_key_index;
    dsm_handle_t subnet_handle;
    mesh_application_secmat_t secmat;
    bool key_updated;
    mesh_application_secmat_t secmat_updated;
} appkey_t;
typedef struct
{
    uint16_t address;
    uint8_t publish_count;
    uint8_t subscription_count;
} regular_address_t;
typedef struct
{
    uint16_t address;
    uint8_t publish_count;
    uint8_t subscription_count;
    uint8_t uuid[MESH_UUID_SIZE];
} virtual_address_t;
typedef enum
{
    DSM_ADDRESS_ROLE_SUBSCRIBE,
    DSM_ADDRESS_ROLE_PUBLISH
} dsm_address_role_t;








void dsm_entry_store(uint16_t record_id, dsm_handle_t handle, uint32_t * p_property);
void dsm_entry_invalidate(uint16_t record_id, dsm_handle_t handle, uint32_t * p_property);



#endif //__DSM_COMMON_H__
