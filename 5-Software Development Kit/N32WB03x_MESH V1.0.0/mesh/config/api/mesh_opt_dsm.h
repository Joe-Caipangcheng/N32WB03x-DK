#ifndef MESH_OPT_DSM_H__
#define MESH_OPT_DSM_H__

#include <stdint.h>
#include "mesh_opt.h"
#include "mesh.h"

enum
{
    MESH_OPT_DSM_METADATA_RECORD               = 0x0001,
    MESH_OPT_DSM_UNICAST_ADDR_RECORD           = 0x0002,
    MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD        = 0x1000,
    MESH_OPT_DSM_VIRTUAL_ADDR_RECORD           = 0x2000,
    MESH_OPT_DSM_SUBNETS_RECORD                = 0x3000,
    MESH_OPT_DSM_APPKEYS_RECORD                = 0x4000,
    MESH_OPT_DSM_DEVKEYS_RECORD                = 0x5000,
    MESH_OPT_DSM_LEGACY_SUBNETS_RECORD         = 0xD000,
    MESH_OPT_DSM_REDUCED_LEGACY_APPKEYS_RECORD = 0xE000,
    MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD    = 0xF000,
};

#define MESH_OPT_DSM_METADATA_EID                MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_METADATA_RECORD)
#define MESH_OPT_DSM_UNICAST_ADDR_EID            MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_UNICAST_ADDR_RECORD)
#define MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD_EID  MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_NONVIRTUAL_ADDR_RECORD)
#define MESH_OPT_DSM_VIRTUAL_ADDR_RECORD_EID     MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_VIRTUAL_ADDR_RECORD)
#define MESH_OPT_DSM_SUBNETS_RECORD_EID          MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_SUBNETS_RECORD)
#define MESH_OPT_DSM_APPKEYS_RECORD_EID          MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_APPKEYS_RECORD)
#define MESH_OPT_DSM_DEVKEYS_RECORD_EID          MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_DEVKEYS_RECORD)

#define MESH_OPT_DSM_LEGACY_SUBNETS_RECORD_EID          MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_LEGACY_SUBNETS_RECORD)
#define MESH_OPT_DSM_REDUCED_LEGACY_APPKEYS_RECORD_EID  MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_REDUCED_LEGACY_APPKEYS_RECORD)
#define MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD_EID     MESH_CONFIG_ENTRY_ID(MESH_OPT_DSM_FILE_ID, MESH_OPT_DSM_FULL_LEGACY_APPKEYS_RECORD)


typedef uint16_t mesh_key_index_t;
typedef uint16_t dsm_handle_t;

typedef struct
{
    uint16_t max_subnets;
    uint16_t max_appkeys;
    uint16_t max_devkeys;
    uint16_t max_addrs_nonvirtual;
    uint16_t max_addrs_virtual;
} dsm_entry_metainfo_t;

typedef struct
{
    uint16_t addr;
} dsm_entry_addr_nonvirtual_t;
typedef struct
{
    uint16_t address_start; 
    uint16_t count;         
} dsm_local_unicast_address_t;
typedef struct
{
    dsm_local_unicast_address_t addr;
} dsm_entry_addr_unicast_t;



typedef struct
{
    mesh_key_index_t             key_index;
    mesh_key_refresh_phase_t key_refresh_phase;
    uint8_t                      key[MESH_KEY_SIZE];
    uint8_t                      key_updated[MESH_KEY_SIZE];
} dsm_entry_subnet_t;



typedef struct
{
    mesh_key_index_t key_index;
    dsm_handle_t     subnet_handle;
    bool             is_key_updated;
    uint8_t          key[MESH_KEY_SIZE];
    uint8_t          key_updated[MESH_KEY_SIZE];
} dsm_entry_appkey_t;

typedef struct
{
    mesh_key_index_t key_index;
    dsm_handle_t     subnet_handle;
    uint8_t          key[MESH_KEY_SIZE];
    uint8_t          key_updated[MESH_KEY_SIZE];
} dsm_legacy_entry_appkey_t;

typedef struct
{
    uint16_t     key_owner;
    dsm_handle_t subnet_handle;
    uint8_t      key[MESH_KEY_SIZE];
} dsm_entry_devkey_t;



typedef struct
{
    uint8_t uuid[MESH_UUID_SIZE];
} dsm_entry_addr_virtual_t;


#endif /* MESH_OPT_DSM_H__ */
