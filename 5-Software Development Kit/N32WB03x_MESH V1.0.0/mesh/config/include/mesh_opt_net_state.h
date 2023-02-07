#ifndef MESH_OPT_NET_STATE_H__
#define MESH_OPT_NET_STATE_H__

#include <stdint.h>

#include "mesh_opt.h"
#include "mesh.h"


enum
{
    MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_RECORD = 1,
    MESH_OPT_NET_STATE_IV_INDEX_LEGACY_RECORD,
    MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_RECORD,
    MESH_OPT_NET_STATE_IV_INDEX_RECORD
};

#define MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID  MESH_CONFIG_ENTRY_ID(MESH_OPT_NET_STATE_FILE_ID, MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_RECORD)
#define MESH_OPT_NET_STATE_IV_INDEX_EID       MESH_CONFIG_ENTRY_ID(MESH_OPT_NET_STATE_FILE_ID, MESH_OPT_NET_STATE_IV_INDEX_RECORD)
#define MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_EID  MESH_CONFIG_ENTRY_ID(MESH_OPT_NET_STATE_FILE_ID, MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_RECORD)
#define MESH_OPT_NET_STATE_IV_INDEX_LEGACY_EID       MESH_CONFIG_ENTRY_ID(MESH_OPT_NET_STATE_FILE_ID, MESH_OPT_NET_STATE_IV_INDEX_LEGACY_RECORD)

typedef struct __attribute((packed))
{
    uint32_t iv_index;
    net_state_iv_update_t  iv_update_in_progress;
    uint16_t  iv_update_timeout_counter;
    uint8_t  synchro_index;
} mesh_opt_iv_index_persist_data_t;

typedef struct __attribute((packed))
{
    uint32_t next_block;
    uint8_t  synchro_index;
} mesh_opt_seqnum_persist_data_t;

typedef struct
{
    uint32_t iv_index;
    uint8_t  iv_update_in_progress;
} mesh_opt_iv_index_persist_data_legacy_t;

typedef struct
{
    uint32_t next_block;
} mesh_opt_seqnum_persist_data_legacy_t;


#endif /* MESH_OPT_NET_STATE_H__ */
