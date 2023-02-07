#ifndef MESH_OPT_ACCESS_H__
#define MESH_OPT_ACCESS_H__

#include <stdint.h>

#include "mesh_opt.h"


enum
{
    MESH_OPT_ACCESS_METADATA_RECORD      = 0x0001,
    MESH_OPT_ACCESS_ELEMENTS_RECORD      = 0x1000,
    MESH_OPT_ACCESS_MODELS_RECORD        = 0x2000,
    MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD = 0x3000
};

#define MESH_OPT_ACCESS_METADATA_EID       MESH_CONFIG_ENTRY_ID(MESH_OPT_ACCESS_FILE_ID, MESH_OPT_ACCESS_METADATA_RECORD)
#define MESH_OPT_ACCESS_SUBSCRIPTIONS_EID  MESH_CONFIG_ENTRY_ID(MESH_OPT_ACCESS_FILE_ID, MESH_OPT_ACCESS_SUBSCRIPTIONS_RECORD)
#define MESH_OPT_ACCESS_ELEMENTS_EID       MESH_CONFIG_ENTRY_ID(MESH_OPT_ACCESS_FILE_ID, MESH_OPT_ACCESS_ELEMENTS_RECORD)
#define MESH_OPT_ACCESS_MODELS_EID         MESH_CONFIG_ENTRY_ID(MESH_OPT_ACCESS_FILE_ID, MESH_OPT_ACCESS_MODELS_RECORD)

#endif /* MESH_OPT_ACCESS_H__ */
