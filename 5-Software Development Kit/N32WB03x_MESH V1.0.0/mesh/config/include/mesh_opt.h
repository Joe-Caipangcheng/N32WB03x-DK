#ifndef MESH_OPT_H__
#define MESH_OPT_H__
#include "mesh_config_entry.h"


enum
{
    MESH_OPT_NET_STATE_FILE_ID  = 0x0000,
    MESH_OPT_DSM_FILE_ID        = 0x0001,
    MESH_OPT_ACCESS_FILE_ID     = 0x0002,
    MESH_OPT_CORE_FILE_ID       = 0x0003,
    MESH_OPT_FIRST_FREE_ID
};
#define MESH_OPT_CORE_ID(id)      MESH_CONFIG_ENTRY_ID(MESH_OPT_CORE_FILE_ID, id)

#define MESH_OPT_CORE_ID_START (0x0100)
#define MESH_OPT_CORE_ID_END   (0x01FF)
#define MESH_OPT_PROV_ID_START (0x0200)
#define MESH_OPT_PROV_ID_END   (0x020F)
#define MESH_OPT_GATT_ID_START (0x0210)
#define MESH_OPT_GATT_ID_END   (0x021F)
#define MESH_OPT_FRND_ID_START (0x0220)
#define MESH_OPT_FRND_ID_END   (0x022F)
#define MESH_OPT_HEALTH_ID_START (0x0230)
#define MESH_OPT_HEALTH_ID_END   (0x024F)

#define MESH_OPT_HEALTH_PRIMARY_EID     MESH_OPT_CORE_ID(MESH_OPT_HEALTH_ID_START + 0)

void mesh_opt_init(void);
void mesh_opt_clear(void);




#endif /* MESH_OPT_H__ */
