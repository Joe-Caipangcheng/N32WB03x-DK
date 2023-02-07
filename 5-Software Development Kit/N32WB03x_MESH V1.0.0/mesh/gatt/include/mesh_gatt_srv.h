#ifndef __MESH_GATT_SRV_H__
#define __MESH_GATT_SRV_H__




#include "prf_types.h"
#include "prf.h"






struct mesh_gatt_db_cfg
{
    uint8_t max_nb_att;
    uint8_t *att_tbl;
    uint8_t *cfg_flag;
    uint16_t features;
    uint8_t is_proxy;
};


enum
{
    MESH_GATT_IDX_SVC,	
    MESH_GATT_IDX_CHAR_IN_CHAR,
    MESH_GATT_IDX_CHAR_IN_VAL,
    MESH_GATT_IDX_CHAR_OUT_CHAR,
    MESH_GATT_IDX_CHAR_OUT_VAL,
    MESH_GATT_IDX_CHAR_OUT_CFG,	
    MESH_GATT_IDX_NB,
};
struct mesh_gatt_env_tag
{
    prf_env_t prf_env;
    uint16_t shdl;
    uint8_t max_nb_att;
    struct ke_msg *operation;
    uint8_t cursor;
    uint8_t ccc_idx;
    struct co_list values;
    ke_state_t state[MESH_GATT_IDX_NB];
};
enum
{
    MESH_GATT_IDLE,
    MESH_GATT_BUSY,
    MESH_GATT_STATE_MAX,
};

enum
{
    MESH_GATT_CREATE_DB_REQ = TASK_FIRST_MSG(TASK_ID_MESH_GATT),  
    MESH_GATT_VALUE_REQ_IND,
    MESH_GATT_VAL_WRITE_IND,
    MESH_GATT_ATT_INFO_REQ,
    MESH_GATT_CONNECT,
	MESH_GATT_DISCONNECT,
    MESH_GATT_TX_COMPLETE,
};

struct mesh_gatt_val_write_ind
{
    uint8_t  conidx;
    uint16_t handle;
    uint16_t length;
    uint8_t  value[__ARRAY_EMPTY];
};

struct mesh_gatt_val_elmt
{
    struct co_list_hdr hdr;
    uint8_t att_idx;
    uint8_t length;
    uint8_t data[__ARRAY_EMPTY];
};
struct mesh_gatt_value_req_ind
{
    uint8_t  conidx;
    uint16_t att_idx;
};
struct mesh_gatt_att_info_req
{
    uint8_t  conidx;
    uint16_t att_idx;
};

struct mesh_gatt_connection_req_ind
{
    uint16_t conhdl;
    uint8_t con_index;    
};



const struct prf_task_cbs* mesh_gatt_prf_itf_get(void);
extern struct mesh_gatt_env_tag *p_mesh_gatt_env;
void mesh_gatt_srv_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);


#endif //__MESH_GATT_SRV_H__
