/**
 ****************************************************************************************
 *
 * @file ns_ius_task.h
 *
 * @brief NS IUS profile task header file
 *
 * Copyright (C) Nation 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef __NS_IUSS_TASK_H__
#define __NS_IUSS_TASK_H__


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_NS_IUS)


#include "prf_types.h"
#include "prf.h"
#include "attm.h"		











struct ns_ius_db_cfg
{
    uint8_t max_nb_att;
    uint8_t *att_tbl;
    uint8_t *cfg_flag;
    uint16_t features;
};


enum
{
    NS_IUS_CREATE_DB_REQ = TASK_FIRST_MSG(TASK_ID_NS_IUS),  
    NS_IUS_VALUE_REQ_IND,
    NS_IUS_VAL_WRITE_IND,
    NS_IUS_ATT_INFO_REQ,
	  NS_IUS_DISCONNECT,
};

enum ns_ius_state
{
    NS_IUS_IDLE,
    NS_IUS_BUSY,
    NS_IUS_STATE_MAX,
};

struct ns_ius_val_write_ind
{
    uint8_t  conidx;
    uint16_t handle;
    uint16_t length;
    uint8_t  value[__ARRAY_EMPTY];
};
struct ns_ius_value_req_ind
{
    uint8_t  conidx;
    uint16_t att_idx;
};

struct ns_ius_att_info_req
{
    uint8_t  conidx;
    uint16_t att_idx;
};


struct ns_ius_disconnect
{
    uint8_t  conidx;
};


extern ke_task_id_t  ns_ble_ius_task;


/** 
 * @brief Set value of CCC for given attribute and connection index.
 * @param[in] conidx   Connection index.
 * @param[in] att_idx  CCC attribute index.
 * @param[in] cc       Value to store. 
 */
void ns_ius_set_ccc_value(uint8_t conidx, uint8_t att_idx, uint16_t ccc);
/** 
 * @brief Initialize Client Characteristic Configuration fields.
 * @details Function initializes all CCC fields to default value.
 * @param[in] att_db         Id of the message received.
 * @param[in] max_nb_att     Pointer to the parameters of the message. 
 */
void ns_ius_init_ccc_values(const struct attm_desc_128 *att_db, int max_nb_att);



#endif  //BLE_APP_NS_IUS







#endif //__NS_IUSS_TASK_H__
