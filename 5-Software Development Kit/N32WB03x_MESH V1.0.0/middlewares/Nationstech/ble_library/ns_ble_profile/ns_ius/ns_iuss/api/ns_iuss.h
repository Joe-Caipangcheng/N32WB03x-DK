#ifndef __NS_IUSS_H__
#define __NS_IUSS_H__



#include "rwip_config.h"              // SW configuration


#if (BLE_APP_NS_IUS)


#include <stdint.h>
#include "prf_types.h"
#include "prf.h"
#include "attm.h" 


#define NS_IUS_IDX_MAX        (1)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/* Public define ------------------------------------------------------------*/
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/  
/* Public constants ---------------------------------------------------------*/

/// Value element
struct ns_ius_val_elmt
{
    struct co_list_hdr hdr;
    uint8_t att_idx;
    uint8_t length;
    uint8_t data[__ARRAY_EMPTY];
};

struct ns_ius_env_tag
{
    prf_env_t prf_env;
    uint16_t shdl;
    uint8_t max_nb_att;
    struct ke_msg *operation;
    uint8_t cursor;
    uint8_t ccc_idx;
    struct co_list values;
    ke_state_t state[NS_IUS_IDX_MAX];
};


const struct prf_task_cbs* ns_ius_prf_itf_get(void);




/* Public function prototypes -----------------------------------------------*/


/**
 * @brief Initialization of the NS IUS module.
 * This function performs all the initializations of the Profile module.
 *  - Creation of database (if it's a service)
 *  - Allocation of profile required memory
 *  - Initialization of task descriptor to register application
 *      - Task State array
 *      - Number of tasks
 *      - Default task handler
 *
 * @param[out]    p_env         Collector or Service allocated environment data.
 * @param[in|out] p_start_hdl   Service start handle (0 - dynamically allocated), only applies for services.
 * @param[in]     app_task      Application task number.
 * @param[in]     sec_lvl       Security level (AUTH, EKS and MI field of @see enum attm_value_perm_mask)
 * @param[in]     p_params      Configuration parameters of profile collector or service (32 bits aligned)
 *
 * @return status code to know if profile initialization succeed or not.
 */
void ns_ius_task_init(struct ke_task_desc *p_task_desc);



#endif





#endif //__NS_IUSS_H__
