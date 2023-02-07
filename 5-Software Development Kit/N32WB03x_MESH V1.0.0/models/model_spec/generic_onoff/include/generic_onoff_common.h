#ifndef GENERIC_ONOFF_COMMON_H__
#define GENERIC_ONOFF_COMMON_H__

#include <stdint.h>
#include "model_common.h"


#define GENERIC_ONOFF_COMPANY_ID 0xFFFF
#define GENERIC_ONOFF_MAX        (0x01)


typedef struct
{
    bool on_off;                                       
    uint8_t tid;                                       
} generic_onoff_state_t;

typedef struct
{
    bool on_off;                                       
    uint8_t tid;                                       
} generic_onoff_set_params_t;

typedef struct
{
    uint8_t present_on_off;                            
    uint8_t target_on_off;                             
    uint32_t remaining_time_ms;                        
} generic_onoff_status_params_t;


#endif /* GENERIC_ONOFF_COMMON_H__ */
