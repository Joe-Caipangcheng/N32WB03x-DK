#ifndef __CO_TIMER_SCHEDULE_H__
#define __CO_TIMER_SCHEDULE_H__




#include "rwip.h"
#include <stdint.h>





#define TIMER_DIFF(timestamp, reference) timer_diff(timestamp, reference)

typedef double timestamp_t;

#define TIMER_EVENT_INTERVAL_SINGLE_SHOT    (0)
typedef void (*timer_sch_callback_t)(timestamp_t timestamp, void * p_context);

typedef enum
{
    TIMER_EVENT_STATE_UNUSED,      
    TIMER_EVENT_STATE_ADDED,       
    TIMER_EVENT_STATE_IN_CALLBACK  
} timer_event_state_t;

typedef struct timer_event
{
    volatile timer_event_state_t state;     
    timestamp_t                  timestamp; 
    timer_sch_callback_t         cb;        
    uint32_t                     interval;  
    void *                       p_context; 
    struct timer_event*          p_next;    
} timer_event_t;



typedef void(* timer_callback_t)(timestamp_t);



#define TIMER_OLDER_THAN(time, ref) (((uint32_t) (time)) - ((uint32_t) (ref)) > 0x7FFFFFFF )

static inline timestamp_t timer_diff(timestamp_t time1, timestamp_t time2)
{
    if (time1 - time2 > UINT32_MAX / 2)
    {
        return time2 - time1;
    }
    else
    {
        return time1 - time2;
    }
}

static inline timestamp_t bsp_time_now(void)
{
    rwip_time_t time = rwip_time_get();
    time.hs += 3;   
    return time.hs*325;
}



#endif //__CO_TIMER_SCHEDULE_H__
