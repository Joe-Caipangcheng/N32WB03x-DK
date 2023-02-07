#ifndef BEARER_EVENT_H__
#define BEARER_EVENT_H__

#include <stdint.h>

#include "co_timer_schedule.h"
#include "mesh_queue.h"


#define BEARER_EVENT_FLAG_INVALID   0xFFFFFFFF

typedef void(*bearer_event_callback_t)(void* p_context);
typedef bool(*bearer_event_flag_callback_t)(void);
typedef uint32_t bearer_event_flag_t;

typedef struct
{
    queue_elem_t queue_elem; 
    bearer_event_callback_t callback; 
    void * p_context; 
    volatile bool event_pending; 
} bearer_event_sequential_t;

void bearer_event_init(void);
void bearer_event_start(void);
bearer_event_flag_t bearer_event_flag_add(bearer_event_flag_callback_t callback);
void bearer_event_flag_set(bearer_event_flag_t flag);
void bearer_event_sequential_add(bearer_event_sequential_t * p_seq, bearer_event_callback_t callback, void * p_context);
uint32_t bearer_event_sequential_post(bearer_event_sequential_t * p_seq);
bool bearer_event_sequential_pending(bearer_event_sequential_t * p_seq);


#endif /* BEARER_EVENT_H__ */
