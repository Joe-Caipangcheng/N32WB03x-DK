#ifndef TIMER_SCHEDULER_H__
#define TIMER_SCHEDULER_H__

#include <stdint.h>
#include <stdbool.h>

#include "co_timer_schedule.h"




void timer_sch_init(void);
void timer_sch_schedule(timer_event_t* p_timer_evt);
void timer_sch_abort(timer_event_t* p_timer_evt);
void timer_sch_reschedule(timer_event_t* p_timer_evt, timestamp_t new_timestamp);
bool timer_sch_is_scheduled(const timer_event_t * p_timer_evt);


#endif /* TIMER_SCHEDULER_H__ */
