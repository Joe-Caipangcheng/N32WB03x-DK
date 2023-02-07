#ifndef ACCESS_PUBLISH_H__
#define ACCESS_PUBLISH_H__

#include <stdint.h>
#include "access.h"


typedef struct __access_model_publication_state_t
{
    access_model_handle_t model_handle;
    access_publish_period_t period;
    access_publish_timeout_cb_t publish_timeout_cb;
    uint32_t target;
    struct __access_model_publication_state_t * p_next;
} access_model_publication_state_t;






void access_publish_init(void);
void access_publish_period_set(access_model_publication_state_t * p_pubstate, access_publish_resolution_t resolution, uint8_t step_number);
void access_publish_period_get(const access_model_publication_state_t * p_pubstate, access_publish_resolution_t * p_resolution, uint8_t * p_step_number);

#endif

