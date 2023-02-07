#ifndef MESH_EVENT_H__
#define MESH_EVENT_H__

#include <stdint.h>
#include "mesh_events.h"


void event_handle(const mesh_evt_t * p_evt);
void event_handler_add(mesh_evt_handler_t * p_handler_params);
void event_handler_remove(mesh_evt_handler_t * p_handler_params);

#endif

