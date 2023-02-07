#ifndef CONFIG_SERVER_H__
#define CONFIG_SERVER_H__

#include <stdint.h>
#include "device_state_manager.h"
#include "access_publish.h"
#include "config_server_events.h"






uint32_t config_server_init(config_server_evt_cb_t evt_cb);
uint32_t config_server_bind(dsm_handle_t devkey_handle);





#endif

