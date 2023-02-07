/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file app_onoff.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#ifndef APP_ONOFF_H__
#define APP_ONOFF_H__

#include <stdint.h>

#include "generic_onoff_server.h"

/**
 * APP_ONOFF Generic OnOff server behaviour
 * Application level OnOff server behavioral structures, functions, and callbacks.
 *
 * This module implements the behavioral requirements of the Generic OnOff server model.
 *
 * The application should use the set callback provided by this module to set the
 * hardware state. The hardware state could be changed by reflecting the value provided by the
 * set callback on the GPIO. Similarly, the application should use the get callback provided 
 * by this module to read the hardware state.
 *
 * This module triggers the set callback only when it determins that it is time to
 * inform the user application. 
 */

/**
 * Macro to create application level app_onoff_server_t context.
 *
 * @param[in] _name                 Name of the app_onoff_server_t instance
 * @param[in] _force_segmented      If the Generic OnOff server shall use force segmentation of messages
 * @param[in] _mic_size             MIC size to be used by Generic OnOff server
 * @param[in] _set_cb               Callback for setting the application state to given value.
 * @param[in] _get_cb               Callback for reading the state from the application.
 */
#define APP_ONOFF_SERVER_DEF(_name, _force_segmented, _mic_size, _set_cb, _get_cb)  \
    static app_onoff_server_t _name =  \
    {  \
        .server.settings.force_segmented = _force_segmented,  \
        .server.settings.transmic_size = _mic_size,  \
        .onoff_set_cb = _set_cb,  \
        .onoff_get_cb = _get_cb  \
    };

    
    
/**
 * @brief  Internal structure to hold state and timing information.
 */    
typedef struct
{
	/** Present value of the OnOff state */
    bool present_onoff;
	/** Target value of the OnOff state, as received from the model interface. */
    bool target_onoff;
	/** Remaining transition time in milliseconds */
    uint32_t remaining_time_ms;
	/** Message execution delay in milliseconds */
    uint32_t delay_ms;
} app_onoff_state_t;

typedef struct __app_onoff_server_t app_onoff_server_t;

/**
 * @brief  Application state set callback prototype.
 * @param  p_app	Pointer to app_onoff_server_t context.
 * @param  onoff    New onoff value to be used by the application
 */
typedef void (*app_onoff_set_cb_t)(const app_onoff_server_t * p_server, bool onoff);

/**
 * @brief  Application state read callback prototype.
 * @param  p_app	Pointer to app_onoff_server_t context.
 * @param  onoff    User application fills this value with the value retrived from the hardware interface.
 */
typedef void (*app_onoff_get_cb_t)(const app_onoff_server_t * p_server, bool * p_present_onoff);

/** Application level structure holding the OnOff server model context and OnOff state representation */
struct __app_onoff_server_t
{
	/** OnOff server model interface context structure */
    generic_onoff_server_t server;
	/** Callaback to be called for informing the user application to update the value*/
    app_onoff_set_cb_t  onoff_set_cb;
	/** Callback to be called for requesting current value from the user application */
    app_onoff_get_cb_t onoff_get_cb;
	/** Internal variable. Representation of the OnOff state related data for behavioral implementation, 
	and for communicating with the application */
    app_onoff_state_t state;
	/** Internal variable. It is used for acquiring RTC counter value. */
    uint32_t last_rtc_counter;
	/** Internal flag. It is used for representation of the state change. */
    bool value_updated;
};

/**
 * @brief	Pressing button 1/2 on the Development Kit will result in LED state to trigger
			the STATUS message to inform client about the state change. This is a demonstration of
			state change publication due to local event. 
 * @param  p_server Pointer to app_onoff_server_t context.
*/
void app_onoff_status_publish(app_onoff_server_t * p_server);

/**
 * @brief  Instantiate onoff server on element index.
 * @param  p_server Pointer to app_onoff_server_t context.
 * @param  element_index element that onoff server is located on.
 */
uint32_t app_onoff_init(app_onoff_server_t * p_server, uint8_t element_index);

#endif /* APP_ONOFF_H__ */


