#ifndef NET_STATE_H__
#define NET_STATE_H__

#include <stdint.h>
#include <stdbool.h>
#include "mesh.h"
#include "mesh_events.h"
#include "mesh_opt_net_state.h"




#define NETWORK_IVI_MASK     (0x00000001)
typedef struct
{
    uint32_t seqnum; 
    uint32_t seqnum_max_available; 
    uint32_t iv_index; 
    bool     key_refresh;               
    struct
    {
        net_state_iv_update_t state;   
        bool pending; 
        bool locked; 
        uint16_t timeout_counter; 
        uint16_t ivr_timeout_counter;
    } iv_update;
} network_state_t;

typedef struct
{
    uint8_t is_seqnum_restored : 1;
    uint8_t is_iv_index_restored : 1;
    uint8_t is_seqnum_allocation_in_progress : 1;
    uint8_t is_test_mode : 1;
    uint8_t is_enabled : 1;
    uint8_t is_iv_state_set_externally : 1;
} net_state_status_t;
extern network_state_t m_net_state;
extern net_state_status_t m_net_state_staus;
extern mesh_opt_seqnum_persist_data_t m_pst_seqnum;



typedef enum
{
	NET_STATE_TO_IV_UPDATE_IN_PROGRESS_SIGNAL,
	NET_STATE_TO_NORMAL_SIGNAL,
} net_state_iv_update_signals_t;

void net_state_init(void);
void net_state_enable(void);
uint32_t net_state_iv_index_set(uint32_t iv_index, bool iv_update);
void net_state_reset(void);
uint32_t net_state_seqnum_alloc(uint32_t * p_seqnum);
void net_state_iv_update_test_mode_set(bool test_mode_on);
uint32_t net_state_test_mode_transition_run(net_state_iv_update_signals_t signal);
uint32_t net_state_iv_update_start(void);
void net_state_key_refresh_phase_changed(uint16_t subnet_index,
                                         const uint8_t * p_network_id,
                                         mesh_key_refresh_phase_t new_phase);
void net_state_iv_index_lock(bool lock);
uint32_t net_state_beacon_iv_index_get(void);
uint32_t net_state_tx_iv_index_get(void);
uint32_t net_state_rx_iv_index_get(uint8_t ivi);

                                         
                                         
                                         
static inline net_state_iv_update_t net_state_iv_update_get(void)
{
    return m_net_state.iv_update.state;
}                                         
                                         


                                     
                                         
                                         
                                         
                                         
#endif /* NET_STATE_H__ */

