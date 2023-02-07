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
 * @file ns_ble.h
 * @author Nations Firmware Team
 * @version v1.0.3
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#ifndef _NS_BLE_H_
#define _NS_BLE_H_

/**
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief Application entry point.
 *
 * @{
 **/

/* Includes ------------------------------------------------------------------*/

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_PRESENT)
#include "global_func.h"
/* Define ------------------------------------------------------------*/

#define APP_ADV_DURATION_MAX           (655)

#define SECS_UNIT_1_25_MS              (800)
#define SECS_UNIT_10MS                 (100)
#define SECS_TO_UNIT(sec,uint)         ((sec)*(uint))

#define MSECS_UNIT_1_25_MS              (1250)
#define MSECS_UNIT_10_MS                (10000)
#define MSECS_TO_UNIT(msec,uint)        ((msec*1000)/(uint))

/// Maximal length of the Device Name value
#define APP_DEVICE_NAME_MAX_LEN      (18)
#define APP_MESH_DEMO_TYPE_LEN        (1)
/// Device IRK used for Resolvable Private Address generation (LSB first)
#define SEC_DEFAULT_IRK  "\x50\x19\x21\x90\x1f\x04\xd5\x62\x4f\xa4\x89\xab\x90\xd0\xcf\x23"

/*
 * MACROS
 **/

#define APP_HANDLERS(subtask)    {&subtask##_msg_handler_list[0], ARRAY_LEN(subtask##_msg_handler_list)}


/* Typedef -----------------------------------------------------------*/
typedef void (* ble_hw_check_t)(uint8_t,uint32_t);
typedef void (*ns_ble_add_prf_func_t)(void);

enum current_op_t
{
    CURRENT_OP_NULL, 
    //adv
    CURRENT_OP_CREATE_ADV,
    CURRENT_OP_SET_ADV_DATA,
    CURRENT_OP_SET_RSP_DATA,
    CURRENT_OP_START_ADV,
    CURRENT_OP_STOP_ADV,
    CURRENT_OP_DELETE_ADV,
    //scan
    CURRENT_OP_CREATE_SCAN,
    CURRENT_OP_START_SCAN,
    CURRENT_OP_STOP_SCAN,
    CURRENT_OP_DELETE_SCAN,
    
    //init (master connect)
    CURRENT_OP_CREATE_INIT,
    CURRENT_OP_START_INIT,
    CURRENT_OP_STOP_INIT,
    CURRENT_OP_DELETE_INIT,
};

/// Advertising state machine
enum app_adv_state
{
    /// Advertising activity does not exists
    APP_ADV_STATE_IDLE = 0,
    #if BLE_APP_PRF
    /// Creating advertising activity
    APP_ADV_STATE_CREATING,
    /// Setting advertising data
    APP_ADV_STATE_SETTING_ADV_DATA,
    /// Setting scan response data
    APP_ADV_STATE_SETTING_SCAN_RSP_DATA,

    /// Advertising activity created
    APP_ADV_STATE_CREATED,
    /// Starting advertising activity
    APP_ADV_STATE_STARTING,
    /// Advertising activity started
    APP_ADV_STATE_STARTED,
    /// Stopping advertising activity
    APP_ADV_STATE_STOPPING,
    #endif //(BLE_APP_PRF)
};

enum app_adv_mode
{
    APP_ADV_MODE_IDLE = 0,
    APP_ADV_MODE_ENABLE,
    APP_ADV_MODE_DIRECTED,
    APP_ADV_MODE_FAST,
    APP_ADV_MODE_SLOW,
    APP_ADV_MODE_STOP,
    
};

enum app_ble_msg
{
    APP_BLE_NULL_MSG= 0,
    APP_BLE_OS_READY,
    APP_BLE_GAP_CONNECTED,
    APP_BLE_GAP_DISCONNECTED,
    APP_BLE_GAP_PARAMS_REQUEST,
    APP_BLE_GAP_CMP_EVT,
    APP_BLE_GAP_RSSI_IND,
    APP_BLE_GATTC_MTU_IND,
    APP_BLE_GAP_PARAMS_IND,
    APP_BLE_GATTC_CMP_EVT,
};

struct ble_msg_t
{
    enum app_ble_msg msg_id;
    
    //income message
    union 
    {
        struct gapm_cmp_evt const*              p_gapm_cmp;
        struct gapc_connection_req_ind const*   p_connection_ind;
        struct gapc_disconnect_ind const*       p_disconnect_ind;
        struct gapc_param_update_req_ind const* p_param_req;
        struct gapc_cmp_evt const*              p_gapc_cmp;
        struct gapc_con_rssi_ind const*         p_gapc_rssi;    
        struct gattc_mtu_changed_ind const*     p_gattc_mtu;
        struct gapc_param_updated_ind const*    p_param_updated;
        struct gattc_cmp_evt const*             p_gattc_cmp;
    }msg;
    // output command
    union 
    {
        struct gapm_set_dev_config_cmd*         p_dev_config;
        struct gapc_connection_cfm*             p_connection_cfm;
        struct gapc_param_update_cfm*           p_param_cfm;
    }cmd;
};


typedef enum
{
    BLE_LSC_LSI_32000HZ = 0,
    BLE_LSC_LSI_32768HZ,
    BLE_LSC_LSI_28800HZ,
    BLE_LSC_LSE_32768HZ
}ble_lsc_cfg_t;

/* Public define ------------------------------------------------------------*/

/// Structure containing information about the handlers for an application subtask
struct app_subtask_handlers
{
    /// Pointer to the message handler table
    const struct ke_msg_handler *p_msg_handler_tab;
    /// Number of messages handled
    uint16_t msg_cnt;
};

//APP_CON_IDX_MAX less than BLE_CONNECTION_MAX-1
/// Application connection environment structure
enum app_connection_num
{
    APP_CON_IDX_0 = 0,
    APP_CON_IDX_1,
    APP_CON_IDX_2,
    APP_CON_IDX_MAX,
};

struct app_con_env_tag
{
       /// Connection handle
    uint16_t conhdl;
    /// Connection Index
    uint8_t  conidx;
    /// Last paired peer address type
    uint8_t peer_addr_type;

    /// Last paired peer address
    struct bd_addr peer_addr;
    /// Maximum device MTU size
    uint16_t max_mtu;
    ///DLE, the maximum number of payload octets in TX
    uint16_t tx_pkt_size;    
    /// Role of device in connection (0 = Master / 1 = Slave)
    uint8_t role;        
    
};

/// Application environment structure
struct app_env_tag
{
    /// Connection handle
    uint16_t conhdl;
    /// Connection Index
    uint8_t  conidx;

    /// Advertising activity index
    uint8_t adv_actv_idx;
    /// Current advertising state (@see enum app_adv_state)
    uint8_t adv_state;
    /// Next expected operation completed event
    uint8_t current_op;  
    /// Advertising mode
    enum app_adv_mode adv_mode;

    /// Last initialized profile
    uint8_t next_svc;
    
    /// Last paired peer address type
    uint8_t peer_addr_type;

    /// Last paired peer address
    struct bd_addr peer_addr;

    /// Local device IRK
    uint8_t loc_irk[KEY_LEN];

    struct gapc_ltk ltk;

    /// Counter used to generate IRK
    uint8_t rand_cnt;

    /// Maximum device MTU size
    uint16_t max_mtu;
    ///DLE, the maximum number of payload octets in TX
    uint16_t tx_pkt_size;
       
    //master device 
    uint8_t scan_actv_idx;
    uint8_t init_actv_idx;

    // found the target near by
    uint8_t target_found;
    /// target device address type
    uint8_t target_addr_type;
    /// target device address going to connect
    struct bd_addr target_addr;
    
    //ble msg hanlder
    void (*ble_msg_handler)(struct ble_msg_t const *);
    //user msg handler
    void (*user_msg_handler)(ke_msg_id_t const, void const *);

    #if (BLE_APP_NS_IUS)    
    uint8_t manual_conn_param_update;
    uint8_t manual_mtu_update;
    #endif
    struct app_con_env_tag conn_env[APP_CON_IDX_MAX];

    uint32_t rssi_intv;
    ble_lsc_cfg_t lsc_cfg;
};


/// Application GAP device infomation structure
struct ns_gap_params_t
{
    /// Attribute database configuration (@see enum gapm_att_cfg_flag)
    uint16_t att_cfg;
    /// Device role (@see enum gap_role)
    uint8_t dev_role;
    /// Device MAC address type (enum gapm_own_addr)
    uint8_t mac_addr_type;
    /// Device MAC address
    struct bd_addr mac_addr;
    /// Device appearance
    uint16_t appearance;
    /// Device Name length
    uint8_t dev_name_len;
    /// Device Name
    uint8_t dev_name[APP_DEVICE_NAME_MAX_LEN];
    /// Device connection parameters
    struct gapc_conn_param dev_conn_param;
    /// Delay time of update connection parameters, 0 mean not active
    uint16_t conn_param_update_delay;
};

struct adv_time_t
{
    uint8_t enable;
    /// Advertising duration (in unit of 10ms). 0 means that advertising continues
    /// until the host disable it
    uint16_t duration;
    /// advertising interval (in unit of 625us). Must be greater than 20ms
    uint16_t adv_intv;
};

struct ns_adv_params_t
{    
    void (*ble_adv_msg_handler)(enum app_adv_mode);
    uint8_t adv_data[ADV_DATA_LEN];
    uint8_t adv_data_len;
    uint8_t scan_rsp_data[ADV_DATA_LEN];
    uint8_t scan_rsp_data_len;
    uint8_t attach_appearance;
    uint8_t attach_name;
    uint8_t adv_phy;
    uint8_t  ex_adv_enable;
    uint8_t* ex_adv_p_data;
    uint8_t  ex_adv_data_len;
    
    // beacon mode without presence of AD_TYPE_FLAG in advertising data
    uint8_t  beacon_enable;   
    
    struct adv_time_t directed_adv;
    struct adv_time_t fast_adv;
    struct adv_time_t slow_adv;
};

enum scan_state_t
{
    SCAN_STATE_IDLE = 0,
    SCAN_STATE_SCANING,
    SCAN_STATE_FOUND,
    SCAN_STATE_CONNECTING,
    
};

enum scan_filter_type
{
    SCAN_FILTER_DISABLE = 0,
    SCAN_FILTER_BY_ADDRESS,
    SCAN_FILTER_BY_NAME,
    SCAN_FILTER_BY_UUID128,
    SCAN_FILTER_BY_UUID16,
    SCAN_FILTER_BY_APPEARANCE,
};
struct ns_scan_params_t
{  
    /// Initiating type (@see enum gapm_init_type)
    uint8_t type;
    // if enable scan
    uint8_t scan_enable         :1;   
    // if connect the device if match
    uint8_t connect_enable      :1;    
    /// if use coded phy mode  
    uint8_t phy_coded_enable    :1;    
    /// Properties for the scan active mode, enable will require scan respond data 
    uint8_t prop_active_enable  :1;
    /// Duplicate packet filtering policy
    uint8_t dup_filt_pol        :2;
    /// Scan interval
    uint16_t scan_intv;
    /// Scan window
    uint16_t scan_wd;
    /// Scan duration (in unit of 10ms). 0 means that the controller will scan continuously until
    /// reception of a stop command from the application
    uint16_t duration;
    /// Scan period (in unit of 1.28s). Time interval betweem two consequent starts of a scan duration
    /// by the controller. 0 means that the scan procedure is not periodic
    uint16_t period;
    
    void (*ble_scan_state_handler)(enum scan_state_t);
    void (*ble_scan_data_handler)(struct gapm_ext_adv_report_ind const*);

    enum scan_filter_type filter_type;
    const uint8_t         *filter_data;
    
};


struct ns_stack_cfg_t
{
    //ble msg hanlder
    void (*ble_msg_handler)(struct ble_msg_t const *);
    //user msg handler
    void (*user_msg_handler)(ke_msg_id_t const, void const *);
    //system lsc selected
    ble_lsc_cfg_t lsc_cfg;

};


struct prf_task_t
{
    uint16_t prf_task_id;
    struct app_subtask_handlers const* prf_task_handler;
    
};

struct ns_ble_prf_evn_t
{
    uint8_t prf_num;
    struct prf_task_t prf_task_list[BLE_NB_PROFILES];
};



struct ns_ble_add_prf_evn_t
{
    uint8_t prf_num;
    ns_ble_add_prf_func_t add_prf_func_list[BLE_NB_PROFILES]; 
};


/* Public variables ---------------------------------------------------------*/
/// Application environment
extern struct app_env_tag           app_env;
extern struct ns_gap_params_t       gap_env;
extern struct ns_adv_params_t       adv_env;
extern struct ns_scan_params_t      scan_env;
extern struct ns_ble_prf_evn_t      ble_prf_evn;
/* Public function prototypes -----------------------------------------------*/
void llhwc_modem_setmode(uint8_t phy);
bool ns_ble_add_svc(void);
void ns_ble_adv_fsm_next(void);

//General function for master and slave
void ns_ble_resolv_addr(struct gap_sec_key *irk, uint8_t nb_key,struct bd_addr* addr);
void ns_ble_list_set_ral(struct gap_ral_dev_info *ral_list,uint8_t ral_cnt);
void ns_ble_set_slave_latency(uint16_t latency_cfg);
bool ns_ble_update_param(struct gapc_conn_param *conn_param);
void ns_ble_mtu_set(uint16_t mtu);
void ns_ble_phy_set(enum gap_phy_val phy);
void ns_ble_get_peer_info(uint8_t op);
void ns_ble_dle_set(uint16_t tx_octets, uint16_t tx_time);
void ns_ble_active_rssi(uint32_t interval);
void ns_ble_disconnect(void);
//stack and profile init function for master and slave
void ns_ble_lsc_config(ble_lsc_cfg_t lsc_set);
void ns_ble_stack_init(struct ns_stack_cfg_t const* p_handler);
void ns_ble_gap_init(struct ns_gap_params_t const* p_dev_info);
bool ns_ble_add_prf_func_register(ns_ble_add_prf_func_t func);
bool ns_ble_prf_task_register(struct prf_task_t *prf);

//function for slave role 
void ns_ble_adv_init(struct ns_adv_params_t const* p_adv_init);
void ns_ble_adv_start(void);
void ns_ble_adv_stop(void);
void ns_ble_adv_data_set(uint8_t* p_dat, uint16_t len);
void ns_ble_scan_rsp_data_set(uint8_t* p_dat, uint16_t len);
void ns_ble_ex_adv_data_set(uint8_t* p_dat, uint16_t len);

//function for master role 
void ns_ble_scan_init(struct ns_scan_params_t *p_init);
void ns_ble_create_scan(void);
void ns_ble_start_scan(void);
void ns_ble_delete_scan(void);
void ns_ble_stop_scan(void);
void ns_ble_create_init(void);
void ns_ble_start_init(uint8_t *addr, uint8_t addr_type);
void ns_ble_delete_init(void);
bool ns_ble_scan_data_find(uint8_t types, const uint8_t *p_filter_data, uint8_t *p_data, uint8_t len);


void    ns_ble_set_active_connection(uint8_t  conidx);
uint8_t ns_ble_get_active_connection(void);
bool    ns_ble_get_connection_state(uint8_t conidx);
uint8_t ns_ble_get_connection_num(void);
uint8_t ns_ble_get_conidx_by_role(uint8_t role);
uint8_t ns_ble_master_connection_num(void);
uint8_t ns_ble_slave_connection_num(void);


void ns_ble_prod_test_cmd_send(struct gapm_le_test_mode_ctrl_cmd *p_params, bool cw_mode);
/// @} APP
///

#endif //(BLE_APP_PRESENT)

#endif // _NS_BLE_H_
