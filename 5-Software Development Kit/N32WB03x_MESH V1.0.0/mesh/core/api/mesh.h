#ifndef MESH_H__
#define MESH_H__

#include <stdint.h>
#include <stdbool.h>
#include "mesh_defines.h"
#include "co_timer_schedule.h"



#define MESH_INITIAL_TOKEN          0x00000000ul
#define MESH_SERVICE_BORDER_TOKEN   0xF0000000ul


#define MESH_FRIEND_TOKEN_BEGIN     0xFFFFFE00ul
#define MESH_FRIEND_TOKEN_END       0xFFFFFEFFul

#define MESH_FRIEND_POLL_TOKEN      0xFFFFFFF8ul
#define MESH_FRIEND_REQUEST_TOKEN   0xFFFFFFF9ul
#define MESH_FRIEND_CLEAR_TOKEN     0xFFFFFFFAul
#define MESH_SUBMAN_ADD_TOKEN       0xFFFFFFFBul
#define MESH_SUBMAN_REMOVE_TOKEN    0xFFFFFFFCul
#define MESH_HEARTBEAT_TOKEN        0xFFFFFFFDul
#define MESH_SAR_TOKEN              0xFFFFFFFEul
#define MESH_RELAY_TOKEN            0xFFFFFFFFul


typedef bool (*mesh_relay_check_cb_t)(uint16_t src, uint16_t dst, uint8_t ttl);
typedef uint32_t mesh_tx_token_t;


typedef struct
{
    mesh_relay_check_cb_t relay_cb; 
    uint8_t irq_priority; 
    const uint8_t * p_uuid; 
} mesh_init_params_t;



typedef enum
{
    MESH_ADDRESS_TYPE_INVALID,
    MESH_ADDRESS_TYPE_UNICAST,
    MESH_ADDRESS_TYPE_VIRTUAL,
    MESH_ADDRESS_TYPE_GROUP,
} mesh_address_type_t;

typedef struct
{
    mesh_address_type_t type;
    uint16_t value;
    const uint8_t * p_virtual_uuid;
} mesh_address_t;





typedef struct
{
    uint8_t nid;
    uint8_t encryption_key[MESH_KEY_SIZE];
    uint8_t privacy_key[MESH_KEY_SIZE];
} mesh_network_secmat_t;




typedef enum
{
    MESH_KEY_REFRESH_PHASE_0, 
    MESH_KEY_REFRESH_PHASE_1,
    MESH_KEY_REFRESH_PHASE_2,
    MESH_KEY_REFRESH_PHASE_3,
} mesh_key_refresh_phase_t;


typedef enum
{
    NET_STATE_IV_UPDATE_NORMAL,
    NET_STATE_IV_UPDATE_IN_PROGRESS,
} net_state_iv_update_t;



#define BLE_GAP_ADDR_LEN (6)
typedef struct
{
  uint8_t addr_id_peer : 1;       
  uint8_t addr_type    : 7;       
  uint8_t addr[BLE_GAP_ADDR_LEN]; 
} ble_gap_addr_t;

typedef struct
{
    bool is_device_key;
    uint8_t aid;
    uint8_t key[MESH_KEY_SIZE];
} mesh_application_secmat_t;
typedef struct
{
    const mesh_network_secmat_t * p_net;
    const mesh_application_secmat_t * p_app;
} mesh_secmat_t;


typedef enum
{
    MESH_RX_SOURCE_SCANNER, 
    MESH_RX_SOURCE_GATT,    
    MESH_RX_SOURCE_INSTABURST,  
    MESH_RX_SOURCE_LOOPBACK,  
} mesh_rx_source_t;
typedef struct
{
    timestamp_t timestamp; 
    uint32_t access_addr; 
    uint8_t  channel; 
    int8_t   rssi; 
    ble_gap_addr_t adv_addr; 
    uint8_t adv_type;  
} mesh_rx_metadata_scanner_t;
typedef struct
{
    uint16_t data_id; 
    uint8_t set_id;  
} mesh_instaburst_event_id_t;
typedef struct
{
    timestamp_t timestamp; 
    uint8_t  channel; 
    int8_t   rssi; 
    struct
    {
        mesh_instaburst_event_id_t id; 
        uint8_t packet_index; 
        bool is_last_in_chain; 
    } event;
} mesh_rx_metadata_instaburst_t;
typedef struct
{
    timestamp_t timestamp; 
    uint16_t connection_index; 
} mesh_rx_metadata_gatt_t;
typedef struct
{
    mesh_tx_token_t tx_token; 
} mesh_rx_metadata_loopback_t;
typedef struct
{
    mesh_rx_source_t source; 
    union
    {
        mesh_rx_metadata_scanner_t scanner;
        mesh_rx_metadata_instaburst_t instaburst;
        mesh_rx_metadata_gatt_t gatt;
        mesh_rx_metadata_loopback_t loopback;
    } params;
} mesh_rx_metadata_t;


typedef struct
{
    uint8_t key[MESH_KEY_SIZE];
    uint8_t net_id[MESH_NETID_SIZE];
    uint8_t identity_key[MESH_KEY_SIZE];
} mesh_beacon_secmat_t;



typedef struct
{
    uint16_t rx_count;
    uint8_t observation_count;
    uint16_t interval;
    timestamp_t tx_timestamp;
} mesh_beacon_tx_info_t;

typedef struct
{
    bool iv_update_permitted;
    mesh_beacon_tx_info_t * p_tx_info;
    mesh_beacon_secmat_t secmat;
    mesh_beacon_secmat_t secmat_updated;
} mesh_beacon_info_t;

typedef enum
{
    MESH_TRANSMIC_SIZE_SMALL,
    MESH_TRANSMIC_SIZE_LARGE,
    MESH_TRANSMIC_SIZE_DEFAULT,
    MESH_TRANSMIC_SIZE_INVALID
} mesh_transmic_size_t;

typedef struct
{
    mesh_address_t dst;
    uint16_t src;
    uint8_t ttl;
    bool force_segmented;
    mesh_transmic_size_t transmic_size;
    const uint8_t * p_data;
    uint16_t data_len;
    mesh_secmat_t security_material;
    mesh_tx_token_t tx_token;
} mesh_tx_params_t;










bool mesh_is_address_rx(const mesh_address_t * p_addr);
bool mesh_is_device_provisioned(void);
void mesh_unicast_address_get(uint16_t * p_addr_start, uint16_t * p_addr_count);
void mesh_app_secmat_next_get(const mesh_network_secmat_t * p_network_secmat, uint8_t aid, const mesh_application_secmat_t ** pp_app_secmat);
void mesh_net_secmat_next_get(uint8_t nid, const mesh_network_secmat_t ** pp_secmat, const mesh_network_secmat_t ** pp_secmat_secondary);


uint32_t mesh_disable(void);
uint32_t mesh_enable(void);
uint32_t mesh_init(const mesh_init_params_t * p_init_params);

void mesh_beacon_info_next_get(const uint8_t * p_network_id, const mesh_beacon_info_t ** pp_beacon_info,
        mesh_key_refresh_phase_t * p_kr_phase);

bool mesh_rx_address_get(uint16_t raw_address, mesh_address_t * p_address);


uint32_t mesh_packet_send(const mesh_tx_params_t * p_params,
                              uint32_t * const p_packet_reference);
mesh_tx_token_t mesh_unique_token_get(void);


void mesh_stack_init(void (*model_init)(void));
void mesh_stack_enable(uint32_t scan_delay);


#endif  /* MESH_H__ */
