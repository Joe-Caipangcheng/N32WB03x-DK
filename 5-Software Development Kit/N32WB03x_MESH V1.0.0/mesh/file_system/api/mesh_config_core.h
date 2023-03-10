#ifndef MESH_CONFIG_CORE_H__
#define MESH_CONFIG_CORE_H__


#ifndef PERSISTENT_STORAGE
#define PERSISTENT_STORAGE 1
#endif

#ifndef MESH_UECC_ENABLE
#define MESH_UECC_ENABLE 1
#endif

#define FLASH_MANAGER_BACKEND

#ifndef AES_USE_HARDWARE
#define AES_USE_HARDWARE 1
#endif
#ifndef CORE_TX_QUEUE_BUFFER_SIZE_ORIGINATOR
#define CORE_TX_QUEUE_BUFFER_SIZE_ORIGINATOR 256
#endif

#ifndef CORE_TX_QUEUE_BUFFER_SIZE_RELAY
#define CORE_TX_QUEUE_BUFFER_SIZE_RELAY 512
#endif

#ifndef CORE_TX_QUEUE_BUFFER_SIZE_INSTABURST_ORIGINATOR
#define CORE_TX_QUEUE_BUFFER_SIZE_INSTABURST_ORIGINATOR 4096
#endif

#ifndef CORE_TX_QUEUE_BUFFER_SIZE_INSTABURST_RELAY
#define CORE_TX_QUEUE_BUFFER_SIZE_INSTABURST_RELAY 2048
#endif

#ifndef CORE_TX_INSTABURST_CHANNELS
#define CORE_TX_INSTABURST_CHANNELS                                                                \
    {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18,                   \
      19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36}
#endif

#ifndef CORE_TX_REPEAT_ORIGINATOR_DEFAULT
#define CORE_TX_REPEAT_ORIGINATOR_DEFAULT 3
#endif

#ifndef CORE_TX_REPEAT_RELAY_DEFAULT
#define CORE_TX_REPEAT_RELAY_DEFAULT 5
#endif

#ifndef MESH_FEATURE_RELAY_ENABLED
#define MESH_FEATURE_RELAY_ENABLED (1)
#endif

#ifndef CCM_DEBUG_MODE_ENABLED
#define CCM_DEBUG_MODE_ENABLED 1
#endif

#ifndef FIFO_STATS
#define FIFO_STATS 0
#endif

#ifndef INTERNAL_EVT_ENABLE
#define INTERNAL_EVT_ENABLE 0
#endif

#ifndef INTERNAL_EVENT_BUFFER_SIZE
#define INTERNAL_EVENT_BUFFER_SIZE 32
#endif

#ifndef MESH_LOG_ENABLE
#define MESH_LOG_ENABLE 1
#endif

#ifndef LOG_LEVEL_DEFAULT
#define LOG_LEVEL_DEFAULT LOG_LEVEL_WARN
#endif

#ifndef LOG_MSK_DEFAULT
#define LOG_MSK_DEFAULT LOG_GROUP_STACK
#endif

#ifndef LOG_ENABLE_RTT
#define LOG_ENABLE_RTT 1
#endif

#define LOG_CALLBACK_DEFAULT log_callback_rtt


#ifndef MSG_CACHE_ENTRY_COUNT
#define MSG_CACHE_ENTRY_COUNT 32
#endif

#ifndef NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD
#define NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD (NETWORK_SEQNUM_MAX / 2)
#endif

#ifndef NETWORK_SEQNUM_IV_UPDATE_END_THRESHOLD
#define NETWORK_SEQNUM_IV_UPDATE_END_THRESHOLD (NETWORK_SEQNUM_MAX - TRANSPORT_SAR_SEQNUM_DIFF_MAX)
#endif

#ifndef NETWORK_SEQNUM_FLASH_BLOCK_SIZE
#define NETWORK_SEQNUM_FLASH_BLOCK_SIZE 8192ul
#endif

#ifndef NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD
#define NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD 64
#endif

#ifndef TRANSPORT_SAR_SESSIONS_MAX
#define TRANSPORT_SAR_SESSIONS_MAX (4)
#endif

#ifndef TRANSPORT_CANCELED_SAR_RX_SESSIONS_CACHE_LEN
#define TRANSPORT_CANCELED_SAR_RX_SESSIONS_CACHE_LEN (8)
#endif

#ifndef TRANSPORT_SAR_SEGACK_TTL_DEFAULT
#define TRANSPORT_SAR_SEGACK_TTL_DEFAULT (8)
#endif

#ifndef PACKET_MGR_DEBUG_MODE
#define PACKET_MGR_DEBUG_MODE 0
#endif

#ifndef PACKET_MGR_MEMORY_POOL_SIZE
#define PACKET_MGR_MEMORY_POOL_SIZE 4096
#endif

#ifndef PACKET_MGR_BLAME_MODE
#define PACKET_MGR_BLAME_MODE 0
#endif

#ifndef PACKET_BUFFER_DEBUG_MODE
#define PACKET_BUFFER_DEBUG_MODE 0
#endif

#ifndef REPLAY_CACHE_ENTRIES
#define REPLAY_CACHE_ENTRIES 40
#endif

#ifndef FLASH_MANAGER_PAGE_COUNT_MAX
#define FLASH_MANAGER_PAGE_COUNT_MAX 255
#endif

#ifndef FLASH_MANAGER_POOL_SIZE
#define FLASH_MANAGER_POOL_SIZE 256
#endif

#ifndef FLASH_MANAGER_ENTRY_MAX_SIZE
#define FLASH_MANAGER_ENTRY_MAX_SIZE 128
#endif

#ifndef FLASH_MANAGER_RECOVERY_PAGE_OFFSET_PAGES
#define FLASH_MANAGER_RECOVERY_PAGE_OFFSET_PAGES 0
#endif



#ifndef MESH_GATT_PROXY_FILTER_ADDR_COUNT
#define MESH_GATT_PROXY_FILTER_ADDR_COUNT 32
#endif

#ifndef MESH_GATT_PROXY_NETWORK_ID_ADV_INT_MS
#define MESH_GATT_PROXY_NETWORK_ID_ADV_INT_MS 1000
#endif

#ifndef MESH_GATT_PROXY_NODE_IDENTITY_ADV_INT_MS
#define MESH_GATT_PROXY_NODE_IDENTITY_ADV_INT_MS 200
#endif

#ifndef MESH_GATT_PROXY_NODE_IDENTITY_DURATION_MS
#define MESH_GATT_PROXY_NODE_IDENTITY_DURATION_MS 60000
#endif

#ifndef MESH_GATT_PROXY_BEACON_CACHE_SIZE
#define MESH_GATT_PROXY_BEACON_CACHE_SIZE 8
#endif

#ifndef ACCESS_MODEL_PUBLISH_PERIOD_RESTORE
#define ACCESS_MODEL_PUBLISH_PERIOD_RESTORE 0
#endif

#ifndef FSM_DEBUG
#define FSM_DEBUG (0)
#endif

#ifndef MESH_FEATURE_LPN_ENABLED
#define MESH_FEATURE_LPN_ENABLED 0
#endif

#ifndef MESH_FEATURE_LPN_ACT_AS_REGULAR_NODE_OUT_OF_FRIENDSHIP
#define MESH_FEATURE_LPN_ACT_AS_REGULAR_NODE_OUT_OF_FRIENDSHIP 0
#endif



#ifndef MESH_FRIEND_FRIENDSHIP_COUNT
#define MESH_FRIEND_FRIENDSHIP_COUNT 2
#endif

#ifndef MESH_FRIEND_SUBLIST_SIZE
#define MESH_FRIEND_SUBLIST_SIZE 16
#endif

#ifndef MESH_FRIEND_QUEUE_SIZE
#define MESH_FRIEND_QUEUE_SIZE 35
#endif


#endif
