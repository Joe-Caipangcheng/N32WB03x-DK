#ifndef ACCESS_INTERNAL_H__
#define ACCESS_INTERNAL_H__

#include <stdint.h>
#include "bitfield.h"
#include "device_state_manager.h"
#include "utils.h"
#include "access.h"
#include "access_publish.h"


#define ACCESS_PACKET_OPCODE_FORMAT_MASK   (0xC0)
#define ACCESS_PACKET_OPCODE_FORMAT_1BYTE0 (0x00)
#define ACCESS_PACKET_OPCODE_FORMAT_1BYTE1 (0x40)
#define ACCESS_PACKET_OPCODE_FORMAT_2BYTE  (0x80)
#define ACCESS_PACKET_OPCODE_FORMAT_3BYTE  (0xC0)
#define ACCESS_OPCODE_INVALID              (0x7F)

#define ACCESS_INTERNAL_STATE_ALLOCATED   (1 << 0)
#define ACCESS_INTERNAL_STATE_REFRESHED   (1 << 1)
#define ACCESS_INTERNAL_STATE_INVALIDATED (1 << 2)
#define ACCESS_INTERNAL_STATE_ALLOCATED_SET(INTERNAL_STATE) (INTERNAL_STATE |= ACCESS_INTERNAL_STATE_ALLOCATED)
#define ACCESS_INTERNAL_STATE_ALLOCATED_CLR(INTERNAL_STATE) (INTERNAL_STATE &= ~ACCESS_INTERNAL_STATE_ALLOCATED)
#define ACCESS_INTERNAL_STATE_IS_ALLOCATED(INTERNAL_STATE)  ((bool)((INTERNAL_STATE) & ACCESS_INTERNAL_STATE_ALLOCATED))
#define ACCESS_INTERNAL_STATE_REFRESHED_SET(INTERNAL_STATE) (INTERNAL_STATE |= ACCESS_INTERNAL_STATE_REFRESHED)
#define ACCESS_INTERNAL_STATE_REFRESHED_CLR(INTERNAL_STATE) (INTERNAL_STATE &= ~ACCESS_INTERNAL_STATE_REFRESHED)
#define ACCESS_INTERNAL_STATE_IS_REFRESHED(INTERNAL_STATE)  ((bool)((INTERNAL_STATE) & ACCESS_INTERNAL_STATE_REFRESHED))
#define ACCESS_INTERNAL_STATE_INVALIDATED_SET(INTERNAL_STATE) (INTERNAL_STATE |= ACCESS_INTERNAL_STATE_INVALIDATED)
#define ACCESS_INTERNAL_STATE_INVALIDATED_CLR(INTERNAL_STATE) (INTERNAL_STATE &= ~ACCESS_INTERNAL_STATE_INVALIDATED)
#define ACCESS_INTERNAL_STATE_IS_INVALIDATED(INTERNAL_STATE)  ((bool)((INTERNAL_STATE) & ACCESS_INTERNAL_STATE_INVALIDATED))

typedef struct
{
    uint8_t internal_state; 
    uint32_t bitfield[BITFIELD_BLOCK_COUNT(DSM_ADDR_MAX)];
} access_subscription_list_t;

typedef struct
{
    uint16_t location;
    uint8_t sig_model_count;
    uint8_t vendor_model_count;
    uint8_t attention_timer;
    uint8_t internal_state;
} access_element_t;

typedef struct
{
    access_model_id_t model_id;
    dsm_handle_t publish_appkey_handle;
    dsm_handle_t publish_address_handle;
    uint32_t application_keys_bitfield[BITFIELD_BLOCK_COUNT(DSM_APP_MAX + DSM_DEVICE_MAX)];
    uint16_t element_index;
    uint16_t subscription_pool_index;
    bool friendship_credential_flag;
    uint8_t publish_ttl;
    access_publish_period_t publication_period;
    access_publish_retransmit_t publication_retransmit;
} access_model_state_data_t;

typedef struct
{
    access_model_state_data_t model_info;
    access_model_publication_state_t publication_state;
    uint16_t publish_divisor;
    const access_opcode_handler_t * p_opcode_handlers;
    uint16_t opcode_count;
    void  * p_args;
    uint8_t internal_state;
} access_common_t;

typedef struct
{
    uint16_t subscription_list_count;
    uint16_t element_count;
    uint16_t model_count;
} access_flash_metadata_t;

typedef struct
{
    uint32_t inverted_bitfield[BITFIELD_BLOCK_COUNT(DSM_ADDR_MAX)];
} access_flash_subscription_list_t;







#endif /* ACCESS_INTERNAL_H__ */
