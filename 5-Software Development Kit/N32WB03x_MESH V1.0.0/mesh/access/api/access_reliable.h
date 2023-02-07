#ifndef ACCESS_RELIABLE_H__
#define ACCESS_RELIABLE_H__

#include <stdint.h>
#include "access.h"

#include "utils.h"

#define BEARER_ADV_INT_DEFAULT_MS 40
#define ACCESS_RELIABLE_TIMEOUT_MIN  (SEC_TO_US(2))
#define ACCESS_RELIABLE_TIMEOUT_MAX  (SEC_TO_US(60))
#define ACCESS_RELIABLE_HOP_PENALTY (MS_TO_US(BEARER_ADV_INT_DEFAULT_MS))
#define ACCESS_RELIABLE_SEGMENT_COUNT_PENALTY MS_TO_US(BEARER_ADV_INT_DEFAULT_MS)
#define ACCESS_RELIABLE_INTERVAL_DEFAULT (MS_TO_US(BEARER_ADV_INT_DEFAULT_MS) * 10)
#define ACCESS_RELIABLE_BACK_OFF_FACTOR (2)
#define ACCESS_RELIABLE_TIMEOUT_MARGIN (MS_TO_US(1))
#define ACCESS_RELIABLE_RETRY_DELAY (MS_TO_US(BEARER_ADV_INT_DEFAULT_MS) * 2)

typedef enum
{
    ACCESS_RELIABLE_TRANSFER_SUCCESS,
    ACCESS_RELIABLE_TRANSFER_TIMEOUT,
    ACCESS_RELIABLE_TRANSFER_CANCELLED
} access_reliable_status_t;

typedef void (*access_reliable_cb_t)(access_model_handle_t model_handle,
                                     void * p_args,
                                     access_reliable_status_t status);

typedef struct
{
    access_model_handle_t model_handle;
    access_message_tx_t message;
    access_opcode_t reply_opcode;
    uint32_t timeout;
    access_reliable_cb_t status_cb;
} access_reliable_t;
void access_reliable_init(void);
void access_reliable_cancel_all(void);
uint32_t access_model_reliable_publish(const access_reliable_t * p_reliable);
uint32_t access_model_reliable_cancel(access_model_handle_t model_handle);
void access_reliable_message_rx_cb(access_model_handle_t model_handle, const access_message_rx_t * p_message, void * p_args);
bool access_reliable_model_is_free(access_model_handle_t model_handle);

#endif /* ACCESS_RELIABLE_H__ */
