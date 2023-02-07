#ifndef ACCESS_PUBLISH_RETRANSMISSION_H__
#define ACCESS_PUBLISH_RETRANSMISSION_H__

#include "access.h"


void access_publish_retransmission_init(void);
void access_publish_retransmission_message_add(access_model_handle_t model_handle,
                                               const access_publish_retransmit_t *p_publication_retransmit,
                                               const access_message_tx_t *p_tx_message,
                                               const uint8_t *p_access_payload,
                                               uint16_t access_payload_len);


#endif /* ACCESS_PUBLISH_RETRANSMISSION_H__ */
