#ifndef PROVISIONING_H__
#define PROVISIONING_H__
#include "prov_pdu.h"
#include "mesh_prov.h"
#include "mesh_prov_types.h"
#include "utils.h"
#include "mesh.h"
#include "mesh_utils.h"

static inline mesh_prov_ctx_t * prov_bearer_ctx_get(prov_bearer_t * p_bearer)
{
    return (mesh_prov_ctx_t *) p_bearer->p_parent;
}
static inline bool prov_data_is_valid(const mesh_prov_provisioning_data_t * p_data)
{
    return (p_data->netkey_index <= MESH_GLOBAL_KEY_INDEX_MAX &&
            mesh_address_type_get(p_data->address) == MESH_ADDRESS_TYPE_UNICAST);
}
static inline bool prov_address_is_valid(const mesh_prov_provisioning_data_t * p_data, uint8_t num_elements)
{
    return (mesh_address_type_get(p_data->address) == MESH_ADDRESS_TYPE_UNICAST &&
            mesh_address_type_get(p_data->address + num_elements - 1) == MESH_ADDRESS_TYPE_UNICAST);
}
bool prov_packet_length_valid(const uint8_t * p_buffer, uint16_t length);
uint32_t prov_tx_public_key(prov_bearer_t * p_bearer, const uint8_t * p_public_key);
uint32_t prov_tx_confirmation(prov_bearer_t * p_bearer, const uint8_t * p_confirmation_value);
uint32_t prov_tx_random(prov_bearer_t * p_bearer, const uint8_t * p_random);
uint32_t prov_tx_invite(prov_bearer_t * p_bearer, uint8_t attention_duration_s, uint8_t * p_confirmation_inputs);
uint32_t prov_tx_input_complete(prov_bearer_t * p_bearer);
uint32_t prov_tx_complete(prov_bearer_t * p_bearer);
uint32_t prov_tx_failed(prov_bearer_t * p_bearer, mesh_prov_failure_code_t failure_code);
uint32_t prov_tx(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length);

uint32_t prov_tx_start(prov_bearer_t * p_bearer, const prov_pdu_prov_start_t * p_start, uint8_t * p_confirmation_inputs);
uint32_t prov_tx_data(prov_bearer_t * p_bearer, const prov_pdu_data_t * p_data);
uint32_t prov_tx_capabilities(prov_bearer_t * p_bearer, const prov_pdu_caps_t * p_caps, uint8_t * p_confirmation_inputs);

#endif /* PROVISIONING_H__ */
