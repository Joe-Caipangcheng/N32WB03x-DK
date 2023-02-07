#ifndef PROV_UTILS_H__
#define PROV_UTILS_H__

#include <stdbool.h>
#include <stdint.h>
#include "mesh.h"
#include "mesh_prov.h"
#include "provisioning.h"



#define PROV_CONFIRM_INPUTS_INVITE_OFFSET   (0)
#define PROV_CONFIRM_INPUTS_CAPS_OFFSET     (PROV_CONFIRM_INPUTS_INVITE_OFFSET + sizeof(prov_pdu_invite_t) - 1)
#define PROV_CONFIRM_INPUTS_START_OFFSET    (PROV_CONFIRM_INPUTS_CAPS_OFFSET + sizeof(prov_pdu_caps_t) - 1)




bool prov_utils_use_ecdh_offloading(void);
void prov_utils_authentication_values_derive(const mesh_prov_ctx_t * p_ctx,
        uint8_t * p_confirmation_salt,
        uint8_t * p_confirmation,
        uint8_t * p_random);
uint32_t prov_utils_keys_generate(uint8_t * p_public, uint8_t * p_private);
void prov_utils_derive_keys(const mesh_prov_ctx_t * p_ctx,
        uint8_t * p_session_key,
        uint8_t * p_session_nonce,
        uint8_t * p_device_key);
uint32_t prov_utils_calculate_shared_secret(const mesh_prov_ctx_t * p_ctx, uint8_t * p_shared_secret);
void prov_utils_generate_oob_data(const mesh_prov_ctx_t * p_ctx, uint8_t * p_auth_value);
bool prov_utils_confirmation_check(const mesh_prov_ctx_t * p_ctx);
bool prov_utils_auth_data_is_alphanumeric(const uint8_t * p_data, uint8_t size);
bool prov_utils_auth_data_is_valid_number(const uint8_t * p_data, uint8_t size);
bool prov_utils_is_valid_pdu(mesh_prov_role_t role, mesh_prov_state_t state, prov_pdu_type_t pdu_type);
bool prov_utils_is_valid_public_key(const uint8_t * p_public_key);


#endif
