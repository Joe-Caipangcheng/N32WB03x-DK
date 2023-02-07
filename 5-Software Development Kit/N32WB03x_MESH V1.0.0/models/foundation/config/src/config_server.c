#include <stdint.h>
#include <string.h>
#include "event.h"
#include "config_server.h"
#include "access.h"
#include "stdio.h"
#include "mesh_opt_core.h"
#include "mesh_utils.h"
#include "access_config.h"
#include "access.h"
#include "config_opcodes.h"
#include "composition_data.h"
#include "config_messages.h"
#include "mesh.h"
#include "bearer_defines.h"
#include "access_status.h"
#include "dsm_appkey.h"
#include "core_tx_adv_config.h"
#include "proxy.h"
#include "device_state_manager.h"
#include "heartbeat.h"
#include "packed_index_list.h"

#define CONFIG_SERVER_MODEL_ID  0x0000


#define IS_PACKET_LENGTH_VALID_WITH_ID(packet_type, p_packet) \
    ((p_packet)->length == sizeof(packet_type) || (p_packet)->length == sizeof(packet_type) - sizeof(uint16_t))
        
#define PACKET_LENGTH_WITH_ID(packet_type, sig_model) \
    (sizeof(packet_type) - ((sig_model) ? sizeof(uint16_t) : 0))  


static access_model_handle_t m_config_server_handle = ACCESS_HANDLE_INVALID;
static void mesh_event_cb(const mesh_evt_t * p_evt);
static mesh_evt_handler_t m_mesh_evt_handler = { .evt_cb = mesh_event_cb };
static config_server_evt_cb_t m_evt_cb;
static mesh_tx_token_t m_reset_token;

typedef enum
{
    NODE_RESET_IDLE,
    NODE_RESET_PENDING,
    NODE_RESET_PENDING_PROXY,
    NODE_RESET_FLASHING,
} node_reset_state_t;
static node_reset_state_t m_node_reset_pending = NODE_RESET_IDLE;






static inline void app_evt_send(const config_server_evt_t * p_evt)
{
    if (m_evt_cb)
    {
        m_evt_cb(p_evt);
    }
}




static void send_reply(access_model_handle_t handle, const access_message_rx_t * p_message, uint16_t opcode,
        const uint8_t * p_reply, uint16_t reply_length, uint32_t tx_token)
{
    const access_message_tx_t reply =
    {
        .opcode =
        {
            .opcode = opcode,
            .company_id = ACCESS_COMPANY_ID_NONE
        },
        .p_buffer = p_reply,
        .length = reply_length,
        .access_token = (mesh_tx_token_t)tx_token,
        .force_segmented = false,
        .transmic_size = MESH_TRANSMIC_SIZE_DEFAULT
    };
    core_tx_type_set(CORE_TX_TYPE_PROV);
    

    
    (void) access_model_reply(handle, p_message, &reply);
}







static void handle_config_default_ttl_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != 0)
    {
        return;
    }
    uint8_t ttl = access_default_ttl_get();
    send_reply(handle, p_message, CONFIG_OPCODE_DEFAULT_TTL_STATUS, (const uint8_t *) &ttl, sizeof(ttl), mesh_unique_token_get());
}

static void handle_composition_data_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_composition_data_get_t))
    {
        return;
    }
    uint8_t buffer[sizeof(config_msg_composition_data_status_t) + CONFIG_COMPOSITION_DATA_SIZE];
    config_msg_composition_data_status_t * p_response = (config_msg_composition_data_status_t *) buffer;
    p_response->page_number = 0;
    uint16_t size = CONFIG_COMPOSITION_DATA_SIZE;
    config_composition_data_get(p_response->data, &size); 
    send_reply(handle, p_message, CONFIG_OPCODE_COMPOSITION_DATA_STATUS, buffer,
               sizeof(config_msg_composition_data_status_t) + size, mesh_unique_token_get());    
}
static void send_network_transmit_status(access_model_handle_t handle, const access_message_rx_t * p_message)
{
    
    config_msg_network_transmit_status_t status_message = { 0 };
    mesh_opt_core_adv_t net;
    mesh_opt_core_adv_get(CORE_TX_ROLE_ORIGINATOR,&net);
    status_message.network_transmit_count = 2;
    status_message.network_transmit_interval_steps = 1;
    send_reply(handle, p_message, CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), mesh_unique_token_get());
}
static void handle_config_network_transmit_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_network_transmit_set_t * p_pdu = (const config_msg_network_transmit_set_t *) p_message->p_data;
    if (p_message->length != sizeof(config_msg_network_transmit_set_t))
    {
        return;
    }
    mesh_opt_core_adv_t net;
    net.enabled = true;
    net.tx_count = 3;
    net.tx_interval_ms = BEARER_ADV_INT_MIN_MS;   
    mesh_opt_core_adv_set(CORE_TX_ROLE_ORIGINATOR, &net);
    send_network_transmit_status(handle, p_message);
    
    const config_server_evt_t evt = {
        .type = CONFIG_SERVER_EVT_NETWORK_TRANSMIT_SET,
        .params.network_transmit_set.retransmit_count = p_pdu->network_transmit_count,
        .params.network_transmit_set.interval_steps = p_pdu->network_transmit_interval_steps
    };
    app_evt_send(&evt);
}
static void send_appkey_status(access_model_handle_t handle,
                               const access_message_rx_t * p_message,
                               access_status_t status,
                               config_msg_key_index_24_t key_indexes)
{
    const config_msg_appkey_status_t response =
    {
        .status = status,
        .key_indexes = key_indexes
    }; 
    send_reply(handle, p_message, CONFIG_OPCODE_APPKEY_STATUS,
               (const uint8_t *) &response, sizeof(response), mesh_unique_token_get());
}


static void handle_appkey_add(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
//    printf("handle_appkey_add\r\n");
    if (p_message->length != sizeof(config_msg_appkey_add_t))
    {
        return;
    }
    const config_msg_appkey_add_t * p_pdu = (const config_msg_appkey_add_t *) p_message->p_data;
    config_msg_key_index_24_t key_indexes = p_pdu->key_indexes;

    uint16_t netkey_index, appkey_index;
    config_msg_key_index_24_get(&key_indexes, &netkey_index, &appkey_index);

    uint32_t status;
    access_status_t status_code;
    config_server_evt_t evt;

    dsm_handle_t network_handle = dsm_net_key_index_to_subnet_handle(netkey_index);
    evt.type = CONFIG_SERVER_EVT_APPKEY_ADD;

    status = dsm_appkey_add(appkey_index, network_handle, p_pdu->appkey, &evt.params.appkey_add.appkey_handle);
   
    switch (status)
    {
        case MESH_SUCCESS:
            status_code = ACCESS_STATUS_SUCCESS;
            break;
        case MESH_ERROR_INTERNAL:
        {
            dsm_handle_t related_subnet;
            dsm_appkey_handle_to_subnet_handle(evt.params.appkey_add.appkey_handle,&related_subnet);
            status_code = related_subnet == network_handle ? ACCESS_STATUS_SUCCESS :
                                                             ACCESS_STATUS_INVALID_NETKEY;
            break;
        }
        case MESH_ERROR_INVALID_PARAM:
            status_code = ACCESS_STATUS_INVALID_APPKEY;
            break;
        case MESH_ERROR_FORBIDDEN:
            status_code = ACCESS_STATUS_KEY_INDEX_ALREADY_STORED;
            break;
        case MESH_ERROR_NO_MEM:
            status_code = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
            break;
        case MESH_ERROR_NOT_FOUND:
            status_code = ACCESS_STATUS_INVALID_NETKEY;
            break;
        default:
            status_code = ACCESS_STATUS_UNSPECIFIED_ERROR;
            break;
    }

    send_appkey_status(handle, p_message, status_code, key_indexes);

    if (status == MESH_SUCCESS)
    {
        app_evt_send(&evt);
    }
}



static inline bool model_id_extract(access_model_id_t * p_dest,
                             const config_model_id_t * p_id,
                             const access_message_rx_t * p_incoming,
                             uint16_t msg_size)
{
    bool sig_model = (p_incoming->length == ((msg_size) - sizeof(uint16_t)));

    if (sig_model)
    {
        p_dest->model_id = p_id->sig.model_id;
        p_dest->company_id = ACCESS_COMPANY_ID_NONE;
    }
    else
    {
        p_dest->model_id = p_id->vendor.model_id;
        p_dest->company_id = p_id->vendor.company_id;
    }

    return sig_model;
}
static inline uint16_t get_element_index(uint16_t element_address)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);

    if (element_address < node_address.address_start)
    {
        return ACCESS_ELEMENT_INDEX_INVALID;
    }

    uint16_t retval = element_address - node_address.address_start;
    if (retval >= (uint16_t) ACCESS_ELEMENT_COUNT)
    {
        return ACCESS_ELEMENT_INDEX_INVALID;
    }
    else
    {
        return retval;
    }
}
static void handle_model_app_bind_unbind(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
//    printf("handle_model_app_bind_unbind\r\n");
    
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_app_bind_unbind_t, p_message))
    {
        return;
    }

    const config_msg_app_bind_unbind_t * p_pdu = (const config_msg_app_bind_unbind_t *) p_message->p_data;
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_app_bind_unbind_t));

    config_msg_app_status_t response;
    response.status = ACCESS_STATUS_SUCCESS;
    response.element_address = p_pdu->element_address;
    response.appkey_index = p_pdu->appkey_index;
    response.model_id = p_pdu->model_id;

    access_model_handle_t model_handle;

    if (sig_model && model_id.model_id == CONFIG_SERVER_MODEL_ID)
    {
        response.status = ACCESS_STATUS_CANNOT_BIND;
        send_reply(handle, p_message, CONFIG_OPCODE_MODEL_APP_STATUS, (const uint8_t *) &response,
                PACKET_LENGTH_WITH_ID(config_msg_app_status_t, sig_model), mesh_unique_token_get());
        return;
    }

    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (ACCESS_ELEMENT_INDEX_INVALID == element_index)
    {
        response.status = ACCESS_STATUS_INVALID_ADDRESS;
        send_reply(handle, p_message, CONFIG_OPCODE_MODEL_APP_STATUS, (const uint8_t *) &response,
                PACKET_LENGTH_WITH_ID(config_msg_app_status_t, sig_model), mesh_unique_token_get());
        return;
    }
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != MESH_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        response.status = ACCESS_STATUS_INVALID_MODEL;
        send_reply(handle, p_message, CONFIG_OPCODE_MODEL_APP_STATUS, (const uint8_t *) &response,
                PACKET_LENGTH_WITH_ID(config_msg_app_status_t, sig_model), mesh_unique_token_get());
        return;
    }

    uint16_t appkey_index = p_pdu->appkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;
    dsm_handle_t appkey_handle = dsm_appkey_index_to_appkey_handle(appkey_index);
    config_server_evt_t evt;
    if (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_APP_BIND)
    {
        evt.type = CONFIG_SERVER_EVT_MODEL_APP_BIND;
        evt.params.model_app_bind.model_handle = model_handle;
        evt.params.model_app_bind.appkey_handle = appkey_handle;
        status = access_model_application_bind(model_handle, appkey_handle);
    }
    else
    {
        evt.type = CONFIG_SERVER_EVT_MODEL_APP_UNBIND;
        evt.params.model_app_unbind.model_handle = model_handle;
        evt.params.model_app_unbind.appkey_handle = appkey_handle;       
        status = access_model_application_unbind(model_handle, appkey_handle);
        if (status == MESH_SUCCESS)
        {
            dsm_handle_t active_appkey_handle;
            access_model_publish_application_get(model_handle, &active_appkey_handle);
            if (active_appkey_handle == appkey_handle)
            {
                access_model_publication_stop(model_handle);
            }
        }
    }
    switch (status)
    {
        case MESH_SUCCESS:
            response.status = ACCESS_STATUS_SUCCESS;
            break;
        case MESH_ERROR_NOT_FOUND:
            response.status = ACCESS_STATUS_INVALID_MODEL;
            break;
        case MESH_ERROR_INVALID_PARAM:
            response.status = ACCESS_STATUS_INVALID_APPKEY;
            break;
        default:
            response.status = ACCESS_STATUS_UNSPECIFIED_ERROR;
            break;
    }

    send_reply(handle, p_message, CONFIG_OPCODE_MODEL_APP_STATUS, (const uint8_t *) &response,
            PACKET_LENGTH_WITH_ID(config_msg_app_status_t, sig_model), mesh_unique_token_get());

    if (response.status == ACCESS_STATUS_SUCCESS)
    {
        app_evt_send(&evt);
    }
}
static void status_error_pub_send(access_model_handle_t this_handle, const access_message_rx_t * p_incoming, bool sig_model, uint8_t status_opcode)
{
    switch (p_incoming->opcode.opcode)
    {
        case CONFIG_OPCODE_MODEL_PUBLICATION_GET:
        case CONFIG_OPCODE_MODEL_PUBLICATION_SET:
        case CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET:
        {
            config_msg_publication_status_t response;

            memset(&response, 0, sizeof(config_msg_publication_status_t));
            response.status = status_opcode;

            switch(p_incoming->opcode.opcode)
            {
                case CONFIG_OPCODE_MODEL_PUBLICATION_GET:
                {
                    const config_msg_publication_get_t * p_pdu = (const config_msg_publication_get_t *) p_incoming->p_data;
                    response.element_address = p_pdu->element_address;
                    response.state.model_id = p_pdu->model_id;
                    break;
                }

                case CONFIG_OPCODE_MODEL_PUBLICATION_SET:
                {
                    const config_msg_publication_set_t * p_pdu = (const config_msg_publication_set_t *) p_incoming->p_data;
                    response.element_address = p_pdu->element_address;
                    response.state.model_id = p_pdu->state.model_id;
                    break;
                }

                case CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET:
                {
                    const config_msg_publication_virtual_set_t * p_pdu = (const config_msg_publication_virtual_set_t *) p_incoming->p_data;
                    response.element_address = p_pdu->element_address;
                    response.state.model_id = p_pdu->state.model_id;
                    break;
                }

                default:
                    break;
            }
            send_reply(this_handle, p_incoming, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, (const uint8_t *) &response,
                       PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, sig_model), mesh_unique_token_get());
            break;
        }
    }
}
static void send_publication_status(access_model_handle_t this_handle, const access_message_rx_t * p_incoming,
        uint16_t element_address, access_model_handle_t model_handle)
{
    config_msg_publication_status_t response;
    uint32_t status;
    bool credential_flag;

    memset(&response.state, 0, sizeof(response.state));
    response.status = ACCESS_STATUS_SUCCESS;
    response.element_address = element_address;
    access_model_publish_friendship_credential_flag_get(model_handle, &credential_flag);
    response.state.credential_flag = credential_flag;
    response.state.rfu = 0;
    access_model_id_t model_id;
    access_model_id_get(model_handle, &model_id);

    bool sig_model = model_id.company_id == ACCESS_COMPANY_ID_NONE;
    config_msg_model_id_set(&response.state.model_id, &model_id, sig_model);

    dsm_handle_t publish_address_handle;
    access_model_publish_address_get(model_handle, &publish_address_handle);

    if (publish_address_handle ==  DSM_HANDLE_INVALID)
    {
        response.publish_address = MESH_ADDR_UNASSIGNED;
    }
    else
    {
        mesh_address_t publish_address;
        dsm_address_get(publish_address_handle, &publish_address);
        response.publish_address = publish_address.value;

        dsm_handle_t appkey_handle;
        status = access_model_publish_application_get(model_handle, &appkey_handle);

        if (status == MESH_ERROR_NOT_FOUND)
        {
            response.publish_address = MESH_ADDR_UNASSIGNED;
        }
        else if (status == MESH_SUCCESS)
        {
            mesh_key_index_t appkey_index;
            status = dsm_appkey_handle_to_appkey_index(appkey_handle, &appkey_index);

            if (status == MESH_ERROR_NOT_FOUND)
            {
                response.publish_address = MESH_ADDR_UNASSIGNED;
            }
            else if (status == MESH_SUCCESS)
            {
                response.state.appkey_index = appkey_index;
                (void ) access_model_publish_ttl_get(model_handle, &response.state.publish_ttl);
                access_publish_resolution_t publish_resolution;
                uint8_t publish_steps;

                memset(&publish_resolution, 0, sizeof(access_publish_resolution_t));
                (void) access_model_publish_period_get(model_handle, &publish_resolution, &publish_steps);
                response.state.publish_period  = publish_resolution << ACCESS_PUBLISH_STEP_NUM_BITS | publish_steps;
                access_publish_retransmit_t publish_retransmit;
                (void) access_model_publish_retransmit_get(model_handle, &publish_retransmit);
                response.state.retransmit_count = publish_retransmit.count;             /* TODO: MBTLE-2388 */
                response.state.retransmit_interval = publish_retransmit.interval_steps; /* TODO: MBTLE-2388 */
            }
        }
    }
    send_reply(this_handle, p_incoming, CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, (const uint8_t *) &response,
            PACKET_LENGTH_WITH_ID(config_msg_publication_status_t, sig_model), mesh_unique_token_get());
}
static void handle_config_model_publication_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
//    printf("handle_config_model_publication_set\r\n");
    
    
    if (
            (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_SET &&
                !IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_publication_set_t, p_message))
            ||
            (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET &&
                !IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_publication_virtual_set_t, p_message))
       )
    {
        return;
    }


    bool sig_model;
    access_model_id_t model_id;

    uint16_t element_address, publish_address = MESH_ADDR_UNASSIGNED;
    const config_publication_params_t * p_pubstate;
    if (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_SET)
    {
        const config_msg_publication_set_t * p_pdu = (const config_msg_publication_set_t *) p_message->p_data;
        sig_model = model_id_extract(&model_id, &p_pdu->state.model_id, p_message, sizeof(config_msg_publication_set_t));
        element_address = p_pdu->element_address;
        p_pubstate = &p_pdu->state;
        publish_address = p_pdu->publish_address;
    }
    else
    {
        const config_msg_publication_virtual_set_t * p_pdu = (const config_msg_publication_virtual_set_t *) p_message->p_data;
        sig_model = model_id_extract(&model_id, &p_pdu->state.model_id, p_message, sizeof(config_msg_publication_virtual_set_t));
        element_address = p_pdu->element_address;
        p_pubstate = &p_pdu->state;
    }

    uint16_t element_index = get_element_index(element_address);

    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != MESH_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }
    dsm_handle_t publish_appkey_handle = dsm_appkey_index_to_appkey_handle(p_pubstate->appkey_index);
    if (publish_appkey_handle == DSM_HANDLE_INVALID)
    {
        status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_APPKEY);
        return;
    }
    dsm_handle_t publish_address_handle = DSM_HANDLE_INVALID;
    mesh_address_t publish_address_stored;
    mesh_address_type_t publish_addr_type = mesh_address_type_get(publish_address);
    if (p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_SET)
    {
        if (publish_addr_type == MESH_ADDRESS_TYPE_VIRTUAL)
        {
            status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
            return;
        }
        else if (publish_addr_type == MESH_ADDRESS_TYPE_UNICAST || publish_addr_type == MESH_ADDRESS_TYPE_GROUP)
        {
            if (access_model_publish_address_get(model_handle, &publish_address_handle) != MESH_SUCCESS)
            {
                status = dsm_address_publish_add(publish_address, &publish_address_handle);
            }
            else
            {
                if (dsm_address_get(publish_address_handle, &publish_address_stored) == MESH_SUCCESS)
                {

                    if ((publish_address_stored.type == MESH_ADDRESS_TYPE_VIRTUAL) ||
                        (publish_address_stored.type != MESH_ADDRESS_TYPE_VIRTUAL  &&
                         publish_address_stored.value != publish_address))
                    {
                        dsm_address_publish_remove(publish_address_handle);
                        status = dsm_address_publish_add(publish_address, &publish_address_handle);
                    }
                    else
                    {
                        
                    }
                }
                else
                {
                    status = dsm_address_publish_add(publish_address, &publish_address_handle);
                }
            }
        }
    }
    else
    {
        const uint8_t * publish_address_uuid = ((const config_msg_publication_virtual_set_t *) p_message->p_data)->publish_uuid;
        if (access_model_publish_address_get(model_handle, &publish_address_handle) != MESH_SUCCESS)
        {
            status = dsm_address_publish_virtual_add(publish_address_uuid, &publish_address_handle);
        }
        else
        {
            if (dsm_address_get(publish_address_handle, &publish_address_stored) == MESH_SUCCESS)
            {

                if ((publish_address_stored.type == MESH_ADDRESS_TYPE_VIRTUAL &&
                     memcmp(publish_address_stored.p_virtual_uuid, publish_address_uuid, MESH_UUID_SIZE) != 0) ||
                    (publish_address_stored.type != MESH_ADDRESS_TYPE_VIRTUAL))
                {
                    dsm_address_publish_remove(publish_address_handle);
                    status = dsm_address_publish_virtual_add(publish_address_uuid, &publish_address_handle);
                }
            }
            else
            {
                status = dsm_address_publish_virtual_add(publish_address_uuid, &publish_address_handle);
            }
        }
    }

    switch (status)
    {
        case MESH_ERROR_NO_MEM:
            status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INSUFFICIENT_RESOURCES);
            return;

        case MESH_SUCCESS:
            break;

        default:
            status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
            return;
    }

    if (publish_address != MESH_ADDR_UNASSIGNED || p_message->opcode.opcode == CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET)
    {
        access_publish_period_t publish_period;
        access_publish_retransmit_t publish_retransmit;
        publish_period.step_res = p_pubstate->publish_period >> ACCESS_PUBLISH_STEP_NUM_BITS;
        publish_period.step_num = p_pubstate->publish_period & ~(0xff << ACCESS_PUBLISH_STEP_NUM_BITS);
        publish_retransmit.count = p_pubstate->retransmit_count;
        publish_retransmit.interval_steps = p_pubstate->retransmit_interval;

        status = access_model_publish_period_set(model_handle, ACCESS_PUBLISH_RESOLUTION_100MS, 0);
        switch (status)
        {
            case MESH_SUCCESS:
                break;

            case MESH_ERROR_NOT_SUPPORTED:
                if (publish_period.step_num != 0)
                {
                    status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_NOT_A_PUBLISH_MODEL);
                    return;
                }
                break;

            default:
                return;
        }

        if (status == MESH_SUCCESS && publish_period.step_num != 0)
        {
            access_model_publish_period_set(model_handle, (access_publish_resolution_t) publish_period.step_res,publish_period.step_num);
        }

        access_model_publish_retransmit_set(model_handle, publish_retransmit);
        access_model_publish_address_set(model_handle, publish_address_handle);
        access_model_publish_application_set(model_handle, publish_appkey_handle);
        access_model_publish_friendship_credential_flag_set(model_handle, p_pubstate->credential_flag);
        access_model_publish_ttl_set(model_handle, p_pubstate->publish_ttl);
    }
    else
    {
        access_model_publication_stop(model_handle);
    }

    send_publication_status(handle, p_message, element_address, model_handle);

    const config_server_evt_t evt = {.type = CONFIG_SERVER_EVT_MODEL_PUBLICATION_SET,
                                     .params.model_publication_set.model_handle = model_handle};
    app_evt_send(&evt);
                                                      
}




static void status_error_sub_send(access_model_handle_t this_handle, const access_message_rx_t * p_incoming,
                                      bool sig_model, uint8_t status_opcode)
{
    switch (p_incoming->opcode.opcode)
    {
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_ADD:
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE:
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_OVERWRITE:
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD:
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE:
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE:
        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE_ALL:
        {
            config_msg_subscription_status_t response;

            memset(&response, 0, sizeof(config_msg_subscription_status_t));
            response.status = status_opcode;

            switch(p_incoming->opcode.opcode)
            {
                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_ADD:
                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE:
                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_OVERWRITE:
                {
                    const config_msg_subscription_add_del_owr_t * p_pdu = (const config_msg_subscription_add_del_owr_t *) p_incoming->p_data;
                    response.element_address = p_pdu->element_address;
                    response.address  = p_pdu->address;
                    response.model_id = p_pdu->model_id;
                    break;
                }

                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD:
                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE:
                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE:
                {
                    const config_msg_subscription_virtual_add_del_owr_t * p_pdu = (const config_msg_subscription_virtual_add_del_owr_t *) p_incoming->p_data;
                    response.element_address = p_pdu->element_address;
                    response.address = 0; 
                    response.model_id = p_pdu->model_id;
                    break;
                }

                case CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE_ALL:
                {
                    const config_msg_subscription_delete_all_t * p_pdu = (const config_msg_subscription_delete_all_t *) p_incoming->p_data;
                    response.element_address = p_pdu->element_address;
                    response.model_id = p_pdu->model_id;
                    break;
                }

                default:
                    
                    break;
            }

            send_reply(this_handle, p_incoming, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, (const uint8_t *) &response,
                       PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model), mesh_unique_token_get());
            break;
        }

        case CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_GET:
        {
            config_msg_sig_model_subscription_list_t response;

            memset(&response, 0, sizeof(config_msg_sig_model_subscription_list_t));
            response.status = status_opcode;

            const config_msg_model_subscription_get_t * p_pdu = (const config_msg_model_subscription_get_t *) p_incoming->p_data;
            response.element_address = p_pdu->element_address;
            response.sig_model_id    = p_pdu->model_id.sig.model_id;

            send_reply(this_handle, p_incoming, CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST, (const uint8_t *) &response,
                       sizeof(config_msg_sig_model_subscription_list_t), mesh_unique_token_get());
            break;
        }

        case CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_GET:
        {
            config_msg_vendor_model_subscription_list_t response;

            memset(&response, 0, sizeof(config_msg_vendor_model_subscription_list_t));
            response.status = status_opcode;

            const config_msg_model_subscription_get_t * p_pdu = (const config_msg_model_subscription_get_t *) p_incoming->p_data;
            response.element_address = p_pdu->element_address;
            response.vendor_model_id = p_pdu->model_id.vendor.model_id;
            response.vendor_company_id = p_pdu->model_id.vendor.company_id;

            send_reply(this_handle, p_incoming, CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST, (const uint8_t *) &response,
                       sizeof(config_msg_vendor_model_subscription_list_t), mesh_unique_token_get());
            break;
        }

    }
}
static void send_subscription_status(access_model_handle_t this_handle, const access_message_rx_t * p_message,
        uint16_t element_address, uint16_t subscription_address, config_model_id_t model_id, bool sig_model)
{
    config_msg_subscription_status_t packet;

    packet.status = ACCESS_STATUS_SUCCESS;
    packet.element_address = element_address;
    packet.address = subscription_address;
    packet.model_id = model_id;

    send_reply(this_handle, p_message, CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, (const uint8_t *) &packet,
            PACKET_LENGTH_WITH_ID(config_msg_subscription_status_t, sig_model), mesh_unique_token_get());
}
static void handle_config_model_subscription_add(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_add_del_owr_t, p_message))
    {
        return;
    }
    const config_msg_subscription_add_del_owr_t * p_pdu = (const config_msg_subscription_add_del_owr_t *) p_message->p_data;
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_subscription_add_del_owr_t));
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }
    mesh_address_type_t address_type = mesh_address_type_get(p_pdu->address);
    if ((address_type != MESH_ADDRESS_TYPE_GROUP) || (p_pdu->address == MESH_ALL_NODES_ADDR))
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != MESH_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }
    dsm_handle_t subscription_address_handle;
    status = dsm_address_subscription_add(p_pdu->address, &subscription_address_handle);
    if (status != MESH_SUCCESS)
    {
        access_status_t access_status;
        switch (status)
        {
            case MESH_ERROR_INVALID_ADDR:
                access_status = ACCESS_STATUS_INVALID_ADDRESS;
                break;
            case MESH_ERROR_NO_MEM:
                access_status = ACCESS_STATUS_INSUFFICIENT_RESOURCES;
                break;
            default:
                access_status = ACCESS_STATUS_UNSPECIFIED_ERROR;
                break;
        }
        status_error_sub_send(handle, p_message, sig_model, access_status);
        return;
    }
    status = access_model_subscription_add(model_handle, subscription_address_handle);
    if (status != MESH_SUCCESS)
    {
        dsm_address_subscription_remove(subscription_address_handle);

        if (status == MESH_ERROR_NOT_SUPPORTED)
        {
            status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL);
        }
        else
        {
            status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
        }
    }
    else
    {
        send_subscription_status(handle, p_message, p_pdu->element_address, p_pdu->address,
                p_pdu->model_id, sig_model);
        const config_server_evt_t evt = {
            .type = CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_ADD,
            .params.model_subscription_add.model_handle = model_handle,
            .params.model_subscription_add.address_handle = subscription_address_handle
        };
        app_evt_send(&evt);
    }
}
static void handle_node_reset(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
//    printf("handle_node_reset\r\n");
    m_reset_token = mesh_unique_token_get();
    m_node_reset_pending = NODE_RESET_PENDING;
    send_reply(handle, p_message, CONFIG_OPCODE_NODE_RESET_STATUS, NULL, 0, m_reset_token);
}


static void handle_config_default_ttl_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
//    printf("handle_config_default_ttl_set\r\n");
    const config_msg_default_ttl_set_t * p_pdu = (const config_msg_default_ttl_set_t *) p_message->p_data;

    if (p_message->length != sizeof(config_msg_default_ttl_set_t))
    {
        return;
    }

    uint8_t ttl = p_pdu->ttl;
    if (access_default_ttl_set(ttl) == MESH_SUCCESS)
    {
        const config_server_evt_t evt = {.type = CONFIG_SERVER_EVT_DEFAULT_TTL_SET,
                                         .params.default_ttl_set.default_ttl = ttl};
        app_evt_send(&evt);

        send_reply(handle, p_message, CONFIG_OPCODE_DEFAULT_TTL_STATUS, (const uint8_t *) &ttl, sizeof(ttl), mesh_unique_token_get());
    }
}

static void handle_heartbeat_publication_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    config_msg_heartbeat_publication_status_t status_message;
//    const heartbeat_publication_state_t * p_hb_pub = heartbeat_publication_get();
    if (p_message->length != 0)
    {
        return;
    }
    status_message.status       = ACCESS_STATUS_SUCCESS;
    status_message.destination  = 0;//p_hb_pub->dst;
    status_message.count_log    = 0;//heartbeat_publication_count_encode(p_hb_pub->count);
    status_message.period_log   = 0;//heartbeat_pubsub_period_encode(p_hb_pub->period);
    status_message.ttl          = 0;//p_hb_pub->ttl;
    status_message.features     = 0;//p_hb_pub->features;
    status_message.netkey_index = 0;//p_hb_pub->netkey_index;
 
    send_reply(handle, p_message, CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS,
              (const uint8_t *) &status_message, sizeof(status_message), mesh_unique_token_get());
}
static void handle_heartbeat_publication_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    config_msg_heartbeat_publication_status_t status_message;
    const config_msg_heartbeat_publication_set_t *p_pdu = (config_msg_heartbeat_publication_set_t *) p_message->p_data;

    if (p_message->length != sizeof(config_msg_heartbeat_publication_set_t))
    {
        return;
    }

    const heartbeat_publication_state_t hb_pub = {
        .dst          = p_pdu->destination,
        .count        = 0,//heartbeat_publication_count_decode(p_pdu->count_log),
        .period       = 0,//heartbeat_pubsub_period_decode(p_pdu->period_log),
        .ttl          = p_pdu->ttl,
        .features     = p_pdu->features & HEARTBEAT_TRIGGER_TYPE_RFU_MASK,
        .netkey_index = p_pdu->netkey_index
    };

    /* This is specifically required for INVALID_NETKEY status code */
    if (dsm_net_key_index_to_subnet_handle(p_pdu->netkey_index) == DSM_HANDLE_INVALID)
    {
        status_message.status  = ACCESS_STATUS_INVALID_NETKEY;
    }
    else
    {
//        status_message.status = (heartbeat_publication_set(&hb_pub) == MESH_SUCCESS)
//                                    ? ACCESS_STATUS_SUCCESS
//                                    : ACCESS_STATUS_CANNOT_SET;
    }

    status_message.destination  =  p_pdu->destination;
    status_message.period_log   =  p_pdu->period_log;
    status_message.count_log    =  p_pdu->count_log;
    status_message.ttl          =  p_pdu->ttl;
    status_message.features     =  p_pdu->features & HEARTBEAT_TRIGGER_TYPE_RFU_MASK;
    status_message.netkey_index =  p_pdu->netkey_index;

    send_reply(handle, p_message, CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS,
              (const uint8_t *) &status_message, sizeof(status_message), mesh_unique_token_get());

    if (status_message.status == ACCESS_STATUS_SUCCESS)
    {
        const config_server_evt_t evt = {
            .type = CONFIG_SERVER_EVT_HEARTBEAT_PUBLICATION_SET,
            .params.heartbeat_publication_set.p_publication_state = &hb_pub
        };
        app_evt_send(&evt);
    }
}
static void handle_heartbeat_subscription_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    config_msg_heartbeat_subscription_status_t status_message;
//    const heartbeat_subscription_state_t * p_hb_sub;

    if (p_message->length != 0)
    {
        return;
    }
//    p_hb_sub = heartbeat_subscription_get();
    status_message.status = ACCESS_STATUS_SUCCESS;
//    if (p_hb_sub->src == MESH_ADDR_UNASSIGNED ||
//        p_hb_sub->dst == MESH_ADDR_UNASSIGNED)
//    {
//        status_message.source      = MESH_ADDR_UNASSIGNED;
//        status_message.destination = MESH_ADDR_UNASSIGNED;
//        status_message.period_log  = 0x00;
//        status_message.count_log   = 0x00;
//        status_message.min_hops    = 0x00;
//        status_message.max_hops    = 0x00;
//    }
//    else
    {
        status_message.source      = 0;//p_hb_sub->src;
        status_message.destination = 0;//p_hb_sub->dst;
        status_message.count_log   = 0;//heartbeat_subscription_count_encode(p_hb_sub->count);
        status_message.period_log  = 0;//heartbeat_pubsub_period_encode(p_hb_sub->period);
        status_message.min_hops    = 0;//p_hb_sub->min_hops;
        status_message.max_hops    = 0;//p_hb_sub->max_hops;
    }

    send_reply(handle, p_message, CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), mesh_unique_token_get());
}

static void handle_heartbeat_subscription_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    config_msg_heartbeat_subscription_status_t status_message;
    const config_msg_heartbeat_subscription_set_t *p_pdu = (config_msg_heartbeat_subscription_set_t *) p_message->p_data;

    if (p_message->length != sizeof(config_msg_heartbeat_subscription_set_t))
    {
        return;
    }

    heartbeat_subscription_state_t new_subscription_state = {
        .src    = p_pdu->source,
        .dst    = p_pdu->destination,
        .period = 0,//heartbeat_pubsub_period_decode(p_pdu->period_log)
    };

//    if (heartbeat_subscription_set(&new_subscription_state) != MESH_SUCCESS)
//    {
//        return;
//    }

//    const heartbeat_subscription_state_t * p_hb_sub = heartbeat_subscription_get();

    status_message.status = ACCESS_STATUS_SUCCESS;
    status_message.source = 0;//p_hb_sub->src;
    status_message.destination = 0;//p_hb_sub->dst;
    status_message.period_log = 0;//heartbeat_pubsub_period_encode(p_hb_sub->period);
    status_message.count_log = 0;//heartbeat_subscription_count_encode(p_hb_sub->count);
    status_message.min_hops = 0;//p_hb_sub->min_hops;
    status_message.max_hops = 0;//p_hb_sub->max_hops;

    send_reply(handle, p_message, CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), mesh_unique_token_get());

    const config_server_evt_t evt = {
        .type = CONFIG_SERVER_EVT_HEARTBEAT_SUBSCRIPTION_SET,
        .params.heartbeat_subscription_set.p_subscription_state = 0,//p_hb_sub
    };
    app_evt_send(&evt);
}
static void send_relay_status(access_model_handle_t handle, const access_message_rx_t * p_message)
{
    config_msg_relay_status_t status_message = { 0 };
    mesh_opt_core_adv_t relay;
    mesh_opt_core_adv_get(CORE_TX_ROLE_RELAY, &relay);
    status_message.relay_state = (relay.enabled ?
                                  CONFIG_RELAY_STATE_SUPPORTED_ENABLED :
                                  CONFIG_RELAY_STATE_SUPPORTED_DISABLED);

    status_message.relay_retransmit_count = relay.tx_count - 1;
    if ((relay.tx_interval_ms == BEARER_ADV_INT_MIN_MS) &&
        (status_message.relay_retransmit_count == 0))
    {
        status_message.relay_retransmit_interval_steps = 0;
    }
    else
    {
        status_message.relay_retransmit_interval_steps =
            CONFIG_RETRANSMIT_INTERVAL_MS_TO_STEP(relay.tx_interval_ms) - 1;
    }
    send_reply(handle, p_message, CONFIG_OPCODE_RELAY_STATUS,
               (const uint8_t *) &status_message, sizeof(status_message), mesh_unique_token_get());
}
static void handle_config_relay_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != 0) 
    {
        return;
    }
    send_relay_status(handle, p_message);
}
static void handle_config_relay_set(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_relay_set_t * p_pdu = (const config_msg_relay_set_t *) p_message->p_data;
    if (p_message->length != sizeof(config_msg_relay_set_t) ||
        p_pdu->relay_state >= CONFIG_RELAY_STATE_UNSUPPORTED)
    {
        return;
    }
    mesh_opt_core_adv_t relay_state;

    relay_state.enabled = (p_pdu->relay_state == CONFIG_RELAY_STATE_SUPPORTED_ENABLED);
    relay_state.tx_count = p_pdu->relay_retransmit_count + 1;

    if ((p_pdu->relay_retransmit_count == 0) && (p_pdu->relay_retransmit_interval_steps == 0))
    {
        relay_state.tx_interval_ms = BEARER_ADV_INT_MIN_MS;
    }
    else
    {
        relay_state.tx_interval_ms = CONFIG_RETRANSMIT_INTERVAL_STEP_TO_MS(p_pdu->relay_retransmit_interval_steps + 1);
    }

    relay_state.tx_interval_ms = MAX(BEARER_ADV_INT_MIN_MS, relay_state.tx_interval_ms);
//    printf("relay enable --> %d\r\n",relay_state.enabled);
    mesh_opt_core_adv_set(CORE_TX_ROLE_RELAY, &relay_state);

    send_relay_status(handle, p_message);

    const config_server_evt_t evt = {
        .type = CONFIG_SERVER_EVT_RELAY_SET,
        .params.relay_set.enabled = (p_pdu->relay_state == CONFIG_RELAY_STATE_SUPPORTED_ENABLED),
        .params.relay_set.retransmit_count = p_pdu->relay_retransmit_count,
        .params.relay_set.interval_steps = p_pdu->relay_retransmit_interval_steps
    };
    app_evt_send(&evt);
}
static void handle_config_model_subscription_delete(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    printf("handle_config_model_subscription_delete\r\n");
    
    
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_subscription_add_del_owr_t, p_message))
    {
        return;
    }

    const config_msg_subscription_add_del_owr_t * p_pdu = (const config_msg_subscription_add_del_owr_t *) p_message->p_data;

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_subscription_add_del_owr_t));

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Check that the subscription address is valid before continuing: */
    mesh_address_type_t address_type = mesh_address_type_get(p_pdu->address);
    if (address_type != MESH_ADDRESS_TYPE_GROUP)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != MESH_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }

    /* Get the handle for the subscription address: */
    dsm_handle_t subscription_address_handle;
    mesh_address_t group_address = { MESH_ADDRESS_TYPE_GROUP, p_pdu->address, NULL };
    status = dsm_address_handle_get(&group_address, &subscription_address_handle);

    if (status == MESH_SUCCESS)
    {
         /* Remove the subscription from the model: */
        status = access_model_subscription_remove(model_handle, subscription_address_handle);
        if (status == MESH_ERROR_NOT_SUPPORTED)
        {
            status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL);
            return;
        }
        else if (status != MESH_SUCCESS)
        {
            status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
            return;
        }

        dsm_address_subscription_remove(subscription_address_handle);
    }
    else if (status != MESH_ERROR_NOT_FOUND)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_UNSPECIFIED_ERROR);
        return;
    }

    send_subscription_status(handle, p_message, p_pdu->element_address, p_pdu->address, p_pdu->model_id, sig_model);
    const config_server_evt_t evt = {
        .type = CONFIG_SERVER_EVT_MODEL_SUBSCRIPTION_DELETE,
        .params.model_subscription_delete.model_handle = model_handle,
        .params.model_subscription_delete.address_handle = subscription_address_handle
    };
    app_evt_send(&evt);
}
static void handle_appkey_delete(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(config_msg_appkey_delete_t))
    {
        return;
    }

    const config_msg_appkey_delete_t * p_pdu = (const config_msg_appkey_delete_t *) p_message->p_data;
    config_msg_key_index_24_t key_indexes = p_pdu->key_indexes;

    access_status_t status_code = ACCESS_STATUS_SUCCESS;
    uint16_t netkey_index, appkey_index;
    config_server_evt_t evt;
    evt.type = CONFIG_SERVER_EVT_APPKEY_DELETE;
    config_msg_key_index_24_get(&key_indexes, &netkey_index, &appkey_index);
    evt.params.appkey_delete.appkey_handle = dsm_appkey_index_to_appkey_handle(appkey_index);

    if (DSM_HANDLE_INVALID != evt.params.appkey_delete.appkey_handle)
    {
        dsm_handle_t network_handle = dsm_net_key_index_to_subnet_handle(netkey_index);
        if (DSM_HANDLE_INVALID == network_handle)
        {
            status_code = ACCESS_STATUS_INVALID_NETKEY;
        }

        if (ACCESS_STATUS_SUCCESS == status_code)
        {
            dsm_handle_t related_subnet;
            dsm_appkey_handle_to_subnet_handle(evt.params.appkey_delete.appkey_handle, &related_subnet);
            if (network_handle == related_subnet)
            {
                access_model_publication_by_appkey_stop(evt.params.appkey_delete.appkey_handle);
                dsm_appkey_delete(evt.params.appkey_delete.appkey_handle);
            }
            else
            {
                status_code = ACCESS_STATUS_INVALID_BINDING;
            }
        }
    }

    send_appkey_status(handle, p_message, status_code, key_indexes);

    if (ACCESS_STATUS_SUCCESS == status_code &&
        DSM_HANDLE_INVALID != evt.params.appkey_delete.appkey_handle)
    {
        app_evt_send(&evt);
    }
}
static uint8_t model_app_response_create(uint16_t opcode,
                                         uint8_t * p_buffer,
                                         access_status_t status,
                                         uint16_t element_address,
                                         access_model_id_t * p_model_id,
                                         mesh_key_index_t * p_appkey_list,
                                         uint16_t appkey_count)
{
    uint8_t size;

    if (opcode == CONFIG_OPCODE_SIG_MODEL_APP_GET)
    {
        config_msg_sig_model_app_list_t * p_response = (config_msg_sig_model_app_list_t *)p_buffer;
        p_response->status = status;
        p_response->element_address = element_address;
        p_response->sig_model_id = p_model_id->model_id;

        if (NULL != p_appkey_list)
        {
            packed_index_list_create(p_appkey_list, p_response->key_indexes, appkey_count);
        }

        size = sizeof(config_msg_sig_model_app_list_t);
    }
    else
    {
        config_msg_vendor_model_app_list_t * p_response = (config_msg_vendor_model_app_list_t *)p_buffer;
        p_response->status = status;
        p_response->element_address = element_address;
        p_response->vendor_company_id = p_model_id->company_id;
        p_response->vendor_model_id = p_model_id->model_id;

        if (NULL != p_appkey_list)
        {
            packed_index_list_create(p_appkey_list, p_response->key_indexes, appkey_count);
        }

        size = sizeof(config_msg_vendor_model_app_list_t);
    }

    return size + PACKED_INDEX_LIST_SIZE(appkey_count);
}
static void handle_model_app_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_model_app_get_t, p_message))
    {
        return;
    }

    const config_msg_model_app_get_t * p_pdu = (const config_msg_model_app_get_t *) p_message->p_data;
    /* Get the model handle: */
    access_model_handle_t model_handle;
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_model_app_get_t));
    uint16_t element_index = get_element_index(p_pdu->element_address);
    uint16_t response_opcode = p_message->opcode.opcode == CONFIG_OPCODE_SIG_MODEL_APP_GET ?
            CONFIG_OPCODE_SIG_MODEL_APP_LIST : CONFIG_OPCODE_VENDOR_MODEL_APP_LIST;
    uint8_t response_buffer[sizeof(config_msg_vendor_model_app_list_t) + PACKED_INDEX_LIST_SIZE(DSM_APP_MAX)];

    if (ACCESS_ELEMENT_INDEX_INVALID == element_index)
    {
        uint8_t response_size = model_app_response_create(p_message->opcode.opcode,
                                                          response_buffer,
                                                          ACCESS_STATUS_INVALID_ADDRESS,
                                                          p_pdu->element_address,
                                                          &model_id,
                                                          NULL,
                                                          0);
        send_reply(handle, p_message, response_opcode, response_buffer, response_size, mesh_unique_token_get());
        return;
    }

    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != MESH_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        uint8_t response_size = model_app_response_create(p_message->opcode.opcode,
                                                          response_buffer,
                                                          ACCESS_STATUS_INVALID_MODEL,
                                                          p_pdu->element_address,
                                                          &model_id,
                                                          NULL,
                                                          0);
        send_reply(handle, p_message, response_opcode, response_buffer, response_size, mesh_unique_token_get());
        return;
    }

    /* Get the application list: */
    uint16_t appkey_instances[DSM_APP_MAX];
    uint16_t appkey_count = DSM_APP_MAX;
    access_model_applications_get(model_handle, appkey_instances, &appkey_count);

    /* Retrieve the appkey indexes from the DSM: */
    for (uint16_t i = 0; i < appkey_count; ++i)
    {
        dsm_appkey_handle_to_appkey_index(appkey_instances[i], &appkey_instances[i]);
    }

    uint8_t response_size = model_app_response_create(p_message->opcode.opcode,
                                                      response_buffer,
                                                      ACCESS_STATUS_SUCCESS,
                                                      p_pdu->element_address,
                                                      &model_id,
                                                      appkey_instances,
                                                      appkey_count);
    send_reply(handle, p_message, response_opcode, response_buffer, response_size, mesh_unique_token_get());
}
static inline access_status_t get_subscription_list(access_model_handle_t model_handle, uint16_t * p_sublist, uint16_t * p_subcount)
{
    uint32_t status = access_model_subscriptions_get(model_handle, p_sublist, p_subcount);
    switch (status)
    {
        case MESH_SUCCESS:
        {
            /* Resolve address handles to actual addresses */
            mesh_address_t addr;
            for (uint32_t i = 0; i < *p_subcount; i++)
            {
                if (dsm_address_get(*p_sublist, &addr) != MESH_SUCCESS)
                {
                    return ACCESS_STATUS_UNSPECIFIED_ERROR;
                }
                else
                {
                    if (addr.type == MESH_ADDRESS_TYPE_VIRTUAL ||
                        addr.type == MESH_ADDRESS_TYPE_GROUP)
                    {
                        *(p_sublist++) = addr.value;
                    }
                    else
                    {
                        return ACCESS_STATUS_UNSPECIFIED_ERROR;
                    }
                }
            }
            return ACCESS_STATUS_SUCCESS;
        }
        case MESH_ERROR_NOT_SUPPORTED:
            return ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL;
        case MESH_ERROR_NOT_FOUND:
            return ACCESS_STATUS_INVALID_MODEL;
        default:
            return ACCESS_STATUS_UNSPECIFIED_ERROR;
    }
}
static void handle_config_sig_model_subscription_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const config_msg_model_subscription_get_t * p_pdu = (const config_msg_model_subscription_get_t *) p_message->p_data;
    if (p_message->length != PACKET_LENGTH_WITH_ID(config_msg_model_subscription_get_t, true))
    {
        return;
    }

    /* Get the model handle: */
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_model_subscription_get_t));

    /* Get the element index corresponding to the requested address: */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != MESH_SUCCESS)
    {
        status_error_sub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
        return;
    }

    uint16_t subscription_count = DSM_ADDR_MAX;
    uint16_t subscription_list[DSM_ADDR_MAX];
    access_status_t error_code = get_subscription_list(model_handle, subscription_list, &subscription_count);
    if (error_code != ACCESS_STATUS_SUCCESS)
    {
        status_error_sub_send(handle, p_message, sig_model, error_code);
        return;
    }

    uint8_t response_buffer[sizeof(config_msg_sig_model_subscription_list_t) + DSM_ADDR_MAX * sizeof(uint16_t)];
    config_msg_sig_model_subscription_list_t * p_response = (config_msg_sig_model_subscription_list_t *) response_buffer;

    p_response->status = ACCESS_STATUS_SUCCESS;
    p_response->element_address = p_pdu->element_address;
    p_response->sig_model_id = model_id.model_id;
    memcpy((void *) p_response->subscriptions, subscription_list, subscription_count * sizeof(uint16_t));

    send_reply(handle, p_message, CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST, response_buffer,
            sizeof(config_msg_sig_model_subscription_list_t) + subscription_count * sizeof(uint16_t), mesh_unique_token_get());
}
static void handle_config_model_publication_get(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (!IS_PACKET_LENGTH_VALID_WITH_ID(config_msg_publication_get_t, p_message))
    {
        return;
    }

    const config_msg_publication_get_t * p_pdu = (const config_msg_publication_get_t *) p_message->p_data;

    /* Extract the model ID from the PDU: */
    access_model_id_t model_id;
    bool sig_model = model_id_extract(&model_id, &p_pdu->model_id, p_message, sizeof(config_msg_publication_get_t));

    /* Get element index from element address */
    uint16_t element_index = get_element_index(p_pdu->element_address);
    if (element_index == ACCESS_ELEMENT_INDEX_INVALID)
    {
        status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_ADDRESS);
        return;
    }

    /* Get the model handle: */
    access_model_handle_t model_handle;
    uint32_t status = access_handle_get(element_index, model_id, &model_handle);
    if (status != MESH_SUCCESS || (!sig_model && model_id.company_id == ACCESS_COMPANY_ID_NONE))
    {
        status_error_pub_send(handle, p_message, sig_model, ACCESS_STATUS_INVALID_MODEL);
    }
    else
    {
        send_publication_status(handle, p_message, p_pdu->element_address, model_handle);
    }
}



static const access_opcode_handler_t opcode_handlers[] =
{
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_COMPOSITION_DATA_GET)       , handle_composition_data_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_DEFAULT_TTL_GET)            , handle_config_default_ttl_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NETWORK_TRANSMIT_SET)       , handle_config_network_transmit_set},
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_APPKEY_ADD)                 , handle_appkey_add },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_APP_BIND)             , handle_model_app_bind_unbind },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_APP_UNBIND)           , handle_model_app_bind_unbind },    
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_PUBLICATION_SET)      , handle_config_model_publication_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_SUBSCRIPTION_ADD)     , handle_config_model_subscription_add },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_NODE_RESET)                 , handle_node_reset },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_DEFAULT_TTL_SET)            , handle_config_default_ttl_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_GET)  , handle_heartbeat_publication_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_SET)  , handle_heartbeat_publication_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_GET) , handle_heartbeat_subscription_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_SET) , handle_heartbeat_subscription_set }, 
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_RELAY_GET)                  , handle_config_relay_get },    
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_RELAY_SET)                  , handle_config_relay_set },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE)  , handle_config_model_subscription_delete },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_APPKEY_DELETE)              , handle_appkey_delete },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_SIG_MODEL_APP_GET)          , handle_model_app_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_GET) , handle_config_sig_model_subscription_get },
    { ACCESS_OPCODE_SIG(CONFIG_OPCODE_MODEL_PUBLICATION_GET)      , handle_config_model_publication_get },
    
};
static void apply_reset(void)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);                  
    Qflash_Erase_Sector(0x103B000);
    Qflash_Erase_Sector(0x103C000);
    Qflash_Erase_Sector(0x103D000);
    Qflash_Erase_Sector(0x103E000);
    Qflash_Erase_Sector(0x103F000);  
    NVIC_SystemReset();
}
static void mesh_event_cb(const mesh_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case MESH_EVT_PROXY_STOPPED:
            if (m_node_reset_pending == NODE_RESET_PENDING_PROXY)
            {
                apply_reset();
            }
            break;
        case MESH_EVT_TX_COMPLETE:
            if (p_evt->params.tx_complete.token == m_reset_token)
            {
                if (NODE_RESET_PENDING == m_node_reset_pending)
                {
                    if (proxy_is_connected())
                    {
                        m_node_reset_pending = NODE_RESET_PENDING_PROXY;
                        (void) proxy_stop();
                    }
                    else
                    {
                        apply_reset();
                    }
                }
            }
            break;

        case MESH_EVT_FLASH_STABLE:

            break;

        default:
            break;
    }
}
uint32_t config_server_init(config_server_evt_cb_t evt_cb)
{
    const access_model_add_params_t init_params =
    {
        .model_id = {
            .model_id = CONFIG_SERVER_MODEL_ID,
            .company_id = ACCESS_COMPANY_ID_NONE
        },
        .element_index = 0,
        .p_opcode_handlers = opcode_handlers,
        .opcode_count = sizeof(opcode_handlers) / sizeof(access_opcode_handler_t)
    };
    mesh_evt_handler_add(&m_mesh_evt_handler);

    m_evt_cb = evt_cb;
    return access_model_add(&init_params, &m_config_server_handle);
}
uint32_t config_server_bind(dsm_handle_t devkey_handle)
{
    return access_model_application_bind(m_config_server_handle, devkey_handle);
}











