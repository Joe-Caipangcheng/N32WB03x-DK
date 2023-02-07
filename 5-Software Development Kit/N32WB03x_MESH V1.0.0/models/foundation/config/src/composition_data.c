#include "composition_data.h"
#include <stdint.h>
#include <string.h>
#include "access.h"
#include "mesh_config_core.h"
#include "access_config.h"



static uint16_t composition_data_sig_models_write(uint16_t element_index, const access_model_handle_t * p_handles, uint16_t count, uint8_t * p_buffer)
{
    uint16_t bytes = 0;
    access_model_id_t model_id;
    for (uint32_t i = 0; i < count; ++i)
    {
        access_model_id_get(p_handles[i], &model_id);
        if (model_id.company_id == ACCESS_COMPANY_ID_NONE)
        {
            /* Little-endian */
            memcpy(&p_buffer[bytes], (void *) &model_id.model_id, sizeof(model_id.model_id));
            bytes += sizeof(model_id.model_id);
        }
    }
    return bytes;
}

static uint16_t composition_data_vendor_models_write(uint16_t element_index, const access_model_handle_t * p_handles, uint16_t count, uint8_t * p_buffer)
{
    uint16_t bytes = 0;
    access_model_id_t model_id;
    for (uint32_t i = 0; i < count; ++i)
    {
        access_model_id_get(p_handles[i], &model_id);
        if (model_id.company_id != ACCESS_COMPANY_ID_NONE)
        {
            memcpy(&p_buffer[bytes], (void *) &model_id.company_id, sizeof(model_id.company_id));
            bytes += sizeof(model_id.company_id);
            memcpy(&p_buffer[bytes], (void *) &model_id.model_id, sizeof(model_id.model_id));
            bytes += sizeof(model_id.model_id);
        }
    }
    return bytes;
}


void config_composition_data_get(uint8_t * p_data, uint16_t * p_size)
{
    access_model_handle_t model_handles[ACCESS_MODEL_COUNT];
    uint16_t element_models_count = 0;
    config_composition_data_header_t device;

    device.company_id = DEVICE_COMPANY_ID;
    device.product_id = DEVICE_PRODUCT_ID;
    device.version_id = DEVICE_VERSION_ID;
    device.replay_cache_entries = REPLAY_CACHE_ENTRIES;
    device.features = 0;
    device.features |= CONFIG_FEATURE_PROXY_BIT;
    device.features |= CONFIG_FEATURE_RELAY_BIT;
    memcpy(&p_data[0], &device, sizeof(device));
    *p_size = sizeof(device);

    for (int i = 0; i < ACCESS_ELEMENT_COUNT; ++i)
    {
        config_composition_element_header_t element;
        access_element_sig_model_count_get(i, &element.sig_model_count);
        access_element_vendor_model_count_get(i, &element.vendor_model_count);
        access_element_location_get(i, (uint16_t*) &element.location);
        element_models_count = ACCESS_MODEL_COUNT;
        access_element_models_get(i, &model_handles[0], &element_models_count);
        memcpy(&p_data[*p_size], &element, sizeof(element));
        *p_size += sizeof(element);

        if (element.sig_model_count > 0)
        {
            *p_size += composition_data_sig_models_write(i, model_handles, element_models_count, &p_data[*p_size]);
        }
        if (element.vendor_model_count > 0)
        {
            *p_size += composition_data_vendor_models_write(i, model_handles, element_models_count, &p_data[*p_size]);
        }
    }
}








