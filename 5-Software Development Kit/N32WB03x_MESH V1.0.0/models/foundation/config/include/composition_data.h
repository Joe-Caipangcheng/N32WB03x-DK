#ifndef COMPOSITION_DATA_H
#define COMPOSITION_DATA_H

#include <stdint.h>


#define CONFIG_SIG_MODEL_ID_SIZE (2)
#define CONFIG_VENDOR_MODEL_ID_SIZE (4)
#define CONFIG_MODEL_ID_SIZE_MAX (CONFIG_VENDOR_MODEL_ID_SIZE)
#define COMPOSITION_DATA_LENGTH_MIN (sizeof(config_msg_composition_data_status_t) + \
                                     sizeof(config_composition_data_header_t) + \
                                     sizeof(config_composition_element_header_t) + \
                                     CONFIG_SIG_MODEL_ID_SIZE)

typedef struct __attribute((packed))
{
    uint16_t company_id;
    uint16_t product_id;
    uint16_t version_id;
    uint16_t replay_cache_entries;
    uint16_t features;
} config_composition_data_header_t;

typedef struct __attribute((packed))
{
    uint16_t location;
    uint8_t sig_model_count;
    uint8_t vendor_model_count;
} config_composition_element_header_t;

#define CONFIG_COMPOSITION_DATA_SIZE                                    \
    (sizeof(config_composition_data_header_t) +                         \
     (ACCESS_ELEMENT_COUNT) * sizeof(config_composition_element_header_t) + \
     (ACCESS_MODEL_COUNT) * CONFIG_MODEL_ID_SIZE_MAX)                   \


typedef enum
{
    CONFIG_FEATURE_RELAY_BIT      = (1 << 0), 
    CONFIG_FEATURE_PROXY_BIT      = (1 << 1), 
    CONFIG_FEATURE_FRIEND_BIT     = (1 << 2), 
    CONFIG_FEATURE_LOW_POWER_BIT  = (1 << 3)  
} config_feature_bit_t;


void config_composition_data_get(uint8_t * p_data, uint16_t * p_size);

#endif /* CONFIG_SERVER_H__ */
