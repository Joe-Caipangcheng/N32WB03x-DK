#ifndef AD_TYPE_FILTER_H__
#define AD_TYPE_FILTER_H__

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
    AD_FILTER_WHITELIST_MODE, 
    AD_FILTER_BLACKLIST_MODE  
} ad_type_mode_t;

void bearer_adtype_filtering_set(bool onoff);
void bearer_adtype_remove(uint8_t type);
void bearer_adtype_add(uint8_t type);
void bearer_adtype_clear(void);
void bearer_adtype_mode_set(ad_type_mode_t mode);
uint32_t bearer_invalid_length_amount_get(void);
uint32_t bearer_adtype_filtered_amount_get(void);


#endif /* AD_TYPE_FILTER_H__ */
