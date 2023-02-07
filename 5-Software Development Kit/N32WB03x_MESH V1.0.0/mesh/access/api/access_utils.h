#ifndef ACCESS_UTILS_H__
#define ACCESS_UTILS_H__

#include <stdint.h>
#include "access.h"


#define ACCESS_UTILS_SIG_OPCODE_SIZE(OPCODE) (((OPCODE) > 0x00FF) ? 2 : 1)
#define ACCESS_UTILS_VENDOR_OPCODE_SIZE      3


static inline uint16_t access_utils_opcode_size_get(access_opcode_t opcode)
{
    if (opcode.company_id != ACCESS_COMPANY_ID_NONE)
    {
        return ACCESS_UTILS_VENDOR_OPCODE_SIZE;
    }
    else
    {
        return ACCESS_UTILS_SIG_OPCODE_SIZE(opcode.opcode);
    }
}

#endif /* ACCESS_UTILS_H__ */

