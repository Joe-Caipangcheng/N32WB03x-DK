#ifndef UTILS_H__
#define UTILS_H__

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "mesh_common.h"


#define PAGE_SIZE        (0x1000)

#define LE2BE24(n) ((((n) & 0x00FF0000) >> 16) |        \
                    ( (n) & 0x0000FF00)        |        \
                    (((n) & 0x000000FF) << 16))

#define BE2LE24(n) LE2BE24(n)
#define WORD_SIZE (sizeof(int))


#define IS_PAGE_ALIGNED(p) (((uint32_t)(p) & (PAGE_SIZE - 1)) == 0)
#define IS_WORD_ALIGNED(p) (((uint32_t)(p) & (WORD_SIZE - 1)) == 0)
#define ALIGN_VAL(X, A)          (((X)+((A)-1))&~((A)-1))
#define IS_POWER_OF_2(VALUE) ((VALUE) && (((VALUE) & ((VALUE) - 1)) == 0))


#ifndef ROUNDED_DIV
#define ROUNDED_DIV(A, B) (((A) + ((B) / 2)) / (B))
#endif

#define HOURS_TO_SECONDS(t) ((t) * 60 * 60)
#define MIN_TO_MS(t) ((t) * 60000ul)
#define SEC_TO_US(t) ((t) * 1000000ul)
#define SEC_TO_MS(t) ((t) * 1000)
#define MS_TO_US(t) ((t) * 1000)
#define MS_TO_SEC(t) (ROUNDED_DIV(t, 1000))
#define MS_TO_MIN(t) ((t) / 60000ul)
#define US_TO_MS(t) (ROUNDED_DIV(t, 1000))
#define US_TO_SEC(t) (ROUNDED_DIV(t, 1000000ul))


#define IS_IN_RANGE(val, min, max) (((min) <= (val) && (val) <= (max)))
#define PARENT_BY_FIELD_GET(STRUCT_TYPE, FIELD_NAME, FIELD_POINTER) \
    ((STRUCT_TYPE *) (((uint8_t *)FIELD_POINTER) - offsetof(STRUCT_TYPE, FIELD_NAME)))
#ifndef CEIL_DIV
#define CEIL_DIV(A, B)   (((A) + (B) - 1) / (B))
#endif


#define BE2LE16(n) ((((n) << 8) & 0xff00) | (((n) >> 8) & 0x00ff)) /**< Converts a 16-bit big-endian number to little-endian. */
#define LE2BE16(n) BE2LE16(n)                                      /**< Converts a 16-bit little-endian number to big-endian. */
#define BE2LE32(n) LE2BE32(n)    /**< Converts a 32-bit big-endian number to little-endian. */
#define LE2BE32(n) (((uint32_t) (n) << 24) | (((uint32_t) (n) << 8) & 0xff0000) \
        | (((n) >> 8) & 0xff00) | ((n) >> 24))  /**< Converts a 32-bit little-endian number to big-endian. */







static inline bool is_power_of_two(uint32_t value)
{
    return IS_POWER_OF_2(value);
}

static inline void utils_reverse_memcpy(uint8_t * p_dst, const uint8_t * p_src, uint16_t size)
{
    p_src += size;
    while (size--)
    {
        *((uint8_t *) p_dst++) = *((uint8_t *) --p_src);
    }
}

static inline void utils_reverse_array(uint8_t * p_array, uint16_t size)
{
    for(uint16_t i = 0; i < size / 2; ++i)
    {
        uint8_t temp = p_array[i];
        p_array[i] = p_array[size - i - 1];
        p_array[size - i - 1] = temp;
    }
}

static inline void utils_xor(uint8_t * p_dst, const uint8_t * p_src1, const uint8_t * p_src2, uint16_t size)
{
    while (0 != size)
    {
        size--;
        p_dst[size] = p_src1[size] ^ p_src2[size];
    }
}

static inline void utils_lshift(uint8_t * p_dst, const uint8_t * p_src, uint16_t size)
{
    for (uint16_t i = 0; i < size - 1; ++i)
    {
        p_dst[i] = (p_src[i] << 1);
        p_dst[i] |= !!(p_src[i + 1] & 0x80);
    }
    p_dst[size - 1] = (p_src[size - 1] << 1);
}

static inline void utils_pad(uint8_t * p_dst, const uint8_t * p_src, uint16_t size)
{
    memcpy(p_dst, p_src, size);
    p_dst[size] = 0x80;

    for (int i = size+1; i < 16; ++i)
        p_dst[i] = 0x00;
}

static inline uint8_t log2_get(uint32_t value)
{
    uint8_t log_val = 0;
    while ((value >>= 1) != 0)
    {
        log_val++;
    }

    return log_val;
}

#endif //UTILS_H__
