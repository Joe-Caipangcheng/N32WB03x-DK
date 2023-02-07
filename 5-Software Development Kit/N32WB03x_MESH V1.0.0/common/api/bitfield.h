#ifndef BITFIELD_H__
#define BITFIELD_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>


#define BITFIELD_BLOCK_SIZE (32)
#define BITFIELD_MASK(BIT) (1u << ((BIT) & (BITFIELD_BLOCK_SIZE - 1)))
#define BITFIELD_BLOCK_COUNT(BITS) (((BITS) + BITFIELD_BLOCK_SIZE - 1) / BITFIELD_BLOCK_SIZE)

static inline bool bitfield_get(const uint32_t * p_bitfield, uint32_t bit)
{
    return !!(p_bitfield[bit / BITFIELD_BLOCK_SIZE] & BITFIELD_MASK(bit));
}

static inline void bitfield_set(uint32_t * p_bitfield, uint32_t bit)
{
    p_bitfield[bit / BITFIELD_BLOCK_SIZE] |= BITFIELD_MASK(bit);
}

static inline void bitfield_clear(uint32_t * p_bitfield, uint32_t bit)
{
    p_bitfield[bit / BITFIELD_BLOCK_SIZE] &= ~(BITFIELD_MASK(bit));
}

static inline void bitfield_set_all(uint32_t * p_bitfield, uint32_t bits)
{
    memset(p_bitfield, 0xFF, (BITFIELD_BLOCK_SIZE / 8) * BITFIELD_BLOCK_COUNT(bits));
}

static inline void bitfield_clear_all(uint32_t * p_bitfield, uint32_t bits)
{
    memset(p_bitfield, 0x00, (BITFIELD_BLOCK_SIZE / 8) * BITFIELD_BLOCK_COUNT(bits));
}

static inline uint32_t bitfield_next_get(const uint32_t * p_bitfield, uint32_t bits, uint32_t start)
{
    /* Lookup table for offset into a single bitfield block. */
    static const uint8_t multiply_de_bruijn_bit_position[32] =
    {
        0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15, 25, 17, 4, 8,
        31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9
    };

    if (start >= bits)
    {
        return bits;
    }

    uint32_t i = start / BITFIELD_BLOCK_SIZE;
    uint32_t v = p_bitfield[i] & ~(BITFIELD_MASK(start) - 1);
    while (v == 0)
    {
        ++i;
        if (i >= BITFIELD_BLOCK_COUNT(bits))
        {
            return bits;
        }
        else
        {
            v = p_bitfield[i];
        }
    }
    return (i * BITFIELD_BLOCK_SIZE) + (uint32_t) multiply_de_bruijn_bit_position[((uint32_t)((v & -v) * 0x077CB531U)) >> 27];
}

static inline uint32_t bitfield_popcount(const uint32_t * p_bitfield, uint32_t bits)
{
    uint32_t popcount = 0;
    for(uint32_t i = bitfield_next_get(p_bitfield, bits, 0); i != bits; i = bitfield_next_get(p_bitfield, bits, i + 1))
    {
        popcount++;
    }

    return popcount;
}

static inline bool bitfield_is_all_clear(const uint32_t * p_bitfield, uint32_t bits)
{
    for (uint32_t i = 0; i < BITFIELD_BLOCK_COUNT(bits); ++i)
    {
        uint32_t mask = 0xFFFFFFFF;
        if (i == (BITFIELD_BLOCK_COUNT(bits) - 1))
        {
            /* Only check the bits that actually are part of the bitfield */
            mask = (1 << (bits - i * BITFIELD_BLOCK_SIZE)) - 1;
        }
        if ((p_bitfield[i] & mask) != 0)
        {
            return false;
        }
    }
    return true;
}

static inline bool bitfield_is_all_set(const uint32_t * p_bitfield, uint32_t bits)
{
    for (uint32_t i = 0; i < BITFIELD_BLOCK_COUNT(bits); ++i)
    {
        uint32_t mask = 0xFFFFFFFF;
        if (i == (BITFIELD_BLOCK_COUNT(bits) - 1))
        {
            /* Only check the bits that actually are part of the bitfield */
            mask = (1 << (bits - i * BITFIELD_BLOCK_SIZE)) - 1;
        }

        if ((p_bitfield[i] & mask) != mask)
        {
            return false;
        }
    }
    return true;
}

static inline bool bitfield_is_subset_of(const uint32_t * p_b1, const uint32_t * p_b2, const uint32_t bits)
{
    for (uint32_t i = 0; i < BITFIELD_BLOCK_COUNT(bits); ++i)
    {
        if ((~p_b1[i] & p_b2[i]) != 0)
        {
            return false;
        }
    }
    return true;
}



#endif /* BITFIELD_H__ */

