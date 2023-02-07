#ifndef MESH_RAND_H__
#define MESH_RAND_H__

#include <stdint.h>



typedef struct
{
    uint32_t a; /**< PRNG state variable A */
    uint32_t b; /**< PRNG state variable B */
    uint32_t c; /**< PRNG state variable C */
    uint32_t d; /**< PRNG state variable D */
} prng_t;
void rand_prng_seed(prng_t* p_prng);
uint32_t rand_prng_get(prng_t* p_prng);
void rand_hw_rng_get(uint8_t* p_result, uint16_t len);


#endif /* MESH_RAND_H__ */
