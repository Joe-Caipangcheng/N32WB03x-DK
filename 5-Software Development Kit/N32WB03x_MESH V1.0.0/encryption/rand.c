#include <stddef.h>
#include "n32wb03x.h"
#include "rand.h"


/*****************************************************************************
* Local defines
*****************************************************************************/
#define ROT(x,k) (((x)<<(k))|((x)>>(32-(k)))) /** PRNG cyclic leftshift */
#define SMALL_PRNG_BASE_SEED    (0xf1ea5eed)  /** Base seed for PRNG, defined by the author of the generator. */
/*****************************************************************************
* Interface functions
*****************************************************************************/
void rand_prng_seed(prng_t* p_prng)
{
    uint32_t seed = 0;
    /* Get true random seed from HW. */
    rand_hw_rng_get((uint8_t*) &seed, sizeof(seed));

    /* establish base magic numbers */
    p_prng->a = SMALL_PRNG_BASE_SEED;
    p_prng->b = seed;
    p_prng->c = seed;
    p_prng->d = seed;

    /* run the prng a couple of times to flush out the seeds */
    for (uint32_t i = 0; i < 20; ++i)
    {
        (void) rand_prng_get(p_prng);
    }
}

uint32_t rand_prng_get(prng_t* p_prng)
{
    /* Bob Jenkins' small PRNG
        http://burtleburtle.net/bob/rand/smallprng.html */
    uint32_t e = p_prng->a - ROT(p_prng->b, 27);
    p_prng->a = p_prng->b ^ ROT(p_prng->c, 17);
    p_prng->b = p_prng->c + p_prng->d;
    p_prng->c = p_prng->d + e;
    p_prng->d = e + p_prng->a;
    return p_prng->d;
}
#include "rwip.h"
#include "co_math.h"
#include "rwip.h"
#include "bsp_lpuart.h"
void rand_hw_rng_get(uint8_t* p_result, uint16_t len)
{
//    printf("..rand_hw_rng_get..\r\n");
    rwip_time_t current_time;
    uint16_t count = 0;
    while (count < len)
    {
        current_time = rwip_time_get();    
        uint32_t pin_code = ((100000 + (co_rand_word()%900000) + current_time.hus) % 999999);
        p_result[count] = (uint8_t)pin_code;
        count++;
    }    
}


