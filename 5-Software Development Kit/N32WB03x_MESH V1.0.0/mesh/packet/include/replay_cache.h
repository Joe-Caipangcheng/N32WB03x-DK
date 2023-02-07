#ifndef REPLAY_CACHE_H__
#define REPLAY_CACHE_H__

#include <stdint.h>
#include <stdbool.h>


void replay_cache_init(void);
void replay_cache_enable(void);
uint32_t replay_cache_add(uint16_t src, uint32_t seqno, uint32_t iv_index);
uint32_t replay_cache_seqauth_add(uint16_t src, uint32_t seqno, uint32_t iv_index, uint16_t seqzero);
bool replay_cache_has_elem(uint16_t src, uint32_t seqno, uint32_t iv_index);
bool replay_cache_has_seqauth(uint16_t src, uint32_t seqno, uint32_t iv_index, uint16_t seqzero);
bool replay_cache_is_seqauth_last(uint16_t src, uint32_t seqno, uint32_t iv_index, uint16_t seqzero);
void replay_cache_on_iv_update(void);
void replay_cache_clear(void);

#endif  /* REPLAY_CACHE_H__ */
