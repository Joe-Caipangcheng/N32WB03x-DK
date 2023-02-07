//
// Created by Liming Shao on 2018/4/24.
//

#ifndef AES_AES_H
#define AES_AES_H



#include <stdint.h>

typedef struct{
    uint32_t eK[44], dK[44];    // encKey, decKey
    int Nr; // 10 rounds
}AesKey;


int aesEncrypt(const uint8_t *key, uint32_t keyLen, const uint8_t *pt, uint8_t *ct, uint32_t len);




#endif //AES_AES_H
