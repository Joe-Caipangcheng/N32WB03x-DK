#ifndef GENERIC_ONOFF_MESSAGES_H__
#define GENERIC_ONOFF_MESSAGES_H__

#include <stdint.h>


#define GENERIC_ONOFF_SET_MINLEN 2
#define GENERIC_ONOFF_SET_MAXLEN 4
#define GENERIC_ONOFF_STATUS_MINLEN 1
#define GENERIC_ONOFF_STATUS_MAXLEN 3

typedef enum
{
    GENERIC_ONOFF_OPCODE_SET = 0x8202,
    GENERIC_ONOFF_OPCODE_SET_UNACKNOWLEDGED = 0x8203,
    GENERIC_ONOFF_OPCODE_GET = 0x8201,
    GENERIC_ONOFF_OPCODE_STATUS = 0x8204
} generic_onoff_opcode_t;

typedef struct __attribute((packed))
{
    uint8_t on_off;                                  
    uint8_t tid;                                     
    uint8_t transition_time;                         
    uint8_t delay;                                   
} generic_onoff_set_msg_pkt_t;

typedef struct __attribute((packed))
{
    uint8_t present_on_off;                          
    uint8_t target_on_off;                           
    uint8_t remaining_time;                          
} generic_onoff_status_msg_pkt_t;

#endif /* GENERIC_ONOFF_MESSAGES_H__ */
