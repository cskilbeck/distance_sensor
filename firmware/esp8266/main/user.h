#pragma once

#if defined(__cplusplus)
extern "C" {
#endif

typedef unsigned char byte;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long long uint64;
typedef char int8;
typedef short int16;
typedef int int32;
typedef long long int64;

void user_main();

typedef struct
{
    uint64 timestamp;    // # of 100uS ticks since epoch midnight 1st Jan 1970
    uint32 options;      // 32 option bits
    uint16 signature;    // signature must be 'DC'
} __attribute__((packed)) message_body_t;

typedef struct
{
    message_body_t msg;    // payload
    uint16 crc;            // 16 bit crc of previous fields
} __attribute__((packed)) message_t;

#if defined(__cplusplus)
}
#endif
