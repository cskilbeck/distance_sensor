#pragma once

//////////////////////////////////////////////////////////////////////

#include <stdint.h>

//////////////////////////////////////////////////////////////////////

typedef uint8_t byte;

typedef uint64_t uint64;
typedef uint32_t uint32;
typedef uint16_t uint16;
typedef uint8_t uint8;

typedef int64_t int64;
typedef int32_t int32;
typedef int16_t int16;
typedef int8_t int8;

#if defined(__cplusplus)
constexpr nullptr_t null = nullptr;
#else
#define null ((void *)0)
#endif
