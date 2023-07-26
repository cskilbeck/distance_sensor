#pragma once

//////////////////////////////////////////////////////////////////////

#define SUPPRESS_DEPRECATED _Pragma("GCC diagnostic push") _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")
#define SUPPRESS_POP _Pragma("GCC diagnostic pop")

//////////////////////////////////////////////////////////////////////

template <typename T, size_t N> constexpr size_t countof(T const (&)[N]) noexcept
{
    return N;
}

void log_buffer(const char *tag, void const *buffer, uint16_t buff_len, esp_log_level_t log_level);
