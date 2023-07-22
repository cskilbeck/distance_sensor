#pragma once

//////////////////////////////////////////////////////////////////////

template <typename T, size_t N> constexpr size_t countof(T const (&)[N]) noexcept
{
    return N;
}

void log_buffer(const char *tag, void const *buffer, uint16_t buff_len, esp_log_level_t log_level);
