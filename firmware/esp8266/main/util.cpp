#include <stdio.h>
#include <cstdint>
#include "esp_log.h"
#include "util.h"

//////////////////////////////////////////////////////////////////////

void log_buffer(const char *tag, void const *buffer, uint16_t buff_len, esp_log_level_t log_level)
{
    constexpr uint16_t bytes_per_line = 32;

    char hex_buffer[2 * bytes_per_line + 1];

    uint8_t const *ptr = reinterpret_cast<uint8_t const *>(buffer);

    while(buff_len != 0) {

        int len = buff_len;
        if(len > bytes_per_line) {
            len = bytes_per_line;
        }

        for(int i = 0; i < len; i++) {
            sprintf(hex_buffer + 2 * i, "%02x", ptr[i]);
        }

        ESP_LOG_LEVEL(log_level, tag, "%s", hex_buffer);

        ptr += len;
        buff_len -= len;
    }
}

//////////////////////////////////////////////////////////////////////
