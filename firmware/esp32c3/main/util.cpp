#include <stdio.h>
#include <cstdint>
#include "esp_log.h"
#include "util.h"

LOG_CONTEXT("util");

//////////////////////////////////////////////////////////////////////

namespace
{
    char hex_digit(int x)
    {
        if(x < 10) {
            return x + '0';
        }
        return x + ('a' - 10);
    }
}

//////////////////////////////////////////////////////////////////////

void log_buffer(char const *tag, void const *buffer, uint16 buff_len, esp_log_level_t log_level)
{
    constexpr uint16_t bytes_per_line = 16;

    char hex_buffer[3 * bytes_per_line + bytes_per_line + 1];

    uint8 const *ptr = reinterpret_cast<uint8 const *>(buffer);

    while(buff_len != 0) {

        int len = min(buff_len, bytes_per_line);

        char *d = hex_buffer;

        for(int i = 0; i < len; ++i) {
            uint8_t b = ptr[i];
            d[0] = hex_digit(b >> 4);
            d[1] = hex_digit(b & 0xf);
            d[2] = ' ';
            d += 3;
        }

        while(d < (hex_buffer + 3 * bytes_per_line)) {
            *d++ = ' ';
        }

        for(int i = 0; i < len; ++i) {
            uint8 c = ptr[i];
            if(c < 32 || c > 127) {
                c = '.';
            }
            *d++ = static_cast<uint8>(c);
        }

        *d++ = 0;

        ESP_LOG_LEVEL(log_level, tag, "%s", hex_buffer);

        ptr += len;
        buff_len -= len;
    }
}

//////////////////////////////////////////////////////////////////////

extern "C" esp_err_t i2c_master_write_write_device(i2c_port_t i2c_num, uint8_t dev_addr, uint8_t const *b1, size_t s1, uint8_t const *b2, size_t s2,
                                                   TickType_t timeout)
{
    static uint8_t i2c_buffer[I2C_LINK_RECOMMENDED_SIZE(8)];

    i2c_cmd_handle_t handle = i2c_cmd_link_create_static(i2c_buffer, sizeof(i2c_buffer));

    DEFER(i2c_cmd_link_delete_static(handle));

    ESP_RET(i2c_master_start(handle));

    ESP_RET(i2c_master_write_byte(handle, dev_addr << 1 | I2C_MASTER_WRITE, true));

    ESP_RET(i2c_master_write(handle, b1, s1, true));

    ESP_RET(i2c_master_write(handle, b2, s2, true));

    ESP_RET(i2c_master_stop(handle));

    ESP_RET(i2c_master_cmd_begin(i2c_num, handle, timeout));

    return ESP_OK;
}
