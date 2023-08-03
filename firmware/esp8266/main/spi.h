#pragma once

#include <stdint.h>
#include <memory.h>
#include "crc.h"

typedef void (*on_spi_callback)(message_t const *msg);

void init_spi(on_spi_callback callback, on_spi_callback error_callback);
void spi_send(message_t const *msg);
bool spi_send_now(message_t const *msg, message_t *reply);

template <typename T> void spi_send_msg(message_t *msg)
{
    init_message<T>(msg);
    spi_send(msg);
}

template <typename T> bool spi_send_msg_now(message_t *msg, message_t *reply)
{
    init_message<T>(msg);
    return spi_send_now(msg, reply);
}
