//////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include <memory.h>
#include "crc.h"

//////////////////////////////////////////////////////////////////////

void init_spi();

bool spi_send_now(message_t const *msg, message_t *reply);

//////////////////////////////////////////////////////////////////////

template <typename T> bool spi_send_msg_now(message_t *msg, message_t *reply)
{
    init_message<T>(msg);
    return spi_send_now(msg, reply);
}
