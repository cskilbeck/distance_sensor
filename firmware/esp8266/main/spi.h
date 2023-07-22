#pragma once

#include <stdint.h>
#include <memory.h>
#include "crc.h"

typedef void (*on_spi_callback)(message_t const *msg);

void init_spi(on_spi_callback callback);
void spi_send(message_t const *msg);