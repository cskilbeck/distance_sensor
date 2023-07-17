#pragma once

uint32_t calc_crc32(uint8_t const *buf, uint32_t len, uint32_t init = 0xffffffff);
