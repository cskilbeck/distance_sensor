//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

#define SPI_DATA_SIZE 32

#define NUM_DISTANCE_READINGS 12    // (SPI_DATA_SIZE - crc(4) - ident(1) - length(1) - vbat(2) ) / sizeof(uint16)

//////////////////////////////////////////////////////////////////////

struct message_body_t
{
    uint8_t ident;                                 // type of message
    uint8_t length;                                // how many bytes in the payload
    uint8_t payload[SPI_DATA_SIZE - 4 - 1 - 1];    // message contents
};

//////////////////////////////////////////////////////////////////////

struct message_t
{
    message_body_t body;
    uint32_t crc;
};

static_assert(sizeof(message_t) == SPI_DATA_SIZE);

//////////////////////////////////////////////////////////////////////

enum msg_index
{
    msg_id_ch32_readings = 1,
    msg_id_esp_status
};

//////////////////////////////////////////////////////////////////////
// CH32->ESP here are some readings

struct ch32_reading_payload_t
{
    static uint8_t constexpr id = msg_id_ch32_readings;

    uint16_t vbat;
    uint16_t distance[NUM_DISTANCE_READINGS];
};

//////////////////////////////////////////////////////////////////////
// ESP->CH32 ESP has booted

struct esp_status_payload_t
{
    static uint8_t constexpr id = msg_id_esp_status;

    uint32_t status;
};

//////////////////////////////////////////////////////////////////////

uint32_t calc_crc32(uint8_t const *buf, uint32_t len, uint32_t crc = 0xffffffff);

bool check_crc32(message_t const *msg);

//////////////////////////////////////////////////////////////////////

template <typename T> void init_message(message_t *msg)
{
    static_assert(sizeof(T) <= sizeof(message_body_t::payload));
    msg->body.ident = T::id;
    msg->body.length = sizeof(T);
    memset(msg->body.payload + sizeof(T), 0, sizeof(message_body_t::payload) - sizeof(T));
    msg->crc = calc_crc32(reinterpret_cast<uint8_t const *>(msg), sizeof(message_body_t));
}
