//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

#define SPI_DATA_SIZE 32

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
    uint16_t distance;
    uint16_t flags;
};

static constexpr uint32_t ch32_flag_factory_reset = 1 << 0;
static constexpr uint32_t ch32_flag_error_reading_vbat = 1 << 1;
static constexpr uint32_t ch32_flag_error_reading_distance = 1 << 2;
static constexpr uint32_t ch32_flag_error_distance_invalid = 1 << 3;

//////////////////////////////////////////////////////////////////////
// ESP->CH32 ESP has booted/connected/sent etc

struct esp_status_payload_t
{
    static uint8_t constexpr id = msg_id_esp_status;

    uint16_t flags;
};

static constexpr uint32_t esp_status_booted = 1 << 0;
static constexpr uint32_t esp_status_connected = 1 << 1;
static constexpr uint32_t esp_status_got_reading = 1 << 2;
static constexpr uint32_t esp_status_sent_reading = 1 << 3;
static constexpr uint32_t esp_status_send_error = 1 << 4;
static constexpr uint32_t esp_status_spi_error = 1 << 5;
static constexpr uint32_t esp_status_wifi_reset = 1 << 6;
static constexpr uint32_t esp_status_wifi_timeout = 1 << 7;
static constexpr uint32_t esp_status_factory_resetting = 1 << 8;
static constexpr uint32_t esp_status_done = 1 << 9;

//////////////////////////////////////////////////////////////////////

uint32_t calc_crc32(uint8_t const *buf, uint32_t len, uint32_t crc = 0xffffffff);

bool check_crc32(message_t const *msg);

//////////////////////////////////////////////////////////////////////

template <typename T> void init_message(message_t *msg)
{
    static_assert(sizeof(T) <= sizeof(message_body_t::payload));
    msg->body.ident = T::id;
    msg->body.length = sizeof(T);
    for(int i = sizeof(T); i < sizeof(message_body_t::payload); ++i) {
        msg->body.payload[i] = 0;
    }
    msg->crc = calc_crc32(reinterpret_cast<uint8_t const *>(msg), sizeof(message_body_t));
}
