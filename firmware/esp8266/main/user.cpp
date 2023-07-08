//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp8266/gpio_struct.h"
#include "esp_log.h"

#include "user.h"

//////////////////////////////////////////////////////////////////////

#define BUF_SIZE (128)

static char const *TAG = "user";

//////////////////////////////////////////////////////////////////////

uint16 crc16(message_body_t const *m)
{
    uint16 crc = 0xffff;
    byte const *p = reinterpret_cast<byte const *>(m);
    byte const *e = p + sizeof(message_body_t);
    for(; p < e; ++p) {
        uint16 x = (crc >> 8) ^ *p;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    }
    return crc;
}

//////////////////////////////////////////////////////////////////////

bool is_valid_message(message_t *m)
{
    return m->msg.signature == 'DC' && crc16(&m->msg) == m->crc;
}

//////////////////////////////////////////////////////////////////////

void init_message(message_t *message, uint64_t timestamp, uint32_t options)
{
    message->msg.timestamp = timestamp;
    message->msg.options = options;
    message->msg.signature = 'DC';
    message->crc = crc16(&message->msg);
}

//////////////////////////////////////////////////////////////////////

message_t message;

extern "C" void user_main()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, "CPU cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "Silicon revision: %d, ", chip_info.revision);
    ESP_LOGI(TAG, "Flash: %dMB %s", spi_flash_get_chip_size() / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI(TAG, "==============================");

    // uart_config_t uart_config = { .baud_rate = 115200,
    //                               .data_bits = UART_DATA_8_BITS,
    //                               .parity = UART_PARITY_DISABLE,
    //                               .stop_bits = UART_STOP_BITS_1,
    //                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //                               .rx_flow_ctrl_thresh = 0 };
    // uart_param_config(UART_NUM_1, &uart_config);
    // uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    while(true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
