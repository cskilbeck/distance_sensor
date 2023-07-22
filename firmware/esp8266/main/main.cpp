//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "esp_spi_flash.h"

#include "util.h"
#include "types.h"
#include "crc.h"
#include "wifi.h"
#include "spi.h"

//////////////////////////////////////////////////////////////////////

static char const *TAG = "main";

//////////////////////////////////////////////////////////////////////

void spi_received(message_t const *msg)
{
    ESP_LOGI(TAG, "GOT msg id %d (len %d)", msg->body.ident, msg->body.length);
}

//////////////////////////////////////////////////////////////////////

extern "C" void app_main()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, "CPU cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "Silicon revision: %d, ", chip_info.revision);
    ESP_LOGI(TAG, "Flash: %dMB %s", spi_flash_get_chip_size() / (1024 * 1024), (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI(TAG, "==============================");

    init_spi(spi_received);

    ESP_ERROR_CHECK(nvs_flash_init());
    initialise_wifi();

    message_t msg;
    esp_status_payload_t *payload = reinterpret_cast<esp_status_payload_t *>(&msg.body.payload);
    payload->status = 0xffffffff;
    init_message<esp_status_payload_t>(&msg);

    while(true) {

        vTaskDelay(2000 / portTICK_PERIOD_MS);

        spi_send(&msg);
    }
}
