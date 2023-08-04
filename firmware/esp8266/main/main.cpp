//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp8266/gpio_register.h"
#include "esp8266/gpio_struct.h"
#include "driver/gpio.h"

#include "esp_spi_flash.h"

#include "util.h"
#include "types.h"
#include "crc.h"
#include "wifi.h"
#include "spi.h"
#include "led.h"
#include "http_client.h"

//////////////////////////////////////////////////////////////////////

namespace
{
    static char const *TAG = "main";

    // default mac address is used as unique device id

    char mac_addr[13];

    // char const *server_host = "192.168.4.52";
    char const *server_host = "vibue.com";
    char const *server_port = "5002";
    char const *server_path = "reading";

    constexpr int http_retries = 2;

    // for notifying main task about all the things

    EventGroupHandle_t main_event_bits;

    constexpr uint32 main_wifi_connected = BIT0;
    constexpr uint32 main_wifi_disconnected = BIT1;
    constexpr uint32 main_bit_mask = BIT0 | BIT1;

    // spi message from ch32

    message_t spi_msg_received;

    // spi message to send esp status to ch32

    message_t status_msg;

    esp_status_payload_t *status = reinterpret_cast<esp_status_payload_t *>(&status_msg.body.payload);

    //////////////////////////////////////////////////////////////////////
    // wifi got connected so notify ch32 and send reading (if we have one)

    void wifi_connected_callback()
    {
        xEventGroupSetBits(main_event_bits, main_wifi_connected);
        ESP_LOGI(TAG, "WIFI CONNECTED!");
    }

    //////////////////////////////////////////////////////////////////////

    void wifi_disconnected_callback()
    {
        ESP_LOGI(TAG, "WIFI DISCONNECTED!");
        xEventGroupSetBits(main_event_bits, main_wifi_disconnected);
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

extern "C" void app_main()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint32_t flash_size = spi_flash_get_chip_size() / (1024 * 1024);
    char const *flash_type = (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external";

    ESP_LOGI(TAG, "\n\n==============================");
    ESP_LOGI(TAG, "CPU cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "Silicon revision: %d, ", chip_info.revision);
    ESP_LOGI(TAG, "Flash: %dMB %s", flash_size, flash_type);
    ESP_LOGI(TAG, "==============================");

    // use hex representation of default mac address as device_id

    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    sprintf(mac_addr, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "MAC ADDRESS: %s", mac_addr);

    led_init();
    spi_init();

    // give CH32 ~1/50th of a second to get SPI slave ready
    vTaskDelay(2);

    main_event_bits = xEventGroupCreate();

    // start the wifi connecting

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    on_wifi_connected = wifi_connected_callback;
    on_wifi_disconnected = wifi_disconnected_callback;

    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();

    status->flags = esp_status_booted;

    TickType_t tick_start = xTaskGetTickCount();

    while(true) {

        // do the SPI transaction
        if(!spi_send_msg_now<esp_status_payload_t>(&status_msg, &spi_msg_received) || spi_msg_received.body.ident != msg_id_ch32_readings) {

            // if SPI error, report and wait for 1/2 a second
            ESP_LOGE(TAG, "Expected ID %d, got ID %d, expected CRC %08x, got CRC %08x", msg_id_ch32_readings, spi_msg_received.body.ident,
                     calc_crc32(reinterpret_cast<uint8_t const *>(&spi_msg_received.body), sizeof(message_body_t)), spi_msg_received.crc);
            status->flags |= esp_status_spi_error;
            log_buffer(TAG, &spi_msg_received, 32, ESP_LOG_INFO);
            vTaskDelay(50);

        } else {

            status->flags &= ~esp_status_spi_error;

            ch32_reading_payload_t const *payload = reinterpret_cast<ch32_reading_payload_t const *>(&spi_msg_received.body.payload);

            // if factory reset requested
            if((payload->flags & ch32_flag_factory_reset) != 0) {

                // notify ch32 that we will factory reset
                status->flags |= esp_status_factory_resetting;

            } else {

                // factory reset was requested, ch32 has cleared its factory reset flag so... do it
                if((status->flags & esp_status_factory_resetting) != 0) {

                    // murder wifi settings and reboot
                    ESP_LOGI(TAG, "Factory reset baby!");
                    fflush(stdout);

                    esp_wifi_restore();

                    SUPPRESS_DEPRECATED
                    esp_wifi_set_auto_connect(false);
                    SUPPRESS_POP

                    esp_restart();

                } else {

                    // if wifi is up and we haven't sent (or failed to send) the reading to the server
                    if((status->flags & esp_status_connected) != 0 && (status->flags & (esp_status_sent_reading | esp_status_send_error)) == 0) {

                        // send reading to the server
                        for(int tries = 0; tries < http_retries; ++tries) {

                            char const *url_format = "http://%s:%s/%s?vbat=%d&distance=%d&flags=%d&device=%s";

                            static char buffer[500];
                            sprintf(buffer, url_format, server_host, server_port, server_path, payload->vbat, payload->distance, payload->flags, mac_addr);

                            if(http_get(buffer) == ESP_OK) {

                                status->flags |= esp_status_sent_reading;
                                break;
                            }
                            vTaskDelay(10);
                        }

                        // notify ch32 whether that was successful (either way, it will switch us off)
                        if((status->flags & esp_status_sent_reading) == 0) {

                            status->flags |= esp_status_send_error;
                        }

                    } else {

                        // if wifi connect takes > 30 seconds
                        if((xTaskGetTickCount() - tick_start) > pdMS_TO_TICKS(30000)) {

                            // notify ch32 to give up and switch us off
                            status->flags |= esp_status_wifi_timeout;
                        }
                    }

                    // monitor wifi status at 10Hz
                    uint32_t bits = xEventGroupWaitBits(main_event_bits, main_bit_mask, true, false, 100);

                    if((bits & main_wifi_connected) != 0) {

                        status->flags |= esp_status_connected;

                    } else if((bits & main_wifi_disconnected) != 0) {

                        status->flags &= ~esp_status_connected;
                    }
                }
            }
        }
    }
}