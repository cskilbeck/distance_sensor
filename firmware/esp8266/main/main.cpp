//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "cJSON.h"

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

    // for notifying main task about wifi status

    EventGroupHandle_t wifi_event_bits;

    uint32 constexpr main_wifi_connected = BIT0;
    uint32 constexpr main_wifi_disconnected = BIT1;
    uint32 constexpr main_wifi_bit_mask = BIT0 | BIT1;

    // longest valid sleep_count (20000 = ~10000 minutes = ~7 days (very roughly))

    int constexpr max_sleep_count = 20000;

    int8 wifi_rssi;

    // spi message from ch32

    message_t spi_rx_msg;

    ch32_reading_payload_t const &payload = *reinterpret_cast<ch32_reading_payload_t const *>(&spi_rx_msg.body.payload);

    // spi message to send esp status to ch32

    message_t spi_tx_msg;

    esp_status_payload_t &status = *reinterpret_cast<esp_status_payload_t *>(&spi_tx_msg.body.payload);

    //////////////////////////////////////////////////////////////////////
    // wifi got connected so notify ch32 and send reading (if we have one)

    void wifi_connected_callback()
    {
        wifi_rssi = wifi_get_rssi();
        xEventGroupSetBits(wifi_event_bits, main_wifi_connected);
        ESP_LOGI(TAG, "WIFI CONNECTED!");
    }

    //////////////////////////////////////////////////////////////////////

    void wifi_disconnected_callback()
    {
        ESP_LOGI(TAG, "WIFI DISCONNECTED!");
        xEventGroupSetBits(wifi_event_bits, main_wifi_disconnected);
    }

    //////////////////////////////////////////////////////////////////////

    void read_settings(char const *response_buffer)
    {
        status.sleep_count = 0;

        cJSON *json = cJSON_Parse(response_buffer);

        if(json == nullptr) {
            ESP_LOGE(TAG, "Failed to parse response");
            return;
        }
        DEFER(cJSON_Delete(json));

        cJSON *settings = cJSON_GetObjectItem(json, "settings");

        if(settings == nullptr) {
            ESP_LOGE(TAG, "Can't find settings object");
            return;
        }

        cJSON *sleep_count = cJSON_GetObjectItem(settings, "sleep_count");

        if(sleep_count == nullptr) {
            ESP_LOGE(TAG, "Can't find sleep_count object");
            return;
        }

        if(sleep_count->type != cJSON_Number) {
            ESP_LOGE(TAG, "sleep_count is not a number");
            return;
        }

        int sleep_count_value = sleep_count->valueint;

        if(sleep_count_value > max_sleep_count || sleep_count_value < 1) {
            ESP_LOGE(TAG, "sleep_count %d out of range (1..%d)", sleep_count_value, max_sleep_count);
            return;
        }

        ESP_LOGI(TAG, "sleep_count: %d", sleep_count_value);
        status.sleep_count = static_cast<uint16>(sleep_count_value);
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

extern "C" void app_main()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint32_t flash_size = spi_flash_get_chip_size() / (1024 * 1024);
    char const *flash_type = (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external";

    ESP_LOGI(TAG, "\n\n");
    ESP_LOGI(TAG, "==============================");
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

    // give CH32 at least 0.1 seconds to get SPI slave ready (probly not necessary)
    vTaskDelay(pdMS_TO_TICKS(10));

    // start the wifi connecting

    wifi_event_bits = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    on_wifi_connected = wifi_connected_callback;
    on_wifi_disconnected = wifi_disconnected_callback;

    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();

    // main loop

    status.flags = esp_status_booted;

    TickType_t tick_start = xTaskGetTickCount();

    while(true) {

        // do an SPI transaction
        if(!spi_send_msg_now<esp_status_payload_t>(&spi_tx_msg, &spi_rx_msg) || spi_rx_msg.body.ident != msg_id_ch32_readings) {

            // if SPI error, report and wait for 1/2 a second
            ESP_LOGE(TAG, "Expected ID %d, got ID %d, expected CRC %08x, got CRC %08x", msg_id_ch32_readings, spi_rx_msg.body.ident, calc_msg_crc(&spi_rx_msg),
                     spi_rx_msg.crc);

            log_buffer(TAG, &spi_rx_msg, 32, ESP_LOG_INFO);

            status.flags |= esp_status_spi_error;
            vTaskDelay(pdMS_TO_TICKS(500));

        } else {

            status.flags &= ~esp_status_spi_error;

            // if factory reset requested
            if((payload.flags & ch32_flag_factory_reset) != 0) {

                // notify ch32 that we will factory reset
                status.flags |= esp_status_factory_resetting;

            } else {

                // factory reset was requested, ch32 has cleared its factory reset flag so... do it
                if((status.flags & esp_status_factory_resetting) != 0) {

                    // murder wifi settings and reboot
                    ESP_LOGI(TAG, "Factory reset baby!");
                    fflush(stdout);

                    esp_wifi_restore();

                    SUPPRESS_DEPRECATED
                    esp_wifi_set_auto_connect(false);
                    SUPPRESS_POP

                    esp_restart();

                    // haven't sent (or failed to send) the reading yet
                } else if((status.flags & (esp_status_sent_reading | esp_status_send_error)) == 0) {

                    // if wifi is up
                    if((status.flags & esp_status_connected) != 0) {

                        // send reading to the server
                        for(int tries = 0; tries < http_retries; ++tries) {

                            char const *url_format = "http://%s:%s/%s?vbat=%d&distance=%d&flags=%d&device=%s&rssi=%d&resolution=%d";

                            static char url[256];
                            sprintf(url, url_format, server_host, server_port, server_path, payload.vbat, payload.distance, payload.flags, mac_addr, wifi_rssi,
                                    payload.resolution);

                            static char response_buffer[256];

                            static size_t response_size = sizeof(response_buffer);

                            int response_code;

                            if(http_request(HTTP_METHOD_PUT, url, &response_code, response_buffer, &response_size) == ESP_OK) {


                                if(response_code < 300) {
                                    read_settings(response_buffer);
                                }
                                status.flags |= esp_status_sent_reading;
                                break;
                            }
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }

                        // notify ch32 whether that was successful (either way, it will switch us off)
                        if((status.flags & esp_status_sent_reading) == 0) {

                            status.flags |= esp_status_send_error;
                        }

                        // wifi still not up, if wifi connect takes > 60 seconds
                    } else if((xTaskGetTickCount() - tick_start) > pdMS_TO_TICKS(60000)) {

                        // notify ch32 to give up and switch us off
                        status.flags |= esp_status_wifi_timeout;

                    } else {

                        // monitor wifi status at 10Hz
                        uint32_t events = xEventGroupWaitBits(wifi_event_bits, main_wifi_bit_mask, true, false, pdMS_TO_TICKS(100));

                        if((events & main_wifi_connected) != 0) {

                            status.flags |= esp_status_connected;

                        } else if((events & main_wifi_disconnected) != 0) {

                            status.flags &= ~esp_status_connected;
                        }
                    }

                } else {

                    break;
                }
            }
        }
    }
    ESP_LOGI(TAG, "All done, waiting to get switched off...");
    vTaskDelay(portMAX_DELAY);
}
