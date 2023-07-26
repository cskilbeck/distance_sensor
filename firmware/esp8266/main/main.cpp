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

#include "esp_spi_flash.h"

#include "util.h"
#include "types.h"
#include "crc.h"
#include "wifi.h"
#include "spi.h"
#include "http_client.h"

//////////////////////////////////////////////////////////////////////

namespace
{
    static char const *TAG = "main";

    // default mac address is used as unique device id

    char mac_addr_str[13];

    // server is here

    // char const *server_ip = "192.168.4.52";
    char const *server_ip = "vibue.com";
    char const *server_port = "5002";
    char const *server_path = "reading";

    // for notifying when to send readings to server

    EventGroupHandle_t send_event_bits;

    // copy of the message from ch32 with the readings in it

    message_t spi_msg_received;

    // a message for sending esp status to ch32

    message_t status_msg;
    esp_status_payload_t *status = reinterpret_cast<esp_status_payload_t *>(&status_msg.body.payload);

    //////////////////////////////////////////////////////////////////////
    // if all is in order, signal main task to send reading to the server
    // if we're not ready yet, do nothing

    void send_reading()
    {
        uint32_t constexpr mask = esp_status_connected | esp_status_got_reading | esp_status_sent_reading;
        uint32_t constexpr ready = esp_status_connected | esp_status_got_reading;

        if((status->flags & mask) == ready) {
            ESP_LOGI(TAG, "Send reading (%04x): Yes", status->flags);
            xEventGroupSetBits(send_event_bits, 1);
        } else {
            ESP_LOGI(TAG, "Send reading (%04x): No", status->flags);
        }
    }

    //////////////////////////////////////////////////////////////////////
    // a spi transaction completed
    // if the incoming was a reading packet (msg_id_ch32_readings)
    // then copy off the message and send reading to server (if we're connected)

    void spi_received(message_t const *msg)
    {
        ESP_LOGI(TAG, "GOT msg id %d (len %d)", msg->body.ident, msg->body.length);

        if(msg->body.ident == msg_id_ch32_readings) {

            memcpy(&spi_msg_received, msg, sizeof(message_t));
            status->flags |= esp_status_got_reading;

            send_reading();
        }
    }

    //////////////////////////////////////////////////////////////////////

    void spi_error(message_t const *msg)
    {
        ESP_LOGI(TAG, "SPI ERROR");
        status->flags |= esp_status_spi_error;
    }

    //////////////////////////////////////////////////////////////////////
    // wifi got connected so notify ch32 and send reading (if we have one)

    void wifi_connected_callback()
    {
        ESP_LOGI(TAG, "WIFI CONNECTED!");
        status->flags |= esp_status_connected;
        spi_send_msg<esp_status_payload_t>(&status_msg);
        send_reading();
    }

    //////////////////////////////////////////////////////////////////////

    void wifi_disconnected_callback()
    {
        ESP_LOGI(TAG, "WIFI DISCONNECTED!");
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
    sprintf(mac_addr_str, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "MAC ADDRESS: %s", mac_addr_str);

    // setup the spi hardware and set the callback for when stuff is received

    init_spi(spi_received, spi_error);

    // give CH32 1/10th of a second to get SPI slave ready
    vTaskDelay(10);

    // create the task which will send the reading to the server

    send_event_bits = xEventGroupCreate();

    // start the wifi connecting

    on_wifi_connected = wifi_connected_callback;
    on_wifi_disconnected = wifi_disconnected_callback;

    ESP_ERROR_CHECK(nvs_flash_init());
    initialise_wifi();

    // notify ch32 that we've booted up

    status->flags = esp_status_booted;
    spi_send_msg<esp_status_payload_t>(&status_msg);

    // now wait for the signal to send the reading to the server

    ESP_LOGI(TAG, "Waiting to send...");
    xEventGroupWaitBits(send_event_bits, 1, false, true, portMAX_DELAY);

    ESP_LOGI(TAG, "sending!!!");

    bool sent = false;

    ch32_reading_payload_t const *reading = reinterpret_cast<ch32_reading_payload_t const *>(&spi_msg_received.body.payload);

    if((reading->flags & ch32_flag_factory_reset) != 0) {

        ESP_LOGI(TAG, "FACTORY RESET BNABNY!");
        // clear the wifi persistent credentials flag
        esp_wifi_restore();

    } else {

        for(int tries = 0; tries < 2; ++tries) {

            static char url[500];
            sprintf(url, "http://%s:%s/%s?vbat=%d&distance=%d&device=%s", server_ip, server_port, server_path, reading->vbat, reading->distance, mac_addr_str);
            if(http_get(url) == ESP_OK) {
                status->flags |= esp_status_sent_reading;
                sent = true;
                break;
            }
            ESP_LOGI(TAG, "failed, waiting...");
            vTaskDelay(100);
        }
    }

    if(!sent) {
        status->flags |= esp_status_send_error;
    }

    // keep sending status to ch32, it should switch us off when it gets this

    while(true) {
        spi_send_msg<esp_status_payload_t>(&status_msg);
        vTaskDelay(100);
    }
}
