//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_smartconfig.h"
#include "smartconfig_ack.h"

#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi.h"
#include "esp8266/gpio_struct.h"
#include "esp8266/spi_register.h"
#include "esp8266/spi_struct.h"

#include "util.h"
#include "types.h"
#include "crc.h"
#include "wifi.h"

//////////////////////////////////////////////////////////////////////

wifi_callback on_wifi_connected;
wifi_callback on_wifi_disconnected;

//////////////////////////////////////////////////////////////////////

namespace
{
    char const *TAG = "wifi";

    EventGroupHandle_t s_wifi_event_group;

    const int CONNECTED_BIT = BIT0;
    const int ESPTOUCH_DONE_BIT = BIT1;

    //////////////////////////////////////////////////////////////////////

    void smartconfig_example_task(void *parm)
    {
        ESP_LOGI(TAG, "SmartConfig task begins...");

        EventBits_t uxBits;

        ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS));

        smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));

        while(true) {

            uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

            if(uxBits & CONNECTED_BIT) {
                ESP_LOGI(TAG, "WiFi Connected to ap");
            }

            if(uxBits & ESPTOUCH_DONE_BIT) {
                esp_smartconfig_stop();
                ESP_LOGI(TAG, "SmartConfig STOPPING");
                vTaskDelete(NULL);
            }
        }
    }

    //////////////////////////////////////////////////////////////////////

    void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
    {
        ESP_LOGI(TAG, "WIFI EVENT: %s (%d)", event_base, event_id);

        if(event_base == WIFI_EVENT) {

            switch(event_id) {

            case WIFI_EVENT_STA_STOP:
                ESP_LOGI(TAG, "WIFI Stopped");
                break;

            case WIFI_EVENT_STA_START:
                xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
                if(on_wifi_disconnected != null) {
                    on_wifi_disconnected();
                }
                ESP_LOGI(TAG, "Reconnecting...");
                esp_wifi_connect();
                break;
            }
        }

        else if(event_base == IP_EVENT) {

            switch(event_id) {

            case IP_EVENT_STA_GOT_IP:
                xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
                if(on_wifi_connected != null) {
                    on_wifi_connected();
                }
                break;
            }
        }

        else if(event_base == SC_EVENT) {

            switch(event_id) {

            case SC_EVENT_SCAN_DONE:
                ESP_LOGI(TAG, "SC_EVENT_SCAN_DONE");
                break;

            case SC_EVENT_FOUND_CHANNEL:
                ESP_LOGI(TAG, "SC_EVENT_FOUND_CHANNEL");
                break;

            case SC_EVENT_GOT_SSID_PSWD: {
                ESP_LOGI(TAG, "SC_EVENT_GOT_SSID_PSWD");

                wifi_config_t wifi_config;
                uint8_t rvd_data[33] = { 0 };

                smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;

                bzero(&wifi_config, sizeof(wifi_config_t));
                memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
                memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
                wifi_config.sta.bssid_set = evt->bssid_set;

                if(wifi_config.sta.bssid_set == true) {
                    memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
                }

                // uint8_t ssid[33] = { 0 };
                // uint8_t password[65] = { 0 };
                // memcpy(ssid, evt->ssid, sizeof(evt->ssid));
                // memcpy(password, evt->password, sizeof(evt->password));
                // ESP_LOGI(TAG, "SSID:%s", ssid);
                // ESP_LOGI(TAG, "PASSWORD:%s", password);

                if(evt->type == SC_TYPE_ESPTOUCH_V2) {
                    ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
                    ESP_LOGI(TAG, "RVD_DATA:%s", rvd_data);
                }

                ESP_ERROR_CHECK(esp_wifi_disconnect());
                ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

                SUPPRESS_DEPRECATED
                ESP_ERROR_CHECK(esp_wifi_set_auto_connect(true));
                SUPPRESS_POP

                ESP_ERROR_CHECK(esp_wifi_connect());
            } break;

            case SC_EVENT_SEND_ACK_DONE:
                ESP_LOGI(TAG, "SC_EVENT_SEND_ACK_DONE");
                xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
                break;
            }
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

void deinit_wifi()
{
    ESP_LOGI(TAG, "de-init");

    esp_wifi_stop();
    esp_wifi_start();
}

//////////////////////////////////////////////////////////////////////

void init_wifi()
{
    ESP_LOGI(TAG, "init");

    tcpip_adapter_init();

    s_wifi_event_group = xEventGroupCreate();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    wifi_config_t saved_config;
    memset(&saved_config, 0, sizeof(saved_config));

    ESP_ERROR_CHECK(esp_wifi_get_config(ESP_IF_WIFI_STA, &saved_config));

    ESP_LOGI(TAG, "Saved SSID: [%s]", saved_config.ap.ssid);

    if(saved_config.ap.ssid[0] == 0 || saved_config.ap.password[0] == 0) {
        ESP_LOGI(TAG, "NO SAVED WIFI CONFIG!");
    }

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_LOGI(TAG, "esp_wifi_start()...");

    ESP_ERROR_CHECK(esp_wifi_start());

    bool auto_connect;

    SUPPRESS_DEPRECATED
    esp_wifi_get_auto_connect(&auto_connect);
    SUPPRESS_POP

    ESP_LOGI(TAG, "Auto connect: %d", auto_connect);

    if(auto_connect) {
        esp_wifi_connect();
    }
}
