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
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "smartconfig_ack.h"

#include "util.h"
#include "wifi.h"

LOG_TAG("wifi");

//////////////////////////////////////////////////////////////////////

wifi_callback on_wifi_connected;
wifi_callback on_wifi_disconnected;

//////////////////////////////////////////////////////////////////////

namespace
{
    EventGroupHandle_t s_wifi_event_group;

    const int CONNECTED_BIT = BIT0;
    const int ESPTOUCH_DONE_BIT = BIT1;

    bool wifi_got_credentials = false;

    int8 rssi = 0;

    //////////////////////////////////////////////////////////////////////

    void smartconfig_task(void *parm)
    {
        EventBits_t uxBits;

        ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS));

        smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));

        while(true) {

            uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

            if(uxBits & CONNECTED_BIT) {
                LOG_I("WiFi Connected to ap");
            }

            if(uxBits & ESPTOUCH_DONE_BIT) {
                esp_smartconfig_stop();
                LOG_I("SmartConfig STOPPING");
                vTaskDelete(NULL);
            }
        }
    }

    //////////////////////////////////////////////////////////////////////

    void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
    {
        LOG_I("WIFI EVENT: %s (%ld)", event_base, event_id);

        if(event_base == WIFI_EVENT) {

            switch(event_id) {

            case WIFI_EVENT_STA_STOP:
                LOG_I("WIFI Stopped");
                break;

            case WIFI_EVENT_STA_START:
                if(!wifi_got_credentials) {
                    xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
                }
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                rssi = 0;
                xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
                if(on_wifi_disconnected != null) {
                    on_wifi_disconnected();
                }
                LOG_I("Reconnecting...");
                esp_wifi_connect();
                break;
            }
        }

        else if(event_base == IP_EVENT) {

            switch(event_id) {

            case IP_EVENT_STA_GOT_IP:
                xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
                wifi_ap_record_t access_point;
                esp_wifi_sta_get_ap_info(&access_point);
                rssi = access_point.rssi;
                if(on_wifi_connected != null) {
                    on_wifi_connected();
                }
                break;
            }
        }

        else if(event_base == SC_EVENT) {

            switch(event_id) {

            case SC_EVENT_SCAN_DONE:
                LOG_I("SC_EVENT_SCAN_DONE");
                break;

            case SC_EVENT_FOUND_CHANNEL:
                LOG_I("SC_EVENT_FOUND_CHANNEL");
                break;

            case SC_EVENT_GOT_SSID_PSWD: {
                LOG_I("SC_EVENT_GOT_SSID_PSWD");

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
                // LOG_I("SSID:%s", ssid);
                // LOG_I("PASSWORD:%s", password);

                if(evt->type == SC_TYPE_ESPTOUCH_V2) {
                    ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
                    LOG_I("RVD_DATA:%s", rvd_data);
                }

                ESP_ERROR_CHECK(esp_wifi_disconnect());
                ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

                ESP_ERROR_CHECK(esp_wifi_connect());
                // led_set_flash_mode(led_flash_mode_t::fast, 100);
            } break;

            case SC_EVENT_SEND_ACK_DONE:
                LOG_I("SC_EVENT_SEND_ACK_DONE");
                xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
                break;
            }
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

esp_err_t wifi_factory_reset()
{
    ESP_RET(nvs_flash_erase());
    ESP_RET(nvs_flash_init());
    esp_restart();
    return ESP_OK;    // unreachable
}

//////////////////////////////////////////////////////////////////////

esp_err_t wifi_init()
{
    LOG_I("init");

    rssi = 0;

    ESP_RET(esp_netif_init());

    s_wifi_event_group = xEventGroupCreate();

    ESP_RET(esp_event_loop_create_default());

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    if(sta_netif == null) {
        return ESP_ERR_ESP_NETIF_NO_MEM;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_RET(esp_wifi_init(&cfg));

    wifi_config_t saved_config;
    memset(&saved_config, 0, sizeof(saved_config));

    ESP_RET(esp_wifi_get_config(WIFI_IF_STA, &saved_config));

    wifi_got_credentials = saved_config.ap.ssid[0] != 0 && saved_config.ap.password[0] != 0;

    // LOG_D("Saved SSID: [%s]", saved_config.ap.ssid);
    // LOG_D("Saved PASS: [%s]", saved_config.ap.password);

    ESP_RET(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_RET(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_RET(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    ESP_RET(esp_wifi_set_mode(WIFI_MODE_STA));

    LOG_I("esp_wifi_start()...");

    ESP_RET(esp_wifi_start());

    if(wifi_got_credentials) {
        ESP_RET(esp_wifi_connect());
    } else {
        LOG_I("NO SAVED WIFI CONFIG!");
    }
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

int8 wifi_get_rssi()
{
    return rssi;
}
