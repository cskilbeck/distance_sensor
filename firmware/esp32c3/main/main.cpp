//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "sdkconfig.h"

#include "soc/soc_caps.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "esp_http_client.h"
#include "esp_mac.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "driver/gpio.h"

#include "nvs_flash.h"

#include "cJSON.h"

#include "util.h"
#include "wifi.h"
#include "rtc.h"
#include "distance.h"
#include "http_client.h"
#include "hardware.h"

LOG_CONTEXT("main");

//////////////////////////////////////////////////////////////////////

namespace
{
    namespace config
    {
        // allow factory reset on long button press

        constexpr bool factory_reset_enabled = true;

        // wait this long for wifi to connect

        constexpr int wifi_timeout_ms = 30000;

        // led flash time when waiting for wifi

        constexpr int wifi_led_flash_time_ms = 20;

        // led flash rate when waiting for wifi

        constexpr int wifi_led_flash_rate_ms = 2000;

        // hold button for this long to factory reset

        constexpr int factory_reset_button_time_ms = 5000;

        // flash led for this long when button held

        constexpr int factory_reset_led_flash_time_ms = 50;

        // flash led at this rate when button held

        constexpr int factory_reset_led_flash_rate_ms = 500;

        // send readings to this server/port/path

        // constexpr char const *server_host = "192.168.4.52";
        char const *server_host = "vibue.com";
        constexpr char const *server_port = "5002";
        constexpr char const *server_path = "reading2";

        // retry http this many times (sometimes needs two, never seen it succeed after that)

        constexpr int http_retries = 3;

        // how long to sleep for, default 6 hours, but get it from the server when a reading is uploaded

        constexpr int32 default_sleep_seconds = 60 * 60 * 6;
        constexpr int32 max_sleep_seconds = 60 * 60 * 12;
        constexpr int32 min_sleep_seconds = 30;

    }    // namespace config

    //////////////////////////////////////////////////////////////////////
    // power admin

    void power_on()
    {
        LOG_WARN("Power ON");
        gpio_set_level(GPIO_POWER_ENABLE, 1);
    }

    //////////////////////////////////////////////////////////////////////

    void power_off()
    {
        LOG_WARN("Power OFF");

        LOG_FLUSH();

        gpio_set_level(GPIO_POWER_ENABLE, 0);
    }

    //////////////////////////////////////////////////////////////////////

    void power_init()
    {
        power_on();

        gpio_config_t conf{ .pin_bit_mask = 1 << GPIO_POWER_ENABLE,
                            .mode = GPIO_MODE_OUTPUT,
                            .pull_up_en = GPIO_PULLUP_DISABLE,
                            .pull_down_en = GPIO_PULLDOWN_DISABLE,
                            .intr_type = GPIO_INTR_DISABLE };
        gpio_config(&conf);
    }

    //////////////////////////////////////////////////////////////////////
    // led admin

    void led_on()
    {
        gpio_set_level(GPIO_LED, 0);
    }

    //////////////////////////////////////////////////////////////////////

    void led_off()
    {
        gpio_set_level(GPIO_LED, 1);
    }

    //////////////////////////////////////////////////////////////////////

    void led_init()
    {
        led_off();

        gpio_config_t conf{ .pin_bit_mask = 1 << GPIO_LED,
                            .mode = GPIO_MODE_OUTPUT,
                            .pull_up_en = GPIO_PULLUP_DISABLE,
                            .pull_down_en = GPIO_PULLDOWN_DISABLE,
                            .intr_type = GPIO_INTR_DISABLE };
        gpio_config(&conf);
    }

    //////////////////////////////////////////////////////////////////////

    void led_off_callback(TimerHandle_t t)
    {
        led_off();
    }

    //////////////////////////////////////////////////////////////////////
    // sleep for some milliseconds

    void MAYBE_UNUSED sleep(int ms)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    //////////////////////////////////////////////////////////////////////
    // read VBAT from ADC

    esp_err_t get_vbat(int *mv)
    {
        LOG_CONTEXT("vbat");

        // switch on VBAT_SNS

        gpio_set_level(GPIO_VBSNS_ENABLE, 1);

        gpio_config_t gpio_cfg{ .pin_bit_mask = 1 << GPIO_VBSNS_ENABLE,
                                .mode = GPIO_MODE_OUTPUT,
                                .pull_up_en = GPIO_PULLUP_DISABLE,
                                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                .intr_type = GPIO_INTR_DISABLE };
        gpio_config(&gpio_cfg);

        // allocate a oneshot adc 'unit' (whatever that is)

        adc_oneshot_unit_handle_t adc_handle;

        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_VBSNS_UNIT,
            .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_RET(adc_oneshot_new_unit(&init_config, &adc_handle));

        DEFER(adc_oneshot_del_unit(adc_handle));

        // allocate a calibration scheme

        adc_cali_handle_t cal_handle = NULL;

        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_VBSNS_UNIT,
            .chan = ADC_VBSNS_CHANNEL,
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ESP_RET(adc_cali_create_scheme_curve_fitting(&cali_config, &cal_handle));

        DEFER(adc_cali_delete_scheme_curve_fitting(cal_handle));

        // configure the adc channel

        adc_oneshot_chan_cfg_t chan_config = {
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ESP_RET(adc_oneshot_config_channel(adc_handle, ADC_VBSNS_CHANNEL, &chan_config));

        // do the adc reading

        int adc_raw;
        ESP_RET(adc_oneshot_read(adc_handle, ADC_VBSNS_CHANNEL, &adc_raw));

        // switch off VBAT_SNS

        gpio_set_level(GPIO_VBSNS_ENABLE, 0);

        // convert to millivolts

        int voltage;
        ESP_RET(adc_cali_raw_to_voltage(cal_handle, adc_raw, &voltage));

        LOG_INFO("ADC UNIT %d [%d] = %d (%d mV)", ADC_VBSNS_UNIT, ADC_VBSNS_CHANNEL, adc_raw, voltage);

        // undo the 50% resistor divider to get actual vbat

        *mv = voltage * 2;

        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////
    // get sleep_count from http response

    esp_err_t read_settings(char const *response_buffer, int32 *server_sleep_seconds)
    {
        LOG_CONTEXT("read_settings");

        cJSON *json = cJSON_Parse(response_buffer);

        if(json == null) {
            LOG_ERROR("Failed to parse response");
            return ESP_ERR_INVALID_RESPONSE;
        }
        DEFER(cJSON_Delete(json));

        cJSON const *settings = cJSON_GetObjectItem(json, "settings");

        if(settings == null) {
            LOG_ERROR("Can't find settings object");
            return ESP_ERR_INVALID_RESPONSE;
        }

        cJSON const *sleep_count_obj = cJSON_GetObjectItem(settings, "sleep_count");

        if(sleep_count_obj == null) {
            LOG_ERROR("Can't find sleep_count object");
            return ESP_ERR_INVALID_RESPONSE;
        }

        if(sleep_count_obj->type != cJSON_Number) {
            LOG_ERROR("sleep_count is not a number");
            return ESP_ERR_INVALID_RESPONSE;
        }

        int sleep_count_value = sleep_count_obj->valueint;

        if(sleep_count_value > config::max_sleep_seconds || sleep_count_value < config::min_sleep_seconds) {
            LOG_WARN("sleep_count %d out of range (%ld..%ld)", sleep_count_value, config::min_sleep_seconds, config::max_sleep_seconds);
            return ESP_ERR_INVALID_RESPONSE;
        }

        LOG_INFO("server sleep seconds: %d", sleep_count_value);

        *server_sleep_seconds = sleep_count_value;

        return ESP_OK;
    }

    //////////////////////////////////////////////////////////////////////
    // send a reading to the server and process the response

    esp_err_t send_reading(int16 distance_mm, int vbat_mv, int8 wifi_rssi, int32 *server_sleep_seconds)
    {
        // default mac address is used as unique device id

        uint8 mac[6];
        esp_efuse_mac_get_default(mac);

        char mac_address[13];
        sprintf(mac_address, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        LOG_VERBOSE("MAC ADDRESS: %s", mac_address);

        for(int tries = 0; tries < config::http_retries; ++tries) {

            char const *url_format = "http://%s:%s/%s?vbat=%d&distance=%d&flags=%d&device=%s&rssi=%d";

            static char url[256];
            sprintf(url, url_format, config::server_host, config::server_port, config::server_path, vbat_mv, distance_mm, 0, mac_address, wifi_rssi);

            static char response_buffer[256];

            static size_t response_size = sizeof(response_buffer);

            int response_code;

            if(http_request(HTTP_METHOD_PUT, url, &response_code, response_buffer, &response_size) == ESP_OK) {

                if(response_code < 300) {

                    return read_settings(response_buffer, server_sleep_seconds);
                }
                return ESP_OK;
            }
            sleep(100);
        }
        return ESP_ERR_TIMEOUT;
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

extern "C" void app_main(void)
{
    // disable brownout reboot, seems to be spurious?

    REG_CLR_BIT(RTC_CNTL_FIB_SEL_REG, RTC_CNTL_FIB_BOD_RST);
    REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ANA_RST_EN);

    esp_log_level_set("*", ESP_LOG_INFO);

    LOG_ERROR("===== MAIN =====");

    LOG_SET_LEVEL("gpio", ESP_LOG_WARN);
    LOG_SET_LEVEL("pp", ESP_LOG_WARN);
    LOG_SET_LEVEL("phy_init", ESP_LOG_WARN);
    LOG_SET_LEVEL("net80211", ESP_LOG_WARN);
    LOG_SET_LEVEL("wifi", ESP_LOG_WARN);
    LOG_SET_LEVEL("wifi_init", ESP_LOG_WARN);

    // init power (and switch it on)

    power_init();

    // get vbat

    int vbat_mv = 0;
    ESP_LOG(get_vbat(&vbat_mv));

    LOG_INFO("VBAT: %dmV", vbat_mv);

    // get the RTC clock

    rtc_clock_data_t clock_data;
    memset(&clock_data, 0, sizeof(clock_data));
    rtc_get_clock(&clock_data);

    LOG_INFO("Time %02x:%02x.%02x", clock_data.hours_bcd, clock_data.minutes_bcd, clock_data.seconds_bcd & 0x7f);

    // init the led (and make sure it's off)

    led_init();

    // start the wifi connecting

    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOG(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_LOG(ret);

    wifi_init();

    // read sleep_seconds from NVS

    int32 sleep_seconds = config::default_sleep_seconds;

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if(err != ESP_OK) {
        LOG_ERROR("Error opening NVS\\storage: %s", esp_err_to_name(err));
    } else {
        err = nvs_get_i32(nvs_handle, "sleep_seconds", &sleep_seconds);
        switch(err) {
        case ESP_OK:
            LOG_VERBOSE("Got NVS:sleep_seconds: %ld", sleep_seconds);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            LOG_INFO("sleep_seconds not initialized yet, defaulting to %ld seconds", config::default_sleep_seconds);
            break;
        default:
            LOG_ERROR("Error (%s) reading NVS:sleep_seconds", esp_err_to_name(err));
        }
    }

    // get value from TOF distance sensor

    int16 distance = 0;

    int distance_status = get_distance(&distance);

    if(distance_status == DISTANCE_SUCCESS) {

        LOG_INFO("DISTANCE: %d", distance);

    } else {

        LOG_ERROR("DISTANCE ERROR: %d", distance_status);
    }

    // flash LED while waiting for wifi to connect

    static StaticTimer_t led_timer;

    TimerHandle_t led_flash_timer = xTimerCreateStatic("led_off", pdMS_TO_TICKS(config::wifi_led_flash_time_ms), pdFALSE, null, led_off_callback, &led_timer);

    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed = 0;

    while(elapsed < pdMS_TO_TICKS(config::wifi_timeout_ms)) {

        LOG_INFO("Wait for wifi %lu", pdTICKS_TO_MS(pdMS_TO_TICKS(config::wifi_timeout_ms) - elapsed));

        uint32_t events = wifi_wait_for_event(pdMS_TO_TICKS(config::wifi_led_flash_rate_ms));

        if((events & wifi_event_connected) != 0) {

            // wifi connected, send the reading to the server

            LOG_INFO("Wifi connected");

            int32 server_sleep_seconds = config::default_sleep_seconds;

            ESP_LOG(send_reading(distance, vbat_mv, wifi_get_rssi(), &server_sleep_seconds));

            // if we got a new value for sleep_seconds, update it in NVS

            if(server_sleep_seconds != sleep_seconds) {

                sleep_seconds = server_sleep_seconds;

                LOG_INFO("saving new sleep_seconds (%ld) to NVS", sleep_seconds);

                ESP_LOG(nvs_set_i32(nvs_handle, "sleep_seconds", sleep_seconds));
                ESP_LOG(nvs_commit(nvs_handle));
            }

            // setup the alarm, must sleep for at least 30 seconds

            sleep_seconds = max(sleep_seconds, 30l);

            // get seconds elapsed today from initial clock reading

            int32 elapsed_seconds = rtc_seconds_from_clock_data(clock_data);

            // set the alarm

            rtc_set_alarm(elapsed_seconds + sleep_seconds, clock_data);

            // we're done, go to power off or factory reset

            break;
        }

        // flash led if event wait timed out

        if(events == 0) {

            led_on();
            xTimerStart(led_flash_timer, 0);
        }
        elapsed = xTaskGetTickCount() - now;
    }

    // try to power down everything except the RTC (including the ESP running this code...!)

    power_off();

    // If we're still here, they must be pressing the button.
    // So flash the led and if they hold it down for >N seconds, factory reset.
    // If they release the button during this time, it just powers off.

    LOG_WARN(">>> FACTORY RESET? <<<");

    now = xTaskGetTickCount();

    do {

        led_on();
        sleep(config::factory_reset_led_flash_time_ms);

        led_off();
        sleep(config::factory_reset_led_flash_rate_ms - config::factory_reset_led_flash_time_ms);

        elapsed = xTaskGetTickCount() - now;

        LOG_INFO("Factory reset maybe (%lu/%d)", pdTICKS_TO_MS(elapsed), config::factory_reset_button_time_ms);

    } while(elapsed < pdMS_TO_TICKS(config::factory_reset_button_time_ms));

    // power back on so factory reset completes even if they release the button at the wrong moment

    power_on();

    LOG_WARN("********** FACTORY RESET! **********");

    // flash LED rapidly for one second to let them know it's factory resetting

    for(int i = 0; i < 10; ++i) {
        led_on();
        sleep(20);
        led_off();
        sleep(80);
    }

    if(config::factory_reset_enabled) {
        wifi_factory_reset();
    } else {
        LOG_WARN("Factory reset disabled, just rebooting");
    }

    // must reboot after factory reset, can't seem to reinit the wifi properly...

    LOG_FLUSH();
    esp_restart();
}
