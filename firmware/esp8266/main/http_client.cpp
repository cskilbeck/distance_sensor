//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_client.h"

//////////////////////////////////////////////////////////////////////

namespace
{
    char const *TAG = "http_client";

    size_t constexpr MAX_HTTP_OUTPUT_BUFFER = 1024;

    //////////////////////////////////////////////////////////////////////

    esp_err_t http_event_handler(esp_http_client_event_t *evt)
    {
        static char *output_buffer;    // Buffer to store response of http request from event handler
        static int output_len;         // Stores number of bytes read

        switch(evt->event_id) {

        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;

        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;

        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;

        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;

        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);

            //  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
            //  However, event handler can also be used in case chunked encoding is used.

            if(!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                if(evt->user_data) {
                    memcpy(reinterpret_cast<uint8_t *>(evt->user_data) + output_len, evt->data, evt->data_len);
                } else {
                    if(output_buffer == NULL) {
                        output_buffer = (char *)malloc(esp_http_client_get_content_length(evt->client));
                        output_len = 0;
                        if(output_buffer == NULL) {
                            ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    memcpy(output_buffer + output_len, evt->data, evt->data_len);
                }
                output_len += evt->data_len;
            }
            break;

        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if(output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
                output_len = 0;
            }
            break;

        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        }
        return ESP_OK;
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

esp_err_t http_get(char const *url)
{
    char const *TAG = "http_get";

    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = { 0 };

    esp_http_client_config_t config;
    memset(&config, 0, sizeof(config));

    config.url = url;
    config.event_handler = http_event_handler;
    config.user_data = local_response_buffer;
    config.method = HTTP_METHOD_GET;
    config.timeout_ms = 500;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if(err == ESP_OK) {

        ESP_LOGI(TAG, "Status = %d, content_length = %d", esp_http_client_get_status_code(client), esp_http_client_get_content_length(client));
        ESP_LOGI(TAG, "Response: %s", local_response_buffer);

    } else {

        ESP_LOGE(TAG, "Request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}
