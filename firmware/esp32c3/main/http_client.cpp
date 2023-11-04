//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_client.h"
#include "util.h"

LOG_TAG("http_client");

//////////////////////////////////////////////////////////////////////

namespace
{
    char const *HTTP_METHOD_MAPPING[] MAYBE_UNUSED = { "GET", "POST", "PUT", "PATCH", "DELETE", "HEAD", "NOTIFY", "SUBSCRIBE", "UNSUBSCRIBE", "OPTIONS" };

    //////////////////////////////////////////////////////////////////////

    esp_err_t http_event_handler(esp_http_client_event_t *evt)
    {
        static char *out_buf;    // Buffer to store response of http request from event handler
        static int out_len;      // Stores number of bytes read

        switch(evt->event_id) {

        case HTTP_EVENT_ERROR:
            LOG_D("HTTP_EVENT_ERROR");
            break;

        case HTTP_EVENT_ON_CONNECTED:
            LOG_D("HTTP_EVENT_ON_CONNECTED");
            break;

        case HTTP_EVENT_HEADER_SENT:
            LOG_D("HTTP_EVENT_HEADER_SENT");
            break;

        case HTTP_EVENT_ON_HEADER:
            LOG_D("HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;

        case HTTP_EVENT_ON_DATA:
            LOG_D("HTTP_EVENT_ON_DATA, len=%d", evt->data_len);

            //  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary
            //  data. However, event handler can also be used in case chunked encoding is used.

            if(!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                if(evt->user_data) {
                    memcpy(reinterpret_cast<uint8_t *>(evt->user_data) + out_len, evt->data, evt->data_len);
                } else {
                    if(out_buf == nullptr) {
                        out_len = 0;
                        out_buf = reinterpret_cast<char *>(malloc(esp_http_client_get_content_length(evt->client)));
                        if(out_buf == nullptr) {
                            LOG_E("Failed to allocate memory for output buffer");
                            return ESP_ERR_NO_MEM;
                        }
                    }
                    memcpy(out_buf + out_len, evt->data, evt->data_len);
                }
                out_len += evt->data_len;
            }
            break;

        case HTTP_EVENT_ON_FINISH:
            LOG_D("HTTP_EVENT_ON_FINISH");
            if(out_buf != nullptr) {
                free(out_buf);
                out_buf = nullptr;
                out_len = 0;
            }
            break;

        case HTTP_EVENT_DISCONNECTED:
            LOG_D("HTTP_EVENT_DISCONNECTED");
            break;

        case HTTP_EVENT_REDIRECT:
            break;
        }
        return ESP_OK;
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

esp_err_t http_request(esp_http_client_method_t method, char const *url, int *response_code, char *response_buffer, size_t *response_buffer_size)
{
    LOG_TAG("http_request");

    LOG_I("%s %s", HTTP_METHOD_MAPPING[method], url);

    esp_http_client_config_t config;
    memset(&config, 0, sizeof(config));

    config.url = url;
    config.event_handler = http_event_handler;
    config.user_data = response_buffer;
    config.method = method;
    config.timeout_ms = 3000;

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if(client == nullptr) {
        LOG_E("esp_http_client_init failed");
        return ESP_FAIL;
    }

    DEFER(esp_http_client_cleanup(client));

    ESP_RET(esp_http_client_perform(client));

    *response_buffer_size = esp_http_client_get_content_length(client);

    *response_code = esp_http_client_get_status_code(client);

    if(*response_code >= 300) {
        LOG_W("Status = %d", *response_code);
    }

    LOG_V("Response: %s", response_buffer);

    return ESP_OK;
}
