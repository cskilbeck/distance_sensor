#pragma once

esp_err_t http_request(esp_http_client_method_t method, char const *url, int *response_code, char *response_buffer, size_t *response_buffer_size,
                       void const *post_data, size_t post_size);
