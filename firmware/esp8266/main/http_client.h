#pragma once

esp_err_t http_request(esp_http_client_method_t method, char const *url, char *response_buffer, size_t *response_buffer_size);
