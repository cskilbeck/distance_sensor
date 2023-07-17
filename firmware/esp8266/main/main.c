#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "user.h"

static char const *TAG = "main";

void app_main()
{
    ESP_LOGI(TAG, "app_main");
    user_main();
}
