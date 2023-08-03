//////////////////////////////////////////////////////////////////////

//#include "esp8266/gpio_register.h"
#include "esp8266/gpio_struct.h"
#include "driver/gpio.h"
#include "led.h"

//////////////////////////////////////////////////////////////////////

void init_led()
{
    led_off();

    gpio_config_t p4_config;
    p4_config.pin_bit_mask = GPIO_Pin_LED;
    p4_config.mode = GPIO_MODE_OUTPUT;
    p4_config.pull_up_en = GPIO_PULLUP_DISABLE;
    p4_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    p4_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&p4_config);
}
