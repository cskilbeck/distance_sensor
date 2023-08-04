//////////////////////////////////////////////////////////////////////

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp8266/gpio_struct.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "util.h"
#include "led.h"

//////////////////////////////////////////////////////////////////////

namespace
{
    char const *TAG = "led";

    struct led_command_t
    {
        uint32 flash_mode : 2;
        uint32 timeout : 30;
    };

    struct flash_config_t
    {
        int on_ticks;
        int off_ticks;
    };

    TaskHandle_t led_task;
    QueueHandle_t led_command_queue;

    flash_config_t const flash_configs[num_flash_modes] = {

        { 0, 9 },    // OFF     on for 0.0, off for 0.9
        { 1, 9 },    // SLOW    on for 0.1, off for 0.9
        { 1, 4 },    // MEDIUM  on for 0.2, off for 0.4
        { 1, 1 }     // FAST    on for 0.1, off for 0.1
    };

    //////////////////////////////////////////////////////////////////////

    uint32 constexpr GPIO_Pin_LED = GPIO_Pin_4;

    //////////////////////////////////////////////////////////////////////

    inline void led_on()
    {
        GPIO.out_w1tc |= GPIO_Pin_LED;
    }

    //////////////////////////////////////////////////////////////////////

    inline void led_off()
    {
        GPIO.out_w1ts |= GPIO_Pin_LED;
    }

    //////////////////////////////////////////////////////////////////////

    inline void led_set(bool on_or_off)
    {
        if(on_or_off) {
            led_on();
        } else {
            led_off();
        }
    }

    //////////////////////////////////////////////////////////////////////

    inline void led_toggle()
    {
        if((GPIO.out & GPIO_Pin_LED) != 0) {
            led_on();
        } else {
            led_off();
        }
    }

    //////////////////////////////////////////////////////////////////////

    void led_task_function(void *)
    {
        led_command_t command;

        command.timeout = 0;
        command.flash_mode = 0;

        int ticks = 0;
        int timeout = 0;

        while(true) {

            if(xQueueReceive(led_command_queue, &command, pdMS_TO_TICKS(100))) {

                timeout = 0;
                ticks = 0;
            }

            if(timeout > command.timeout) {

                led_off();
                continue;
            }

            flash_config_t const &flash_config = flash_configs[command.flash_mode];

            int total = flash_config.on_ticks + flash_config.off_ticks;

            while(ticks >= total) {
                ticks -= total;
            }

            led_set(ticks < flash_config.on_ticks);

            ticks += 1;
            timeout += 1;
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

void led_init()
{
    led_off();

    gpio_config_t p4_config;
    p4_config.pin_bit_mask = GPIO_Pin_LED;
    p4_config.mode = GPIO_MODE_OUTPUT;
    p4_config.pull_up_en = GPIO_PULLUP_DISABLE;
    p4_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    p4_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&p4_config);

    xTaskCreate(led_task_function, "led_task", 1000, null, 1, &led_task);

    led_command_queue = xQueueCreate(2, sizeof(led_command_t));
}

//////////////////////////////////////////////////////////////////////

void led_set_flash_mode(led_flash_mode_t mode, uint32 timeout)
{
    ASSERT(mode < num_flash_modes);

    led_command_t cmd;
    cmd.flash_mode = mode;
    cmd.timeout = timeout / 10;
    xQueueSend(led_command_queue, &cmd, portMAX_DELAY);
}
