//////////////////////////////////////////////////////////////////////

#include <string.h>
#include "driver/i2c.h"

#include "esp_log.h"

#include "util.h"
#include "rtc.h"
#include "hardware.h"

LOG_TAG("rtc");

//////////////////////////////////////////////////////////////////////

#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 200

#define RTC_DEFAULT_I2C_ADDRESS (static_cast<uint16_t>(0xA2))

#define DEVICE I2C_MASTER_NUM
#define TIMEOUT (pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS))
#define I2C_ADDR (RTC_DEFAULT_I2C_ADDRESS >> 1)

//////////////////////////////////////////////////////////////////////

esp_err_t rtc_init()
{
    i2c_config_t conf{ .mode = I2C_MODE_MASTER,
                       .sda_io_num = I2C_MASTER_SDA_IO,
                       .scl_io_num = I2C_MASTER_SCL_IO,
                       .sda_pullup_en = GPIO_PULLUP_DISABLE,
                       .scl_pullup_en = GPIO_PULLUP_DISABLE,
                       .master = { .clk_speed = I2C_MASTER_FREQ_HZ },
                       .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL

    };
    ESP_RET(i2c_param_config(I2C_MASTER_NUM, &conf));

    ESP_RET(i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

esp_err_t rtc_cleanup()
{
    i2c_driver_delete(I2C_MASTER_NUM);
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

int rtc_get_clock_seconds(rtc_clock_data_t const &data)
{
    int s = bcd_to_int(data.seconds_bcd);
    int m = bcd_to_int(data.minutes_bcd);
    int h = bcd_to_int(data.hours_bcd);
    return s + m * 60 + h * 3600;
}

//////////////////////////////////////////////////////////////////////

void rtc_set_alarm_seconds(int32 seconds, rtc_clock_data_t &data)
{
    data.seconds_alarm_bcd = int_to_bcd(seconds % 60);
    data.minutes_alarm_bcd = int_to_bcd((seconds / 60) % 60);
    data.hours_alarm_bcd = int_to_bcd((seconds / 3600) % 24);
}

//////////////////////////////////////////////////////////////////////

esp_err_t rtc_get_clock(rtc_clock_data_t *data)
{
    byte reg_address = 0;

    ESP_RET(rtc_init());
    DEFER(rtc_cleanup());

    ESP_RET(i2c_master_write_read_device(DEVICE, I2C_ADDR, &reg_address, 1, reinterpret_cast<uint8_t *>(data), sizeof(*data), TIMEOUT));

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

esp_err_t rtc_set_clock(rtc_clock_data_t const *data)
{
    byte reg_address = 0;

    ESP_RET(rtc_init());
    DEFER(rtc_cleanup());

    ESP_RET(i2c_master_write_write_device(DEVICE, I2C_ADDR, &reg_address, 1, reinterpret_cast<uint8_t const *>(data), sizeof(*data), TIMEOUT));

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

esp_err_t rtc_set_alarm(rtc_clock_data_t const *data)
{
    ESP_RET(rtc_init());
    DEFER(rtc_cleanup());

    // set ctl2 (to switch off the alarm irq)
    byte ctl2_reg_address = offsetof(rtc_clock_data_t, control2);
    ESP_RET(i2c_master_write_write_device(DEVICE, I2C_ADDR, &ctl2_reg_address, 1, &data->control2, 1, TIMEOUT));

    // set alarm
    byte alarm_reg_address = offsetof(rtc_clock_data_t, seconds_alarm_bcd);
    ESP_RET(i2c_master_write_write_device(DEVICE, I2C_ADDR, &alarm_reg_address, 1, &data->seconds_alarm_bcd, 5, TIMEOUT));

    return ESP_OK;
}
