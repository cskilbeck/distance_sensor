//////////////////////////////////////////////////////////////////////

#include <string.h>
#include "driver/i2c.h"

#include "esp_log.h"

#include "util.h"
#include "rtc.h"
#include "hardware.h"

LOG_CONTEXT("rtc");

//////////////////////////////////////////////////////////////////////

namespace
{
    constexpr uint32 RTC_I2C_FREQ_HZ = 400000;
    constexpr uint32 RTC_I2C_TIMEOUT_MS = 200;
    constexpr uint8 RTC_I2C_ADDRESS = 0xA2;

    constexpr TickType_t TIMEOUT = pdMS_TO_TICKS(RTC_I2C_TIMEOUT_MS);
    constexpr uint8_t I2C_ADDR = RTC_I2C_ADDRESS >> 1;
}

//////////////////////////////////////////////////////////////////////

esp_err_t rtc_init()
{
    i2c_config_t conf{ .mode = I2C_MODE_MASTER,
                       .sda_io_num = I2C_MASTER_SDA_IO,
                       .scl_io_num = I2C_MASTER_SCL_IO,
                       .sda_pullup_en = GPIO_PULLUP_DISABLE,
                       .scl_pullup_en = GPIO_PULLUP_DISABLE,
                       .master = { .clk_speed = RTC_I2C_FREQ_HZ },
                       .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL

    };
    ESP_RET(i2c_param_config(I2C_MASTER_NUM, &conf));

    ESP_RET(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

esp_err_t rtc_cleanup()
{
    i2c_driver_delete(I2C_MASTER_NUM);
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

int32 rtc_seconds_from_clock_data(rtc_clock_data_t const &data)
{
    int s = bcd_to_int(data.seconds_bcd & 0x7f);
    int m = bcd_to_int(data.minutes_bcd);
    int h = bcd_to_int(data.hours_bcd);
    return s + m * 60 + h * 3600;
}

//////////////////////////////////////////////////////////////////////
// get the whole RTC register set into a struct

esp_err_t rtc_get_clock(rtc_clock_data_t *data)
{
    byte reg_address = 0;

    ESP_RET(rtc_init());
    DEFER(rtc_cleanup());

    ESP_RET(i2c_master_write_read_device(I2C_MASTER_NUM, I2C_ADDR, &reg_address, 1, reinterpret_cast<uint8_t *>(data), sizeof(rtc_clock_data_t), TIMEOUT));

    // LOG_BUFFER(ESP_LOG_INFO, data, sizeof(*data));

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////
// overwrite the whole RTC register set

esp_err_t rtc_set_clock(rtc_clock_data_t const *data)
{
    byte reg_address = 0;

    ESP_RET(rtc_init());
    DEFER(rtc_cleanup());

    ESP_RET(i2c_master_write_write_device(I2C_MASTER_NUM, I2C_ADDR, &reg_address, 1, reinterpret_cast<uint8_t const *>(data), sizeof(*data), TIMEOUT));

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////
// set the alarm to go off at some # of seconds in the day

esp_err_t rtc_set_alarm(int32 seconds, rtc_clock_data_t &data)
{
    data.control2 = RTC_CTL2_ALARM_IRQ_ENABLE;

    // setup HH:MM.SS, wraps around if necessary

    data.seconds_alarm_bcd = int_to_bcd(seconds % 60);
    data.minutes_alarm_bcd = int_to_bcd((seconds / 60) % 60);
    data.hours_alarm_bcd = int_to_bcd((seconds / 3600) % 24);

    // top bit disables date and weekly alarms

    data.date_alarm_bcd = 0x80;
    data.weekday_alarm_bcd = 0x80;

    ESP_RET(rtc_init());
    DEFER(rtc_cleanup());

    byte const rtc_control2_register = offsetof(rtc_clock_data_t, control2);
    byte const rtc_alarm_register = offsetof(rtc_clock_data_t, seconds_alarm_bcd);

    size_t const rtc_alarm_num_registers = offsetof(rtc_clock_data_t, weekday_alarm_bcd) - rtc_alarm_register + 1;

    // set control2 (alarm irq setting)
    ESP_RET(i2c_master_write_write_device(I2C_MASTER_NUM, I2C_ADDR, &rtc_control2_register, 1, &data.control2, 1, TIMEOUT));

    // set alarm registers
    ESP_RET(i2c_master_write_write_device(I2C_MASTER_NUM, I2C_ADDR, &rtc_alarm_register, 1, &data.seconds_alarm_bcd, rtc_alarm_num_registers, TIMEOUT));

    LOG_INFO("Alarm set for %02x:%02x.%02x", data.hours_alarm_bcd, data.minutes_alarm_bcd, data.seconds_alarm_bcd);

    return ESP_OK;
}
