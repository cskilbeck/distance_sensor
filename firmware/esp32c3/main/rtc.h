//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////
// this must match the register layout in the RTC

struct rtc_clock_data_t
{
    uint8_t control1;
    uint8_t control2;
    uint8_t offset;
    uint8_t ram;
    uint8_t seconds_bcd;
    uint8_t minutes_bcd;
    uint8_t hours_bcd;
    uint8_t date_bcd;
    uint8_t weekday;
    uint8_t month_bcd;
    uint8_t year_bcd;
    uint8_t seconds_alarm_bcd;
    uint8_t minutes_alarm_bcd;
    uint8_t hours_alarm_bcd;
    uint8_t date_alarm_bcd;
    uint8_t weekday_alarm_bcd;
    uint8_t timer_value;
    uint8_t timer_mode;
};

//////////////////////////////////////////////////////////////////////

#define RTC_CTL1_TEST (1 << 7)
#define RTC_CTL1_SW_RESET1 (1 << 6)
#define RTC_CTL1_STOP_CLOCK (1 << 5)
#define RTC_CTL1_SW_RESET2 (1 << 4)
#define RTC_CTL1_SW_RESET3 (1 << 3)
#define RTC_CTL1_COMP_IRQ_ENABLE (1 << 2)
#define RTC_CTL1_12_HOUR_MODE (1 << 1)
#define RTC_CTL1_ZERO (1 << 0)

//////////////////////////////////////////////////////////////////////

#define RTC_CTL2_ALARM_IRQ_ENABLE (1 << 7)
#define RTC_CTL2_ALARM_IRQ_DISABLE 0

#define RTC_CTL2_ALARM_IRQ_ACTIVE (1 << 6)
#define RTC_CTL2_ALARM_IRQ_INACTIVE 0

#define RTC_CTL2_MINUTE_IRQ_ENABLE (1 << 5)
#define RTC_CTL2_HALF_MINUTE_IRQ_ENABLE (1 << 4)
#define RTC_CTL2_TIMER_IRQ_ACTIVE (1 << 3)

#define RTC_CTL2_CLKOUT_32768 0x0
#define RTC_CTL2_CLKOUT_16384 0x1
#define RTC_CTL2_CLKOUT_8192 0x2
#define RTC_CTL2_CLKOUT_4906 0x3
#define RTC_CTL2_CLKOUT_2048 0x4
#define RTC_CTL2_CLKOUT_1024 0x5
#define RTC_CTL2_CLKOUT_1 0x6
#define RTC_CTL2_CLKOUT_OFF 0x7

//////////////////////////////////////////////////////////////////////

esp_err_t rtc_init();
esp_err_t rtc_cleanup();

esp_err_t rtc_get_clock(rtc_clock_data_t *data);
esp_err_t rtc_set_clock(rtc_clock_data_t const *data);

esp_err_t rtc_set_alarm(int32 seconds, rtc_clock_data_t &data);

int32 rtc_seconds_from_clock_data(rtc_clock_data_t const &data);
