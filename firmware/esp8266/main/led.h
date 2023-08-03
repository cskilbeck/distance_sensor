//////////////////////////////////////////////////////////////////////

#pragma once

#include "types.h"

//////////////////////////////////////////////////////////////////////

uint32 constexpr GPIO_Pin_LED = GPIO_Pin_4;

void init_led();

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