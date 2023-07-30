/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v00x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include <ch32v00x_it.h>
#include <ch32v00x_gpio.h>
#include "user_gpio.h"
#include "user_pins.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
    GPIO_Setup(GPIO_PORT_LED, GPIO_PIN_LED, GPIO_OUT_PP_2MHZ);
    GPIO_Set(GPIO_PORT_LED, GPIO_MASK_LED);
    int x = 0;

    while (1) {
        x = (x + 12) & 4095;
        GPIO_Clear(GPIO_PORT_LED, GPIO_MASK_LED);
        int delay1 = x;
        for(volatile int i=0; i<delay1; ++i) {
        }
        GPIO_Set(GPIO_PORT_LED, GPIO_MASK_LED);
        int delay2 = (4095 - x) << 2;
        for(volatile int i=0; i<delay2; ++i) {
        }
    }
}

