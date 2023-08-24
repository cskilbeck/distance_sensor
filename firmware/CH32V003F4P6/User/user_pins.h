//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////
//
// Pin          Fn          Name
//
// 1    D4   OUT_PP      VBAT_SNS_EN
// 2    D5   OUT_AF_PP   TX
// 3    D6   IN_FLOAT    RX
// 4                     RST
// 5    A1   IN_FLOAT    VBAT_SNS
// 6    A2   IN_PU       BUTTON
// 7                     GND
// 8                     -
// 9                     VCC
// 10   C0   OUT_PP      LED
// 11   C1   OUT_PP      SPI_CS
// 12   C2   OUT_PP      5VSW
// 13   C3               -
// 14   C4               -
// 15   C5   OUT_AF_PP   SPI_CLK
// 16   C6   OUT_AF_PP   SPI_MOSI
// 17   C7   IN_FLOAT    SPI_MISO
// 18                    SWIO
// 19   D2   OUT_PP      SNS_TX
// 20   D3   IN_FLOAT    SNS_RX
//
//////////////////////////////////////////////////////////////////////

#define GPIO_MASK_ALL 0xffff

// GPIOA
#define GPIO_PIN_VBAT_SNS 1
#define GPIO_PIN_BUTTON 2

#define GPIO_MASK_VBAT_SNS GPIO_Pin_1
#define GPIO_MASK_BUTTON GPIO_Pin_2

#define GPIO_PORT_VBAT_SNS GPIOA
#define GPIO_PORT_BUTTON GPIOA

// GPIOC
#define GPIO_PIN_LED 0
#define GPIO_PIN_SPI_CS 1
#define GPIO_PIN_5VSW 2
#define GPIO_PIN_SPI_CLK 5
#define GPIO_PIN_SPI_MOSI 6
#define GPIO_PIN_SPI_MISO 7

#define GPIO_MASK_LED GPIO_Pin_0
#define GPIO_MASK_SPI_CS GPIO_Pin_1
#define GPIO_MASK_5VSW GPIO_Pin_2
#define GPIO_MASK_SPI_CLK GPIO_Pin_5
#define GPIO_MASK_SPI_MOSI GPIO_Pin_6
#define GPIO_MASK_SPI_MISO GPIO_Pin_7

#define GPIO_PORT_LED GPIOC
#define GPIO_PORT_SPI_CS GPIOC
#define GPIO_PORT_5VSW GPIOC
#define GPIO_PORT_SPI_CLK GPIOC
#define GPIO_PORT_SPI_MOSI GPIOC
#define GPIO_PORT_SPI_MISO GPIOC

// GPIOD
#define GPIO_PIN_SNS_RX 2
#define GPIO_PIN_SNS_TX 3
#define GPIO_PIN_VBAT_SNS_EN 4
#define GPIO_PIN_TX 5
#define GPIO_PIN_RX 6

#define GPIO_MASK_SNS_RX GPIO_Pin_2
#define GPIO_MASK_SNS_TX GPIO_Pin_3
#define GPIO_MASK_VBAT_SNS_EN GPIO_Pin_4
#define GPIO_MASK_TX GPIO_Pin_5
#define GPIO_MASK_RX GPIO_Pin_6

#define GPIO_PORT_SNS_TX GPIOD
#define GPIO_PORT_SNS_RX GPIOD
#define GPIO_PORT_VBAT_SNS_EN GPIOD
#define GPIO_PORT_RX GPIOD
#define GPIO_PORT_TX GPIOD

