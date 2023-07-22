//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

#define GPIO_IN_ANALOG       0b0000
#define GPIO_IN_FLOATING     0b0100
#define GPIO_IN_PULLUP_DOWN  0b1000
#define GPIO_IN_RESERVED     0b1100

#define GPIO_OUT_PP_10MHZ    0b0001
#define GPIO_OUT_OD_10MHZ    0b0101
#define GPIO_OUT_AF_PP_10MHZ 0b1001
#define GPIO_OUT_AF_OD_10MHZ 0b1101

#define GPIO_OUT_PP_2MHZ     0b0010
#define GPIO_OUT_OD_2MHZ     0b0110
#define GPIO_OUT_AF_PP_2MHZ  0b1010
#define GPIO_OUT_AF_OD_2MHZ  0b1110

#define GPIO_OUT_PP_50MHZ    0b0011
#define GPIO_OUT_OD_50MHZ    0b0111
#define GPIO_OUT_AF_PP_50MHZ 0b1011
#define GPIO_OUT_AF_OD_50MHZ 0b1111

//////////////////////////////////////////////////////////////////////

inline void GPIO_Setup(GPIO_TypeDef *gpio, int pin_number, u32 config)
{
    pin_number *= 4;
    gpio->CFGLR = (gpio->CFGLR & ~(0xf << pin_number)) | (config << pin_number);
}

//////////////////////////////////////////////////////////////////////

inline void GPIO_SetupPins(GPIO_TypeDef *GPIOx, u32 pin_mask, u32 config)
{
    u32 mask = 0;
    u32 cfg = 0;
    for (int i = 0x80; i != 0; i >>= 1) {
        mask <<= 4;
        cfg <<= 4;
        if((pin_mask & i) != 0) {
            mask |= 0xf;
            cfg |= config;
        }
    }
    GPIOx->CFGLR = (GPIOx->CFGLR & ~mask) | cfg;
}

//////////////////////////////////////////////////////////////////////

inline void GPIO_SetPullup(GPIO_TypeDef *GPIOx, u32 mask)
{
    GPIOx->BSHR = mask;
}

//////////////////////////////////////////////////////////////////////

inline void GPIO_SetPullDown(GPIO_TypeDef *GPIOx, u32 mask)
{
    GPIOx->BSHR = mask << 16;
}

//////////////////////////////////////////////////////////////////////

inline void GPIO_Set(GPIO_TypeDef *GPIOx, u32 mask)
{
    GPIOx->BSHR = mask;
}

//////////////////////////////////////////////////////////////////////

inline int GPIO_Get(GPIO_TypeDef *GPIOx, u32 mask)
{
    return (GPIOx->INDR & mask) != 0;
}

//////////////////////////////////////////////////////////////////////

inline void GPIO_Clear(GPIO_TypeDef *GPIOx, u32 mask)
{
    GPIOx->BCR = mask;
}

//////////////////////////////////////////////////////////////////////

inline void GPIO_SetAndClear(GPIO_TypeDef *GPIOx, u32 mask_on, u32 mask_off)
{
    GPIOx->BSHR = mask_on | (mask_off << 16);
}

//////////////////////////////////////////////////////////////////////

inline void GPIO_Toggle(GPIO_TypeDef *GPIOx, u32 mask)
{
    GPIOx->OUTDR ^= mask;
}

//////////////////////////////////////////////////////////////////////

inline void GPIO_SetTo(GPIO_TypeDef *GPIOx, u32 mask, int on_or_off)
{
    GPIOx->BSHR = mask << (on_or_off ? 0 : 16);
}

