//////////////////////////////////////////////////////////////////////

#include "debug.h"
#include "string.h"
#include "memory.h"
#include "user_gpio.h"
#include "user_pins.h"
#include "crc.h"

//////////////////////////////////////////////////////////////////////

using byte = uint8_t;

#define SPI_DATA_SIZE 14

#define NUM_TIMER_READINGS 16

//////////////////////////////////////////////////////////////////////

enum status_t
{
    status_idle,
    status_in_progress,
    status_complete
};

//////////////////////////////////////////////////////////////////////

struct stopwatch_t
{
    u32 now;

    u32 elapsed() const;
    void reset();
};

//////////////////////////////////////////////////////////////////////

struct button_t
{
    u16 history;
    u16 pressed :1;
    u16 released :1;
    u16 held :1;

    void update(int state);
};

//////////////////////////////////////////////////////////////////////

struct message_t
{
    u32 crc;
    u16 data[SPI_DATA_SIZE];
};

//////////////////////////////////////////////////////////////////////

extern "C"
{
    void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
    void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
    void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
    void ADC1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
    void SPI1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
}

//////////////////////////////////////////////////////////////////////

volatile status_t vbat_status = status_idle;
volatile status_t spi_status = status_idle;
volatile status_t distance_status = status_idle;

volatile u32 ticks;

u16 spi_tx_data[SPI_DATA_SIZE] = {
0x0101, 0x0202, 0x0303, 0x0404, 0x0505, 0x0606, 0x1111, 0x1212, 0x1313, 0x1414, 0x1515, 0x1616, 0x1717, 0x1818
};

u16 spi_rx_data[SPI_DATA_SIZE];

message_t spi_message;

u16 *spi_next_word;
u16 *spi_end_addr;

u32 distance_readings[NUM_TIMER_READINGS];
int num_distance_readings;
int cur_distance_reading;
u32 cur_distance_total;
volatile u32 distance_value;

button_t button;

u32 vbat_reading;

volatile u8 char_received;

stopwatch_t spi_timer;
stopwatch_t distance_timer;
stopwatch_t vbat_timer;

//////////////////////////////////////////////////////////////////////

extern "C" void SysTick_Handler(void)
{
    SysTick->SR &= ~(1 << 0);
    button.update(GPIO_Get(GPIO_PORT_BUTTON, GPIO_MASK_BUTTON));
    ticks += 1;
}

//////////////////////////////////////////////////////////////////////

extern "C" void ADC1_IRQHandler(void)
{
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    vbat_reading = ADC_GetConversionValue(ADC1);
    GPIO_Clear(GPIO_PORT_VBAT_SNS_EN, GPIO_MASK_VBAT_SNS_EN);
    vbat_status = status_complete;
}

//////////////////////////////////////////////////////////////////////

extern "C" void TIM1_CC_IRQHandler(void)
{
    if((TIM1->INTFR & TIM_IT_CC2) != RESET) {
        distance_value = TIM1->CH2CVR;
        distance_status = status_complete;
    }
    TIM1->INTFR = (uint16_t)~(TIM_IT_CC1 | TIM_IT_CC2);
}

//////////////////////////////////////////////////////////////////////

extern "C" void USART1_IRQHandler(void)
{
    char_received = USART1->DATAR;
}

//////////////////////////////////////////////////////////////////////

extern "C" void SPI1_IRQHandler(void)
{
    SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_TXE);
    if(spi_next_word < spi_end_addr) {
        SPI_I2S_SendData(SPI1, *spi_next_word++);
    } else {
        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
        spi_status = status_complete;
        GPIO_Set(GPIO_PORT_SPI_CS, GPIO_MASK_SPI_CS);
    }
}

//////////////////////////////////////////////////////////////////////
// UART1 for printf/getchar

void init_uart1(void)
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO;

    GPIO_PinRemapConfig(GPIO_PartialRemap2_USART1, ENABLE);

    GPIO_Setup(GPIO_PORT_TX, GPIO_PIN_TX, GPIO_OUT_AF_PP_50MHZ);
    GPIO_Setup(GPIO_PORT_RX, GPIO_PIN_RX, GPIO_IN_FLOATING);

    {
        USART_InitTypeDef USART_InitStructure;
        USART_InitStructure.USART_BaudRate = 115200;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART_Init(USART1, &USART_InitStructure);
    }

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);

    USART1->CTLR1 |= USART_CTLR1_UE;
}

//////////////////////////////////////////////////////////////////////

u32 stopwatch_t::elapsed() const
{
    return static_cast<s32>(ticks) - now;
}

//////////////////////////////////////////////////////////////////////

void stopwatch_t::reset()
{
    now = ticks;
}

//////////////////////////////////////////////////////////////////////

void init_button()
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA;

    GPIO_Setup(GPIO_PORT_BUTTON, GPIO_PIN_BUTTON, GPIO_IN_PULLUP_DOWN);
    GPIO_SetPullup(GPIO_PORT_BUTTON, GPIO_MASK_BUTTON);

    button.history = 0xffff;
    button.held = 0;
    button.released = 0;
    button.pressed = 0;
}

//////////////////////////////////////////////////////////////////////

void button_t::update(int state)
{
    u16 h = history;
    h = (h << 1) | state;
    history = h;
    held = !state;
    if(h == 0xfffe) {
        pressed = 1;
    } else if(h == 0x0001) {
        released = 1;
    }
}

//////////////////////////////////////////////////////////////////////
// Timer 2 channel 2 (GPIO D3) is SEND pulse

void init_timer2()
{
    RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
    RCC->APB2PCENR |= RCC_APB2Periph_AFIO;

    GPIO_Setup(GPIO_PORT_SNS_TX, GPIO_PIN_SNS_TX, GPIO_OUT_AF_PP_10MHZ);

    {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
        TIM_TimeBaseInitStructure.TIM_Period = 10;
        TIM_TimeBaseInitStructure.TIM_Prescaler = 47;
        TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    }

    {
        TIM_OCInitTypeDef TIM_OCInitStructure;
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;
        TIM_OCInitStructure.TIM_Pulse = 1;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
        TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
        TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    }

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Single);
}

//////////////////////////////////////////////////////////////////////
// Timer 1 channel 1 (GPIO D2) is RECEIVE (measure pulse length)

void init_timer1()
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1;

    GPIO_Setup(GPIO_PORT_SNS_RX, GPIO_PIN_SNS_RX, GPIO_IN_FLOATING);

    {
        TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
        TIM_TimeBaseInitStructure.TIM_Period = 0xffff;
        TIM_TimeBaseInitStructure.TIM_Prescaler = 289;
        TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
        TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    }

    {
        TIM_ICInitTypeDef TIM_ICInitStructure;
        TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
        TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
        TIM_ICInitStructure.TIM_ICFilter = 0x00;
        TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
        TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
        TIM_PWMIConfig(TIM1, &TIM_ICInitStructure);
    }

    {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

    TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
    TIM_Cmd(TIM1, ENABLE);
}

//////////////////////////////////////////////////////////////////////
// ADC Channel 1 (GPIO A1) reads VBAT

void init_adc(void)
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD
                    | RCC_APB2Periph_GPIOA
                    | RCC_APB2Periph_ADC1;

    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    GPIO_Setup(GPIO_PORT_VBAT_SNS, GPIO_PIN_VBAT_SNS, GPIO_IN_AIN);
    GPIO_Setup(GPIO_PORT_VBAT_SNS_EN, GPIO_PIN_VBAT_SNS_EN, GPIO_OUT_PP_2MHZ);

    {
        ADC_InitTypeDef ADC_InitStructure;
        ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_InitStructure.ADC_ScanConvMode = DISABLE;
        ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
        ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_NbrOfChannel = 1;
        ADC_Init(ADC1, &ADC_InitStructure);
    }

    ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1)) {
        __NOP();
    }

    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1)) {
        __NOP();
    }

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_241Cycles);

    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    NVIC_EnableIRQ(ADC_IRQn);
}

//////////////////////////////////////////////////////////////////////

void init_systick()
{
    SysTick->CTLR = 0xF;
    SysTick->SR &= ~(1 << 0);
    SysTick->CMP = (SystemCoreClock / 1000) - 1;
    SysTick->CNT = 0;
    SysTick->CTLR = 0xF;
    NVIC_EnableIRQ(SysTicK_IRQn);
}

//////////////////////////////////////////////////////////////////////

void init_spi(void)
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO;

    GPIO_Setup(GPIO_PORT_SPI_CLK, GPIO_PIN_SPI_CLK, GPIO_OUT_AF_PP_10MHZ);
    GPIO_Setup(GPIO_PORT_SPI_MOSI, GPIO_PIN_SPI_MOSI, GPIO_OUT_AF_PP_10MHZ);
    GPIO_Setup(GPIO_PORT_SPI_MISO, GPIO_PIN_SPI_MISO, GPIO_IN_FLOATING);
    GPIO_Setup(GPIO_PORT_SPI_CS, GPIO_PIN_SPI_CS, GPIO_OUT_PP_10MHZ);
    GPIO_Set(GPIO_PORT_SPI_CS, GPIO_MASK_SPI_CS);

    {
        SPI_InitTypeDef SPI_InitStructure;
        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
        SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_InitStructure.SPI_CRCPolynomial = 7;
        SPI_Init(SPI1, &SPI_InitStructure);
    }

    SPI_Cmd(SPI1, ENABLE);
    NVIC_EnableIRQ(SPI1_IRQn);
}

//////////////////////////////////////////////////////////////////////

extern "C" int user_main(void)
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;

    GPIO_Set(GPIO_PORT_LED, GPIO_MASK_LED);
    GPIO_Setup(GPIO_PORT_LED, GPIO_PIN_LED, GPIO_OUT_PP_2MHZ);

    GPIO_Setup(GPIO_PORT_EN_3V3, GPIO_PIN_EN_3V3, GPIO_OUT_PP_2MHZ);

    GPIO_Setup(GPIO_PORT_LVL_OE, GPIO_PIN_LVL_OE, GPIO_OUT_PP_2MHZ);

    init_uart1();
    init_button();
    init_timer2();
    init_timer1();
    init_adc();
    init_spi();
    init_systick();

    printf("SystemClk:%d\r\n", SystemCoreClock);

    memcpy(spi_message.data, spi_tx_data, sizeof(spi_message.data));

    spi_message.crc = crc32(reinterpret_cast<u8 const *>(spi_message.data), sizeof(spi_message.data));

    printf("CRC:%08x\r\n", spi_message.crc);

    //GPIO_Set(GPIO_PORT_EN_3V3, GPIO_MASK_EN_3V3);
    //GPIO_Set(GPIO_PORT_LVL_OE, GPIO_MASK_LVL_OE);

    vbat_timer.reset();
    spi_timer.reset();
    distance_timer.reset();

    while(true) {

        if(spi_timer.elapsed() >= 1000 && spi_status == status_idle) {
            spi_timer.reset();
            spi_status = status_in_progress;
            spi_next_word = spi_tx_data;
            spi_end_addr = spi_tx_data + SPI_DATA_SIZE;
            GPIO_Clear(GPIO_PORT_SPI_CS, GPIO_MASK_SPI_CS);
            SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
        }

        if(distance_timer.elapsed() > 133 && distance_status == status_idle) {
            distance_timer.reset();
            if(distance_status == status_idle) {
                TIM_Cmd(TIM2, ENABLE);
            }
        }

        if(vbat_timer.elapsed() > 300 && vbat_status == status_idle) {
            vbat_timer.reset();
            GPIO_Set(GPIO_PORT_VBAT_SNS_EN, GPIO_MASK_VBAT_SNS_EN);
            vbat_status = status_in_progress;
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        }

        if(button.pressed) {
            GPIO_Toggle(GPIO_PORT_LED, GPIO_MASK_LED);
            button.pressed = 0;
        }

        if(char_received != 0) {
            GPIO_Toggle(GPIO_PORT_LED, GPIO_MASK_LED);
            char_received = 0;
        }

        if(vbat_status == status_complete) {
            vbat_status = status_idle;
            printf("%d\n", vbat_reading);
        }

        if(distance_status == status_complete) {
            distance_status = status_idle;
            if(distance_value < 500) {
                u32 cur_average = 0;
                cur_distance_total -= distance_readings[cur_distance_reading];
                cur_distance_total += distance_value;
                distance_readings[cur_distance_reading] = distance_value;
                cur_distance_reading = ++cur_distance_reading % NUM_TIMER_READINGS;
                num_distance_readings += 1;
                if(num_distance_readings >= NUM_TIMER_READINGS) {
                    cur_average = cur_distance_total / NUM_TIMER_READINGS;
                }
                printf("%d (%d)\r\n", distance_value, cur_average);
            }
        }

        //PFIC->SCTLR &= ~((1<<3) | (1 << 1));    // WFI mode (not WFE), clear SLEEPONEXIT
        //asm volatile ("wfi");
    }
}
