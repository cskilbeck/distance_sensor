//////////////////////////////////////////////////////////////////////
//
// x    SPI Slave mode
// x    CH32 Sleep/standby for N minutes
// x    Measure power consumption
// !    Common message stuff between ESP12 and CH32
//      ESP12 Tasks (WiFi, SPI etc)
//      Server/Alexa
//
//
//
//////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include "debug.h"
#include "string.h"
#include "memory.h"
#include "user_gpio.h"
#include "user_pins.h"
#include "crc.h"

#define SYSTICK_CNTIF 0x1

//////////////////////////////////////////////////////////////////////

using byte = uint8_t;

#define SPI_DATA_SIZE 32

#define NUM_TIMER_READINGS 16

//////////////////////////////////////////////////////////////////////

enum status_t
{
    status_idle,
    status_in_progress,
    status_complete
};

//////////////////////////////////////////////////////////////////////
// so max data you can send in one message is 32 - 4 - 2 = 26 bytes

struct message_t
{
    u8 id;                                                              // type of message
    u8 length;                                                          // how many bytes in the payload
    u8 payload[SPI_DATA_SIZE - 4 - 2] __attribute__((aligned(1)));      // zero pad so crc makes sense
    u32 crc;
};

static_assert(sizeof(message_t)==SPI_DATA_SIZE);

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
    volatile u16 pressed :1;
    volatile u16 released :1;
    volatile u16 held :1;

    void update(int state);
};

//////////////////////////////////////////////////////////////////////

extern "C"
{
    void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
    void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
    void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
    void ADC1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
    void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
    void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
}

void spi_start();

//////////////////////////////////////////////////////////////////////

volatile status_t vbat_status = status_idle;
volatile status_t spi_status = status_idle;
volatile status_t distance_status = status_idle;

volatile u32 ticks;

u8 spi_tx_data[SPI_DATA_SIZE * 2] __attribute__ ((aligned(4)));
u8 spi_rx_data[SPI_DATA_SIZE * 2] __attribute__ ((aligned(4)));

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
// sleep for N minutes
//
// 61440 / 128000 = 0.48 secs/tick
// 63 ticks per loop = 30.24 seconds
// 30.24 * 2 = 60.48 seconds per minute, so.... near enough
// e.g. if you ask for an hour, you'll get approx 29 extra seconds
// but the 128kHz LSI clock is wobbly anyway so whevs

#define PFIC_SCTLR_SLEEP_DEEP 0x04
#define PFIC_SCTLR_WFITOWFE 0x08
#define PFIC_SCTLR_SETEVENT 0x20

#define PWR_AWUEN 0x02

void sleep_for_minutes_and_reset(u32 minutes)
{
    __disable_irq();

    // switch off the ADC
    ADC1->CTLR2 = 0;

    // switch off all clocks except what we need
    RCC->AHBPCENR = RCC_AHBPeriph_SRAM;
    RCC->APB1PCENR = RCC_APB1Periph_PWR;
    RCC->APB2PCENR = RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO;

    // disable all AFIO remap
    AFIO->PCFR1 = 0;

    // then set all GPIOs input pull down except LED (C0 = pull up)
    GPIOA->CFGLR = 0x88888888;
    GPIOC->CFGLR = 0x88888888;
    GPIOD->CFGLR = 0x88888888;
    GPIOA->OUTDR = 0;
    GPIOC->OUTDR = 0 | GPIO_MASK_LED;
    GPIOD->OUTDR = 0;

    // set clock to 8MHz HSI, switch off the PLL
    RCC->CFGR0 |= RCC_HPRE_DIV3;
    RCC->CTLR &= ~RCC_PLLON;

    // switch off SysTick
    SysTick->CTLR = 0;

    // configure EXTI_Line9 event falling trigger
    EXTI->INTENR = 0;
    EXTI->EVENR = EXTI_Line9;
    EXTI->RTENR = 0;
    EXTI->FTENR = EXTI_Line9;

    // switch on 128kHz LSI
    RCC->RSTSCKR |= RCC_LSION;

    // wait for it to stabilize
    while((RCC->RSTSCKR & RCC_LSIRDY) == 0) {
    }

    // setup auto wakeup every ~30seconds
    PWR->AWUPSC = PWR_AWU_Prescaler_61440;
    PWR->AWUCSR |= PWR_AWUEN;
    PWR->AWUWR = 4;

    // prepare standby mode
    PWR->CTLR = PWR_CTLR_PDDS;

    // go into standby N times for 30 * 2 seconds
    u32 t = NVIC->SCTLR | PFIC_SCTLR_SLEEP_DEEP | PFIC_SCTLR_WFITOWFE | PFIC_SCTLR_SETEVENT;

    for(u32 i = minutes * 2; i != 0; --i) {
        NVIC->SCTLR = t;
        asm volatile ("wfi");
        asm volatile ("wfi");
    }

    // and reboot
    NVIC_SystemReset();
}

//////////////////////////////////////////////////////////////////////

extern "C" void SysTick_Handler(void)
{
    SysTick->SR &= ~SYSTICK_CNTIF;
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
    if((TIM1->INTFR & TIM_IT_CC2) != 0) {
        distance_value = TIM1->CH2CVR;
        distance_status = status_complete;
    }
    TIM1->INTFR = ~(TIM_IT_CC1 | TIM_IT_CC2);
}

//////////////////////////////////////////////////////////////////////

extern "C" void USART1_IRQHandler(void)
{
    char_received = USART1->DATAR;
}

//////////////////////////////////////////////////////////////////////

void DMA1_Channel2_IRQHandler(void)
{
    DMA_ClearFlag(DMA1_FLAG_TC2);
}

//////////////////////////////////////////////////////////////////////

void DMA1_Channel3_IRQHandler(void)
{
    DMA_ClearFlag(DMA1_FLAG_TC3);
    spi_start();
    spi_status = status_complete;
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
    return static_cast<u32>(static_cast<s32>(ticks) - now);
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
    SysTick->CTLR = 0;
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
    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

    GPIO_Setup(GPIO_PORT_SPI_CLK, GPIO_PIN_SPI_CLK, GPIO_IN_FLOATING);
    GPIO_Setup(GPIO_PORT_SPI_MOSI, GPIO_PIN_SPI_MOSI, GPIO_IN_FLOATING);
    GPIO_Setup(GPIO_PORT_SPI_MISO, GPIO_PIN_SPI_MISO, GPIO_OUT_AF_PP_10MHZ);
    GPIO_Setup(GPIO_PORT_SPI_CS, GPIO_PIN_SPI_CS, GPIO_IN_FLOATING);

    // only init when SPI is idle
    while(GPIO_Get(GPIO_PORT_SPI_CS, GPIO_PIN_SPI_CS) == 0) {
    }

    memset(spi_tx_data, 0, sizeof(spi_tx_data));

    SPI_Cmd(SPI1, DISABLE);

    {
        SPI_InitTypeDef SPI_InitStructure;
        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
        SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_InitStructure.SPI_CRCPolynomial = 7;
        SPI_Init(SPI1, &SPI_InitStructure);
    }

    {
        DMA_InitTypeDef dma_rx;
        dma_rx.DMA_PeripheralBaseAddr = (u32)(&SPI1->DATAR);
        dma_rx.DMA_MemoryBaseAddr = (u32)spi_rx_data;
        dma_rx.DMA_DIR = DMA_DIR_PeripheralSRC;
        dma_rx.DMA_BufferSize = SPI_DATA_SIZE * 2;
        dma_rx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma_rx.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma_rx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma_rx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma_rx.DMA_Mode = DMA_Mode_Normal;
        dma_rx.DMA_Priority = DMA_Priority_Medium;
        dma_rx.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel2, &dma_rx);
    }

    {
        DMA_InitTypeDef dma_tx;
        dma_tx.DMA_PeripheralBaseAddr = (u32)(&SPI1->DATAR);
        dma_tx.DMA_MemoryBaseAddr = (u32)spi_tx_data;
        dma_tx.DMA_DIR = DMA_DIR_PeripheralDST;
        dma_tx.DMA_BufferSize = SPI_DATA_SIZE * 2;
        dma_tx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma_tx.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma_tx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma_tx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma_tx.DMA_Mode = DMA_Mode_Normal;
        dma_tx.DMA_Priority = DMA_Priority_High;
        dma_tx.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel3, &dma_tx);
    }

    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);

    DMA_Cmd(DMA1_Channel2, ENABLE);
    DMA_Cmd(DMA1_Channel3, ENABLE);

    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);
}

//////////////////////////////////////////////////////////////////////

void spi_start()
{
    SPI1->CTLR1 &= ~SPI_CTLR1_SPE;
    DMA1_Channel2->CFGR &= ~DMA_CFGR1_EN;
    DMA1_Channel3->CFGR &= ~DMA_CFGR1_EN;
    DMA1_Channel2->MADDR = (u32)spi_rx_data;
    DMA1_Channel3->MADDR = (u32)spi_tx_data;
    DMA1_Channel2->CNTR = SPI_DATA_SIZE * 2;
    DMA1_Channel3->CNTR = SPI_DATA_SIZE * 2;
    DMA1_Channel2->CFGR |= DMA_CFGR1_EN;
    DMA1_Channel3->CFGR |= DMA_CFGR1_EN;
    SPI1->CTLR1 |= SPI_CTLR1_SPE;
}

//////////////////////////////////////////////////////////////////////
// we're the slave so just setup the buffer which will get picked
// up next time the master (ESP12) does a transaction
//
// Only call this just after a transaction!

void spi_send(u8 id, u8 length, void const *data)
{
    message_t *m = reinterpret_cast<message_t *>(spi_tx_data + SPI_DATA_SIZE);    // 1st half of buffer is ignored by ESP12
    m->id = id;
    m->length = length;
    for(int x = 0; x < length; ++x) {
        m->payload[x] = ((u8*)data)[x];
    }
    for(int x = length; x < sizeof(m->payload); ++x) {
        m->payload[x] = 0;
    }
    m->crc = calc_crc32(spi_tx_data, sizeof(message_t) - sizeof(u32));
}

//////////////////////////////////////////////////////////////////////

template<typename T> void spi_sender(u8 id, T const &msg)
{
    static_assert(sizeof(T) < sizeof(message_t::payload));

    spi_send(id, sizeof(T), &msg);
}

//////////////////////////////////////////////////////////////////////

void init_led()
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;

    GPIO_Set(GPIO_PORT_LED, GPIO_MASK_LED);
    GPIO_Setup(GPIO_PORT_LED, GPIO_PIN_LED, GPIO_OUT_PP_2MHZ);
}

//////////////////////////////////////////////////////////////////////

void init_power()
{
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;

    GPIO_Setup(GPIO_PORT_EN_3V3, GPIO_PIN_EN_3V3, GPIO_OUT_PP_2MHZ);
    GPIO_Setup(GPIO_PORT_LVL_OE, GPIO_PIN_LVL_OE, GPIO_OUT_PP_2MHZ);
}

//////////////////////////////////////////////////////////////////////

extern "C" int user_main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    init_led();
    init_power();
    init_uart1();
    init_button();
    init_timer2();
    init_timer1();
    init_adc();
    init_spi();
    init_systick();

    printf("SysClk:%d\r\n", SystemCoreClock);

    GPIO_Set(GPIO_PORT_EN_3V3, GPIO_MASK_EN_3V3);
    GPIO_Set(GPIO_PORT_LVL_OE, GPIO_MASK_LVL_OE);

    vbat_timer.reset();
    spi_timer.reset();
    distance_timer.reset();

    struct bobbins
    {
        u8 foo[4];
    };

    bobbins bobby;

    bobby.foo[0] = 0xDE;
    bobby.foo[1] = 0xAD;
    bobby.foo[2] = 0xBE;
    bobby.foo[3] = 0xEF;

    spi_sender(1, bobby);

    spi_start();

    u32 x = 0;

    while(true) {

        if(spi_status == status_complete) {
            printf("GOT %5d,%08x\r\n", x++, ((u32 *)spi_rx_data)[0]);
            spi_status = status_idle;
        }

        if(distance_timer.elapsed() > 133 && distance_status == status_idle) {
            distance_timer.reset();
            TIM_Cmd(TIM2, ENABLE);
        }

        if(vbat_timer.elapsed() > 3000 && vbat_status == status_idle) {
            vbat_timer.reset();
            GPIO_Set(GPIO_PORT_VBAT_SNS_EN, GPIO_MASK_VBAT_SNS_EN);
            vbat_status = status_in_progress;
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        }

        if(button.pressed) {
            button.pressed = 0;
            printf("Sleep\r\n");
            u32 now = ticks;
            while(((s32)ticks - now) < 500) {
            }
            sleep_for_minutes_and_reset(3);
        }

        if(char_received != 0) {
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
    }
}
