//////////////////////////////////////////////////////////////////////
//
// x    Common message stuff between ESP12 and CH32
// x    ESP12 Tasks (x WiFi, SPI etc)
// x    ESP http send
// x    Server/Database
// x    SPI Slave mode
// x    CH32 Sleep/standby for N minutes
// x    Button wakeup
// x    Handshake ESP/CH32 boot via MOSI? But.. janky current thing is working...
// x    Make server thing a service
// x    Measure power consumption
// !    Factory reset (long button press)
//      Status/history web page
//      Alexa notification
//      Email notification
//      OTA ESP firmware update? Why tho? Can't update CH32...
//
//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <memory.h>

#include <ch32v00x.h>
#include <ch32v00x_conf.h>
#include <ch32v00x_gpio.h>

#include "int_types.h"
#include "user_gpio.h"
#include "user_pins.h"
#include "crc.h"

#define SYSTICK_SR_CNTIF 0x01

#define SYSTICK_CTLR_ENABLE 0x01
#define SYSTICK_CTLR_INTEN 0x02
#define SYSTICK_CTLR_HCLK 0x04
#define SYSTICK_CTLR_AUTORE 0x08

#define PFIC_SCTLR_SLEEPDEEP 0x04
#define PFIC_SCTLR_WFITOWFE 0x08
#define PFIC_SCTLR_SEVONPEND 0x10
#define PFIC_SCTLR_SETEVENT 0x20

#define PWR_AWUEN 0x02

//////////////////////////////////////////////////////////////////////

namespace
{
    static constexpr int MIN_VALID_DISTANCE = 50;
    static constexpr int MAX_VALID_DISTANCE = 700;
    static constexpr int MAX_DISTANCE_TRIES = 5;

    enum state_t
    {
        state_boot = 0,
        state_reading,
        state_wait_for_esp,
        state_esp,
        state_num
    };

    char const *state_name[state_num] = {
    "boot", "reading", "wait_for_esp", "esp"
    };

    //////////////////////////////////////////////////////////////////////
    // status of a system (spi, vbat adc etc)

    enum status_t
    {
        status_idle,
        status_in_progress,
        status_complete
    };

    //////////////////////////////////////////////////////////////////////
    // what readings have been got so far

    enum readings_t
    {
        reading_vbat = 1,
        reading_distance = 2,
        reading_all = 3
    };

    //////////////////////////////////////////////////////////////////////

    struct stopwatch_t
    {
        uint32 now;

        uint32 elapsed() const;
        void reset();
    };

    //////////////////////////////////////////////////////////////////////

    struct button_t
    {
        uint16 history;
        union
        {
            struct
            {
                volatile uint16 pressed :1;
                volatile uint16 released :1;
                volatile uint16 held :1;
            };
            uint16 flags;
        };

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
    void init_peripherals();
    void set_state(state_t new_state);

    //////////////////////////////////////////////////////////////////////

    state_t state = state_boot;
    uint32 state_start_ticks;

    volatile status_t vbat_status = status_idle;
    volatile status_t spi_status = status_idle;
    volatile status_t distance_status = status_idle;

    volatile uint32 ticks;

    uint8 spi_tx_data[SPI_DATA_SIZE * 2] __attribute__ ((aligned(4)));
    uint8 spi_rx_data[SPI_DATA_SIZE * 2] __attribute__ ((aligned(4)));

    int num_distance_readings;
    volatile uint32 distance_value;

    button_t button;

    volatile uint16 vbat_reading;

    volatile uint8 char_received;

    uint32 got_readings = 0;

    stopwatch_t distance_timer;

    bool factory_reset = false;

    int standby_boot = 0;

    //////////////////////////////////////////////////////////////////////

    uint32 state_elapsed_ticks()
    {
        return static_cast<uint32>(static_cast<int32>(ticks - state_start_ticks));
    }

    //////////////////////////////////////////////////////////////////////

    void set_state(state_t new_state)
    {
        printf("set_state [%s] (%d) after %d ticks\n", state_name[new_state], new_state, state_elapsed_ticks());
        state_start_ticks = ticks;
        state = new_state;
    }

    //////////////////////////////////////////////////////////////////////
    // sleep for some amount of time
    //
    // 61440 / 128000 = 0.48 seconds per tick
    // 63 ticks per loop = 30.24 seconds and then with the loop overhead measured at ~30.6 seconds
    // so to wait for 4 hours, loop count of 470 is about right
    // the 128kHz LSI clock is probably not that accurate anyway...
    //
    // button wakes it up, hold for > N seconds to set factory_reset flag
    //
    // do not call this from an interrupt handler

    void sleep(uint32 count)
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

        // then set all GPIOs input pull down except BUTTON (A2 = pullup), LED (C0 = pull up) and 5VSW (C2 = pullup)
        GPIOA->CFGLR = 0x88888888;
        GPIOC->CFGLR = 0x88888888;
        GPIOD->CFGLR = 0x88888888;
        GPIOA->OUTDR = GPIO_MASK_BUTTON;
        GPIOC->OUTDR = GPIO_MASK_LED | GPIO_MASK_5VSW;
        GPIOD->OUTDR = 0;

        // set clock to 8MHz HSI, switch off the PLL
        RCC->CFGR0 |= RCC_HPRE_DIV3;
        RCC->CTLR &= ~RCC_PLLON;

        // switch off SysTick
        SysTick->CTLR = 0;

        // EXTI line 2 on port A (A2... right?)
        AFIO->EXTICR = GPIO_PortSourceGPIOA << GPIO_PIN_BUTTON;

        // configure EXTI_Line9 (auto wakeup) and EXTI_Line1 (button) event falling triggers
        EXTI->INTENR = 0;
        EXTI->EVENR = EXTI_Line9 | EXTI_Line2;
        EXTI->RTENR = 0;
        EXTI->FTENR = EXTI_Line9 | EXTI_Line2;

        // switch on 128kHz LSI
        RCC->RSTSCKR |= RCC_LSION;

        // wait for it to stabilize
        while((RCC->RSTSCKR & RCC_LSIRDY) == 0) {
        }

        // setup auto wakeup every ~30seconds
        PWR->AWUCSR &= ~PWR_AWUEN;
        PWR->AWUPSC = PWR_AWU_Prescaler_61440;
        PWR->AWUWR = 63;
        PWR->AWUCSR |= PWR_AWUEN;

        // prepare standby mode
        PWR->CTLR = PWR_CTLR_PDDS;

        // go into standby N times for some amount of time, button wakes it up
        uint32 t = NVIC->SCTLR | PFIC_SCTLR_SLEEPDEEP | PFIC_SCTLR_WFITOWFE;

        // 1st WFI seems to be ignored for some reason...? am I not clearing something? irq pending?
        NVIC->SCTLR = t;
        asm volatile ("wfi");

#pragma GCC unroll 0
        for(uint32 i = count; i != 0; --i) {
            asm volatile ("wfi");
            if(GPIO_Get(GPIO_PORT_BUTTON, GPIO_MASK_BUTTON) == 0) {
                break;
            }
        }

        // disable sleep/standby stuff
        EXTI->INTENR = 0;
        EXTI->EVENR = 0;
        EXTI->RTENR = 0;
        EXTI->FTENR = 0;
        RCC->RSTSCKR &= ~RCC_LSION;
        PWR->AWUCSR &= ~PWR_AWUEN;
        PWR->CTLR &= ~PWR_CTLR_PDDS;

        // set systick to 8MHz/8000 = 1ms
        SysTick->CTLR = 0;
        SysTick->SR = 0;
        SysTick->CMP = 8000 - 1;
        SysTick->CNT = 0;
        SysTick->CTLR = SYSTICK_CTLR_ENABLE | SYSTICK_CTLR_HCLK | SYSTICK_CTLR_AUTORE;

        // debounce button and if button held for >5 seconds, set factory_reset flag
        factory_reset = false;
        uint32 button_held_ticks = 0;
        uint32 tick = 0;
        while(tick < 10) {
            if((SysTick->SR & SYSTICK_SR_CNTIF) != 0) {
                SysTick->SR = 0;
                tick += 1;
                if(GPIO_Get(GPIO_PORT_BUTTON, GPIO_MASK_BUTTON) == 0) {
                    tick = 0;
                    ++button_held_ticks;
                    if(button_held_ticks == 5000) {
                        factory_reset = true;
                        break;
                    }
                }
            }
        }

        // reinit System Clock to 48MHz HSI
        SystemInit();

        __enable_irq();

        standby_boot += 1;

        init_peripherals();
        state_start_ticks = ticks;
        set_state(state_boot);
    }

    //////////////////////////////////////////////////////////////////////

    constexpr message_t *outbound_message()
    {
        return reinterpret_cast<message_t *>(spi_tx_data + SPI_DATA_SIZE);
    }

    //////////////////////////////////////////////////////////////////////

    template<typename T> constexpr T *outbound_payload()
    {
        return reinterpret_cast<T *>(&outbound_message()->body.payload);
    }

    //////////////////////////////////////////////////////////////////////

    extern "C" void SysTick_Handler(void)
    {
        SysTick->SR &= ~SYSTICK_SR_CNTIF;
        button.update(!GPIO_Get(GPIO_PORT_BUTTON, GPIO_MASK_BUTTON));
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

    void flush_printf()
    {
        // wait for transfer empty and transfer complete
        uint32 constexpr flags = (USART_STATR_TC | USART_STATR_TXE);
        while((USART1->STATR & flags) != flags) {
        }
    }

    //////////////////////////////////////////////////////////////////////

    uint32 stopwatch_t::elapsed() const
    {
        return static_cast<uint32>(static_cast<int32>(ticks) - now);
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

        button.history = 0;
        button.flags = 0;
    }

    //////////////////////////////////////////////////////////////////////

    void button_t::update(int state)
    {
        uint16 h = history;
        h = (h << 1) | state;
        history = h;
        held = state;
        if(h == 0x0001) {
            pressed = 1;
        } else if(h == 0xfffe) {
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

        GPIO_Setup(GPIO_PORT_VBAT_SNS, GPIO_PIN_VBAT_SNS, GPIO_IN_ANALOG);
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
        SysTick->SR &= ~SYSTICK_CTLR_ENABLE;
        SysTick->CMP = (SystemCoreClock / 1000) - 1;
        SysTick->CNT = 0;
        SysTick->CTLR = SYSTICK_CTLR_ENABLE | SYSTICK_CTLR_INTEN | SYSTICK_CTLR_HCLK | SYSTICK_CTLR_AUTORE;
        NVIC_EnableIRQ(SysTicK_IRQn);
        ticks = 0;
    }

    //////////////////////////////////////////////////////////////////////

    void init_spi(void)
    {
        RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO;
        RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

        GPIO_Setup(GPIO_PORT_SPI_CLK, GPIO_PIN_SPI_CLK, GPIO_IN_FLOATING);
        GPIO_Setup(GPIO_PORT_SPI_MOSI, GPIO_PIN_SPI_MOSI, GPIO_IN_FLOATING);
        GPIO_Setup(GPIO_PORT_SPI_MISO, GPIO_PIN_SPI_MISO, GPIO_OUT_AF_PP_10MHZ);
        GPIO_Setup(GPIO_PORT_SPI_CS, GPIO_PIN_SPI_CS, GPIO_IN_PULLUP_DOWN);
        GPIO_SetPullup(GPIO_PORT_SPI_CS, GPIO_MASK_SPI_CS);

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
            dma_rx.DMA_PeripheralBaseAddr = (uint32)(&SPI1->DATAR);
            dma_rx.DMA_MemoryBaseAddr = (uint32)spi_rx_data;
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
            dma_tx.DMA_PeripheralBaseAddr = (uint32)(&SPI1->DATAR);
            dma_tx.DMA_MemoryBaseAddr = (uint32)spi_tx_data;
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

        DMA_ClearFlag(0xffff);

        SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

        NVIC_EnableIRQ(DMA1_Channel2_IRQn);
        NVIC_EnableIRQ(DMA1_Channel3_IRQn);

        DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
        DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);

        spi_status = status_idle;
    }

    //////////////////////////////////////////////////////////////////////

    void spi_start()
    {
        SPI1->CTLR1 &= ~SPI_CTLR1_SPE;
        DMA1_Channel2->CFGR &= ~DMA_CFGR1_EN;
        DMA1_Channel3->CFGR &= ~DMA_CFGR1_EN;
        DMA1_Channel2->MADDR = (uint32)spi_rx_data;
        DMA1_Channel3->MADDR = (uint32)spi_tx_data;
        DMA1_Channel2->CNTR = SPI_DATA_SIZE * 2;
        DMA1_Channel3->CNTR = SPI_DATA_SIZE * 2;
        DMA1_Channel2->CFGR |= DMA_CFGR1_EN;
        DMA1_Channel3->CFGR |= DMA_CFGR1_EN;
        SPI1->CTLR1 |= SPI_CTLR1_SPE;
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
        GPIO_Setup(GPIO_PORT_5VSW, GPIO_PIN_5VSW, GPIO_OUT_PP_2MHZ);
        GPIO_Set(GPIO_PORT_5VSW, GPIO_MASK_5VSW);
    }

    //////////////////////////////////////////////////////////////////////

    void init_peripherals()
    {
        RCC->RSTSCKR |= RCC_RMVF;
        RCC->INTR = 0xffff;

        uint32 apb1_reset = RCC_TIM2RST | RCC_TIM3RST | RCC_WWDGRST | RCC_I2C1RST;
        uint32 apb2_reset = RCC_AFIORST | RCC_IOPARST | RCC_IOPBRST | RCC_IOPCRST | RCC_IOPDRST | RCC_ADC1RST
                        | RCC_TIM1RST
                        | RCC_SPI1RST;

        RCC->APB1PRSTR |= apb1_reset;
        RCC->APB2PRSTR |= apb2_reset;
        RCC->APB1PRSTR &= ~apb1_reset;
        RCC->APB2PRSTR &= ~apb2_reset;

        // setup all the things (except SPI)
        init_led();
        init_power();
        init_uart1();
        init_button();
        init_timer2();
        init_timer1();
        init_adc();
        init_systick();

        // don't init SPI yet, just listen on CS line for ESP to drive it high
        GPIO_Setup(GPIO_PORT_SPI_CS, GPIO_PIN_SPI_CS, GPIO_IN_PULLUP_DOWN);
        GPIO_SetPullDown(GPIO_PORT_SPI_CS, GPIO_MASK_SPI_CS);

        distance_timer.reset();

        // switch on VBAT_SNS
        GPIO_Set(GPIO_PORT_VBAT_SNS_EN, GPIO_MASK_VBAT_SNS_EN);

        // don't switch on the 5V rail for level shifter, distance sensor yet
        GPIO_Clear(GPIO_PORT_5VSW, GPIO_MASK_5VSW);

        // don't switch on ESP12 yet
        GPIO_Clear(GPIO_PORT_EN_3V3, GPIO_MASK_EN_3V3);

        // LED off to start with
        GPIO_Set(GPIO_PORT_LED, GPIO_MASK_LED);

        printf("\n\nSysClk:%d (boot %d)\n", SystemCoreClock, standby_boot);
    }
}

//////////////////////////////////////////////////////////////////////

extern "C" int user_main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    init_peripherals();

    set_state(state_boot);

    while(true) {

        switch(state)
        {
            ////////////////////////////////////////

            case state_boot:
            {
                if(state_elapsed_ticks() > 2) {

                    // kick off vbat read
                    vbat_status = status_in_progress;
                    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

                    // switch on the 5V rail for level shifter, distance sensor
                    GPIO_Clear(GPIO_PORT_5VSW, GPIO_MASK_5VSW);

                    // and distance reading
                    num_distance_readings = 0;
                    distance_timer.reset();
                    distance_status = status_idle;
                    got_readings = 0;
                    button.released = 0;
                    button.pressed = 0;
                    GPIO_Clear(GPIO_PORT_LED, GPIO_MASK_LED);
                    set_state(state_reading);
                }
            }
            break;

            ////////////////////////////////////////

            case state_reading:
            {
                ch32_reading_payload_t *payload = outbound_payload<ch32_reading_payload_t>();

                uint32 elapsed = state_elapsed_ticks();

                payload->flags &= ~ch32_flag_factory_reset;
                if(factory_reset) {
                    payload->flags |= ch32_flag_factory_reset;
                    if(elapsed < 1000 || button.held) {
                        GPIO_SetTo(GPIO_PORT_LED, GPIO_MASK_LED, (elapsed >> 6) & 1);
                        break;
                    }
                } else if(state_elapsed_ticks() > 2) {
                    GPIO_Set(GPIO_PORT_LED, GPIO_MASK_LED);
                }

                // if we got a VBAT reading
                if(vbat_status == status_complete) {

                    // stuff it into the SPI payload
                    payload->vbat = vbat_reading;
                    printf("VBAT: %d (0x%04x)\n", vbat_reading, vbat_reading);
                    vbat_status = status_idle;
                    got_readings |= reading_vbat;
                }

                // if we're ready to kick off another distance reading
                if(distance_status == status_idle) {

                    if(distance_timer.elapsed() > 10) {

                        distance_status = status_in_progress;
                        TIM_Cmd(TIM2, ENABLE);
                    }

                } else if(distance_status == status_complete) {

                    printf("DISTANCE[%d] = %d [0x%04x]\n", num_distance_readings, distance_value, distance_value);

                    num_distance_readings += 1;

                    if(MIN_VALID_DISTANCE <= distance_value && distance_value <= MAX_VALID_DISTANCE) {

                        payload->distance = distance_value;
                        got_readings |= reading_distance;

                    } else if(num_distance_readings < MAX_DISTANCE_TRIES) {

                        distance_timer.reset();
                        distance_status = status_idle;

                    } else {

                        payload->distance = 0;
                        got_readings |= reading_distance;
                    }
                }

                // got VBAT and distance readings?
                if(got_readings == reading_all) {

                    // switch off the blasted led for sure
                    GPIO_Set(GPIO_PORT_LED, GPIO_MASK_LED);

                    // finalize the spi payload
                    init_message<ch32_reading_payload_t>(outbound_message());

                    // switch on ESP, start listening for it on SPI_CS
                    GPIO_Set(GPIO_PORT_EN_3V3, GPIO_MASK_EN_3V3);
                    set_state(state_wait_for_esp);
                }

                // timeout on the readings... what to do?
                if(state_elapsed_ticks() > 3000) {
                    printf("Where's the readings?\n");
                    NVIC_SystemReset();
                }
            }
            break;

            ////////////////////////////////////////
            // don't switch on the SPI until the ESP has driven SPI CS high
            // otherwise spurious CS line shenanigans cause a headache

            case state_wait_for_esp:
            {
                if(state_elapsed_ticks() > 100 && GPIO_Get(GPIO_PORT_SPI_CS, GPIO_MASK_SPI_CS) != 0) {

                    init_spi();
                    spi_start();
                    set_state(state_esp);
                }
            }
            break;

            ////////////////////////////////////////

            case state_esp:
            {
                // SPI transaction completed
                if(spi_status == status_complete) {
                    spi_status = status_idle;

                    message_t const *msg = reinterpret_cast<message_t const *>(spi_rx_data);
                    bool ok = check_crc32(msg);

                    esp_status_payload_t const *status = reinterpret_cast<esp_status_payload_t const *>(&msg->body.payload);

                    if((status->flags & esp_status_spi_error) != 0) {
                        printf("SPI ERROR! Resetting SPI\n");
                        init_spi();
                        spi_start();
                    }

                    if((status->flags & (esp_status_send_error | esp_status_sent_reading)) != 0) {
                        printf("ESP done, switch off/sleep etc...\n");
                        flush_printf();
                        sleep(705);    // should be about 6 hours
                    }

                    if(state_elapsed_ticks() < 5000) {
                        printf("SPI (%d) ", ok);
                        for(int i=0; i<SPI_DATA_SIZE; ++i) {
                            printf("%02x", spi_rx_data[i]);
                        }
                        printf("\n");
                    }
                }

                // @DEBUG button puts it to sleep for ~30 seconds

                if(button.pressed) {
                    while(button.held) {
                    }
                    printf("Sleeping...\n");
                    flush_printf();
                    sleep(1);
                }
            }
            break;
        }

        if(char_received != 0) {
            char_received = 0;
        }
    }
}
