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
// x    Measure power consumption
// x    Factory reset (long button press)
// x    B2B Connector
//      Alexa notification
//      Email notification
//      Enclosure
//
//      Status/history web page (Go templates + FastCGI) (Grafana?)
//
//////////////////////////////////////////////////////////////////////

#define DEBUG

#if defined(DEBUG)
#include <stdio.h>
#endif

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

#if defined(DEBUG)
#define debug printf
#else
#define debug(...) do {} while (false)
#endif

#if defined(DEBUG)
#define SLEEP_DELAY_TICKS 100
#else
#define SLEEP_DELAY_TICKS 2
#endif

//////////////////////////////////////////////////////////////////////

namespace
{
    static constexpr int MAX_VALID_DISTANCE = 7000;
    static constexpr int MAX_DISTANCE_TRIES = 5;

    enum state_t
    {
        state_boot = 0,
        state_read_vbat,
        state_read_distance,
        state_wait_for_esp,
        state_esp,
        state_reboot_esp,
        state_done,
        state_num
    };

#if defined(DEBUG)
    char const *state_name[state_num] =
    {
                    "boot",
                    "read_vbat",
                    "read_distance",
                    "wait_for_esp",
                    "esp",
                    "reboot_esp",
                    "done"
    };
#endif

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
        : uint32
    {
        got_reading_vbat = 1,
        got_reading_distance = 2,
        got_reading_all = 3
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
        void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
        void ADC1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
        void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
        void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((used));
    }

    void spi_start();
    void init_peripherals();
    void set_state(state_t new_state);

    //////////////////////////////////////////////////////////////////////

    // main system state

    state_t state = state_boot;
    uint32 state_start_ticks;

    // status of some peripheral handlers

    volatile status_t vbat_status = status_idle;
    volatile status_t spi_status = status_idle;
    volatile status_t distance_status = status_idle;

    // systicks

    volatile uint32 ticks;

    // SPI buffers are double size - 1st half of tx and second half of rx are discarded
    // because ch32 spi is in full duplex mode but esp12 can't seem to do that, it
    // sends then receives the packet sequentially

    uint8 spi_tx_data[SPI_DATA_SIZE * 2] __attribute__ ((aligned(4)));
    uint8 spi_rx_data[SPI_DATA_SIZE * 2] __attribute__ ((aligned(4)));

    // NOTE spi data transactions are guaranteed to happen at <= 10Hz
    // so no need to copy the buffers, use in place if we're quick about it

    message_t &tx_msg = *reinterpret_cast<message_t * const >(spi_tx_data + SPI_DATA_SIZE);
    message_t const &rx_msg = *reinterpret_cast<message_t const * const >(spi_rx_data);

    ch32_reading_payload_t &payload = *reinterpret_cast<ch32_reading_payload_t *>(tx_msg.body.payload);
    esp_status_payload_t const &rx_payload = *reinterpret_cast<esp_status_payload_t const *>(rx_msg.body.payload);

    // distance sensor admin

    int num_distance_readings;
    volatile uint32 distance_value;
    stopwatch_t distance_timer;
    int distance_delay;

    //

    volatile uint16 vbat_reading;
    stopwatch_t vbat_timer;

    button_t button;

    // what have we read so far? (vbat, distance)

    uint32 got_readings = 0;

    // how many boots from standby mode since power on

    int standby_boot = 0;

    uint16 sleep_count = 705;

    //////////////////////////////////////////////////////////////////////

    uint32 state_elapsed_ticks()
    {
        return static_cast<uint32>(static_cast<int32>(ticks) - state_start_ticks);
    }

    //////////////////////////////////////////////////////////////////////

    void set_state(state_t new_state)
    {
        debug("set_state [%s] (%d) after %d ticks\n", state_name[new_state], new_state, state_elapsed_ticks());
        state_start_ticks = ticks;
        state = new_state;
    }

    //////////////////////////////////////////////////////////////////////
    // sleep for some amount of time
    //
    // 61440 / 128000 = 0.48 seconds per tick
    // 63(64?) ticks per loop = 30.24(30.72?) seconds - measured w/PPK at ~30.6 seconds (loop overhead?)
    // so to wait for 6 hours, loop count of 705 is about right (seems to be ~3 to 8 seconds shy)
    // the 128kHz LSI clock is probably not that accurate anyway...
    //
    // button wakes it up, hold for > N seconds to set factory_reset flag
    //
    // do not call this from an interrupt handler

    void sleep(uint32 count)
    {
        __disable_irq();

        payload.flags = 0;

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

        // configure EXTI_Line9 (auto wakeup) and EXTI_Line2 (button) event falling triggers
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
        NVIC->SCTLR |= PFIC_SCTLR_SLEEPDEEP | PFIC_SCTLR_WFITOWFE;

        // 1st WFI seems to be ignored for some reason...? am I not clearing something? irq pending?
        asm volatile ("wfi");

        // go into standby N times for some amount of time, button wakes it up

#pragma GCC unroll 0
        for(uint32 i = count; i != 0; --i) {
            asm volatile ("wfi");
            if(GPIO_Get(GPIO_PORT_BUTTON, GPIO_MASK_BUTTON) == 0) {
                payload.flags |= ch32_flag_button_boot;
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

        __enable_irq();

        SystemInit();        // reinit System Clock to 48MHz HSI

        standby_boot += 1;

        init_peripherals();

        button.pressed = 0;
        button.released = 0;

        state_start_ticks = 0;
        set_state(state_boot);
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
        ADC1->STATR = ~ADC_EOC;
        vbat_reading = (uint16_t)ADC1->RDATAR;
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

    void DMA1_Channel2_IRQHandler(void)
    {
        DMA1->INTFCR = DMA1_FLAG_TC2;
    }

    //////////////////////////////////////////////////////////////////////

    void DMA1_Channel3_IRQHandler(void)
    {
        spi_status = status_complete;
        DMA1->INTFCR = DMA1_FLAG_TC3;
        spi_start();
    }

    //////////////////////////////////////////////////////////////////////
    // UART1 for printf/getchar

    void init_uart1(void)
    {
        RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO;

        GPIO_Setup(GPIO_PORT_TX, GPIO_PIN_TX, GPIO_OUT_AF_PP_10MHZ);

        {
            USART_InitTypeDef USART_InitStructure;
            USART_InitStructure.USART_BaudRate = 115200;
            USART_InitStructure.USART_WordLength = USART_WordLength_8b;
            USART_InitStructure.USART_StopBits = USART_StopBits_1;
            USART_InitStructure.USART_Parity = USART_Parity_No;
            USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
            USART_InitStructure.USART_Mode = USART_Mode_Tx;
            USART_Init(USART1, &USART_InitStructure);
        }

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

    void button_t::update(int state)
    {
        held = state;
        history = (history << 1) | state;
        if(history == 0x0001) {
            pressed = 1;
        } else if(history == 0xfffe) {
            released = 1;
        }
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
        GPIO_Setup(GPIO_PORT_VBAT_SNS_EN, GPIO_PIN_VBAT_SNS_EN, GPIO_OUT_PP_10MHZ);

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

        SPI1->CTLR1 &= ~SPI_CTLR1_SPE;

        uint32 constexpr SPI_CTLR1_CLEAR_Mask = 0x00003040;
        uint32 constexpr DMA_CFGR_CLEAR_Mask = 0xFFFF800F;

        SPI1->CTLR1 = (SPI1->CTLR1 & SPI_CTLR1_CLEAR_Mask)
                        | SPI_Direction_2Lines_FullDuplex
                        | SPI_CPOL_High
                        | SPI_CPHA_1Edge
                        | SPI_NSS_Hard
                        | SPI_BaudRatePrescaler_2
                        | SPI_FirstBit_MSB;

        SPI1->CRCR = 7;

        DMA1_Channel2->CFGR = (DMA1_Channel2->CFGR & DMA_CFGR_CLEAR_Mask)
                        | DMA_DIR_PeripheralSRC
                        | DMA_Mode_Normal
                        | DMA_PeripheralInc_Disable
                        | DMA_MemoryInc_Enable
                        | DMA_PeripheralDataSize_Byte
                        | DMA_MemoryDataSize_Byte
                        | DMA_Priority_Medium
                        | DMA_M2M_Disable;

        DMA1_Channel2->CNTR = SPI_DATA_SIZE * 2;
        DMA1_Channel2->PADDR = reinterpret_cast<uint32>(&SPI1->DATAR);
        DMA1_Channel2->MADDR = reinterpret_cast<uint32>(spi_rx_data);

        DMA1_Channel3->CFGR = (DMA1_Channel3->CFGR & DMA_CFGR_CLEAR_Mask)
                        | DMA_DIR_PeripheralDST
                        | DMA_Mode_Normal
                        | DMA_PeripheralInc_Disable
                        | DMA_MemoryInc_Enable
                        | DMA_PeripheralDataSize_Byte
                        | DMA_MemoryDataSize_Byte
                        | DMA_Priority_Medium
                        | DMA_M2M_Disable;

        DMA1_Channel3->CNTR = SPI_DATA_SIZE * 2;
        DMA1_Channel3->PADDR = reinterpret_cast<uint32>(&SPI1->DATAR);
        DMA1_Channel3->MADDR = reinterpret_cast<uint32>(spi_tx_data);

        DMA1->INTFCR = 0xffff;

        SPI1->CTLR2 |= SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx;

        NVIC_EnableIRQ(DMA1_Channel2_IRQn);
        NVIC_EnableIRQ(DMA1_Channel3_IRQn);

        DMA1_Channel2->CFGR |= DMA_IT_TC;
        DMA1_Channel3->CFGR |= DMA_IT_TC;

        spi_status = status_idle;
    }

    //////////////////////////////////////////////////////////////////////
    // order here is important:
    // disable SPI, disable DMA, setup DMA, enable SPI, enable DMA

    void spi_start()
    {
        SPI1->CTLR1 &= ~SPI_CTLR1_SPE;
        DMA1_Channel2->CFGR &= ~DMA_CFGR1_EN;
        DMA1_Channel3->CFGR &= ~DMA_CFGR1_EN;

        DMA1_Channel2->MADDR = reinterpret_cast<uint32>(spi_rx_data);
        DMA1_Channel3->MADDR = reinterpret_cast<uint32>(spi_tx_data);
        DMA1_Channel2->CNTR = SPI_DATA_SIZE * 2;
        DMA1_Channel3->CNTR = SPI_DATA_SIZE * 2;

        SPI1->CTLR1 |= SPI_CTLR1_SPE;    // NOTE: you must enable the SPI before the DMA otherwise you get spurious bytes!
        DMA1_Channel2->CFGR |= DMA_CFGR1_EN;
        DMA1_Channel3->CFGR |= DMA_CFGR1_EN;
    }

    //////////////////////////////////////////////////////////////////////

    void led_off()
    {
        GPIO_Set(GPIO_PORT_LED, GPIO_MASK_LED);
    }

    //////////////////////////////////////////////////////////////////////

    void led_on()
    {
        GPIO_Clear(GPIO_PORT_LED, GPIO_MASK_LED);
    }

    //////////////////////////////////////////////////////////////////////

    void init_led()
    {
        RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;

        led_off();
        GPIO_Setup(GPIO_PORT_LED, GPIO_PIN_LED, GPIO_OUT_PP_10MHZ);
    }

    //////////////////////////////////////////////////////////////////////

    void init_power()
    {
        RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;

        GPIO_Set(GPIO_PORT_5VSW, GPIO_MASK_5VSW);

        GPIO_Setup(GPIO_PORT_5VSW, GPIO_PIN_5VSW, GPIO_OUT_PP_10MHZ);
    }

    //////////////////////////////////////////////////////////////////////

    void init_peripherals()
    {
        // clear the RCC reset flag
        RCC->RSTSCKR |= RCC_RMVF;

        // clear and enable all pending RCC irqs
        RCC->INTR = 0xffff;

        // reset the peripherals on apb1, apb2
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

        // switch on VBAT_SNS
        GPIO_Set(GPIO_PORT_VBAT_SNS_EN, GPIO_MASK_VBAT_SNS_EN);

        // don't switch on the 5V rail for level shifter, distance sensor & ESP12 yet
        GPIO_Set(GPIO_PORT_5VSW, GPIO_MASK_5VSW);

        // LED off to start with
        led_off();

        debug("\n\nSysClk:%d (boot %d)\n", SystemCoreClock, standby_boot);
    }
}

//////////////////////////////////////////////////////////////////////

extern "C" int main()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    init_peripherals();

    payload.flags = 0;

    set_state(state_boot);

    while(true) {

        switch(state)
        {
            ////////////////////////////////////////

            case state_boot:
            {
                if(button.held && state_elapsed_ticks() >= 5000) {

                    payload.flags |= ch32_flag_factory_reset;
                }

                if((!button.held || (payload.flags & ch32_flag_factory_reset) != 0) && state_elapsed_ticks() > 10) {

                    got_readings = 0;

                    payload.distance = 0;
                    payload.vbat = 0;

                    vbat_status = status_idle;
                    vbat_timer.reset();

                    button.released = 0;
                    button.pressed = 0;

                    set_state(state_read_vbat);
                }
            }
            break;

            ////////////////////////////////////////

            case state_read_vbat:
            {
                uint32 elapsed = state_elapsed_ticks();

                if((payload.flags & ch32_flag_factory_reset) != 0) {

                    if(elapsed < 1000 || button.held) {

                        GPIO_SetTo(GPIO_PORT_LED, GPIO_MASK_LED, (elapsed >> 6) & 1);
                        break;
                    }

                }

                // give power 5 ms to stabilize before reading vbat
                if(vbat_status == status_idle && elapsed > 5) {

                    // kick off vbat read
                    vbat_status = status_in_progress;
                    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
                }

                // if we got a VBAT reading
                if(vbat_status == status_complete) {

                    // stuff it into the SPI payload
                    payload.vbat = vbat_reading;
                    debug("VBAT: %d (0x%04x)\n", vbat_reading, vbat_reading);
                    vbat_status = status_idle;
                    got_readings |= got_reading_vbat;

                    // switch on the 5V rail for level shifter, distance sensor
                    GPIO_Clear(GPIO_PORT_5VSW, GPIO_MASK_5VSW);

                    // and distance reading
                    num_distance_readings = 0;
                    distance_timer.reset();
                    distance_status = status_idle;
                    distance_delay = 50;
                    led_on();
                    set_state(state_read_distance);
                }
            }
            break;

            ////////////////////////////////////////

            case state_read_distance:
            {
                uint32 elapsed = state_elapsed_ticks();

                // end led flash at start of distance reading
                if(elapsed > 2) {
                    led_off();
                }

                // if we're ready to kick off another distance reading
                if(distance_status == status_idle) {

                    if(distance_timer.elapsed() > distance_delay) {

                        distance_status = status_in_progress;
                        TIM_Cmd(TIM2, ENABLE);
                        distance_delay = 2;
                    }

                } else if(distance_status == status_complete) {

                    debug("DISTANCE[%d] = %d [0x%04x]\n", num_distance_readings, distance_value, distance_value);

                    num_distance_readings += 1;

                    if(distance_value != 0 && distance_value <= MAX_VALID_DISTANCE) {

                        payload.distance = distance_value;

                    } else if(num_distance_readings < MAX_DISTANCE_TRIES) {

                        distance_timer.reset();
                        distance_status = status_idle;

                    } else {

                        payload.flags |= ch32_flag_error_reading_distance;
                    }
                }

                if(((payload.flags & ch32_flag_error_reading_distance) !=0 ) || elapsed > 500 || payload.distance != 0) {

                    // switch off the blasted led for sure
                    led_off();

                    // finalize the spi payload
                    init_message<ch32_reading_payload_t>(&tx_msg);

                    // switch on ESP, start listening for it on SPI_CS
                    set_state(state_wait_for_esp);
                }

            }
            break;

            ////////////////////////////////////////
            // don't switch on the SPI until the ESP has driven SPI CS high
            // otherwise spurious CS line shenanigans cause a headache

            case state_wait_for_esp:
            {
                if(GPIO_Get(GPIO_PORT_SPI_CS, GPIO_MASK_SPI_CS) != 0) {

                    init_spi();
                    spi_start();
                    set_state(state_esp);

                } else if(state_elapsed_ticks() > 1000) {

                    led_on();
                    GPIO_Set(GPIO_PORT_5VSW, GPIO_MASK_5VSW);
                    set_state(state_reboot_esp);
                }
            }
            break;

            ////////////////////////////////////////

            case state_reboot_esp:
            {
                if(state_elapsed_ticks() > 1000) {
                    GPIO_Clear(GPIO_PORT_5VSW, GPIO_MASK_5VSW);
                    set_state(state_wait_for_esp);
                    led_off();
                }
            }
            break;

            ////////////////////////////////////////

            case state_esp:
            {
                // SPI transaction completed
                if(spi_status == status_complete) {

                    spi_status = status_idle;

                    bool ch32_ok = check_crc32(&rx_msg);
                    bool esp_ok = (rx_payload.flags & esp_status_spi_error) == 0;

                    if(!ch32_ok) {

                        debug("RECV SPI ERROR! Resetting SPI (got %08x, expected %08x) ID %02x, LEN %02x\n",
                                        rx_msg.crc, calc_crc32(reinterpret_cast<uint8_t const *>(&rx_msg.body), sizeof(message_body_t)),
                                        rx_msg.body.ident, rx_msg.body.length);
                        init_spi();
                        spi_start();
                    }

                    else if(!esp_ok) {

                        debug("SEND SPI ERROR! Resetting SPI\n");

                    } else {

                        if((rx_payload.flags & esp_status_factory_resetting) != 0) {

                            payload.flags &= ~ch32_flag_factory_reset;
                            init_message<ch32_reading_payload_t>(&tx_msg);
                        }

                        if((rx_payload.flags & esp_status_done) != 0) {

                            debug("ESP done: Flags: %04x, Sleep count: %d\n", rx_payload.flags, rx_payload.sleep_count);
                            set_state(state_done);
                        }
                    }
                }

                if(state_elapsed_ticks() > 10000) {
                    led_on();
                    GPIO_Set(GPIO_PORT_5VSW, GPIO_MASK_5VSW);
                    set_state(state_reboot_esp);
                }
            }
            break;

            ////////////////////////////////////////

            case state_done:
            {
                if(state_elapsed_ticks() > SLEEP_DELAY_TICKS) {

                    if(rx_payload.sleep_count != 0) {
                        sleep_count = rx_payload.sleep_count;
                    } else {
                        debug("No sleep count, keeping %d\n", sleep_count);
                    }
                    debug("DONE Sleeping for %d\n", sleep_count);
                    flush_printf();
                    sleep(sleep_count);    // should be about 6 hours
                }
            }
            break;
        }

        // @DEBUG button puts it to sleep for ~30 seconds

#if defined(DEBUG)

        if(button.pressed) {
            stopwatch_t debounce;
            debounce.reset();
            while(debounce.elapsed() < 10) {
                if(button.held) {
                    debounce.reset();
                }
            }
            debug("BUTTON Sleeping...\n");
            flush_printf();
            sleep(1);
        }

#endif
    }
}

