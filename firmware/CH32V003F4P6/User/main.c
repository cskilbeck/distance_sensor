//////////////////////////////////////////////////////////////////////

#include "debug.h"
#include "user_gpio.h"
#include "user_pins.h"

//////////////////////////////////////////////////////////////////////

typedef enum
{
    status_idle,
    status_in_progress,
    status_complete
} status_t;

//////////////////////////////////////////////////////////////////////

typedef struct button_t
{
    u16 history;
    u16 held :1;
    u16 pressed :1;
    u16 released :1;
} button_t;

//////////////////////////////////////////////////////////////////////

#define SPI_DATA_SIZE          16

u16 spi_tx_data[SPI_DATA_SIZE] = { 0x0101, 0x0202, 0x0303, 0x0404, 0x0505, 0x0606, 0x1111, 0x1212, 0x1313, 0x1414,
                                   0x1515, 0x1616, 0x2121, 0x2222, 0x2323, 0x2424 };

u16 spi_rx_data[SPI_DATA_SIZE];

//////////////////////////////////////////////////////////////////////

void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void ADC1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SPI1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

//////////////////////////////////////////////////////////////////////

#define NUM_TIMER_READINGS 16

u32 readings[NUM_TIMER_READINGS] = { 0 };
int num_readings = 0;
int cur_reading = 0;
u32 cur_total = 0;

button_t button = { 0 };

volatile status_t vbat_status = status_idle;
volatile status_t spi_status = status_idle;
volatile status_t distance_status = status_idle;

u32 vbat_reading = 0;

volatile u32 distance_value = 0;

volatile u8 char_received = 0;

volatile u16 ticks = 0;

//////////////////////////////////////////////////////////////////////

void button_update(button_t *b, int state)
{
    u32 h = b->history;
    h = (h << 1) | state;
    b->history = h;
    b->held = state;
    if(h == 0xfffe) {
        b->pressed = 1;
    }
    else if(h == 0x0001) {
        b->released = 1;
    }
}

//////////////////////////////////////////////////////////////////////

void SysTick_Handler(void)
{
    SysTick->SR &= ~(1 << 0);
    button_update(&button, GPIO_Get(GPIO_PORT_BUTTON, GPIO_MASK_BUTTON));
    ticks += 1;
}

//////////////////////////////////////////////////////////////////////

void ADC1_IRQHandler(void)
{
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    vbat_reading = ADC_GetConversionValue(ADC1);
    GPIO_Clear(GPIO_PORT_VBAT_SNS_EN, GPIO_MASK_VBAT_SNS_EN);
    vbat_status = status_complete;
}

//////////////////////////////////////////////////////////////////////

void init_uart1(void)
{
    USART_InitTypeDef USART_InitStructure = { 0 };

    GPIO_PinRemapConfig(GPIO_PartialRemap2_USART1, ENABLE);

    GPIO_Setup(GPIO_PORT_TX, GPIO_PIN_TX, GPIO_OUT_AF_PP_50MHZ);
    GPIO_Setup(GPIO_PORT_RX, GPIO_PIN_RX, GPIO_IN_FLOATING);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);

    USART1->CTLR1 |= USART_CTLR1_UE;
}

//////////////////////////////////////////////////////////////////////

void USART1_IRQHandler(void)
{
    char_received = USART1->DATAR;
}

//////////////////////////////////////////////////////////////////////

void init_timer2()
{
    TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = { 0 };

    GPIO_Setup(GPIO_PORT_SNS_TX, GPIO_PIN_SNS_TX, GPIO_OUT_AF_PP_50MHZ);

    TIM_TimeBaseInitStructure.TIM_Period = 10;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 47;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Single);
}

//////////////////////////////////////////////////////////////////////

void init_timer1()
{
    TIM_ICInitTypeDef TIM_ICInitStructure = { 0 };
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    GPIO_Setup(GPIO_PORT_SNS_RX, GPIO_PIN_SNS_RX, GPIO_IN_FLOATING);

    TIM_TimeBaseInitStructure.TIM_Period = 0xffff;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 289;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;

    TIM_PWMIConfig(TIM1, &TIM_ICInitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

    TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
    TIM1->CTLR1 |= TIM_CEN;
}

//////////////////////////////////////////////////////////////////////

void TIM1_CC_IRQHandler(void)
{
    if((TIM1->INTFR & TIM_IT_CC2) != RESET) {
        distance_value = TIM1->CH2CVR;
        distance_status = status_complete;
    }
    TIM1->INTFR = (uint16_t) ~(TIM_IT_CC1 | TIM_IT_CC2);
}

//////////////////////////////////////////////////////////////////////

void adc_7_init(void)
{
    ADC_InitTypeDef ADC_InitStructure = { 0 };

    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    GPIO_Setup(GPIO_PORT_VBAT_SNS, GPIO_PIN_VBAT_SNS, GPIO_IN_AIN);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init( ADC1, &ADC_InitStructure);

    ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
    ADC_Cmd( ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1)) {
    }

    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1)) {
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

void spi_init(void)
{
    SPI_InitTypeDef SPI_InitStructure = { 0 };

    GPIO_Setup(GPIO_PORT_SPI_CLK, GPIO_PIN_SPI_CLK, GPIO_OUT_AF_PP_10MHZ);
    GPIO_Setup(GPIO_PORT_SPI_MOSI, GPIO_PIN_SPI_MOSI, GPIO_OUT_AF_PP_10MHZ);
    GPIO_Setup(GPIO_PORT_SPI_MISO, GPIO_PIN_SPI_MISO, GPIO_IN_FLOATING);

    //GPIO_Setup(GPIO_PORT_SPI_MOSI, GPIO_PIN_SPI_MOSI, GPIO_OUT_AF_PP_10MHZ);
    GPIO_Setup(GPIO_PORT_SPI_CS, GPIO_PIN_SPI_CS, GPIO_IN_PULLUP_DOWN);
    GPIO_SetPullDown(GPIO_PORT_SPI_CS, GPIO_MASK_SPI_CS);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
}

//////////////////////////////////////////////////////////////////////

void dma_tx_init(DMA_Channel_TypeDef *DMA_CHx, u32 periph_addr, u32 mem_addr, u16 buf_size)
{
    DMA_InitTypeDef DMA_InitStructure = { 0 };
    DMA_DeInit(DMA_CHx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = periph_addr;
    DMA_InitStructure.DMA_MemoryBaseAddr = mem_addr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = buf_size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);
}

//////////////////////////////////////////////////////////////////////

void dma_rx_init(DMA_Channel_TypeDef *DMA_CHx, u32 periph_addr, u32 mem_addr, u16 buf_size)
{
    DMA_InitTypeDef DMA_InitStructure = { 0 };
    DMA_DeInit(DMA_CHx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = periph_addr;
    DMA_InitStructure.DMA_MemoryBaseAddr = mem_addr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = buf_size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);
}

//////////////////////////////////////////////////////////////////////

void SPI1_IRQHandler(void)
{
    SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_TXE);
    if(spi_status == status_complete) {
        GPIO_Set(GPIO_PORT_LED, GPIO_MASK_LED);
        spi_status = status_idle;
        SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);
    }
}

//////////////////////////////////////////////////////////////////////

void DMA1_Channel2_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_IT_TC2);
    DMA_ClearFlag(DMA1_FLAG_TC2);
    DMA_Cmd(DMA1_Channel2, DISABLE);
}

//////////////////////////////////////////////////////////////////////

void DMA1_Channel3_IRQHandler(void)
{
    SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
    spi_status = status_complete;
    DMA_ClearITPendingBit(DMA1_IT_TC3);
    DMA_ClearFlag(DMA1_FLAG_TC3);
    DMA_Cmd(DMA1_Channel3, DISABLE);
}

//////////////////////////////////////////////////////////////////////

int main(void)
{
    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

    RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

    RCC->APB2PCENR |= RCC_APB2Periph_GPIOD
            | RCC_APB2Periph_GPIOC
            | RCC_APB2Periph_GPIOA
            | RCC_APB2Periph_TIM1
            | RCC_APB2Periph_SPI1
            | RCC_APB2Periph_AFIO
            | RCC_APB2Periph_USART1
            | RCC_APB2Periph_ADC1;

    GPIO_Set(GPIO_PORT_LED, GPIO_MASK_LED);
    GPIO_Setup(GPIO_PORT_LED, GPIO_PIN_LED, GPIO_OUT_PP_2MHZ);

    GPIO_Setup(GPIO_PORT_VBAT_SNS_EN, GPIO_PIN_VBAT_SNS_EN, GPIO_OUT_PP_2MHZ);

    GPIO_Setup(GPIO_PORT_EN_3V3, GPIO_PIN_EN_3V3, GPIO_OUT_PP_2MHZ);
    GPIO_Setup(GPIO_PORT_LVL_OE, GPIO_PIN_LVL_OE, GPIO_OUT_PP_2MHZ);

    GPIO_Setup(GPIO_PORT_BUTTON, GPIO_PIN_BUTTON, GPIO_IN_PULLUP_DOWN);
    GPIO_SetPullup(GPIO_PORT_BUTTON, GPIO_MASK_BUTTON);

    init_uart1();

    printf("SystemClk:%d\r\n", SystemCoreClock);

    init_timer2();
    init_timer1();
    adc_7_init();
    spi_init();
    GPIO_Set(GPIO_PORT_EN_3V3, GPIO_MASK_EN_3V3);
    GPIO_Set(GPIO_PORT_LVL_OE, GPIO_MASK_LVL_OE);

    dma_rx_init(DMA1_Channel2, (u32) &SPI1->DATAR, (u32) spi_rx_data, SPI_DATA_SIZE);
    dma_tx_init(DMA1_Channel3, (u32) &SPI1->DATAR, (u32) spi_tx_data, SPI_DATA_SIZE);

    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);

    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    NVIC_EnableIRQ(SPI1_IRQn);

    init_systick();

    u16 now = ticks;

    SPI_Cmd(SPI1, ENABLE);

    while(1) {

        u16 elapsed = (s16) ticks - now;

        if(elapsed >= 1000) {
            printf("%d\r\n", ticks);
            now = ticks;
            GPIO_Clear(GPIO_PORT_LED, GPIO_MASK_LED);
            spi_status = status_in_progress;
            SPI_SSOutputCmd(SPI1, DISABLE);
            DMA_SetCurrDataCounter(DMA1_Channel2, SPI_DATA_SIZE);
            DMA_SetCurrDataCounter(DMA1_Channel3, SPI_DATA_SIZE);
            DMA1_Channel2->MADDR = (u32)spi_rx_data;
            DMA1_Channel3->MADDR = (u32)spi_tx_data;
            DMA_Cmd(DMA1_Channel2, ENABLE);
            DMA_Cmd(DMA1_Channel3, ENABLE);
        }

        if(button.pressed) {

            button.pressed = 0;

            if(vbat_status == status_idle) {
                GPIO_Set(GPIO_PORT_VBAT_SNS_EN, GPIO_MASK_VBAT_SNS_EN);
                vbat_status = status_in_progress;
                ADC_SoftwareStartConvCmd(ADC1, ENABLE);
            }


            if(distance_status == status_idle) {
                TIM2->CTLR1 |= TIM_CEN;
            }
        }

        if(vbat_status == status_complete) {
            vbat_status = status_idle;
            printf("%d\n", vbat_reading);
        }

        if(char_received != 0) {
            char_received = 0;
        }

        if(distance_status == status_complete) {
            distance_status = status_idle;
            if(distance_value < 500) {
                u32 cur_average = 0;
                cur_total -= readings[cur_reading];
                cur_total += distance_value;
                readings[cur_reading] = distance_value;
                cur_reading = ++cur_reading % NUM_TIMER_READINGS;
                num_readings += 1;
                if(num_readings >= NUM_TIMER_READINGS) {
                    cur_average = cur_total / NUM_TIMER_READINGS;
                }
                printf("%d (%d)\r\n", distance_value, cur_average);
            }
        }
        __WFI();
    }
}

