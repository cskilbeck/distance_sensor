//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <memory.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi.h"
#include "esp8266/gpio_struct.h"
#include "esp8266/spi_register.h"
#include "esp8266/spi_struct.h"
#include "esp_log.h"

#include "user.h"

//////////////////////////////////////////////////////////////////////

static char const *TAG = "user";

//////////////////////////////////////////////////////////////////////

uint16 crc16(message_body_t const *m)
{
    uint16 crc = 0xffff;
    byte const *p = reinterpret_cast<byte const *>(m);
    byte const *e = p + sizeof(message_body_t);
    for(; p < e; ++p) {
        uint16 x = (crc >> 8) ^ *p;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    }
    return crc;
}

//////////////////////////////////////////////////////////////////////

bool is_valid_message(message_t *m)
{
    return m->msg.signature == 'DC' && crc16(&m->msg) == m->crc;
}

//////////////////////////////////////////////////////////////////////

void init_message(message_t *message, uint64_t timestamp, uint32_t options)
{
    message->msg.timestamp = timestamp;
    message->msg.options = options;
    message->msg.signature = 'DC';
    message->crc = crc16(&message->msg);
}

//////////////////////////////////////////////////////////////////////

uint32_t mosi_buffer[8];
uint32_t miso_buffer[8];

#define HSPI_CMD (SPI_CMD(1))
#define HSPI_USER (SPI_USER(1))
#define HSPI_USER1 (SPI_USER1(1))
#define HSPI_USER2 (SPI_USER2(1))
#define HSPI_CLOCK (SPI_CLOCK(1))
#define HSPI_PIN (SPI_PIN(1))
#define HSPI_W(x) (reinterpret_cast<uint32_t *>(SPI_W0(1)) + (x))

#define SPI_INTR_ENABLE() _xt_isr_unmask(1 << ETS_SPI_INUM)
#define SPI_INTR_DISABLE() _xt_isr_mask(1 << ETS_SPI_INUM)
#define SPI_INTR_REGISTER(a, b) _xt_isr_attach(ETS_SPI_INUM, (a), (b))

#define HSPI_IS_BUSY ((READ_PERI_REG(HSPI_CMD) & SPI_USR) != 0)
// SPI[host]->clock.clk_equ_sysclk = false;
// SPI[host]->clock.clkdiv_pre = 0;
// SPI[host]->clock.clkcnt_n = *clk_div - 1;
// // In the master mode clkcnt_h = floor((clkcnt_n+1)/2-1). In the slave mode it must be 0
// SPI[host]->clock.clkcnt_h = *clk_div / 2 - 1;
// // In the master mode clkcnt_l = clkcnt_n. In the slave mode it must be 0
// SPI[host]->clock.clkcnt_l = *clk_div - 1;

#define SPI_MASTER_CLOCK_VAL(c) \
    (((c - 1) & SPI_CLKCNT_N) << SPI_CLKCNT_N_S) | (((((c + 1) >> 1) - 1) & SPI_CLKCNT_H) << SPI_CLKCNT_H_S) | (((c - 1) & SPI_CLKCNT_L) << SPI_CLKCNT_L_S)

#define SPI_TEST_MODE 2

void init_hspi()
{
#if SPI_TEST_MODE == 0 && 0

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO13);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO14);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);

    gpio_config_t cfg;
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    cfg.pin_bit_mask = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio_config(&cfg);

#elif SPI_TEST_MODE == 1

    spi_config_t cfg;
    cfg.interface.cpol = SPI_CPOL_HIGH;
    cfg.interface.cpha = SPI_CPHA_LOW;
    cfg.interface.bit_tx_order = SPI_BIT_ORDER_LSB_FIRST;    // Actually MSB first
    cfg.interface.bit_rx_order = SPI_BIT_ORDER_LSB_FIRST;    // Actually MSB first
    cfg.interface.byte_tx_order = SPI_BYTE_ORDER_LSB_FIRST;
    cfg.interface.byte_rx_order = SPI_BYTE_ORDER_LSB_FIRST;
    cfg.interface.mosi_en = 1;
    cfg.interface.miso_en = 1;
    cfg.interface.cs_en = 1;
    cfg.intr_enable.read_buffer = 0;
    cfg.intr_enable.write_buffer = 0;
    cfg.intr_enable.read_status = 0;
    cfg.intr_enable.write_status = 0;
    cfg.intr_enable.trans_done = 0;
    cfg.event_cb = NULL;
    cfg.mode = SPI_MASTER_MODE;
    cfg.clk_div = SPI_16MHz_DIV;
    spi_init(HSPI_HOST, &cfg);

#elif SPI_TEST_MODE == 2

    CLEAR_PERI_REG_MASK(HSPI_CMD, SPI_USR);

    CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX, SPI1_CLK_EQU_SYS_CLK);
    SET_PERI_REG_MASK(PERIPHS_IO_MUX, PERIPHS_IO_MUX_OE);

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_HSPIQ_MISO);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_HSPI_CLK);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_HSPID_MOSI);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_HSPI_CS0);

    WRITE_PERI_REG(HSPI_CLOCK, SPI_MASTER_CLOCK_VAL(SPI_16MHz_DIV));

    CLEAR_PERI_REG_MASK(HSPI_PIN, SPI_IDLE_EDGE);

    SET_PERI_REG_MASK(HSPI_USER, SPI_RD_BYTE_ORDER | SPI_CK_OUT_EDGE | SPI_USR_MISO_HIGHPART | SPI_USR_MOSI | SPI_USR_MISO);

    CLEAR_PERI_REG_MASK(HSPI_USER, SPI_CK_I_EDGE | SPI_USR_ADDR | SPI_USR_COMMAND | SPI_USR_DUMMY);

#endif
}

//////////////////////////////////////////////////////////////////////

void hspi_transaction()
{
    memset(mosi_buffer, 0, sizeof(mosi_buffer));
    for(uint8_t i = 0; i < sizeof(mosi_buffer); ++i) {
        ((uint8_t *)mosi_buffer)[i] = 0xAA;
    }

#if SPI_TEST_MODE == 1

    spi_trans_t trans;
    memset(&trans, 0, sizeof(trans));
    trans.mosi = mosi_buffer;
    trans.miso = miso_buffer;
    trans.bits.mosi = 128;
    trans.bits.miso = 128;

    spi_trans(HSPI_HOST, &trans);

#elif SPI_TEST_MODE == 2

    while(HSPI_IS_BUSY) {
    }

    WRITE_PERI_REG(HSPI_USER1, (255 << SPI_USR_MOSI_BITLEN_S) | (255 << SPI_USR_MISO_BITLEN_S));
    memcpy(HSPI_W(0), mosi_buffer, sizeof(mosi_buffer));

    SET_PERI_REG_MASK(HSPI_CMD, SPI_USR);

    while(HSPI_IS_BUSY) {
    }

    memcpy(miso_buffer, HSPI_W(8), sizeof(miso_buffer));

#endif
}

//////////////////////////////////////////////////////////////////////

message_t message;

extern "C" void user_main()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, "CPU cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "Silicon revision: %d, ", chip_info.revision);
    ESP_LOGI(TAG, "Flash: %dMB %s", spi_flash_get_chip_size() / (1024 * 1024), (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI(TAG, "==============================");

    init_hspi();

    int x = 0;
    while(true) {
        vTaskDelay(250 / portTICK_PERIOD_MS);
        hspi_transaction();
        uint32_t *p = (uint32_t *)miso_buffer;
        ESP_LOGI(TAG, "Got %4d,0x%08x%08x%08x%08x", x++, p[0], p[1], p[2], p[3]);
    }
}
