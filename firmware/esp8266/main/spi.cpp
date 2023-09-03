//////////////////////////////////////////////////////////////////////

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "driver/spi.h"
#include "driver/gpio.h"
#include "esp8266/gpio_struct.h"
#include "esp8266/spi_register.h"
#include "esp8266/spi_struct.h"
#include "esp_log.h"

#include "types.h"
#include "util.h"
#include "spi.h"

//////////////////////////////////////////////////////////////////////

#define HSPI_CMD (SPI_CMD(1))
#define HSPI_USER (SPI_USER(1))
#define HSPI_USER1 (SPI_USER1(1))
#define HSPI_USER2 (SPI_USER2(1))
#define HSPI_CLOCK (SPI_CLOCK(1))
#define HSPI_PIN (SPI_PIN(1))

#define HSPI_W0 ((uint32 *)(SPI_W0(1)))
#define HSPI_W8 ((uint32 *)(SPI_W8(1)))

#define SPI_INTR_ENABLE() _xt_isr_unmask(1 << ETS_SPI_INUM)
#define SPI_INTR_DISABLE() _xt_isr_mask(1 << ETS_SPI_INUM)
#define SPI_INTR_REGISTER(a, b) _xt_isr_attach(ETS_SPI_INUM, (a), (b))

#define HSPI_IS_BUSY ((READ_PERI_REG(HSPI_CMD) & SPI_USR) != 0)

#define SPI_MASTER_CLOCK_VAL(c) \
    (((c - 1) & SPI_CLKCNT_N) << SPI_CLKCNT_N_S) | (((((c + 1) >> 1) - 1) & SPI_CLKCNT_H) << SPI_CLKCNT_H_S) | (((c - 1) & SPI_CLKCNT_L) << SPI_CLKCNT_L_S)

#define SPI_BITLEN ((SPI_DATA_SIZE * 8) - 1)

//////////////////////////////////////////////////////////////////////

namespace
{
    char const *TAG = "spi";

    void hspi_transaction(uint32 const *send, uint32 *recv)
    {
        while(HSPI_IS_BUSY) {
        }

        HSPI_W0[0] = send[0];
        HSPI_W0[1] = send[1];
        HSPI_W0[2] = send[2];
        HSPI_W0[3] = send[3];
        HSPI_W0[4] = send[4];
        HSPI_W0[5] = send[5];
        HSPI_W0[6] = send[6];
        HSPI_W0[7] = send[7];

        WRITE_PERI_REG(HSPI_USER1, (SPI_BITLEN << SPI_USR_MOSI_BITLEN_S) | (SPI_BITLEN << SPI_USR_MISO_BITLEN_S));

        SET_PERI_REG_MASK(HSPI_CMD, SPI_USR);

        while(HSPI_IS_BUSY) {
        }

        recv[0] = HSPI_W8[0];
        recv[1] = HSPI_W8[1];
        recv[2] = HSPI_W8[2];
        recv[3] = HSPI_W8[3];
        recv[4] = HSPI_W8[4];
        recv[5] = HSPI_W8[5];
        recv[6] = HSPI_W8[6];
        recv[7] = HSPI_W8[7];
    }
}    // namespace

//////////////////////////////////////////////////////////////////////

void spi_init()
{
    ESP_LOGI(TAG, "init_spi");

    CLEAR_PERI_REG_MASK(HSPI_CMD, SPI_USR);

    CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX, SPI1_CLK_EQU_SYS_CLK);

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_HSPIQ_MISO);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_HSPI_CLK);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_HSPID_MOSI);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_HSPI_CS0);

    SET_PERI_REG_MASK(PERIPHS_IO_MUX, PERIPHS_IO_MUX_OE);

    WRITE_PERI_REG(HSPI_CLOCK, SPI_MASTER_CLOCK_VAL(SPI_16MHz_DIV));

    CLEAR_PERI_REG_MASK(HSPI_PIN, SPI_IDLE_EDGE);

    uint32 temp = READ_PERI_REG(HSPI_USER);

    temp &= ~(SPI_CK_I_EDGE | SPI_USR_ADDR | SPI_USR_COMMAND | SPI_USR_DUMMY);
    temp |= SPI_CK_OUT_EDGE | SPI_USR_MISO_HIGHPART | SPI_USR_MOSI | SPI_USR_MISO;

    WRITE_PERI_REG(HSPI_USER, temp);
}

//////////////////////////////////////////////////////////////////////

bool spi_send_now(message_t const *msg, message_t *reply)
{
    hspi_transaction(reinterpret_cast<uint32 const *>(msg), reinterpret_cast<uint32 *>(reply));
    return check_crc32(reply);
}
