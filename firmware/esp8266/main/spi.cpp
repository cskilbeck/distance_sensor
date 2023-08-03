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

#define HSPI_W0 ((uint32_t *)(SPI_W0(1)))
#define HSPI_W8 ((uint32_t *)(SPI_W8(1)))

#define SPI_INTR_ENABLE() _xt_isr_unmask(1 << ETS_SPI_INUM)
#define SPI_INTR_DISABLE() _xt_isr_mask(1 << ETS_SPI_INUM)
#define SPI_INTR_REGISTER(a, b) _xt_isr_attach(ETS_SPI_INUM, (a), (b))

#define HSPI_IS_BUSY ((READ_PERI_REG(HSPI_CMD) & SPI_USR) != 0)

#define SPI_MASTER_CLOCK_VAL(c) \
    (((c - 1) & SPI_CLKCNT_N) << SPI_CLKCNT_N_S) | (((((c + 1) >> 1) - 1) & SPI_CLKCNT_H) << SPI_CLKCNT_H_S) | (((c - 1) & SPI_CLKCNT_L) << SPI_CLKCNT_L_S)

#define SPI_BITLEN ((SPI_DATA_SIZE * 8) - 1)

//////////////////////////////////////////////////////////////////////
// event bits

#define BUF_0_FULL 1
#define BUF_1_FULL 2
#define BUF_0_EMPTY 4
#define BUF_1_EMPTY 8

#define BUF_ANY_FULL (BUF_0_FULL | BUF_1_FULL)
#define BUF_ANY_EMPTY (BUF_0_EMPTY | BUF_1_EMPTY)

//////////////////////////////////////////////////////////////////////

namespace
{
    char const *TAG = "spi";

    uint32_t mosi_buffer[2][SPI_DATA_SIZE / sizeof(uint32_t)];
    uint32_t miso_buffer[2][SPI_DATA_SIZE / sizeof(uint32_t)];

    TaskHandle_t spi_task_handle = null;
    EventGroupHandle_t spi_eventgroup;

    on_spi_callback spi_callback;
    on_spi_callback spi_error_callback;

    void hspi_transaction(uint32_t const *send, uint32_t *recv)
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

    //////////////////////////////////////////////////////////////////////

    void spi_task(void *)
    {
        ESP_LOGI(TAG, "spi task begins...");

        while(true) {

            EventBits_t bits = xEventGroupWaitBits(spi_eventgroup, BUF_ANY_FULL, true, false, -1);

            int index = 0;

            for(int x = BUF_0_FULL; x <= BUF_1_FULL; x <<= 1) {

                if((bits & x) != 0) {

                    hspi_transaction(mosi_buffer[index], miso_buffer[index]);

                    message_t const *msg = reinterpret_cast<message_t const *>(miso_buffer[index]);

                    log_buffer(TAG, miso_buffer[index], 32, ESP_LOG_INFO);

                    if(check_crc32(msg)) {
                        if(spi_callback != null) {
                            spi_callback(msg);
                        }
                    } else {
                        ESP_LOGE(TAG, "CRC err got %08x, expected %08x", msg->crc, calc_crc32((uint8_t const *)miso_buffer[index], sizeof(miso_buffer[index])));
                        if(spi_error_callback != null) {
                            spi_error_callback(msg);
                        }
                    }

                    xEventGroupSetBits(spi_eventgroup, BUF_0_EMPTY << index);
                }
                index += 1;
            }
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

void init_spi(on_spi_callback callback, on_spi_callback error_callback)
{
    ESP_LOGI(TAG, "init_spi");

    spi_error_callback = error_callback;
    spi_callback = callback;

    CLEAR_PERI_REG_MASK(HSPI_CMD, SPI_USR);

    CLEAR_PERI_REG_MASK(PERIPHS_IO_MUX, SPI1_CLK_EQU_SYS_CLK);
    SET_PERI_REG_MASK(PERIPHS_IO_MUX, PERIPHS_IO_MUX_OE);

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_HSPIQ_MISO);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_HSPI_CLK);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_HSPID_MOSI);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_HSPI_CS0);

    WRITE_PERI_REG(HSPI_CLOCK, SPI_MASTER_CLOCK_VAL(SPI_16MHz_DIV));

    CLEAR_PERI_REG_MASK(HSPI_PIN, SPI_IDLE_EDGE);

    uint32 temp = READ_PERI_REG(HSPI_USER);

    temp &= ~(SPI_CK_I_EDGE | SPI_USR_ADDR | SPI_USR_COMMAND | SPI_USR_DUMMY);
    temp |= SPI_CK_OUT_EDGE | SPI_USR_MISO_HIGHPART | SPI_USR_MOSI | SPI_USR_MISO;

    WRITE_PERI_REG(HSPI_USER, temp);

    spi_eventgroup = xEventGroupCreate();
    xEventGroupSetBits(spi_eventgroup, BUF_ANY_EMPTY);    // buffers both available

    xTaskCreate(spi_task, "spi_task", 2048, null, 1, &spi_task_handle);
}

//////////////////////////////////////////////////////////////////////

void spi_send(message_t const *msg)
{
    EventBits_t bits = xEventGroupWaitBits(spi_eventgroup, BUF_ANY_EMPTY, true, false, -1);
    int index = 0;
    for(int x = BUF_0_EMPTY; x <= BUF_1_EMPTY; x <<= 1) {
        if((bits & x) != 0) {
            memcpy(mosi_buffer[index], msg, SPI_DATA_SIZE);
            xEventGroupSetBits(spi_eventgroup, (BUF_0_FULL << index));
            break;
        }
    }
}

//////////////////////////////////////////////////////////////////////

bool spi_send_now(message_t const *msg, message_t *reply)
{
    hspi_transaction(reinterpret_cast<uint32_t const *>(msg), reinterpret_cast<uint32_t *>(reply));
    return check_crc32(reply);
}
