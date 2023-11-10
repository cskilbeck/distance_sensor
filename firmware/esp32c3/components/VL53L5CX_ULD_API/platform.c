//////////////////////////////////////////////////////////////////////

#include "driver/i2c.h"
#include "hal/i2c_hal.h"
#include "platform.h"
#include "esp_log.h"
#include "util.h"

LOG_CONTEXT("platform");

//////////////////////////////////////////////////////////////////////

#define DEVICE I2C_MASTER_NUM
#define TIMEOUT (pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS))
#define I2C_ADDR (VL53L5CX_DEFAULT_I2C_ADDRESS >> 1)

//////////////////////////////////////////////////////////////////////

static uint8_t const *byte_swap(uint8_t data[2], uint16_t x)
{
    data[0] = x >> 8;
    data[1] = x & 0xff;
    return data;
}

//////////////////////////////////////////////////////////////////////

uint8_t RdByte(VL53L5CX_Platform *p_platform, uint16_t reg_addr, uint8_t *p_value)
{
    uint8_t addr[2];
    return i2c_master_write_read_device(DEVICE, I2C_ADDR, byte_swap(addr, reg_addr), 2, p_value, 1, TIMEOUT);
}

//////////////////////////////////////////////////////////////////////

uint8_t RdMulti(VL53L5CX_Platform *p_platform, uint16_t reg_addr, uint8_t *p_values, uint32_t size)
{
    uint8_t addr[2];
    return i2c_master_write_read_device(DEVICE, I2C_ADDR, byte_swap(addr, reg_addr), 2, p_values, size, TIMEOUT);
}

//////////////////////////////////////////////////////////////////////

uint8_t WrByte(VL53L5CX_Platform *p_platform, uint16_t reg_addr, uint8_t value)
{
    uint8_t addr[2];
    uint8_t data[1] = { value };
    return i2c_master_write_write_device(DEVICE, I2C_ADDR, byte_swap(addr, reg_addr), 2, data, 1, TIMEOUT);
}

//////////////////////////////////////////////////////////////////////

uint8_t WrMulti(VL53L5CX_Platform *p_platform, uint16_t reg_addr, uint8_t *p_values, uint32_t size)
{
    uint8_t addr[2];
    return i2c_master_write_write_device(DEVICE, I2C_ADDR, byte_swap(addr, reg_addr), 2, p_values, size, TIMEOUT);
}

//////////////////////////////////////////////////////////////////////
// (Optional) Needs to be implemented by customer. Returns 0 if OK

uint8_t Reset_Sensor(VL53L5CX_Platform *p_platform)
{
    // Set LPN, AVDD, VDDIO = LOW
    WaitMs(p_platform, 100);

    // Set LPN, AVDD, VDDIO = HIGH
    WaitMs(p_platform, 100);

    return 0;
}

//////////////////////////////////////////////////////////////////////

void SwapBuffer(uint8_t *buffer, uint16_t size)
{
    for(uint32_t i = 0; i < size; i = i + 4) {
        uint32_t tmp = (buffer[i] << 24) | (buffer[i + 1] << 16) | (buffer[i + 2] << 8) | buffer[i + 3];
        memcpy(&(buffer[i]), &tmp, 4);
    }
}

//////////////////////////////////////////////////////////////////////

uint8_t WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs)
{
    vTaskDelay(TimeMs / portTICK_PERIOD_MS);
    return 0;
}
