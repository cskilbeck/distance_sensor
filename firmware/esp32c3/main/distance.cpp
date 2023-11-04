//////////////////////////////////////////////////////////////////////

#include <time.h>
#include <sys/time.h>

#include "sdkconfig.h"

#include "soc/soc_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "platform.h"
#include "vl53l5cx_api.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "util.h"
#include "distance.h"

LOG_TAG("distance");

//////////////////////////////////////////////////////////////////////

namespace
{
    VL53L5CX_Configuration vl53;

    VL53L5CX_ResultsData vl53_results;
}

//////////////////////////////////////////////////////////////////////

int get_distance(int16_t *distance)
{
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if(err != ESP_OK) {
        return DISTANCE_ERR_I2C_PARAMS;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if(err != ESP_OK) {
        return DISTANCE_ERR_I2C_DRIVER_INIT;
    }

    DEFER(i2c_driver_delete(I2C_MASTER_NUM));

    vl53.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;

    uint8_t is_alive = 0;

    uint8_t status = vl53l5cx_is_alive(&vl53, &is_alive);
    if(status != 0 || is_alive == 0) {
        LOG_E("IS_ALIVE = %d", is_alive);
        return DISTANCE_ERR_SENSOR_MISSING;
    }

    if(vl53l5cx_init(&vl53) != 0) {
        return DISTANCE_ERR_INIT;
    }

    if(vl53l5cx_set_ranging_frequency_hz(&vl53, 10) != 0) {
        return DISTANCE_ERR_SET_FREQ;
    }

    if(vl53l5cx_set_ranging_mode(&vl53, VL53L5CX_RANGING_MODE_CONTINUOUS) != 0) {
        return DISTANCE_ERR_SET_MODE;
    }

    if(vl53l5cx_set_resolution(&vl53, VL53L5CX_RESOLUTION_8X8) != 0) {
        return DISTANCE_ERR_SET_RESOLUTION;
    }

    if(vl53l5cx_start_ranging(&vl53) != 0) {
        return DISTANCE_ERR_START_RANGING;
    }

    // try and get a decent reading this many times

    int const MAX_DISTANCE_TRIES = 5;

    for(int i = 0; i < MAX_DISTANCE_TRIES; ++i) {

        // wait up to 200ms for a reading to become available

        uint8_t data_ready;

        for(int t = 0; t < 20; ++t) {

            // distance reading available?

            if(vl53l5cx_check_data_ready(&vl53, &data_ready) != 0) {
                return DISTANCE_ERR_DATA_READY;
            }

            // if so

            if(data_ready) {

                // get it

                if(vl53l5cx_get_ranging_data(&vl53, &vl53_results) != 0) {
                    return DISTANCE_ERR_GET_DATA;
                }

                LOG_I("Distance sensor temp %d", vl53_results.silicon_temp_degc);

                int dist4[4] MAYBE_UNUSED;
                int di = 0;

                int total = 0;
                int imax = 0;
                int imin = 0x7fff;
                for(int y = 3; y <= 4; ++y) {
                    for(int x = 3; x <= 4; ++x) {
                        int r = vl53_results.distance_mm[x + y * 8];
                        dist4[di++] = r;
                        imin = min(imin, r);
                        imax = max(imax, r);
                        total += r;
                    }
                }

                int16_t dist = static_cast<int16_t>((total - (imin + imax)) / 2);

                LOG_V("% 4d % 4d % 4d % 4d", dist4[0], dist4[1], dist4[2], dist4[3]);

                // if we got a good reading in some sensible range, return it

                if(dist > 0 && dist < 2000) {
                    *distance = dist;
                    return DISTANCE_SUCCESS;
                }

                // we got a reading but not in range, try again

                break;
            }

            // no data, wait for 10ms
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // distance data not available after 200ms, tough luck

        if(!data_ready) {
            return DISTANCE_ERR_TIMEOUT;
        }
    }

    // we got a reading but it's out of range, tough luck

    return DISTANCE_ERR_BAD_DISTANCE;
}
