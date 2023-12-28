#pragma once

#include "vl53l5cx_api.h"

//////////////////////////////////////////////////////////////////////

#define DISTANCE_SUCCESS 0
#define DISTANCE_ERR_I2C_DRIVER_INIT -1
#define DISTANCE_ERR_SENSOR_MISSING -2
#define DISTANCE_ERR_INIT -3
#define DISTANCE_ERR_SET_FREQ -4
#define DISTANCE_ERR_SET_MODE -5
#define DISTANCE_ERR_SET_RESOLUTION -6
#define DISTANCE_ERR_START_RANGING -7
#define DISTANCE_ERR_DATA_READY -8
#define DISTANCE_ERR_TIMEOUT -9
#define DISTANCE_ERR_GET_DATA -10
#define DISTANCE_ERR_BAD_DISTANCE -11
#define DISTANCE_ERR_I2C_PARAMS -12

//////////////////////////////////////////////////////////////////////

int get_distance(int16_t *distance, VL53L5CX_ResultsData **results);
