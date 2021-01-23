#include <esp_types.h>

#ifndef DESK_STATION_TEMP_SENSOR_H
#define DESK_STATION_TEMP_SENSOR_H

#include "stdint.h"

typedef struct {
    double temperature;
} ds_temp_sensor_data_t;

_Noreturn
void sensor_task();

#endif //DESK_STATION__TEMP_SENSOR_H