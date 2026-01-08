//
// Created by Hugo Trippaers on 03/01/2026.
//

#ifndef SENSORS_H
#define SENSORS_H

#include "FreeRTOS.h"
#include "arm_math.h"

#define GRAVITY (9.80665f)

typedef enum {
    SENSOR_GYRO,
    SENSOR_ACCEL,
    SENSOR_MAG
} sensor_type_t;

typedef struct {
    sensor_type_t type;
    TickType_t    tick;        // FreeRTOS timestamp
    float32_t     data[3];     // xyz
} sensor_sample_t;


#endif //SENSORS_H
