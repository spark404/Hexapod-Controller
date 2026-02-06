/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "servos.h"

#include "arm_math.h"
#include "dynamixel/dynamixel.h"
#include "log.h"

#define XL430_RESOLUTION 4096.0f
#define FEMUR_OFFSET_ANGLE_RAD 0.436332313f
#define TWO_PI (2.0f * (float)M_PI)

float32_t xl430_pulse_to_rad_centered(const uint16_t pulse)
{
    const int32_t signed_pulse = (int32_t)pulse - 2048;
    return ((float32_t)signed_pulse * TWO_PI) / XL430_RESOLUTION;
}

uint16_t xl430_rad_centered_to_pulse(const float32_t rad)
{
    int32_t pulse = (int32_t)((rad * XL430_RESOLUTION) / TWO_PI) + 2048;

    /* clamp to valid range */
    if (pulse < 0) pulse = 0;
    if (pulse > 4095) pulse = 4095;

    return (uint16_t)pulse;
}

void compensate(const float32_t src[3], float32_t dst[3]) {
    dst[0] = src[0];
    dst[1] = -1 * src[1] ;
    dst[2] = src[2] + FEMUR_OFFSET_ANGLE_RAD;
}

void uncompensate(const float32_t src[3], float32_t dst[3]) {
    dst[0] = src[0];
    dst[1] = -1 * src[1];
    dst[2] = src[2] - FEMUR_OFFSET_ANGLE_RAD;
}