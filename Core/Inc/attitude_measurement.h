/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ATTITUDE_MEASUREMENT_H
#define ATTITUDE_MEASUREMENT_H

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

void compute_roll_pitch(
    const float32_t acc[3],
    float32_t *roll,
    float32_t *pitch
);

float32_t compute_yaw_from_mag(
    const float32_t acc[3],
    const float32_t mag[3]
);

#ifdef __cplusplus
}
#endif

#endif //ATTITUDE_MEASUREMENT_H
