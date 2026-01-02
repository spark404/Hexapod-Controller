/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "attitude_measurement.h"

#include "arm_math.h"

/**
 * Compute roll and pitch from accelerometer
 *
 * acc: accelerometer vector (body frame)
 *      does NOT need to be normalized
 *
 * roll  = atan2( ay, az )
 * pitch = atan2( -ax, sqrt(ay^2 + az^2) )
 *
 * Output angles are in radians
 */
void compute_roll_pitch(
    const float32_t acc[3],
    float32_t *roll,
    float32_t *pitch
) {
    float32_t denom;

    // sqrt(ay^2 + az^2)
    arm_sqrt_f32(acc[1] * acc[1] + acc[2] * acc[2], &denom);

    // Roll (rotation around X axis)
    *roll = atan2f(acc[1], acc[2]);

    // Pitch (rotation around Y axis)
    *pitch = atan2f(-acc[0], denom);
}

/**
 * Compute tilt-compensated yaw from accelerometer and magnetometer
 *
 * acc: accelerometer [m/s^2] or normalized (body frame)
 * mag: magnetometer [uT] or normalized (body frame)
 *
 * returns yaw [rad], range [-pi, pi]
 */
float32_t compute_yaw_from_mag(
    const float32_t acc[3],
    const float32_t mag[3]
) {
    // 1. Roll & pitch from accelerometer
    const float32_t roll  = atan2f(acc[1], acc[2]);

    float32_t tmp;
    arm_sqrt_f32(acc[1]*acc[1] + acc[2]*acc[2], &tmp);

    const float32_t pitch = atan2f(
        -acc[0],
        tmp
    );

    // 2. Precompute trig
    const float32_t cr = arm_cos_f32(roll);
    const float32_t sr = arm_sin_f32(roll);
    const float32_t cp = arm_cos_f32(pitch);
    const float32_t sp = arm_sin_f32(pitch);

    // 3. Tilt compensation
    const float32_t mx =
        mag[0]*cp +
        mag[1]*sr*sp +
        mag[2]*cr*sp;

    const float32_t my =
        mag[1]*cr -
        mag[2]*sr;

    // 4. Heading (yaw)
    return atan2f(-my, mx);
}
