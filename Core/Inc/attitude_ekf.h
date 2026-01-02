/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"   // CMSIS-DSP

#ifdef __cplusplus
extern "C" {
#endif

#define EKF_STATE_DIM 6
#define EKF_MEAS_DIM  3

typedef struct {
    // State: [roll, pitch, yaw, bgx, bgy, bgz]
    float32_t x[EKF_STATE_DIM];

    // Covariance
    float32_t P[EKF_STATE_DIM][EKF_STATE_DIM];

    // Process noise
    float32_t Q[EKF_STATE_DIM][EKF_STATE_DIM];

    // Measurement noise
    float32_t R[EKF_MEAS_DIM][EKF_MEAS_DIM];

    bool initialized;
} attitude_ekf_t;

void attitude_ekf_init(attitude_ekf_t *ekf);

void attitude_ekf_predict(
    attitude_ekf_t *ekf,
    const float32_t gyro[3],
    float32_t dt
);

void attitude_ekf_update_accel(
    attitude_ekf_t *ekf,
    float32_t roll,
    float32_t pitch
);

void attitude_ekf_update_mag(
    attitude_ekf_t *ekf,
    float32_t yaw
);

#ifdef __cplusplus
}
#endif

#endif //ATTITUDE_EKF_H
