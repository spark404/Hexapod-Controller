/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "attitude_ekf.h"
#include <string.h>

void attitude_ekf_init(attitude_ekf_t *ekf)
{
    memset(ekf, 0, sizeof(*ekf));

    // Initial covariance
    for (uint32_t i = 0; i < EKF_STATE_DIM; i++) {
        ekf->P[i][i] = 0.1f;
    }

    // Process noise
    const float32_t angle_q = 1e-4f;
    const float32_t bias_q  = 1e-6f;

    for (uint32_t i = 0; i < 3; i++)
        ekf->Q[i][i] = angle_q;

    for (uint32_t i = 3; i < 6; i++)
        ekf->Q[i][i] = bias_q;

    // Measurement noise
    ekf->R[0][0] = 0.05f;  // roll
    ekf->R[1][1] = 0.05f;  // pitch
    ekf->R[2][2] = 0.2f;   // yaw

    ekf->initialized = true;
}

void attitude_ekf_predict(
    attitude_ekf_t *ekf,
    const float32_t gyro[3],
    float32_t dt
) {
    const float32_t wx = gyro[0] - ekf->x[3];
    const float32_t wy = gyro[1] - ekf->x[4];
    const float32_t wz = gyro[2] - ekf->x[5];

    // State prediction
    ekf->x[0] += wx * dt;
    ekf->x[1] += wy * dt;
    ekf->x[2] += wz * dt;

    // Covariance prediction (simplified)
    for (uint32_t i = 0; i < EKF_STATE_DIM; i++) {
        for (uint32_t j = 0; j < EKF_STATE_DIM; j++) {
            ekf->P[i][j] += ekf->Q[i][j];
        }
    }
}

static float32_t wrap_pi(float32_t x)
{
    while (x >  PI) x -= 2.0f * PI;
    while (x < -PI) x += 2.0f * PI;
    return x;
}

void attitude_ekf_update_accel(
    attitude_ekf_t *ekf,
    float32_t roll,
    float32_t pitch
) {
    float32_t y[2] = {
        roll  - ekf->x[0],
        pitch - ekf->x[1]
    };

    float32_t S[2] = {
        ekf->P[0][0] + ekf->R[0][0],
        ekf->P[1][1] + ekf->R[1][1]
    };

    // Kalman gain
    for (uint32_t i = 0; i < EKF_STATE_DIM; i++) {
        ekf->x[i] += (ekf->P[i][0] / S[0]) * y[0]
                   + (ekf->P[i][1] / S[1]) * y[1];
    }

    // Covariance update (diagonal)
    ekf->P[0][0] *= (1.0f - ekf->P[0][0] / S[0]);
    ekf->P[1][1] *= (1.0f - ekf->P[1][1] / S[1]);
}

void attitude_ekf_update_mag(
    attitude_ekf_t *ekf,
    float32_t yaw
) {
    float32_t y = wrap_pi(yaw - ekf->x[2]);
    float32_t S = ekf->P[2][2] + ekf->R[2][2];

    for (uint32_t i = 0; i < EKF_STATE_DIM; i++) {
        ekf->x[i] += (ekf->P[i][2] / S) * y;
    }

    ekf->P[2][2] *= (1.0f - ekf->P[2][2] / S);
}
