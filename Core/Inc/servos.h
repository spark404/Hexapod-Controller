/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SERVOS_H
#define SERVOS_H

#include <stdbool.h>

#include "arm_math.h"
#include "dynamixel/dynamixel.h"

typedef struct {
    float32_t actual_joint_angles[6][3];
    float32_t target_joint_angles[6][3];
    bool initialized;
    bool request_powerdown;
    bool limit_alert_enabled;
} servo_shared_state_t;

typedef enum {
    SERVO_INIT,
    SERVO_SYNC_FROM_HW,
    SERVO_SYNC_TO_HW,
    SERVO_RUNNING,
    SERVO_ERROR,
    SERVO_POWER_DOWN,
    SERVO_IDLE,
    SERVO_POWER_UP,
} servo_state_t;

float32_t xl430_pulse_to_rad_centered(const uint16_t pulse);
uint16_t xl430_rad_centered_to_pulse(const float32_t rad);

void compensate(const float src[3], float dst[3]);
void uncompensate(const float src[3], float dst[3]);


#endif //SERVOS_H
