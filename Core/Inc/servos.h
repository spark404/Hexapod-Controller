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
} servo_shared_state_t;

typedef enum {
    SERVO_INIT,
    SERVO_SYNC_FROM_HW,
    SERVO_SYNC_TO_HW,
    SERVO_RUNNING,
    SERVO_ERROR,
    SERVO_POWER_DOWN,
    SERVO_IDLE,
    SERVO_POWER_UP
} servo_state_t;

void compensate_geometry_from_servo(const float32_t src[3], float32_t compensated[3]);
void compensate_geometry_to_servo(const float32_t src[3], float32_t compensated[3]);

int read_actual_servo_position(dynamixel_servo_t *servos, uint8_t servo_count, float32_t *actual_servo_angles);
int write_next_servo_position(dynamixel_servo_t *servos, uint8_t servo_count, const float32_t *next_servo_angles);

#endif //SERVOS_H
