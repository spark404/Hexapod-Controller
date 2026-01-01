/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SERVOS_H
#define SERVOS_H

#include "arm_math.h"
#include "dynamixel/dynamixel.h"

int read_actual_servo_position(dynamixel_servo_t *servos, const uint8_t servo_count, float32_t *actual_servo_angles);
int write_next_servo_position(dynamixel_servo_t *servos, const uint8_t servo_count, const float32_t *next_servo_angles);

#endif //SERVOS_H
