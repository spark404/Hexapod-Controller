/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "arm_math.h"

#include "dynamixel/dynamixel.h"
#include "hexapodmath/additional_functions.h"

#include "log.h"

#define RAD_PER_PULSE (float)(2 * M_PI / 4096)

static uint32_t angle_to_pulse(float32_t angle) {
    // Due to mounting 0 rad is actually PI rad
    float actual_angle = angle + (float) M_PI;
    return (uint32_t) roundf(actual_angle / RAD_PER_PULSE);
}

static float32_t pulse_to_angle(uint32_t pulse) {
    return (float) pulse * RAD_PER_PULSE - (float) M_PI;
}

void compensate_geometry_from_servo(const float32_t src[3], float32_t compensated[3]) {
    compensated[0] = src[0];
    compensated[1] = -src[1];
    compensated[2] = src[2] + (float32_t)D2R(25);
}

void compensate_geometry_to_servo(const float32_t src[3], float32_t compensated[3]) {
    compensated[0] = src[0];
    compensated[1] = -src[1];
    compensated[2] = src[2] - (float32_t)D2R(25);
}

int read_actual_servo_position(dynamixel_servo_t *servos, const uint8_t servo_count, float32_t *actual_servo_angles) {
    // Read actual position
    uint32_t actual_position[servo_count];
    const dynamixel_result_t res = dynamixel_get_long_parameter_multiple(servos, servo_count,
        XL430_CT_RAM_PRESENT_POSITION, actual_position);
    if (res != DNM_OK) {
        // LOG_ERROR("Failed to get long position using sync read: %d", res);
        return -1;
    }

    float actual_angle[servo_count];
    for (int i = 0; i < servo_count; i++) {
        actual_angle[i] = pulse_to_angle(actual_position[i]);
    }

    arm_vec_copy_f32(actual_angle, actual_servo_angles, servo_count);

    return 0;
}

int write_next_servo_position(dynamixel_servo_t *servos, uint8_t servo_count, const float32_t *next_servo_angles) {
    uint32_t position_next[servo_count];
    for (int i = 0; i < servo_count; i++) {
        position_next[i] = angle_to_pulse(next_servo_angles[i]);
    }
    const dynamixel_result_t res = dynamixel_set_long_parameter_multiple(servos, servo_count, XL430_CT_RAM_GOAL_POSITION, position_next);

    if (res != DNM_OK) {
        LOG_ERROR("Failed to write long position using sync write: %d", res);
        return -1;
    }

    return 0;
}
