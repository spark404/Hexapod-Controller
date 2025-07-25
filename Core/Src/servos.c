//
// Created by Hugo Trippaers on 24/07/2025.
//
#include "arm_math.h"

#include "dynamixel/dynamixel.h"
#include "hexapodmath/additional_functions.h"

#include "log.h"

#include "servos.h"


#define RAD_PER_PULSE (float)(2 * M_PI / 4096)

static uint32_t angle_to_pulse(float32_t angle) {
    // Due to mounting 0 rad is actually PI rad
    float actual_angle = angle + (float) M_PI;
    return (uint32_t) roundf(actual_angle / RAD_PER_PULSE);
}

static float32_t pulse_to_angle(uint32_t pulse) {
    return (float) pulse * RAD_PER_PULSE - (float) M_PI;
}

void read_actual_servo_position(dynamixel_servo_t *servos, const uint8_t servo_count, float32_t *actual_servo_angles) {
    // Read actual position
    uint32_t actual_position[3];
    dynamixel_result_t res = dynamixel_sync_get_long_parameter(servos, XL430_CT_RAM_PRESENT_POSITION,
                                                               actual_position, servo_count);
    if (res != DNM_OK) {
        LOG_ERROR("Failed to get long position using sync read: %d\r\n", res);
    }

    float actual_angle[3];
    for (int i = 0; i < 3; i++) {
        actual_angle[i] = pulse_to_angle(actual_position[i]);
    }

    // Compensate for geometry
    actual_angle[1] = -actual_angle[1];
    actual_angle[2] += D2R(25);

    arm_vec_copy_f32(actual_angle, actual_servo_angles, 3);
}

void write_next_servo_position(dynamixel_servo_t *servos, uint8_t servo_count, const float32_t *next_servo_angles) {
    float32_t angles_next[3];

    arm_vec_copy_f32(next_servo_angles, angles_next, 3);
    angles_next[1] = -angles_next[1];
    angles_next[2] -= D2R(25);

    uint32_t position_next[] = {
        angle_to_pulse(angles_next[0]), angle_to_pulse(angles_next[1]), angle_to_pulse(angles_next[2])
    };
    dynamixel_sync_set_long_parameter(servos, XL430_CT_RAM_GOAL_POSITION, position_next, servo_count);
}
