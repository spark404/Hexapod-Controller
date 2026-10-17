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
    float32_t target_duration_s;       /* time the controller meant the targets to take */
    uint32_t target_seq;               /* incremented with every new set of targets */
    /* As reported by the servos, in the servo frame: the signs are NOT the
     * compensated joint frame of the angles above (see compensate()). */
    float32_t load_pct[6][3];          /* Present Load, % of the maximum torque, signed */
    float32_t velocity_radps[6][3];    /* Present Velocity, rad/s, signed */
    /* The servo's own profile: how far the position trails it, and its velocity */
    float32_t tracking_error_rad[6][3];       /* Position Trajectory - Present Position, rad */
    float32_t profile_velocity_radps[6][3];   /* Velocity Trajectory, rad/s, signed */
    uint16_t input_voltage_dv[6][3];          /* Present Input Voltage, 0.1 V, 0 = not read yet */
    uint8_t temperature_c[6][3];              /* Present Temperature, degrees C */
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
float32_t xl430_load_to_percent(const int16_t raw);
float32_t xl430_velocity_to_radps(const int32_t raw);
uint16_t xl430_rad_centered_to_pulse(const float32_t rad);

void compensate(const float32_t src[3], float32_t dst[3]);
void uncompensate(const float32_t src[3], float32_t dst[3]);
void servo_copy_actual_joint_angles(float32_t dst[6][3], const servo_shared_state_t *state);
void servo_copy_target_command(float32_t dst[6][3], float32_t *duration_s, uint32_t *seq,
                               const servo_shared_state_t *state);
void servo_set_target_joint_angles(servo_shared_state_t *state, float32_t src[6][3], float32_t duration_s);
void servo_set_actual_joint_angles(servo_shared_state_t *state, float32_t src[6][3]);
void servo_set_actual_and_target_joint_angles(servo_shared_state_t *state, float32_t src[6][3]);
void servo_copy_load(float32_t dst[6][3], const servo_shared_state_t *state);
void servo_copy_velocity(float32_t dst[6][3], const servo_shared_state_t *state);
void servo_set_load_and_velocity(servo_shared_state_t *state, float32_t load[6][3], float32_t velocity[6][3]);
void servo_copy_profile_tracking(float32_t error[6][3], float32_t velocity[6][3], const servo_shared_state_t *state);
void servo_set_profile_tracking(servo_shared_state_t *state, float32_t error[6][3], float32_t velocity[6][3]);
void servo_copy_health(uint16_t voltage_dv[6][3], uint8_t temperature_c[6][3], const servo_shared_state_t *state);
void servo_set_health(servo_shared_state_t *state, uint16_t voltage_dv[6][3], uint8_t temperature_c[6][3]);
void servo_get_flags(const servo_shared_state_t *state, bool *request_powerdown, bool *limit_alert_enabled);
void servo_set_request_powerdown(servo_shared_state_t *state, bool request_powerdown);
void servo_set_limit_alert_enabled(servo_shared_state_t *state, bool limit_alert_enabled);


#endif //SERVOS_H
