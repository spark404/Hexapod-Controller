/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Motion profile of the XL430 servos (velocity-based, Drive Mode bit 2 = 0).
 *
 * Every goal is written together with its profile: Profile Acceleration (108),
 * Profile Velocity (112) and Goal Position (116) are adjacent, one 12 byte
 * Sync Write at address 108 per goal. The servo plans a trapezoid from where
 * its own trajectory is to the goal; a new goal during a move is planned from
 * the present trajectory velocity, so chained goals do not stop in between.
 *
 * No RTOS or HAL dependencies, so this file builds on the host
 * (tests/servo_profile_test.c).
 */

#ifndef SERVO_PROFILE_H
#define SERVO_PROFILE_H

#include <stddef.h>
#include <stdint.h>

#include "arm_math.h"

/* Bytes per servo in the profile and goal write: acceleration, velocity, goal */
#define SERVO_PROFILE_BYTES 12

/*
 * Upper limits of the profile. The Velocity Limit register does not cap the
 * profile in position mode (bench T7.14: a profile of 9.5 rad/s was accepted,
 * the fastest joint reached 5.9 rad/s), so the firmware does.
 */
#define SERVO_PROFILE_MAX_VELOCITY_RADPS      4.0f
#define SERVO_PROFILE_MAX_ACCELERATION_RADPS2 150.0f

/*
 * Lower limit of a computed acceleration. The servo also brakes with Profile
 * Acceleration: a tiny move after a fast one would otherwise give an
 * acceleration of almost nothing, and the joint would take seconds to stop
 * and run far past its goal (from 1.6 rad/s at 0.4 rad/s^2: 4 s, 3 rad).
 */
#define SERVO_PROFILE_MIN_ACCELERATION_RADPS2 40.0f

/*
 * Profile Velocity in 0.229 rev/min units, at least 1: 0 means "no limit" to
 * the servo, a jump to the goal. Clamped to SERVO_PROFILE_MAX_VELOCITY_RADPS.
 */
uint32_t xl430_radps_to_profile_velocity(float32_t radps);

/*
 * Profile Acceleration in 214.577 rev/min^2 units, at least 1 (0 means no
 * limit). Clamped to SERVO_PROFILE_MAX_ACCELERATION_RADPS2.
 */
uint32_t xl430_radps2_to_profile_acceleration(float32_t radps2);

/*
 * Profile that moves delta_rad in time_s with ramps of ramp_s at both ends:
 * v = |delta| / (time - ramp), a = v / ramp. A trapezoid with ramps of
 * ramp_s covers v * (time - ramp). ramp_s is limited to half of time_s.
 * a is at least SERVO_PROFILE_MIN_ACCELERATION_RADPS2, and the results are
 * clamped as by the conversions above.
 */
void servo_profile_compute(float32_t delta_rad, float32_t time_s, float32_t ramp_s,
                           uint32_t *velocity, uint32_t *acceleration);

/*
 * Pack count servos as SERVO_PROFILE_BYTES each, little endian: Profile
 * Acceleration, Profile Velocity, Goal Position. out holds
 * count * SERVO_PROFILE_BYTES bytes.
 */
void servo_profile_pack(const uint32_t *acceleration, const uint32_t *velocity, const uint32_t *goal,
                        size_t count, uint8_t *out);

#endif //SERVO_PROFILE_H
