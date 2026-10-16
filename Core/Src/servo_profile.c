/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "servo_profile.h"

#define TWO_PI (2.0f * (float32_t)M_PI)

/* XL430 control table units, confirmed on the bench (T7.14, E1/E2) */
#define XL430_PROFILE_VELOCITY_UNIT_RADPS      (0.229f * TWO_PI / 60.0f)
#define XL430_PROFILE_ACCELERATION_UNIT_RADPS2 (214.577f * TWO_PI / 3600.0f)
#define XL430_PROFILE_MAX_RAW                  32767U

static uint32_t to_profile_units(const float32_t value, const float32_t limit, const float32_t unit)
{
    /* Also catches NaN: 0 would mean "no limit" to the servo */
    if (!(value > 0.0f)) {
        return 1U;
    }
    const float32_t raw = (value < limit ? value : limit) / unit + 0.5f;
    if (raw < 1.0f) {
        return 1U;
    }
    if (raw >= (float32_t) XL430_PROFILE_MAX_RAW) {
        return XL430_PROFILE_MAX_RAW;
    }
    return (uint32_t) raw;
}

uint32_t xl430_radps_to_profile_velocity(const float32_t radps)
{
    return to_profile_units(radps, SERVO_PROFILE_MAX_VELOCITY_RADPS, XL430_PROFILE_VELOCITY_UNIT_RADPS);
}

uint32_t xl430_radps2_to_profile_acceleration(const float32_t radps2)
{
    return to_profile_units(radps2, SERVO_PROFILE_MAX_ACCELERATION_RADPS2, XL430_PROFILE_ACCELERATION_UNIT_RADPS2);
}

void servo_profile_compute(const float32_t delta_rad, const float32_t time_s, float32_t ramp_s,
                           uint32_t *velocity, uint32_t *acceleration)
{
    if (!(time_s > 0.0f) || !(ramp_s > 0.0f)) {
        /* No time to plan in: the fastest profile allowed */
        *velocity = xl430_radps_to_profile_velocity(SERVO_PROFILE_MAX_VELOCITY_RADPS);
        *acceleration = xl430_radps2_to_profile_acceleration(SERVO_PROFILE_MAX_ACCELERATION_RADPS2);
        return;
    }
    if (ramp_s > 0.5f * time_s) {
        ramp_s = 0.5f * time_s;
    }

    float32_t v = fabsf(delta_rad) / (time_s - ramp_s);
    if (v > SERVO_PROFILE_MAX_VELOCITY_RADPS) {
        /* Keep the ramp time: the move takes longer than time_s */
        v = SERVO_PROFILE_MAX_VELOCITY_RADPS;
    }
    *velocity = xl430_radps_to_profile_velocity(v);
    *acceleration = xl430_radps2_to_profile_acceleration(v / ramp_s);
}

static void put_u32(uint8_t *p, const uint32_t value)
{
    p[0] = (uint8_t) value;
    p[1] = (uint8_t) (value >> 8);
    p[2] = (uint8_t) (value >> 16);
    p[3] = (uint8_t) (value >> 24);
}

void servo_profile_pack(const uint32_t *acceleration, const uint32_t *velocity, const uint32_t *goal,
                        const size_t count, uint8_t *out)
{
    for (size_t s = 0; s < count; s++) {
        uint8_t *p = &out[s * SERVO_PROFILE_BYTES];
        put_u32(&p[0], acceleration[s]);
        put_u32(&p[4], velocity[s]);
        put_u32(&p[8], goal[s]);
    }
}
