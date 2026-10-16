//
// Host test for the servo profile units and packing, see Core/Src/servo_profile.c
//
// Build and run from the Hexapod-Controller directory:
//
//   DSP=cmake-build-debug/_deps/cmsis-dsp-src/Include
//   cc -std=c99 -Wall -Wextra -pedantic -Werror -ICore/Inc -I$DSP -IDrivers/CMSIS/Include \
//      tests/servo_profile_test.c Core/Src/servo_profile.c -lm -o /tmp/servo_profile_test && /tmp/servo_profile_test
//
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "servo_profile.h"

static void test_units(void)
{
    /* 1.875 rad/s / 0.02398 = 78.2, 93.75 rad/s^2 / 0.3745 = 250.3 */
    assert(xl430_radps_to_profile_velocity(1.875f) == 78);
    assert(xl430_radps2_to_profile_acceleration(93.75f) == 250);

    /* one unit back and forth: 0.229 rev/min and 214.577 rev/min^2 */
    assert(xl430_radps_to_profile_velocity(0.229f * 2.0f * (float) M_PI / 60.0f) == 1);
    assert(xl430_radps2_to_profile_acceleration(214.577f * 2.0f * (float) M_PI / 3600.0f) == 1);
}

static void test_never_zero(void)
{
    /* 0 means "no limit" to the servo: tiny, zero, negative and NaN all give 1 */
    assert(xl430_radps_to_profile_velocity(0.001f) == 1);
    assert(xl430_radps_to_profile_velocity(0.0f) == 1);
    assert(xl430_radps_to_profile_velocity(-1.0f) == 1);
    assert(xl430_radps_to_profile_velocity(NAN) == 1);
    assert(xl430_radps2_to_profile_acceleration(0.0f) == 1);
    assert(xl430_radps2_to_profile_acceleration(NAN) == 1);
}

static void test_clamp(void)
{
    const uint32_t v_max = xl430_radps_to_profile_velocity(SERVO_PROFILE_MAX_VELOCITY_RADPS);
    const uint32_t a_max = xl430_radps2_to_profile_acceleration(SERVO_PROFILE_MAX_ACCELERATION_RADPS2);
    assert(v_max == 167);   /* 4.0 rad/s */
    assert(a_max == 401);   /* 150 rad/s^2 */
    assert(xl430_radps_to_profile_velocity(50.0f) == v_max);
    assert(xl430_radps_to_profile_velocity(INFINITY) == v_max);
    assert(xl430_radps2_to_profile_acceleration(1e6f) == a_max);
}

static void test_compute(void)
{
    uint32_t v, a;

    /* The femur lift at 100 mm/s: 0.15 rad in one 100 ms tick, ramps of 20 ms */
    servo_profile_compute(0.15f, 0.1f, 0.02f, &v, &a);
    assert(v == 78);
    assert(a == 250);

    /* The direction does not matter, the goal gives it */
    uint32_t v2, a2;
    servo_profile_compute(-0.15f, 0.1f, 0.02f, &v2, &a2);
    assert(v2 == v && a2 == a);

    /* A tiny move still has a profile */
    servo_profile_compute(0.002f, 0.1f, 0.025f, &v, &a);
    assert(v == 1);
    assert(a >= 1);

    /* No move: the slowest profile, not a jump */
    servo_profile_compute(0.0f, 0.1f, 0.025f, &v, &a);
    assert(v == 1 && a == 1);

    /* Too fast: velocity clamped, the ramp time kept */
    servo_profile_compute(1.0f, 0.1f, 0.025f, &v, &a);
    assert(v == 167);
    assert(a == xl430_radps2_to_profile_acceleration(SERVO_PROFILE_MAX_VELOCITY_RADPS / 0.025f));

    /* A ramp longer than half the time is cut to half: a triangle */
    uint32_t v_tri, a_tri;
    servo_profile_compute(0.1f, 0.1f, 0.08f, &v, &a);
    servo_profile_compute(0.1f, 0.1f, 0.05f, &v_tri, &a_tri);
    assert(v == v_tri && a == a_tri);

    /* No time to plan in: the fastest profile allowed */
    servo_profile_compute(0.1f, 0.0f, 0.02f, &v, &a);
    assert(v == 167 && a == 401);
    servo_profile_compute(0.1f, 0.1f, 0.0f, &v, &a);
    assert(v == 167 && a == 401);
}

static void test_pack(void)
{
    const uint32_t acc[2] = {0x01020304U, 250};
    const uint32_t vel[2] = {0x0A0B0C0DU, 78};
    const uint32_t goal[2] = {2048, 4095};
    uint8_t out[2 * SERVO_PROFILE_BYTES];
    memset(out, 0xEE, sizeof(out));

    servo_profile_pack(acc, vel, goal, 2, out);

    const uint8_t expected[2 * SERVO_PROFILE_BYTES] = {
        0x04, 0x03, 0x02, 0x01,  0x0D, 0x0C, 0x0B, 0x0A,  0x00, 0x08, 0x00, 0x00,
        0xFA, 0x00, 0x00, 0x00,  0x4E, 0x00, 0x00, 0x00,  0xFF, 0x0F, 0x00, 0x00,
    };
    assert(memcmp(out, expected, sizeof(expected)) == 0);
}

int main(void)
{
    test_units();
    test_never_zero();
    test_clamp();
    test_compute();
    test_pack();
    printf("servo_profile_test: all tests passed\n");
    return 0;
}
