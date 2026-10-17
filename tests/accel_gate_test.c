//
// Host test for the accelerometer sanity check, see Core/Src/accel_gate.c
//
// The samples are the ones the robot printed in the [Sensors] lines of 2026-10-16 while it stood
// still (frame as used by the filter, m/s^2): 7 of 10 are wrong. They are fed through the real
// attitude filter, with and without the check.
//
// Build and run from the Hexapod-Controller directory:
//
//   DSP=cmake-build-debug/_deps/cmsis-dsp-src/Include
//   LIB=../HexapodSim/Controller/cmake-build-debug/_deps/cmsis-dsp-build/Source/libCMSISDSP.a   # a host build
//   cc -std=c99 -Wall -Wextra -pedantic -Werror -ICore/Inc -I$DSP -I$DSP/../PrivateInclude -IDrivers/CMSIS/Include \
//      tests/accel_gate_test.c Core/Src/accel_gate.c Core/Src/attitude_ekf.c Core/Src/attitude_measurement.c \
//      $LIB -lm -o /tmp/accel_gate_test && /tmp/accel_gate_test
//
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "accel_gate.h"
#include "attitude_ekf.h"
#include "attitude_measurement.h"

static const float SEEN[10][3] = {
    {-0.06f, 19.75f, 9.82f}, {-0.07f, 0.16f, 45.23f}, {-0.07f, 0.45f, 44.85f}, {-0.06f, 21.36f, 9.83f},
    {0.11f, 0.45f, 44.76f},  {0.10f, 24.28f, 9.82f},  {0.11f, 0.45f, 9.82f},   {0.11f, 0.07f, 9.83f},
    {0.11f, 23.14f, 24.54f}, {0.11f, 0.24f, 9.83f},
};

static void test_good_and_bad_samples(void)
{
    accel_gate_t g;
    accel_gate_init(&g, ACCEL_GATE_DEFAULT_TOLERANCE);

    for (int i = 0; i < 10; i++) {
        const bool good = i == 6 || i == 7 || i == 9;      /* the three that are gravity */
        const int16_t raw[3] = {(int16_t) i, (int16_t) (i * 2), (int16_t) (i * 3)};
        assert(accel_gate_check(&g, SEEN[i], raw) == good);
    }
    assert(g.accepted == 3 && g.rejected == 7);
    assert(g.last_rejected_raw[0] == 8 && g.last_rejected_raw[2] == 24);   /* the last bad one, sample 8 */
    assert(fabsf(g.last_rejected_norm - sqrtf(0.11f * 0.11f + 23.14f * 23.14f + 24.54f * 24.54f)) < 1e-3f);
}

static void test_edges_and_nonsense(void)
{
    accel_gate_t g;
    accel_gate_init(&g, 2.0f);
    const float on_edge[3] = {0.0f, 0.0f, ACCEL_GATE_GRAVITY + 2.0f};
    const float just_out[3] = {0.0f, 0.0f, ACCEL_GATE_GRAVITY + 2.1f};
    const float tilted[3] = {ACCEL_GATE_GRAVITY * 0.5f, 0.0f, ACCEL_GATE_GRAVITY * 0.8660254f};   /* 30 degrees, still 1 g */
    const float zero[3] = {0.0f, 0.0f, 0.0f};                                                       /* free fall or a dead sensor */
    const float nan_[3] = {NAN, 0.0f, ACCEL_GATE_GRAVITY};
    const float inf_[3] = {0.0f, INFINITY, 0.0f};

    assert(accel_gate_check(&g, on_edge, NULL));
    assert(!accel_gate_check(&g, just_out, NULL));
    assert(accel_gate_check(&g, tilted, NULL));                /* the length counts, not the direction */
    assert(!accel_gate_check(&g, zero, NULL));
    assert(!accel_gate_check(&g, nan_, NULL));
    assert(!accel_gate_check(&g, inf_, NULL));
    assert(g.accepted == 2 && g.rejected == 4);
}

/* The filter as the EKF task runs it: 100 Hz gyro predictions, 12.5 Hz accelerometer updates. The robot is still. */
static void run(bool gated, float *roll_mean, float *roll_max, float *pitch_max)
{
    attitude_ekf_t ekf;
    accel_gate_t g;
    attitude_ekf_init(&ekf);
    accel_gate_init(&g, ACCEL_GATE_DEFAULT_TOLERANCE);
    const float gyro[3] = {0.0f, 0.0f, 0.0f};

    float sum = 0.0f, rmax = 0.0f, pmax = 0.0f;
    unsigned n = 0;
    for (int k = 0; k < 12500 / 10; k++) {                     /* 100 s */
        for (int s = 0; s < 8; s++) {
            attitude_ekf_predict(&ekf, gyro, 0.01f);
        }
        const float *a = SEEN[(k * 7) % 10];                   /* the observed mix, 70 % wrong */
        if (!gated || accel_gate_check(&g, a, NULL)) {
            float roll, pitch;
            compute_roll_pitch(a, &roll, &pitch);
            attitude_ekf_update_accel(&ekf, roll, pitch);
        }
        if (k >= 100) {                                        /* after the start-up */
            sum += ekf.x[0];
            n++;
            rmax = fmaxf(rmax, fabsf(ekf.x[0]));
            pmax = fmaxf(pmax, fabsf(ekf.x[1]));
        }
    }
    *roll_mean = sum / (float) n;
    *roll_max = rmax;
    *pitch_max = pmax;
}

static void test_filter_with_and_without_the_check(void)
{
    float mean_off, max_off, pmax_off, mean_on, max_on, pmax_on;
    run(false, &mean_off, &max_off, &pmax_off);
    run(true, &mean_on, &max_on, &pmax_on);
    printf("  still robot, roll without the check: mean %.3f rad, peak %.3f; with it: mean %.3f, peak %.3f "
           "(pitch peak %.3f and %.3f)\n", mean_off, max_off, mean_on, max_on, pmax_off, pmax_on);

    assert(mean_off > 0.3f && max_off > 0.5f);                 /* what the robot logged: 0.34 to 0.75 */
    assert(fabsf(mean_on) < 0.06f && max_on < 0.1f);           /* the real tilt is about 0.03 */
    assert(pmax_on < 0.02f);
}

int main(void)
{
    test_good_and_bad_samples();
    test_edges_and_nonsense();
    test_filter_with_and_without_the_check();
    printf("accel_gate_test: all tests passed\n");
    return 0;
}
