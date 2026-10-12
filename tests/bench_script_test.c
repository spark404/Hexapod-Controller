//
// Host test for the bench experiment script, see Core/Src/bench_script.c
//
// Build and run from the Hexapod-Controller directory:
//
//   cc -std=c99 -Wall -Wextra -pedantic -Werror -ICore/Inc tests/bench_script_test.c \
//      Core/Src/bench_script.c -lm -o /tmp/bench_script_test && /tmp/bench_script_test
//
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bench_script.h"

static bench_run_t runs[BENCH_MAX_RUNS];

static size_t count_experiment(size_t n, unsigned experiment)
{
    size_t c = 0;
    for (size_t i = 0; i < n; i++) {
        if (runs[i].experiment == experiment) c++;
    }
    return c;
}

static void test_script_contents(void)
{
    const size_t n = bench_script_build(BENCH_ALL | BENCH_E0, runs, BENCH_MAX_RUNS);
    assert(n < BENCH_MAX_RUNS);
    assert(count_experiment(n, 0) == 1);
    assert(count_experiment(n, 1) == 4);
    assert(count_experiment(n, 2) == 12);
    assert(count_experiment(n, 3) == 36);
    assert(count_experiment(n, 4) == 42);
    assert(count_experiment(n, 5) == 5);
    assert(count_experiment(n, 6) == 4);

    // identifiers run 1..n, the drive mode groups are not mixed (group 0 first)
    unsigned group = 0;
    for (size_t i = 0; i < n; i++) {
        assert(runs[i].id == i + 1);
        const unsigned g = bench_run_group(&runs[i]);
        assert(g >= group);
        group = g;
    }
    // the abort test is the last of group 0 and is the only run of experiment 0
    size_t last_group0 = 0;
    for (size_t i = 0; i < n; i++) {
        if (bench_run_group(&runs[i]) == 0) last_group0 = i;
    }
    assert(runs[last_group0].experiment == 0);

    // only what was asked for
    assert(bench_script_build(BENCH_E1, runs, BENCH_MAX_RUNS) == 4);
    assert(bench_script_build(BENCH_E1 | BENCH_E0, runs, BENCH_MAX_RUNS) == 5);
    assert(bench_script_build(0, runs, BENCH_MAX_RUNS) == 0);
    assert(bench_script_build(BENCH_ALL, runs, 10) == 10);   // limited by the array
}

static void test_runs_are_consistent(void)
{
    const size_t n = bench_script_build(BENCH_ALL | BENCH_E0, runs, BENCH_MAX_RUNS);
    for (size_t i = 0; i < n; i++) {
        const bench_run_t *r = &runs[i];
        assert(r->joints != 0 && r->joints <= 7);
        assert(r->sample_ms >= 1);
        assert(r->duration_ms >= 400);
        if (r->mode == BENCH_MODE_TIME || r->mode == BENCH_MODE_VELOCITY_COMPUTED) {
            assert(r->time_ms > 0 && r->ramp_ms >= 1 && 2 * r->ramp_ms <= r->time_ms);
        }
        if (r->mode == BENCH_MODE_VELOCITY) {
            assert(r->velocity >= 1 && r->accel >= 1);
        }
        if (r->path == BENCH_PATH_RAMP || r->path == BENCH_PATH_SINE) {
            assert(r->period_ms > 0 && r->path_param > 0);
        }
        // a joint that does not move has no travel, one that moves has some
        for (unsigned j = 0; j < BENCH_JOINTS; j++) {
            if (r->joints & (1U << j)) assert(r->delta_mrad[j] != 0);
            else assert(r->delta_mrad[j] == 0);
        }
    }
}

// The guard of the experiment: no ideal path and no goal may leave +-BENCH_RANGE_MRAD
static void test_nothing_leaves_the_range(void)
{
    const size_t n = bench_script_build(BENCH_ALL | BENCH_E0, runs, BENCH_MAX_RUNS);
    const float limit = (float) BENCH_RANGE_MRAD / 1000.0f + 1e-5f;
    for (size_t i = 0; i < n; i++) {
        const bench_run_t *r = &runs[i];
        for (unsigned j = 0; j < BENCH_JOINTS; j++) {
            for (int32_t t = -BENCH_PRE_TRIGGER_MS; t <= (int32_t) r->duration_ms; t++) {
                assert(fabsf(bench_path_offset_rad(r, j, t)) <= limit);
            }
            for (unsigned k = 0; k < bench_goal_count(r); k++) {
                assert(fabsf(bench_goal_offset_rad(r, j, k)) <= limit);
                assert(bench_goal_time_ms(r, k) >= 0 && bench_goal_time_ms(r, k) <= (int32_t) r->duration_ms);
            }
        }
    }
}

// What a joint has to do for a run to be possible at all (the XL430 does about 6.35 rad/s at most,
// the default Velocity Limit register of 265 units). E2 and E3 deliberately ask for more in some runs.
#define SERVO_LIMIT_RADPS (265.0f * 0.229f * 2.0f * 3.14159265f / 60.0f)

static float nominal_move_seconds(const bench_run_t *r, float delta_rad)
{
    if (r->mode == BENCH_MODE_TIME || r->mode == BENCH_MODE_VELOCITY_COMPUTED) {
        return (float) r->time_ms / 1000.0f;
    }
    const float v = (float) r->velocity * 0.229f * 2.0f * 3.14159265f / 60.0f;
    const float a = (float) r->accel * 214.577f * 2.0f * 3.14159265f / 3600.0f;
    return fabsf(delta_rad) / v + v / a;
}

static void test_runs_are_possible_for_the_servo(void)
{
    const size_t n = bench_script_build(BENCH_ALL | BENCH_E0, runs, BENCH_MAX_RUNS);
    for (size_t i = 0; i < n; i++) {
        const bench_run_t *r = &runs[i];
        const float limit = 0.95f * SERVO_LIMIT_RADPS;

        for (unsigned j = 0; j < BENCH_JOINTS; j++) {
            if (!(r->joints & (1U << j))) continue;

            // speed the profile has to reach: travel of one goal over the cruise time of the profile
            if ((r->mode == BENCH_MODE_TIME || r->mode == BENCH_MODE_VELOCITY_COMPUTED) && r->experiment != 3) {
                const float cruise_s = (float) (r->time_ms - r->ramp_ms) / 1000.0f;
                float worst = 0.0f;
                for (unsigned k = 0; k < bench_goal_count(r); k++) {
                    const float from = k == 0 ? bench_path_offset_rad(r, j, -1)
                                              : bench_goal_offset_rad(r, j, k - 1);
                    worst = fmaxf(worst, fabsf(bench_goal_offset_rad(r, j, k) - from));
                }
                assert(worst / cruise_s <= limit);
            }
            if (r->mode == BENCH_MODE_VELOCITY && r->experiment != 2) {
                assert((float) r->velocity * 0.229f * 2.0f * 3.14159265f / 60.0f <= limit);
            }
        }

        // a stop or a reversal has to happen while the joint is still moving
        if (r->path == BENCH_PATH_STOP || r->path == BENCH_PATH_REVERSE) {
            const float move_s = nominal_move_seconds(r, (float) r->delta_mrad[2] / 1000.0f);
            assert((float) r->path_param / 1000.0f < 0.8f * move_s);
            assert((float) r->path_param / 1000.0f > 0.2f * move_s);
        }
    }
}

static bench_run_t make(bench_path_t path, int16_t tibia_mrad, uint16_t param, uint16_t period)
{
    bench_run_t r;
    memset(&r, 0, sizeof(r));
    r.path = path;
    r.joints = 4;
    r.delta_mrad[2] = tibia_mrad;
    r.path_param = param;
    r.period_ms = period;
    r.duration_ms = 1500;
    return r;
}

static void test_paths(void)
{
    // step: from -d/2 to +d/2, one goal at path time 0
    bench_run_t step = make(BENCH_PATH_STEP, 500, 0, 0);
    assert(fabsf(bench_path_offset_rad(&step, 2, -50) + 0.25f) < 1e-6f);   // parked at -d/2 before the goal
    assert(fabsf(bench_path_offset_rad(&step, 2, 0) - 0.25f) < 1e-6f);
    assert(bench_goal_count(&step) == 1);
    assert(bench_goal_time_ms(&step, 0) == 0);
    assert(fabsf(bench_goal_offset_rad(&step, 2, 0) - 0.25f) < 1e-6f);
    assert(bench_path_offset_rad(&step, 0, 100) == 0.0f);                   // joints that do not move

    // negative travel goes the other way
    bench_run_t neg = make(BENCH_PATH_STEP, -300, 0, 0);
    assert(fabsf(bench_goal_offset_rad(&neg, 2, 0) + 0.15f) < 1e-6f);

    // ramp: starts at -d/2, 0.6 rad/s, stops at +d/2 after d / rate = 1 s
    bench_run_t ramp = make(BENCH_PATH_RAMP, 600, 600, 40);
    assert(fabsf(bench_path_offset_rad(&ramp, 2, -30) + 0.3f) < 1e-6f);
    assert(fabsf(bench_path_offset_rad(&ramp, 2, 0) + 0.3f) < 1e-6f);
    assert(fabsf(bench_path_offset_rad(&ramp, 2, 500) - 0.0f) < 1e-5f);
    assert(fabsf(bench_path_offset_rad(&ramp, 2, 1000) - 0.3f) < 1e-5f);
    assert(fabsf(bench_path_offset_rad(&ramp, 2, 1400) - 0.3f) < 1e-5f);   // stays
    assert(bench_goal_count(&ramp) == 1500 / 40);
    assert(bench_goal_time_ms(&ramp, 3) == 120);
    // a goal commands where the joint should be one period later
    assert(fabsf(bench_goal_offset_rad(&ramp, 2, 3) - bench_path_offset_rad(&ramp, 2, 160)) < 1e-6f);

    // sine: 1 Hz, amplitude d/2, zero at 0, peak at a quarter period
    bench_run_t sine = make(BENCH_PATH_SINE, 500, 1000, 20);
    assert(fabsf(bench_path_offset_rad(&sine, 2, -20)) < 1e-6f);           // parked in the middle
    assert(fabsf(bench_path_offset_rad(&sine, 2, 0)) < 1e-6f);
    assert(fabsf(bench_path_offset_rad(&sine, 2, 250) - 0.25f) < 1e-5f);
    assert(fabsf(bench_path_offset_rad(&sine, 2, 750) + 0.25f) < 1e-5f);
    assert(fabsf(bench_path_offset_rad(&sine, 2, 1000)) < 1e-5f);

    // stop: the step, then the second goal is the present position (placeholder 0)
    bench_run_t stop = make(BENCH_PATH_STOP, 800, 150, 0);
    assert(bench_goal_count(&stop) == 2);
    assert(bench_goal_time_ms(&stop, 1) == 150);
    assert(fabsf(bench_goal_offset_rad(&stop, 2, 0) - 0.4f) < 1e-6f);

    assert(fabsf(bench_path_offset_rad(&stop, 2, -1) + 0.4f) < 1e-6f);

    // reversal: back to the start
    bench_run_t rev = make(BENCH_PATH_REVERSE, 800, 150, 0);
    assert(fabsf(bench_path_offset_rad(&rev, 2, -1) + 0.4f) < 1e-6f);
    assert(bench_goal_count(&rev) == 2);
    assert(fabsf(bench_goal_offset_rad(&rev, 2, 0) - 0.4f) < 1e-6f);
    assert(fabsf(bench_goal_offset_rad(&rev, 2, 1) + 0.4f) < 1e-6f);
}

static void test_units(void)
{
    // 0.229 rev/min = 0.02398 rad/s; 214.577 rev/min^2 = 0.37451 rad/s^2
    assert(bench_velocity_units(2.398f) == 100);
    assert(bench_velocity_units(-2.398f) == 100);
    assert(bench_velocity_units(0.0f) == 1);                 // 0 would mean "no limit"
    assert(bench_velocity_units(1e9f) == 32767);
    assert(bench_accel_units(37.451f) == 100);
    assert(bench_accel_units(0.0f) == 1);

    assert(bench_rad_to_pulses(2.0f * 3.14159265f) == 4096);
    assert(bench_rad_to_pulses(0.5f) == 326);
    assert(bench_rad_to_pulses(-0.5f) == -326);
    assert(fabsf(bench_pulses_to_rad(1024) - 3.14159265f / 2.0f) < 1e-5f);
}

// The corrected formula of the review: the move must take the time asked for, ramps included
static void test_computed_profile_arrives_in_time(void)
{
    static const struct { float delta; uint16_t t; uint16_t tr; } cases[] = {
        {0.1f, 80, 20}, {0.5f, 100, 25}, {0.02f, 40, 10}, {0.3f, 32, 8}, {0.05f, 16, 4},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        uint32_t v, a;
        bench_computed_profile(cases[i].delta, cases[i].t, cases[i].tr, 1023, &v, &a);
        const float v_radps = (float) v * 0.229f * 2.0f * 3.14159265f / 60.0f;
        const float a_radps2 = (float) a * 214.577f * 2.0f * 3.14159265f / 3600.0f;
        // symmetric trapezoid: time = distance / v + v / a
        const float t_s = cases[i].delta / v_radps + v_radps / a_radps2;
        const float t_wanted = (float) cases[i].t / 1000.0f;
        // the units are coarse for the small moves (a few percent)
        assert(fabsf(t_s - t_wanted) / t_wanted < 0.08f);
        // the phase 2 note's v = delta / T would have been far too slow for the time
        const float v_note = cases[i].delta / t_wanted;
        assert(v_radps > v_note);
    }

    // limited by the velocity limit register
    uint32_t v, a;
    bench_computed_profile(1.0f, 30, 7, 265, &v, &a);
    assert(v == 265);
    // nonsense ramps are repaired instead of dividing by zero
    bench_computed_profile(0.2f, 20, 0, 1023, &v, &a);
    assert(v >= 1 && a >= 1);
    bench_computed_profile(0.2f, 10, 20, 1023, &v, &a);
    assert(v >= 1 && a >= 1);
}

static void test_window(void)
{
    assert(bench_in_window(2048, 2048, 10));
    assert(bench_in_window(2038, 2048, 10));
    assert(bench_in_window(2058, 2048, 10));
    assert(!bench_in_window(2037, 2048, 10));
    assert(!bench_in_window(2059, 2048, 10));
    assert(bench_clamp_to_window(5000, 2048, 10) == 2058);
    assert(bench_clamp_to_window(-5000, 2048, 10) == 2038);
    assert(bench_clamp_to_window(2050, 2048, 10) == 2050);
}

static void test_start_pose_needs_room(void)
{
    const int32_t half = bench_rad_to_pulses(0.5f) + BENCH_GUARD_MARGIN_PULSES;     // 350 pulses
    assert(half == 350);
    assert(bench_centre_has_room(2048, half));
    assert(bench_centre_has_room(half, half));
    assert(!bench_centre_has_room(half - 1, half));
    assert(bench_centre_has_room(4095 - half, half));
    assert(!bench_centre_has_room(4095 - half + 1, half));
    assert(!bench_centre_has_room(100, half));
    assert(!bench_centre_has_room(4000, half));
}

static void test_estimate(void)
{
    const size_t n = bench_script_build(BENCH_ALL, runs, BENCH_MAX_RUNS);
    const float full = bench_estimate_seconds(runs, n);
    const size_t dry_n = bench_script_build(BENCH_E1 | BENCH_E0, runs, BENCH_MAX_RUNS);
    const float dry = bench_estimate_seconds(runs, dry_n);
    printf("bench script: %zu runs, estimated %.0f s (%.1f min) for the full script, %zu runs %.0f s for the dry run\n",
           n, full, full / 60.0f, dry_n, dry);
    assert(full < 40.0f * 60.0f);    // has to fit in the hour with room to spare
    assert(dry < 5.0f * 60.0f);
}

int main(void)
{
    test_script_contents();
    test_runs_are_consistent();
    test_nothing_leaves_the_range();
    test_runs_are_possible_for_the_servo();
    test_paths();
    test_units();
    test_computed_profile_arrives_in_time();
    test_window();
    test_start_pose_needs_room();
    test_estimate();
    printf("bench_script: all tests passed\n");
    return 0;
}
