//
// Script of the servo profile bench experiment, see bench_script.h
//
#include "bench_script.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f
#define PULSES_PER_TURN 4096.0f

/* Units of the profile registers: 0.229 rev/min and 214.577 rev/min^2 */
#define VELOCITY_UNIT_RADPS (0.229f * 2.0f * PI_F / 60.0f)
#define ACCEL_UNIT_RADPS2 (214.577f * 2.0f * PI_F / 3600.0f)

#define TIBIA_ONLY 0x04U
#define ALL_JOINTS 0x07U

typedef struct {
    bench_run_t *runs;
    size_t max;
    size_t count;
} builder_t;

static bench_run_t *add_run(builder_t *b, unsigned experiment, bench_mode_t mode, bench_path_t path, unsigned joints)
{
    if (b->count >= b->max) {
        return NULL;
    }
    bench_run_t *r = &b->runs[b->count];
    memset(r, 0, sizeof(*r));
    r->id = (uint8_t) (b->count + 1);
    r->experiment = (uint8_t) experiment;
    r->mode = mode;
    r->path = path;
    r->joints = (uint8_t) joints;
    r->sample_ms = 1;
    r->duration_ms = 1000;
    b->count++;
    return r;
}

static void set_delta(bench_run_t *r, int16_t d0, int16_t d1, int16_t d2)
{
    r->delta_mrad[0] = d0;
    r->delta_mrad[1] = d1;
    r->delta_mrad[2] = d2;
}

static void tibia_delta(bench_run_t *r, int16_t mrad)
{
    set_delta(r, 0, 0, mrad);
}

/* E1: each joint a little, to check the identifiers and directions, then the unit check */
static void build_e1(builder_t *b)
{
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        bench_run_t *r = add_run(b, 1, BENCH_MODE_VELOCITY, BENCH_PATH_STEP, 1U << j);
        if (r == NULL) {
            return;
        }
        r->velocity = 50;
        r->accel = 20;
        r->delta_mrad[j] = 100;
        r->duration_ms = 800;
    }
    bench_run_t *r = add_run(b, 1, BENCH_MODE_VELOCITY, BENCH_PATH_STEP, TIBIA_ONLY);
    if (r != NULL) {
        r->velocity = 100;
        r->accel = 50;
        tibia_delta(r, 500);
    }
}

/* E2: velocity based profile, one joint, velocity and acceleration sweep */
static void build_e2(builder_t *b)
{
    static const uint16_t velocities[] = {50, 100, 200, 400};
    static const uint16_t accels[] = {20, 100, 500};
    for (size_t v = 0; v < sizeof(velocities) / sizeof(velocities[0]); v++) {
        for (size_t a = 0; a < sizeof(accels) / sizeof(accels[0]); a++) {
            bench_run_t *r = add_run(b, 2, BENCH_MODE_VELOCITY, BENCH_PATH_STEP, TIBIA_ONLY);
            if (r == NULL) {
                return;
            }
            r->velocity = velocities[v];
            r->accel = accels[a];
            tibia_delta(r, 500);
        }
    }
}

static void add_time_step(builder_t *b, uint16_t time_ms, uint16_t ramp_ms, int16_t delta_mrad)
{
    bench_run_t *r = add_run(b, 3, BENCH_MODE_TIME, BENCH_PATH_STEP, TIBIA_ONLY);
    if (r == NULL) {
        return;
    }
    r->time_ms = time_ms;
    r->ramp_ms = ramp_ms;
    tibia_delta(r, delta_mrad);
    // a move that the servo can not make in the time takes longer: leave room to see how long
    r->duration_ms = (delta_mrad >= 500 && time_ms < 60) ? 1000 : 600;
}

/* E3: time based profile, one joint */
static void build_e3(builder_t *b)
{
    static const uint16_t times[] = {30, 60, 100, 200};
    static const int16_t deltas[] = {50, 200, 500, 1000};
    for (size_t t = 0; t < sizeof(times) / sizeof(times[0]); t++) {
        for (unsigned ramp_div = 4; ramp_div >= 2; ramp_div -= 2) {
            for (size_t d = 0; d < sizeof(deltas) / sizeof(deltas[0]); d++) {
                add_time_step(b, times[t], (uint16_t) (times[t] / ramp_div), deltas[d]);
            }
        }
    }
    // two more times, in between
    static const uint16_t extra_times[] = {40, 80};
    for (size_t t = 0; t < sizeof(extra_times) / sizeof(extra_times[0]); t++) {
        add_time_step(b, extra_times[t], (uint16_t) (extra_times[t] / 4), 200);
        add_time_step(b, extra_times[t], (uint16_t) (extra_times[t] / 4), 500);
    }
}

static void add_chained(builder_t *b, bench_mode_t mode, bench_path_t path, uint16_t period_ms, uint16_t time_ms)
{
    bench_run_t *r = add_run(b, 4, mode, path, TIBIA_ONLY);
    if (r == NULL) {
        return;
    }
    r->period_ms = period_ms;
    r->time_ms = time_ms;
    r->ramp_ms = (uint16_t) (time_ms / 4);
    if (time_ms > 0 && r->ramp_ms == 0) {
        r->ramp_ms = 1;
    }
    if (path == BENCH_PATH_RAMP) {
        tibia_delta(r, 600);
        r->path_param = 600;        // mrad/s
        r->duration_ms = 1300;
    } else {
        tibia_delta(r, 500);        // amplitude 250 mrad
        r->path_param = 1000;       // mHz
        r->duration_ms = 1500;
    }
}

/* E4: chained goals. The modes of drive mode group 0 or 1, as asked. */
static void build_e4(builder_t *b, unsigned group)
{
    static const uint16_t periods[] = {100, 40, 20};
    static const uint16_t factors_x10[] = {8, 12, 15};
    static const bench_path_t paths[] = {BENCH_PATH_RAMP, BENCH_PATH_SINE};

    for (size_t p = 0; p < sizeof(periods) / sizeof(periods[0]); p++) {
        for (size_t q = 0; q < sizeof(paths) / sizeof(paths[0]); q++) {
            if (group == 0) {
                // no profile at all: how the servo behaves with plain goals
                add_chained(b, BENCH_MODE_NONE, paths[q], periods[p], 0);
            }
            for (size_t f = 0; f < sizeof(factors_x10) / sizeof(factors_x10[0]); f++) {
                const uint16_t time_ms = (uint16_t) ((periods[p] * factors_x10[f] + 5) / 10);
                add_chained(b, group == 0 ? BENCH_MODE_VELOCITY_COMPUTED : BENCH_MODE_TIME, paths[q], periods[p],
                            time_ms);
            }
        }
    }
}

static void add_sync(builder_t *b, bench_mode_t mode, uint16_t time_ms)
{
    bench_run_t *r = add_run(b, 5, mode, BENCH_PATH_STEP, ALL_JOINTS);
    if (r == NULL) {
        return;
    }
    // 40, 100 and 150 mrad: even in 40 ms (ramps of 10 ms) the tibia needs only 5 rad/s
    set_delta(r, 40, 100, 150);
    r->sample_ms = 2;
    r->duration_ms = 600;
    if (mode == BENCH_MODE_VELOCITY) {
        r->velocity = 100;      // the same 2.4 rad/s for all joints: they arrive one after the other
        r->accel = 100;
    } else {
        r->time_ms = time_ms;
        r->ramp_ms = (uint16_t) (time_ms / 4);
    }
}

/* E5: three joints with unequal travel */
static void build_e5(builder_t *b, unsigned group)
{
    if (group == 0) {
        add_sync(b, BENCH_MODE_VELOCITY, 0);
        add_sync(b, BENCH_MODE_VELOCITY_COMPUTED, 80);
        add_sync(b, BENCH_MODE_VELOCITY_COMPUTED, 40);
    } else {
        add_sync(b, BENCH_MODE_TIME, 80);
        add_sync(b, BENCH_MODE_TIME, 40);
    }
}

/* E6: stop and reversal in the middle of a move */
static void build_e6(builder_t *b, unsigned group)
{
    static const bench_path_t paths[] = {BENCH_PATH_STOP, BENCH_PATH_REVERSE};
    for (size_t p = 0; p < sizeof(paths) / sizeof(paths[0]); p++) {
        bench_run_t *r = add_run(b, 6, group == 0 ? BENCH_MODE_VELOCITY : BENCH_MODE_TIME, paths[p], TIBIA_ONLY);
        if (r == NULL) {
            return;
        }
        tibia_delta(r, 800);
        r->path_param = 150;        // ms after the start of the move: still moving then
        r->duration_ms = 1200;
        if (group == 0) {
            r->velocity = 200;      // the move takes about 300 ms
            r->accel = 100;
        } else {
            r->time_ms = 300;       // 3.6 rad/s, possible for the servo
            r->ramp_ms = 75;
        }
    }
}

/* E0: the guard has to abort this one */
static void build_e0(builder_t *b)
{
    bench_run_t *r = add_run(b, 0, BENCH_MODE_VELOCITY, BENCH_PATH_STEP, TIBIA_ONLY);
    if (r != NULL) {
        r->velocity = 100;
        r->accel = 50;
        tibia_delta(r, 100);
        r->duration_ms = 400;
    }
}

size_t bench_script_build(uint32_t experiment_mask, bench_run_t *runs, size_t max_runs)
{
    builder_t b = {runs, max_runs, 0};

    // drive mode group 0: no profile and velocity based profiles
    if (experiment_mask & BENCH_E1) build_e1(&b);
    if (experiment_mask & BENCH_E2) build_e2(&b);
    if (experiment_mask & BENCH_E4) build_e4(&b, 0);
    if (experiment_mask & BENCH_E5) build_e5(&b, 0);
    if (experiment_mask & BENCH_E6) build_e6(&b, 0);
    if (experiment_mask & BENCH_E0) build_e0(&b);

    // drive mode group 1: time based profile
    if (experiment_mask & BENCH_E3) build_e3(&b);
    if (experiment_mask & BENCH_E4) build_e4(&b, 1);
    if (experiment_mask & BENCH_E5) build_e5(&b, 1);
    if (experiment_mask & BENCH_E6) build_e6(&b, 1);

    return b.count;
}

unsigned bench_run_group(const bench_run_t *run)
{
    return run->mode == BENCH_MODE_TIME ? 1U : 0U;
}

static float joint_delta_rad(const bench_run_t *run, unsigned joint)
{
    if (joint >= BENCH_JOINTS || !(run->joints & (1U << joint))) {
        return 0.0f;
    }
    return (float) run->delta_mrad[joint] / 1000.0f;
}

float bench_path_offset_rad(const bench_run_t *run, unsigned joint, int32_t t_ms)
{
    const float d = joint_delta_rad(run, joint);
    if (d == 0.0f) {
        return 0.0f;
    }
    const int32_t t = t_ms < 0 ? 0 : t_ms;

    switch (run->path) {
        case BENCH_PATH_STEP:
        case BENCH_PATH_STOP:
            // before the goal the joint waits at the start pose
            return t_ms < 0 ? -d / 2.0f : d / 2.0f;
        case BENCH_PATH_REVERSE:
            if (t_ms < 0) {
                return -d / 2.0f;
            }
            return t >= (int32_t) run->path_param ? -d / 2.0f : d / 2.0f;
        case BENCH_PATH_RAMP: {
            const float rate = (float) run->path_param / 1000.0f;       // rad/s
            float travel = rate * (float) t / 1000.0f;
            const float limit = fabsf(d);
            if (travel > limit) {
                travel = limit;
            }
            return -d / 2.0f + (d < 0.0f ? -travel : travel);
        }
        case BENCH_PATH_SINE: {
            const float f_hz = (float) run->path_param / 1000.0f;
            return (d / 2.0f) * sinf(2.0f * PI_F * f_hz * (float) t / 1000.0f);
        }
    }
    return 0.0f;
}

unsigned bench_goal_count(const bench_run_t *run)
{
    switch (run->path) {
        case BENCH_PATH_STEP:
            return 1;
        case BENCH_PATH_STOP:
        case BENCH_PATH_REVERSE:
            return 2;
        case BENCH_PATH_RAMP:
        case BENCH_PATH_SINE: {
            const unsigned n = run->period_ms > 0 ? run->duration_ms / run->period_ms : 1;
            return n > 0 ? n : 1;
        }
    }
    return 1;
}

int32_t bench_goal_time_ms(const bench_run_t *run, unsigned k)
{
    switch (run->path) {
        case BENCH_PATH_STEP:
            return 0;
        case BENCH_PATH_STOP:
        case BENCH_PATH_REVERSE:
            return k == 0 ? 0 : (int32_t) run->path_param;
        case BENCH_PATH_RAMP:
        case BENCH_PATH_SINE:
            return (int32_t) (k * run->period_ms);
    }
    return 0;
}

float bench_goal_offset_rad(const bench_run_t *run, unsigned joint, unsigned k)
{
    switch (run->path) {
        case BENCH_PATH_STEP:
            return bench_path_offset_rad(run, joint, 0);
        case BENCH_PATH_STOP:
            return k == 0 ? bench_path_offset_rad(run, joint, 0) : 0.0f;
        case BENCH_PATH_REVERSE:
            return bench_path_offset_rad(run, joint, k == 0 ? 0 : (int32_t) run->path_param);
        case BENCH_PATH_RAMP:
        case BENCH_PATH_SINE:
            // a controller tick commands where the joint should be at the next tick
            return bench_path_offset_rad(run, joint, bench_goal_time_ms(run, k) + (int32_t) run->period_ms);
    }
    return 0.0f;
}

int32_t bench_rad_to_pulses(float rad)
{
    return (int32_t) lroundf(rad * PULSES_PER_TURN / (2.0f * PI_F));
}

float bench_pulses_to_rad(int32_t pulses)
{
    return (float) pulses * 2.0f * PI_F / PULSES_PER_TURN;
}

static uint32_t clamp_units(float units, uint32_t max)
{
    if (!(units >= 1.0f)) {
        return 1;
    }
    if (units > (float) max) {
        return max;
    }
    return (uint32_t) lroundf(units);
}

uint32_t bench_velocity_units(float radps)
{
    return clamp_units(fabsf(radps) / VELOCITY_UNIT_RADPS, 32767);
}

uint32_t bench_accel_units(float radps2)
{
    return clamp_units(fabsf(radps2) / ACCEL_UNIT_RADPS2, 32767);
}

void bench_computed_profile(float delta_rad, uint16_t time_ms, uint16_t ramp_ms, uint32_t velocity_limit,
                            uint32_t *velocity, uint32_t *accel)
{
    // Both ramps together take at most the whole time
    uint32_t ramp = ramp_ms;
    uint32_t total = time_ms;
    if (ramp < 1) {
        ramp = 1;
    }
    if (total < 2 * ramp) {
        total = 2 * ramp;
    }

    const float cruise_s = (float) (total - ramp) / 1000.0f;
    const float v_radps = fabsf(delta_rad) / cruise_s;
    const float a_radps2 = v_radps / ((float) ramp / 1000.0f);

    uint32_t v = bench_velocity_units(v_radps);
    if (velocity_limit > 0 && v > velocity_limit) {
        v = velocity_limit;
    }
    *velocity = v;
    *accel = bench_accel_units(a_radps2);
}

int bench_in_window(int32_t pulses, int32_t centre, int32_t half_width)
{
    return pulses >= centre - half_width && pulses <= centre + half_width;
}

int bench_centre_has_room(int32_t centre, int32_t half_width)
{
    return centre - half_width >= 0 && centre + half_width <= 4095;
}

int32_t bench_clamp_to_window(int32_t goal, int32_t centre, int32_t half_width)
{
    if (goal < centre - half_width) {
        return centre - half_width;
    }
    if (goal > centre + half_width) {
        return centre + half_width;
    }
    return goal;
}

float bench_estimate_seconds(const bench_run_t *runs, size_t count)
{
    const float dump_s_per_record = 0.0045f;   // a record line at 115200 baud
    float total = 0.0f;
    unsigned groups_seen = 0;
    int last_group = -1;

    for (size_t i = 0; i < count; i++) {
        const bench_run_t *r = &runs[i];
        unsigned joints = 0;
        for (unsigned j = 0; j < BENCH_JOINTS; j++) {
            joints += (r->joints >> j) & 1U;
        }
        const float records = (float) (BENCH_PRE_TRIGGER_MS + r->duration_ms) / (float) r->sample_ms * (float) joints;

        total += 1.5f;                                          // gentle move to the start pose
        total += 0.3f;                                          // settle
        total += (float) (BENCH_PRE_TRIGGER_MS + r->duration_ms) / 1000.0f;
        total += records * dump_s_per_record + 0.3f;            // dump and overhead

        if ((int) bench_run_group(r) != last_group) {
            groups_seen++;
            last_group = (int) bench_run_group(r);
        }
    }
    // the countdowns: at the start 30 s to hold the leg plus 10 s to let go; between the two drive mode
    // groups 10 s of warning, 45 s to hold the leg in the pose and 10 s to let go; 15 s at the end
    total += 55.0f + (groups_seen > 1 ? 65.0f : 0.0f);
    return total;
}
