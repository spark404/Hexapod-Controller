/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Script of the servo profile bench experiment (docs/servo-profile-bench-experiment.md).
 *
 * The runs, the ideal paths and the unit conversions, without any hardware
 * dependency so they can be tested on the host (tests/bench_script_test.c). The
 * firmware side is bench_profile.c.
 *
 * Conventions:
 *  - joints are the three servos of leg 0: 0 = coxa, 1 = femur, 2 = tibia
 *  - angles are offsets from a centre pose, in servo pulses (4096 per turn)
 *  - a path offset is the ideal position of a joint, in rad relative to the centre
 *  - path time 0 is the moment of the first goal write
 */

#ifndef BENCH_SCRIPT_H
#define BENCH_SCRIPT_H

#include <stddef.h>
#include <stdint.h>

#define BENCH_JOINTS 3
#define BENCH_MAX_RUNS 128

/* Experiments, also the bit numbers of the experiment mask */
#define BENCH_E1 (1U << 0)  /* identify and units */
#define BENCH_E2 (1U << 1)  /* velocity based profile, one joint */
#define BENCH_E3 (1U << 2)  /* time based profile, one joint */
#define BENCH_E4 (1U << 3)  /* chained goals */
#define BENCH_E5 (1U << 4)  /* three joints in sync */
#define BENCH_E6 (1U << 5)  /* servo side stop and reversal */
#define BENCH_E0 (1U << 6)  /* safety self test: the range guard must abort the run */
#define BENCH_ALL (BENCH_E1 | BENCH_E2 | BENCH_E3 | BENCH_E4 | BENCH_E5 | BENCH_E6)

/* How the goals are given to the servo */
typedef enum {
    BENCH_MODE_NONE = 0,                /* no profile: velocity and acceleration 0, the servo jumps */
    BENCH_MODE_VELOCITY = 1,            /* velocity based, fixed velocity and acceleration */
    BENCH_MODE_VELOCITY_COMPUTED = 2,   /* velocity based, computed per goal so it arrives in time_ms */
    BENCH_MODE_TIME = 3                 /* time based profile: time_ms per goal, ramp_ms ramp */
} bench_mode_t;

typedef enum {
    BENCH_PATH_STEP = 0,    /* one goal: from -delta/2 to +delta/2 */
    BENCH_PATH_RAMP = 1,    /* from -delta/2 to +delta/2 at path_param mrad/s, goal every period_ms */
    BENCH_PATH_SINE = 2,    /* delta/2 * sin(2 pi f t), f = path_param mHz, goal every period_ms */
    BENCH_PATH_STOP = 3,    /* step, then at path_param ms the goal is set to the present position */
    BENCH_PATH_REVERSE = 4  /* step, then at path_param ms the goal goes back to the start */
} bench_path_t;

typedef struct {
    uint8_t id;
    uint8_t experiment;         /* 0..6 */
    bench_mode_t mode;
    bench_path_t path;
    uint8_t joints;             /* bit mask of the joints that move */
    uint16_t period_ms;         /* time between goals of a chained path, 0 for the others */
    uint16_t time_ms;           /* time based / computed: time per goal (T) */
    uint16_t ramp_ms;           /* time based / computed: ramp time (Tr), at most T / 2 */
    uint16_t velocity;          /* fixed velocity: Profile Velocity units */
    uint16_t accel;             /* fixed velocity: Profile Acceleration units */
    int16_t delta_mrad[BENCH_JOINTS];   /* total travel per joint, milli radian, signed */
    uint16_t path_param;
    uint8_t sample_ms;          /* sampling period */
    uint16_t duration_ms;       /* how long after path time 0 the run samples */
} bench_run_t;

/* Samples are taken for this long before the first goal, to see the baseline */
#define BENCH_PRE_TRIGGER_MS 50
/* Distance each side of the centre pose that a joint may reach, and the extra margin of the guard */
#define BENCH_RANGE_MRAD 500
#define BENCH_GUARD_MARGIN_PULSES 24

/**
 * Build the list of runs for the experiments in the mask, ordered so that the
 * runs of one drive mode group are together (the group with drive mode 0 first).
 * Returns the number of runs, at most max_runs.
 */
size_t bench_script_build(uint32_t experiment_mask, bench_run_t *runs, size_t max_runs);

/** Drive mode group of a run: 1 for time based profile, 0 for the others */
unsigned bench_run_group(const bench_run_t *run);

/**
 * Ideal offset in rad of a joint at path time t_ms. Before time 0 this is the
 * start pose the joint is parked at: -delta/2 for a step, stop, reversal and
 * ramp, 0 for a sine.
 */
float bench_path_offset_rad(const bench_run_t *run, unsigned joint, int32_t t_ms);

/** Number of goal writes of a run */
unsigned bench_goal_count(const bench_run_t *run);

/** Path time of goal k */
int32_t bench_goal_time_ms(const bench_run_t *run, unsigned k);

/**
 * Offset in rad to command at goal k. For BENCH_PATH_STOP, goal 1 is the
 * present position: the value returned (0) is a placeholder.
 */
float bench_goal_offset_rad(const bench_run_t *run, unsigned joint, unsigned k);

/* Conversions. A servo pulse is 2 pi / 4096 rad. */
int32_t bench_rad_to_pulses(float rad);
float bench_pulses_to_rad(int32_t pulses);

/* Profile Velocity unit 0.229 rev/min = 0.023981 rad/s, Profile Acceleration unit
 * 214.577 rev/min^2 = 0.374508 rad/s^2 (XL430 control table, to be confirmed by E1) */
uint32_t bench_velocity_units(float radps);
uint32_t bench_accel_units(float radps2);

/**
 * Velocity and acceleration units for a velocity based profile that moves
 * delta_rad in time_ms with ramps of ramp_ms: v = |d| / (T - Tr), a = v / Tr.
 * The velocity is limited to velocity_limit units (at least 1).
 */
void bench_computed_profile(float delta_rad, uint16_t time_ms, uint16_t ramp_ms, uint32_t velocity_limit,
                            uint32_t *velocity, uint32_t *accel);

/** 1 when pulses is within half_width of centre */
int bench_in_window(int32_t pulses, int32_t centre, int32_t half_width);

/** 1 when a joint can move half_width pulses either way from centre inside the servo range 0..4095 */
int bench_centre_has_room(int32_t centre, int32_t half_width);

/** goal limited to the window */
int32_t bench_clamp_to_window(int32_t goal, int32_t centre, int32_t half_width);

/** Estimate of the time a script takes on the bench, in seconds, console dump included */
float bench_estimate_seconds(const bench_run_t *runs, size_t count);

#endif /* BENCH_SCRIPT_H */
