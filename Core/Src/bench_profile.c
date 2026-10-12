//
// Servo profile bench experiment, see bench_profile.h and
// docs/servo-profile-bench-experiment.md
//
// Output format (every line starts with "BP,", the console adds nothing else):
//   BP,BEGIN,<mask>,<runs>,<estimated seconds>
//   BP,ID,<servo id>,<register>,<value>                  identification of the servos of leg 0
//   BP,NOTE,<text>                                       what the operator has to do or what happens
//   BP,RUN,<id>,<exp>,<mode>,<path>,<joints>,<period>,<time>,<ramp>,<velocity>,<accel>,
//          <d0>,<d1>,<d2>,<path param>,<sample ms>,<duration ms>
//   BP,START,<id>,<park0>,<park1>,<park2>,<centre0>,<centre1>,<centre2>     pulses
//   BP,G,<id>,<host us>,<joint>,<goal pulses>,<profile velocity>,<profile accel>   a goal write
//   BP,S,<id>,<host us>,<joint>,<servo tick>,<moving>,<load>,<velocity>,<position>,<vel traj>,<pos traj>
//   BP,END,<id>,<samples>,<OK | OVERFLOW | ABORT:reason>
//   BP,DONE,<OK | ABORT>
// host us is the time since the start of the run, measured with the cycle counter.
//
#include "bench_profile.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bench_script.h"
#include "cmsis_os2.h"
#include "log.h"
#include "main.h"

#ifndef BENCH_EXPERIMENT_MASK
#define BENCH_EXPERIMENT_MASK (BENCH_E1 | BENCH_E0)
#endif

/* One Sync Read gives the servo tick, moving flags, PWM, load, velocity, position and both trajectories */
#define READ_ADDRESS XL430_CT_RAM_REALTIME_TICK
#define READ_LENGTH 24
#define MAX_SAMPLES 1600
#define MAX_GOALS 256
#define BUS_ERRORS_BEFORE_ABORT 5
/* A joint that holds the leg against gravity does not reach its goal: the position controller has no
 * integral action, so it stops short by an amount that follows the load (21 pulses at a load of 12 % was
 * measured). Arrival is therefore judged on the profile (trajectory at the goal, no movement), and only
 * a position further than this from the profile counts as a blocked joint. */
#define MAX_FOLLOW_ERROR 80         /* pulses */
/* At the switch to the second drive mode group the leg is held by hand in the pose of the start again. All runs
 * have to be centred on the same pose, so the pose may differ this much (a hand does not hold it exactly). */
#define POSE_TOLERANCE 30           /* pulses, about 2.6 degrees */
#define STILL_VELOCITY_UNITS 2      /* |present velocity| at or below this is standing still */

#define DRIVE_MODE_TIME_BASED 0x04

typedef struct __attribute__((packed)) {
    uint32_t host_us;
    uint16_t tick;
    int16_t load;
    int16_t velocity;
    uint16_t position;
    int16_t velocity_trajectory;
    uint16_t position_trajectory;
    uint8_t joint;
    uint8_t moving;
} sample_t;

typedef struct {
    uint32_t host_us;
    uint32_t velocity;
    uint32_t accel;
    uint16_t goal;
    uint8_t joint;
} goal_t;

static struct {
    dynamixel_servo_t *leg;             /* the three servos of leg 0 */
    int32_t centre[BENCH_JOINTS];       /* pose the experiment is centred on, pulses */
    int32_t goal_half;                  /* goals are limited to centre +- goal_half */
    int32_t guard_centre[BENCH_JOINTS]; /* the run is aborted when a joint leaves guard_centre +- guard_half */
    int32_t guard_half;
    int32_t latest_position[BENCH_JOINTS];
    uint32_t velocity_limit;            /* Velocity Limit register, in profile velocity units */
    uint8_t drive_mode_base;            /* the drive mode of the servos without the time based bit */
    unsigned current_run;
    bool have_centre;                   /* the centre pose has been taken (at the start) */
} bp;

static sample_t g_samples[MAX_SAMPLES];
static unsigned g_sample_count;
static bool g_overflow;
static goal_t g_goals[MAX_GOALS];
static unsigned g_goal_count;
static uint32_t g_t0_cycles;
static bench_run_t g_runs[BENCH_MAX_RUNS];
static size_t g_run_count;

static uint32_t elapsed_us(void)
{
    return (DWT->CYCCNT - g_t0_cycles) / (SystemCoreClock / 1000000U);
}

static dynamixel_result_t write_profile_all(uint32_t accel, uint32_t velocity);

static void note(const char *text)
{
    printf("BP,NOTE,%s\r\n", text);
}

__attribute__((noreturn)) static void park(void)
{
    for (;;) {
        osDelay(10000);
        note("parked, reset the board to run again");
    }
}

/** The LEDs of the servos of leg 0 show which leg is under test */
static void leds_all(uint8_t state)
{
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        dynamixel_set_led(&bp.leg[j], state);
    }
}

static void torque_all(uint8_t state)
{
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        dynamixel_set_torque_enable(&bp.leg[j], state);
    }
}

/**
 * Leave the servos as the normal firmware expects them, which never writes the profile registers: drive
 * mode back to normal and Profile Velocity and Acceleration 0 (no profile). The profile registers are RAM and
 * stay as they are until the servos lose power, also across a reset of the board, so what the experiment
 * wrote would otherwise slow the robot down afterwards. Best effort: used after an abort too.
 */
static void restore_servos(void)
{
    torque_all(0);
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        dynamixel_set_byte_parameter(&bp.leg[j], XL430_CT_EEP_DRIVE_MODE, bp.drive_mode_base);
    }
    write_profile_all(0, 0);
}

static void dump_run(const bench_run_t *r, const int32_t park_pose[BENCH_JOINTS], const char *status)
{
    printf("BP,RUN,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%u,%u,%u\r\n", r->id, r->experiment, (unsigned) r->mode,
           (unsigned) r->path, r->joints, r->period_ms, r->time_ms, r->ramp_ms, r->velocity, r->accel,
           r->delta_mrad[0], r->delta_mrad[1], r->delta_mrad[2], r->path_param, r->sample_ms, r->duration_ms);
    printf("BP,START,%u,%ld,%ld,%ld,%ld,%ld,%ld\r\n", r->id, (long) park_pose[0], (long) park_pose[1],
           (long) park_pose[2], (long) bp.centre[0], (long) bp.centre[1], (long) bp.centre[2]);
    for (unsigned i = 0; i < g_goal_count; i++) {
        const goal_t *g = &g_goals[i];
        printf("BP,G,%u,%lu,%u,%u,%lu,%lu\r\n", r->id, (unsigned long) g->host_us, g->joint, g->goal,
               (unsigned long) g->velocity, (unsigned long) g->accel);
    }
    for (unsigned i = 0; i < g_sample_count; i++) {
        const sample_t *s = &g_samples[i];
        printf("BP,S,%u,%lu,%u,%u,%u,%d,%d,%u,%d,%u\r\n", r->id, (unsigned long) s->host_us, s->joint, s->tick,
               s->moving, s->load, s->velocity, s->position, s->velocity_trajectory, s->position_trajectory);
    }
    printf("BP,END,%u,%u,%s\r\n", r->id, g_sample_count, status);
}

/** Stop everything: torque off, report what there is and park. Does not return. */
__attribute__((noreturn)) static void abort_run(const char *reason, const bench_run_t *run,
                                                const int32_t park_pose[BENCH_JOINTS])
{
    restore_servos();
    char status[48];
    snprintf(status, sizeof(status), "ABORT:%s", reason);
    if (run != NULL) {
        dump_run(run, park_pose, status);
    } else {
        printf("BP,END,%u,0,%s\r\n", bp.current_run, status);
    }
    printf("BP,DONE,ABORT\r\n");
    park();
}

static bool bus_ok(dynamixel_result_t result)
{
    return result == DNM_OK;
}

static void decode_sample(const uint8_t *d, unsigned joint, uint32_t host_us, sample_t *s)
{
    s->host_us = host_us;
    s->tick = dynamixel_decode_u16(&d[0]);
    s->moving = d[2];
    s->load = dynamixel_decode_i16(&d[6]);
    s->velocity = (int16_t) dynamixel_decode_i32(&d[8]);
    s->position = (uint16_t) dynamixel_decode_u32(&d[12]);
    s->velocity_trajectory = (int16_t) dynamixel_decode_i32(&d[16]);
    s->position_trajectory = (uint16_t) dynamixel_decode_u32(&d[20]);
    s->joint = (uint8_t) joint;
}

static dynamixel_result_t write_profile_all(uint32_t accel, uint32_t velocity)
{
    uint8_t data[BENCH_JOINTS * 8];
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        memcpy(&data[j * 8], &accel, 4);        /* little endian target */
        memcpy(&data[j * 8 + 4], &velocity, 4);
    }
    return dynamixel_set_bytes_multiple(bp.leg, BENCH_JOINTS, XL430_CT_RAM_PROFILE_ACC, 8, data);
}

static dynamixel_result_t write_goals_all(const int32_t goal[BENCH_JOINTS])
{
    uint32_t values[BENCH_JOINTS];
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        values[j] = (uint32_t) bench_clamp_to_window(goal[j], bp.centre[j], bp.goal_half);
    }
    return dynamixel_set_long_parameter_multiple(bp.leg, BENCH_JOINTS, XL430_CT_RAM_GOAL_POSITION, values);
}

/** Gentle profile used to move between runs: velocity based units or, in time based mode, milliseconds */
static dynamixel_result_t write_gentle_profile(unsigned group)
{
    return group ? write_profile_all(200, 800) : write_profile_all(20, 50);
}

static bool read_positions(int32_t position[BENCH_JOINTS])
{
    uint32_t values[BENCH_JOINTS];
    if (!bus_ok(dynamixel_get_long_parameter_multiple(bp.leg, BENCH_JOINTS, XL430_CT_RAM_PRESENT_POSITION, values))) {
        return false;
    }
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        position[j] = (int32_t) values[j];
    }
    return true;
}

/** Position, moving flag and trajectories of all three joints while parking between runs */
static bool read_all_blocks(uint8_t data[BENCH_JOINTS * READ_LENGTH])
{
    dynamixel_result_t status[BENCH_JOINTS];
    return bus_ok(dynamixel_get_bytes_multiple(bp.leg, BENCH_JOINTS, READ_ADDRESS, READ_LENGTH, data, status));
}

/** Move all joints to a pose with the gentle profile and wait until they are there */
static bool go_to(const int32_t pose[BENCH_JOINTS], unsigned group)
{
    if (!bus_ok(write_gentle_profile(group)) || !bus_ok(write_goals_all(pose))) {
        note("cannot write the goals for the start pose");
        return false;
    }

    int32_t position[BENCH_JOINTS] = {0};
    int32_t trajectory[BENCH_JOINTS] = {0};
    int32_t target[BENCH_JOINTS] = {0};
    uint8_t moving[BENCH_JOINTS] = {0};

    const uint32_t deadline = osKernelGetTickCount() + 5000;
    unsigned stable = 0;
    while (osKernelGetTickCount() < deadline) {
        osDelay(25);
        uint8_t data[BENCH_JOINTS * READ_LENGTH];
        if (!read_all_blocks(data)) {
            note("cannot read the joints while moving to the start pose");
            return false;
        }
        bool arrived = true;
        for (unsigned j = 0; j < BENCH_JOINTS; j++) {
            const uint8_t *d = &data[j * READ_LENGTH];
            position[j] = (int32_t) dynamixel_decode_u32(&d[12]);
            trajectory[j] = (int32_t) dynamixel_decode_u32(&d[20]);
            moving[j] = d[2];
            const int32_t velocity = dynamixel_decode_i32(&d[8]);
            bp.latest_position[j] = position[j];
            if (!bench_in_window(position[j], bp.guard_centre[j], bp.guard_half)) {
                abort_run("out of range while moving to the start pose", NULL, NULL);
            }
            // arrived: the profile has finished at the goal and the joint stands still
            target[j] = bench_clamp_to_window(pose[j], bp.centre[j], bp.goal_half);
            const int32_t at_goal = trajectory[j] - target[j];
            if (at_goal > 1 || at_goal < -1 || moving[j] != 0 ||
                velocity > STILL_VELOCITY_UNITS || velocity < -STILL_VELOCITY_UNITS) {
                arrived = false;
            }
        }
        stable = arrived ? stable + 1 : 0;
        if (stable >= 3) {
            for (unsigned j = 0; j < BENCH_JOINTS; j++) {
                const int32_t error = position[j] - trajectory[j];
                if (error > MAX_FOLLOW_ERROR || error < -MAX_FOLLOW_ERROR) {
                    printf("BP,NOTE,joint %u stands %ld pulses from its profile: blocked?\r\n", j, (long) error);
                    return false;
                }
            }
            return true;
        }
    }

    // say what the joints were doing, to see why
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        printf("BP,NOTE,start pose timeout joint %u: position %ld profile %ld goal %ld moving %u\r\n", j,
               (long) position[j], (long) trajectory[j], (long) target[j], moving[j]);
    }
    return false;
}

/**
 * Hold the present pose: goal = present position, torque on. At the start (keep_centre false) the pose also
 * becomes the centre of the experiment; later the centre of the start is kept.
 */
static void hold_current_pose(unsigned group, bool keep_centre)
{
    int32_t position[BENCH_JOINTS] = {0};
    bool read = false;
    for (int attempt = 0; attempt < 3 && !read; attempt++) {
        read = read_positions(position);
    }
    if (!read) {
        abort_run("cannot read the present positions", NULL, NULL);
    }

    bp.guard_half = bench_rad_to_pulses((float) BENCH_RANGE_MRAD / 1000.0f) + BENCH_GUARD_MARGIN_PULSES;
    bp.goal_half = bench_rad_to_pulses((float) BENCH_RANGE_MRAD / 1000.0f) + 4;

    /* the experiment moves every joint up to +-0.5 rad from this pose: it must not be near the
     * end of the range of the servo (0..4095), and the leg has to be free to move that far */
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        if (!keep_centre) {
            if (!bench_centre_has_room(position[j], bp.guard_half)) {
                char reason[48];
                snprintf(reason, sizeof(reason), "start pose of joint %u near range end", j);
                abort_run(reason, NULL, NULL);
            }
            bp.centre[j] = position[j];
            bp.guard_centre[j] = position[j];
        }
        bp.latest_position[j] = position[j];
    }
    bp.have_centre = true;

    /* the goal and the profile are written with the torque off, so nothing moves when it comes on */
    if (!bus_ok(write_gentle_profile(group)) || !bus_ok(write_goals_all(position))) {
        abort_run("cannot write the hold pose", NULL, NULL);
    }
    torque_all(1);
}

static void set_drive_mode(unsigned group)
{
    const uint8_t wanted = (uint8_t) (bp.drive_mode_base | (group ? DRIVE_MODE_TIME_BASED : 0));
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        uint8_t current = 0xFF;
        if (!bus_ok(dynamixel_get_byte_parameter(&bp.leg[j], XL430_CT_EEP_DRIVE_MODE, &current))) {
            abort_run("cannot read the drive mode", NULL, NULL);
        }
        if (current != wanted &&
            !bus_ok(dynamixel_set_byte_parameter(&bp.leg[j], XL430_CT_EEP_DRIVE_MODE, wanted))) {
            abort_run("cannot write the drive mode", NULL, NULL);
        }
    }
}

static void countdown(const char *what, unsigned seconds)
{
    char text[96];
    for (unsigned left = seconds; left > 0; left -= (left > 5 ? 5 : left)) {
        snprintf(text, sizeof(text), "%s in %u s", what, left);
        note(text);
        osDelay(left > 5 ? 5000 : left * 1000);
    }
}

/**
 * Start of the experiment and switch of the drive mode.
 *
 * The drive mode (an EEPROM register) can only be written with the torque off, so for a moment the leg has to be
 * limp. At the switch the leg is under torque, holding the last pose of the previous run: the torque goes off
 * after a warning, and the leg has to be held by hand and brought back to the pose of the start, within
 * POSE_TOLERANCE, so that all runs are centred on the same pose. Only then the drive mode is written and the
 * torque comes on again, holding the pose.
 */
static void change_group(unsigned group, unsigned countdown_s)
{
    const bool first = !bp.have_centre;

    if (first) {
        /* the torque is off since the board started: the leg can be positioned by hand */
        note("HOLD LEG 0 in a neutral pose with a hand (the torque is off): the pose is taken at the end of the countdown");
    } else {
        note("SUPPORT LEG 0: the torque goes off in 10 s, so that the drive mode can be changed");
        countdown("torque off", 10);
        torque_all(0);
        note("TORQUE IS OFF: hold leg 0 in the pose of the start, within 2 degrees (see the numbers)");
    }

    for (unsigned attempt = 0;; attempt++) {
        countdown("the pose is taken", attempt == 0 ? countdown_s : 10);
        if (first) {
            break;
        }
        int32_t position[BENCH_JOINTS] = {0};
        bool read = false;
        for (int tries = 0; tries < 3 && !read; tries++) {
            read = read_positions(position);
        }
        if (!read) {
            abort_run("cannot read the positions", NULL, NULL);
        }
        bool in_pose = true;
        for (unsigned j = 0; j < BENCH_JOINTS; j++) {
            const long off = (long) (position[j] - bp.centre[j]);
            printf("BP,NOTE,joint %u is %+ld pulses from the pose of the start\r\n", j, off);
            if (off > POSE_TOLERANCE || off < -POSE_TOLERANCE) {
                in_pose = false;
            }
        }
        if (in_pose) {
            break;
        }
        if (attempt >= 3) {
            abort_run("leg not back in the pose of the start", NULL, NULL);
        }
        note("not in the pose of the start yet: move the leg, trying again in 10 s");
    }

    set_drive_mode(group);
    hold_current_pose(group, !first);
    note("torque is on, the leg holds this pose: RELEASE IT NOW");
    osDelay(10000);

    /* the leg was let go: the first time the pose it settled in is the centre, later it has to be there still */
    int32_t position[BENCH_JOINTS] = {0};
    if (!read_positions(position)) {
        abort_run("cannot read the positions after release", NULL, NULL);
    }
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        if (position[j] - bp.centre[j] > 2 * POSE_TOLERANCE || bp.centre[j] - position[j] > 2 * POSE_TOLERANCE) {
            abort_run("leg moved after release", NULL, NULL);
        }
        if (first) {
            bp.centre[j] = position[j];
            bp.guard_centre[j] = position[j];
        }
        bp.latest_position[j] = position[j];
    }
}

static void send_goal(const bench_run_t *r, const dynamixel_servo_t *list, const unsigned *joint_of, unsigned n,
                      unsigned k, const int32_t park_pose[BENCH_JOINTS])
{
    const bool stop_goal = r->path == BENCH_PATH_STOP && k == 1;
    const bool with_profile = r->mode == BENCH_MODE_VELOCITY_COMPUTED || stop_goal;
    const unsigned group = bench_run_group(r);

    uint8_t data[BENCH_JOINTS * 12];
    uint32_t velocity[BENCH_JOINTS] = {0};
    uint32_t accel[BENCH_JOINTS] = {0};
    int32_t goal[BENCH_JOINTS] = {0};

    for (unsigned i = 0; i < n; i++) {
        const unsigned j = joint_of[i];
        if (stop_goal) {
            goal[i] = bp.latest_position[j];
            // a hard stop: velocity based 400 units with acceleration 500, time based 20 ms with 5 ms ramps
            accel[i] = group ? 5 : 500;
            velocity[i] = group ? 20 : 400;
        } else {
            goal[i] = bp.centre[j] + bench_rad_to_pulses(bench_goal_offset_rad(r, j, k));
        }
        goal[i] = bench_clamp_to_window(goal[i], bp.centre[j], bp.goal_half);

        if (r->mode == BENCH_MODE_VELOCITY_COMPUTED) {
            const float delta = bench_pulses_to_rad(goal[i] - bp.latest_position[j]);
            bench_computed_profile(delta, r->time_ms, r->ramp_ms, bp.velocity_limit, &velocity[i], &accel[i]);
        }

        const uint32_t goal_u = (uint32_t) goal[i];
        if (with_profile) {
            memcpy(&data[i * 12], &accel[i], 4);
            memcpy(&data[i * 12 + 4], &velocity[i], 4);
            memcpy(&data[i * 12 + 8], &goal_u, 4);
        } else {
            memcpy(&data[i * 4], &goal_u, 4);
        }
    }

    const dynamixel_result_t res = with_profile
        ? dynamixel_set_bytes_multiple(list, n, XL430_CT_RAM_PROFILE_ACC, 12, data)
        : dynamixel_set_bytes_multiple(list, n, XL430_CT_RAM_GOAL_POSITION, 4, data);
    const uint32_t host_us = elapsed_us();
    if (!bus_ok(res)) {
        abort_run("goal write failed", r, park_pose);
    }

    for (unsigned i = 0; i < n && g_goal_count < MAX_GOALS; i++) {
        goal_t *g = &g_goals[g_goal_count++];
        g->host_us = host_us;
        g->joint = (uint8_t) joint_of[i];
        g->goal = (uint16_t) goal[i];
        g->velocity = with_profile ? velocity[i] : 0;
        g->accel = with_profile ? accel[i] : 0;
    }
}

static void execute_run(const bench_run_t *r)
{
    bp.current_run = r->id;
    g_sample_count = 0;
    g_goal_count = 0;
    g_overflow = false;

    dynamixel_servo_t list[BENCH_JOINTS];
    unsigned joint_of[BENCH_JOINTS];
    unsigned n = 0;
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        if (r->joints & (1U << j)) {
            list[n] = bp.leg[j];
            joint_of[n] = j;
            n++;
        }
    }

    /* park every joint at its start pose; joints that do not move stay in the centre */
    const unsigned group = bench_run_group(r);
    int32_t park_pose[BENCH_JOINTS];
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        park_pose[j] = bp.centre[j] + bench_rad_to_pulses(bench_path_offset_rad(r, j, -1));
    }
    if (!go_to(park_pose, group)) {
        abort_run("cannot reach the start pose", NULL, NULL);
    }
    osDelay(300);

    /* the profile of the run */
    dynamixel_result_t res = DNM_OK;
    switch (r->mode) {
        case BENCH_MODE_NONE:
            res = write_profile_all(0, 0);
            break;
        case BENCH_MODE_VELOCITY:
            res = write_profile_all(r->accel, r->velocity);
            break;
        case BENCH_MODE_TIME:
            res = write_profile_all(r->ramp_ms, r->time_ms);
            break;
        case BENCH_MODE_VELOCITY_COMPUTED:
            res = write_gentle_profile(0);      // replaced by the profile of each goal
            break;
    }
    if (!bus_ok(res)) {
        abort_run("cannot write the profile", NULL, NULL);
    }

    /* the safety self test: a guard that is too narrow for the move must stop the run */
    if (r->experiment == 0) {
        for (unsigned j = 0; j < BENCH_JOINTS; j++) {
            bp.guard_centre[j] = park_pose[j];
        }
        bp.guard_half = 8;
        note("self test: the guard is narrower than the move, the run has to abort");
    }

    const unsigned goals = bench_goal_count(r);
    unsigned next_goal = 0;
    unsigned bus_errors = 0;
    const uint32_t end_us = (uint32_t) (BENCH_PRE_TRIGGER_MS + r->duration_ms) * 1000U;

    g_t0_cycles = DWT->CYCCNT;
    uint32_t next_tick = osKernelGetTickCount();

    for (;;) {
        uint32_t now = elapsed_us();
        while (next_goal < goals &&
               now >= (uint32_t) (BENCH_PRE_TRIGGER_MS + bench_goal_time_ms(r, next_goal)) * 1000U) {
            send_goal(r, list, joint_of, n, next_goal, park_pose);
            next_goal++;
            now = elapsed_us();
        }
        if (now >= end_us) {
            break;
        }

        uint8_t data[BENCH_JOINTS * READ_LENGTH];
        dynamixel_result_t status[BENCH_JOINTS];
        const dynamixel_result_t read = dynamixel_get_bytes_multiple(list, n, READ_ADDRESS, READ_LENGTH, data, status);
        const uint32_t host_us = elapsed_us();
        (void) read;

        for (unsigned i = 0; i < n; i++) {
            const unsigned j = joint_of[i];
            if (status[i] == 0x80) {
                abort_run("hardware alert", r, park_pose);
            }
            if (status[i] != DNM_OK) {
                bus_errors++;
                if (bus_errors >= BUS_ERRORS_BEFORE_ABORT) {
                    abort_run("bus errors", r, park_pose);
                }
                continue;
            }
            bus_errors = 0;

            sample_t s;
            decode_sample(&data[i * READ_LENGTH], j, host_us, &s);
            bp.latest_position[j] = s.position;
            if (!bench_in_window(s.position, bp.guard_centre[j], bp.guard_half)) {
                if (g_sample_count < MAX_SAMPLES) {
                    g_samples[g_sample_count++] = s;
                }
                abort_run("out of range", r, park_pose);
            }
            if (g_sample_count < MAX_SAMPLES) {
                g_samples[g_sample_count++] = s;
            } else {
                g_overflow = true;
            }
        }

        next_tick += r->sample_ms;
        if (osKernelGetTickCount() >= next_tick) {
            next_tick = osKernelGetTickCount() + 1;     // a transaction took longer than the period
        }
        osDelayUntil(next_tick);
    }

    dump_run(r, park_pose, g_overflow ? "OVERFLOW" : "OK");
}

/** Print the identification registers of the servos of leg 0 and read what the run needs */
static void identify(void)
{
    static const struct { const char *name; uint16_t address; uint8_t size; } registers[] = {
        {"model_number", XL430_CT_EEP_MODEL_NUMBER, 2},
        {"firmware_version", XL430_CT_EEP_FIRMWARE_VERSION, 1},
        {"id", XL430_CT_EEP_ID, 1},
        {"baud_rate", XL430_CT_EEP_BAUD_RATE, 1},
        {"return_delay_time", XL430_CT_EEP_RETURN_DELAY_TIME, 1},
        {"drive_mode", XL430_CT_EEP_DRIVE_MODE, 1},
        {"operating_mode", XL430_CT_EEP_OPERATING_MODE, 1},
        {"pwm_limit", XL430_CT_EEP_PWM_LIMIT, 2},
        {"velocity_limit", XL430_CT_EEP_VELOCITY_LIMIT, 4},
        {"max_position_limit", XL430_CT_EEP_MAX_POSITION_LIMIT, 4},
        {"min_position_limit", XL430_CT_EEP_MIN_POSITION_LIMIT, 4},
        {"profile_acceleration", XL430_CT_RAM_PROFILE_ACC, 4},
        {"profile_velocity", XL430_CT_RAM_PROFILE_VEL, 4},
        {"hardware_error_status", XL430_CT_RAM_HARDWARE_ERR_STATUS, 1},
        /* the position controller: it decides how well a loaded joint follows its profile */
        {"velocity_i_gain", XL430_CT_RAM_VELOCITY_I_GAIN, 2},
        {"velocity_p_gain", XL430_CT_RAM_VELOCITY_P_GAIN, 2},
        {"position_d_gain", XL430_CT_RAM_POSITION_D_GAIN, 2},
        {"position_i_gain", XL430_CT_RAM_POSITION_I_GAIN, 2},
        {"position_p_gain", XL430_CT_RAM_POSITION_P_GAIN, 2},
        {"feedforward_2nd_gain", XL430_CT_RAM_FEEDFORWARD_2ND_GAIN, 2},
        {"feedforward_1st_gain", XL430_CT_RAM_FEEDFORWARD_1ST_GAIN, 2},
        {"present_voltage", 144, 2},
        {"present_temperature", 146, 1},
    };

    bp.velocity_limit = 0;
    bp.drive_mode_base = 0;
    for (unsigned j = 0; j < BENCH_JOINTS; j++) {
        for (size_t i = 0; i < sizeof(registers) / sizeof(registers[0]); i++) {
            uint32_t value = 0;
            dynamixel_result_t res = DNM_OK;
            if (registers[i].size == 1) {
                uint8_t v;
                res = dynamixel_get_byte_parameter(&bp.leg[j], registers[i].address, &v);
                value = v;
            } else if (registers[i].size == 2) {
                uint16_t v;
                res = dynamixel_get_word_parameter(&bp.leg[j], registers[i].address, &v);
                value = v;
            } else {
                res = dynamixel_get_long_parameter(&bp.leg[j], registers[i].address, &value);
            }
            if (!bus_ok(res)) {
                printf("BP,ID,%u,%s,ERROR%u\r\n", bp.leg[j].id, registers[i].name, res);
                continue;
            }
            printf("BP,ID,%u,%s,%lu\r\n", bp.leg[j].id, registers[i].name, (unsigned long) value);

            if (registers[i].address == XL430_CT_EEP_VELOCITY_LIMIT) {
                if (bp.velocity_limit == 0 || value < bp.velocity_limit) {
                    bp.velocity_limit = value;
                }
            }
            if (registers[i].address == XL430_CT_EEP_DRIVE_MODE && j == 0) {
                bp.drive_mode_base = (uint8_t) (value & ~DRIVE_MODE_TIME_BASED);
            }
        }
    }
}

void bench_profile_run(dynamixel_servo_t *servos)
{
    /* keep the console free of other output: every line of the experiment starts with BP, */
    g_log_level = LOG_LEVEL_FATAL;
    bp.leg = servos;

    /* enable the cycle counter in case it is not (the firmware does it at start up) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    g_run_count = bench_script_build(BENCH_EXPERIMENT_MASK, g_runs, BENCH_MAX_RUNS);
    printf("BP,BEGIN,%lu,%u,%.0f\r\n", (unsigned long) BENCH_EXPERIMENT_MASK, (unsigned) g_run_count,
           (double) bench_estimate_seconds(g_runs, g_run_count));

    identify();

    /* show which leg is under test; the normal firmware switches them off again at its next start */
    leds_all(1);
    printf("BP,NOTE,testing leg 0, servos %u %u %u: their LEDs are on\r\n", servos[0].id, servos[1].id,
           servos[2].id);

    if (g_run_count == 0) {
        note("no runs selected");
        printf("BP,DONE,OK\r\n");
        park();
    }

    unsigned group = bench_run_group(&g_runs[0]);
    change_group(group, 30);     // time to see which leg has its LEDs on, and to take hold of it

    for (size_t i = 0; i < g_run_count; i++) {
        if (bench_run_group(&g_runs[i]) != group) {
            group = bench_run_group(&g_runs[i]);
            change_group(group, 45);
        }
        execute_run(&g_runs[i]);
    }

    /* leave the servos as they were: the drive mode back to normal needs the torque off */
    note("done: LEG 0 GOES LIMP, hold it");
    countdown("torque off", 15);
    restore_servos();
    leds_all(0);
    printf("BP,DONE,OK\r\n");
    park();
}
