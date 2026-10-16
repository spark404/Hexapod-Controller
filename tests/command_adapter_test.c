//
// Host test for the twist to controller command adapter, see Core/Src/command_adapter.c
//
// Build and run from the Hexapod-Controller directory:
//
//   cc -std=c99 -Wall -Wextra -pedantic -Werror -ICore/Inc \
//      tests/command_adapter_test.c Core/Src/command_adapter.c -lm -o /tmp/command_adapter_test \
//      && /tmp/command_adapter_test
//
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "command_adapter.h"

#define DT 0.1f
#define TWO_PI 6.28318530717958647692f

static const adapter_cfg_t CFG = {
    .max_speed = 100.0f,
    .max_yaw_rate = 0.1f,
    .min_height = 50.0f,
    .max_height = 170.0f,
    .max_accel = 0.0f,
    .link_timeout_ms = 300,
};

static bool near(float a, float b)
{
    return fabsf(a - b) < 1e-4f;
}

static cmd_frame_t cmd(float vx, float vy, float wz)
{
    cmd_frame_t c = {.seq = 1, .vx = vx, .vy = vy, .wz = wz, .height = 100.0f};
    return c;
}

static void give(adapter_t *a, cmd_frame_t c, uint32_t rx_ms)
{
    adapter_set_command(a, &c, rx_ms);
}

/* One control tick at time ms */
static adapter_output_t step(adapter_t *a, uint32_t ms, adapter_report_t *rep)
{
    adapter_output_t out;
    adapter_step(a, ms, DT, &out, rep);
    return out;
}

static void test_no_command_means_no_link(void)
{
    adapter_t a;
    adapter_report_t r;
    adapter_init(&a, &CFG, 0.0f, 100.0f);

    adapter_output_t o = step(&a, 5000, &r);
    assert(!r.link_ok && !r.limited && !r.stop && !r.powerdown_request);
    assert(o.velocity == 0.0f && near(o.heading, 0.0f) && o.height == 100.0f);
}

static void test_forward_and_turn(void)
{
    adapter_t a;
    adapter_report_t r;
    adapter_init(&a, &CFG, 1.0f, 100.0f);

    cmd_frame_t c = cmd(50.0f, 0.0f, 0.1f);
    c.seq = 9;
    adapter_set_command(&a, &c, 1000);

    adapter_output_t o = step(&a, 1050, &r);
    assert(r.link_ok && !r.limited && r.seq == 9);
    assert(o.velocity == 50.0f);
    assert(near(o.heading, 1.0f + 0.1f * DT));

    /* the heading keeps integrating while the command is held */
    o = step(&a, 1100, &r);
    o = step(&a, 1200, &r);
    assert(r.link_ok);
    assert(near(o.heading, 1.0f + 3.0f * 0.1f * DT));
}

static void test_heading_integration_and_wrap(void)
{
    adapter_t a;
    adapter_report_t r;
    adapter_init(&a, &CFG, 0.02f, 100.0f);

    /* counter-clockwise: wz > 0 raises the heading */
    cmd_frame_t c = cmd(0.0f, 0.0f, 0.1f);
    adapter_set_command(&a, &c, 0);
    adapter_output_t o = step(&a, 0, &r);
    assert(near(o.heading, 0.03f));

    /* clockwise through zero wraps to just below 2 pi, never negative */
    adapter_init(&a, &CFG, 0.005f, 100.0f);
    c = cmd(0.0f, 0.0f, -0.1f);
    adapter_set_command(&a, &c, 0);
    o = step(&a, 0, &r);
    assert(o.heading >= 0.0f && o.heading < TWO_PI);
    assert(near(o.heading, TWO_PI - 0.005f));

    /* counter-clockwise through 2 pi wraps to the start */
    adapter_init(&a, &CFG, TWO_PI - 0.005f, 100.0f);
    c = cmd(0.0f, 0.0f, 0.1f);
    adapter_set_command(&a, &c, 0);
    o = step(&a, 0, &r);
    assert(o.heading >= 0.0f && o.heading < TWO_PI);
    assert(near(o.heading, 0.005f));

    /* the starting heading is brought into range too */
    adapter_init(&a, &CFG, -1.0f, 100.0f);
    o = step(&a, 0, &r);
    assert(near(o.heading, TWO_PI - 1.0f));
}

static void test_clamps_are_reported(void)
{
    adapter_t a;
    adapter_report_t r;
    adapter_init(&a, &CFG, 0.0f, 100.0f);

    adapter_set_command(&a, &(cmd_frame_t) {.vx = 500.0f, .height = 100.0f}, 0);
    adapter_output_t o = step(&a, 0, &r);
    assert(o.velocity == 100.0f && r.limited);

    adapter_init(&a, &CFG, 0.0f, 100.0f);
    adapter_set_command(&a, &(cmd_frame_t) {.wz = 1.0f, .height = 100.0f}, 0);
    o = step(&a, 0, &r);
    assert(near(o.heading, 0.1f * DT) && r.limited);   /* turns at the maximum rate, not faster */

    adapter_set_command(&a, &(cmd_frame_t) {.height = 10.0f}, 0);
    o = step(&a, 0, &r);
    assert(o.height == 50.0f && r.limited);
    adapter_set_command(&a, &(cmd_frame_t) {.height = 300.0f}, 0);
    o = step(&a, 0, &r);
    assert(o.height == 170.0f && r.limited);

    /* exactly at the limits is not limited */
    adapter_set_command(&a, &(cmd_frame_t) {.vx = 100.0f, .wz = -0.1f, .height = 170.0f}, 0);
    o = step(&a, 0, &r);
    assert(!r.limited && o.velocity == 100.0f && o.height == 170.0f);
}

static void test_unsupported_motion_is_dropped_and_reported(void)
{
    adapter_t a;
    adapter_report_t r;
    adapter_init(&a, &CFG, 0.0f, 100.0f);

    /* backward: no motion, reported */
    give(&a, cmd(-30.0f, 0.0f, 0.0f), 0);
    adapter_output_t o = step(&a, 0, &r);
    assert(o.velocity == 0.0f && r.limited);

    /* strafe: the forward part still happens */
    give(&a, cmd(40.0f, 25.0f, 0.0f), 0);
    o = step(&a, 0, &r);
    assert(o.velocity == 40.0f && r.limited);

    /* tilt and another gait */
    cmd_frame_t c = cmd(0.0f, 0.0f, 0.0f);
    c.tilt_p = 0.1f;
    adapter_set_command(&a, &c, 0);
    step(&a, 0, &r);
    assert(r.limited);
    c.tilt_p = 0.0f;
    c.gait = 1;
    adapter_set_command(&a, &c, 0);
    step(&a, 0, &r);
    assert(r.limited);

    /* noise around zero is not a request */
    c = cmd(0.2f, 0.3f, 0.001f);
    adapter_set_command(&a, &c, 0);
    o = step(&a, 0, &r);
    assert(!r.limited);
    c = cmd(-0.3f, -0.4f, -0.001f);
    adapter_set_command(&a, &c, 0);
    o = step(&a, 0, &r);
    assert(!r.limited && o.velocity == 0.0f);
}

static void test_link_timeout(void)
{
    adapter_t a;
    adapter_report_t r;
    adapter_init(&a, &CFG, 2.0f, 120.0f);

    cmd_frame_t c = cmd(60.0f, 0.0f, 0.1f);
    c.height = 140.0f;
    c.seq = 77;
    adapter_set_command(&a, &c, 1000);

    adapter_output_t o = step(&a, 1300, &r);   /* exactly the timeout: still alive */
    assert(r.link_ok && o.velocity == 60.0f);

    float heading_before = o.heading;
    o = step(&a, 1301, &r);                    /* one ms later: gone */
    assert(!r.link_ok);
    assert(o.velocity == 0.0f);
    assert(near(o.heading, heading_before));   /* turning stops, the heading is kept */
    assert(o.height == 140.0f);                /* and so is the height */
    assert(r.seq == 77);                       /* the status still shows the last command */
    assert(!r.powerdown_request && !r.fault_clear);

    /* a new command brings it back */
    c.seq = 78;
    adapter_set_command(&a, &c, 5000);
    o = step(&a, 5050, &r);
    assert(r.link_ok && o.velocity == 60.0f && r.seq == 78);
}

static void test_link_timeout_across_counter_wrap(void)
{
    adapter_t a;
    adapter_report_t r;
    adapter_init(&a, &CFG, 0.0f, 100.0f);

    give(&a, cmd(50.0f, 0.0f, 0.0f), 0xFFFFFF00U);   /* 256 ms before the wrap */
    step(&a, 0x00000020U, &r);                                        /* 288 ms later */
    assert(r.link_ok);
    step(&a, 0x00000040U, &r);                                        /* 320 ms later */
    assert(!r.link_ok);
}

static void test_stop_flag_and_actions(void)
{
    adapter_t a;
    adapter_report_t r;
    adapter_init(&a, &CFG, 0.0f, 100.0f);

    cmd_frame_t c = cmd(50.0f, 0.0f, 0.1f);
    c.flags = CMD_FLAG_STOP;
    adapter_set_command(&a, &c, 0);
    adapter_output_t o = step(&a, 0, &r);
    assert(r.link_ok && r.stop && !r.limited);
    assert(o.velocity == 0.0f && near(o.heading, 0.0f));

    c.flags = CMD_FLAG_POWERDOWN | CMD_FLAG_FAULT_CLEAR;
    adapter_set_command(&a, &c, 0);
    step(&a, 0, &r);
    assert(r.powerdown_request && r.fault_clear && !r.stop);
}

static void test_speed_slew(void)
{
    adapter_cfg_t cfg = CFG;
    cfg.max_accel = 200.0f;   /* 20 mm/s per 100 ms tick */
    adapter_t a;
    adapter_report_t r;
    adapter_init(&a, &cfg, 0.0f, 100.0f);

    give(&a, cmd(50.0f, 0.0f, 0.0f), 0);
    assert(near(step(&a, 0, &r).velocity, 20.0f));
    assert(near(step(&a, 50, &r).velocity, 40.0f));
    assert(near(step(&a, 100, &r).velocity, 50.0f));     /* does not overshoot */
    assert(near(step(&a, 150, &r).velocity, 50.0f));

    /* stop flag decelerates the same way */
    cmd_frame_t c = cmd(50.0f, 0.0f, 0.0f);
    c.flags = CMD_FLAG_STOP;
    adapter_set_command(&a, &c, 150);
    assert(near(step(&a, 200, &r).velocity, 30.0f));

    /* so does a lost link: it brings the robot to a stop in steps, not at once */
    give(&a, cmd(50.0f, 0.0f, 0.0f), 200);
    step(&a, 250, &r);
    step(&a, 300, &r);
    step(&a, 350, &r);
    float v = step(&a, 400, &r).velocity;
    assert(v > 40.0f);
    adapter_output_t o = step(&a, 600, &r);   /* link is gone */
    assert(!r.link_ok && near(o.velocity, v - 20.0f));
    o = step(&a, 700, &r);
    assert(near(o.velocity, v - 40.0f));
    for (int i = 0; i < 5; i++) {
        o = step(&a, 800 + (uint32_t) i * 100, &r);
    }
    assert(o.velocity == 0.0f);
}

int main(void)
{
    test_no_command_means_no_link();
    test_forward_and_turn();
    test_heading_integration_and_wrap();
    test_clamps_are_reported();
    test_unsupported_motion_is_dropped_and_reported();
    test_link_timeout();
    test_link_timeout_across_counter_wrap();
    test_stop_flag_and_actions();
    test_speed_slew();
    printf("command_adapter_test: all tests passed\n");
    return 0;
}
