/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Adapter from the twist command of the SPI link (cmd_frame.h) to what the
 * controller accepts today: a speed along the body's own heading, an absolute
 * heading to turn towards, and a body height (controller_command_t).
 *
 * It exists so the link can have its final form before the motion core does
 * (backlog T7.5 now, T7.8 later): when the controller takes a twist itself this
 * module is deleted and the command goes straight through. Until then:
 *
 *   - speed is max(vx, 0): backward is not possible,
 *   - vy (strafe), tilt and a gait other than tripod are not possible,
 *   - the yaw rate wz is integrated into the heading here, so the heading lives
 *     in one place and survives a restart of the Pi,
 *   - what could not be done is reported (adapter_report_t.limited), and the
 *     status frame passes it on, so the sender knows.
 *
 * It also enforces what the controller should not have to: limits on speed,
 * yaw rate and height, a limit on how fast the speed changes, and the link
 * timeout (design 3.3): without a new command for link_timeout_ms the twist is
 * zero, the robot stops and stands.
 *
 * No HAL, RTOS or controller dependency, builds on the host
 * (tests/command_adapter_test.c). Not thread safe: call everything from the
 * control task, handing over commands received by the SPI task through the
 * shared command (slice 3). Times are milliseconds from a free running 32 bit
 * counter; wrap around is handled.
 */

#ifndef COMMAND_ADAPTER_H
#define COMMAND_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

#include "cmd_frame.h"

typedef struct {
    float max_speed;          /* mm/s, forward */
    float max_yaw_rate;       /* rad/s */
    float min_height;         /* mm */
    float max_height;         /* mm */
    float max_accel;          /* mm/s^2 change of speed, 0 = no limit */
    uint32_t link_timeout_ms; /* no command for this long: zero twist */
} adapter_cfg_t;

typedef struct {
    float velocity;   /* mm/s, >= 0, along the heading of the robot */
    float heading;    /* rad, [0, 2 pi) */
    float height;     /* mm */
} adapter_output_t;

typedef struct {
    bool link_ok;
    bool limited;            /* part of the command was clamped or is not supported */
    bool stop;               /* CMD_FLAG_STOP is set in the command being applied */
    bool powerdown_request;  /* CMD_FLAG_POWERDOWN is set (only while the link is ok) */
    bool fault_clear;        /* CMD_FLAG_FAULT_CLEAR is set (only while the link is ok) */
    uint8_t seq;             /* seq of the command last applied, for the status frame */
} adapter_report_t;

typedef struct {
    adapter_cfg_t cfg;
    float heading;
    float height;
    float velocity;
    bool have_cmd;
    cmd_frame_t cmd;
    uint32_t cmd_ms;
} adapter_t;

/* Start with the robot's current heading and height (the controller starts
 * at yaw 0 and CTRL_BODY_Z). No command yet counts as no link. */
void adapter_init(adapter_t *a, const adapter_cfg_t *cfg, float heading, float height);

/* A new command was received at rx_ms. Hand over only CMD_RX_NEW frames. */
void adapter_set_command(adapter_t *a, const cmd_frame_t *cmd, uint32_t rx_ms);

/* Advance by dt_s (the control period) at time now_ms and produce the command
 * for the controller. */
void adapter_step(adapter_t *a, uint32_t now_ms, float dt_s, adapter_output_t *out, adapter_report_t *report);

#endif /* COMMAND_ADAPTER_H */
