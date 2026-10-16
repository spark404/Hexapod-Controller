/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "command_adapter.h"

#include <math.h>

#define TWO_PI 6.28318530717958647692f

/* Below these a command component counts as zero: joystick noise is no reason to
 * tell the sender it asked for something that is not supported. */
#define EPS_VELOCITY 0.5f   /* mm/s */
#define EPS_RATE 0.005f     /* rad/s */
#define EPS_TILT 0.001f     /* rad */

static float clamp(float v, float lo, float hi, bool *limited)
{
    if (v < lo) {
        *limited = true;
        return lo;
    }
    if (v > hi) {
        *limited = true;
        return hi;
    }
    return v;
}

static float wrap_two_pi(float angle)
{
    angle = fmodf(angle, TWO_PI);
    if (angle < 0.0f) {
        angle += TWO_PI;
    }
    /* fmodf of a tiny negative number plus 2 pi can round up to exactly 2 pi */
    if (angle >= TWO_PI) {
        angle = 0.0f;
    }
    return angle;
}

void adapter_init(adapter_t *a, const adapter_cfg_t *cfg, float heading, float height)
{
    a->cfg = *cfg;
    a->heading = wrap_two_pi(heading);
    a->height = height;
    a->velocity = 0.0f;
    a->have_cmd = false;
    a->cmd = (cmd_frame_t) {0};
    a->cmd_ms = 0;
}

void adapter_set_command(adapter_t *a, const cmd_frame_t *cmd, uint32_t rx_ms)
{
    a->cmd = *cmd;
    a->cmd_ms = rx_ms;
    a->have_cmd = true;
}

void adapter_step(adapter_t *a, uint32_t now_ms, float dt_s, adapter_output_t *out, adapter_report_t *report)
{
    const adapter_cfg_t *cfg = &a->cfg;
    adapter_report_t rep = {0};

    /* Unsigned difference: correct across the wrap of the millisecond counter */
    rep.link_ok = a->have_cmd && (uint32_t) (now_ms - a->cmd_ms) <= cfg->link_timeout_ms;

    float target_speed = 0.0f;
    float yaw_rate = 0.0f;

    if (a->have_cmd) {
        rep.seq = a->cmd.seq;   /* the status keeps showing the last command applied */
    }

    if (rep.link_ok) {
        const cmd_frame_t *c = &a->cmd;
        bool limited = false;

        rep.stop = (c->flags & CMD_FLAG_STOP) != 0U;
        rep.powerdown_request = (c->flags & CMD_FLAG_POWERDOWN) != 0U;
        rep.fault_clear = (c->flags & CMD_FLAG_FAULT_CLEAR) != 0U;

        if (!rep.stop) {
            /* Backward is not possible: a negative vx is dropped, not mirrored */
            if (c->vx < -EPS_VELOCITY) {
                limited = true;
            } else {
                target_speed = clamp(c->vx < 0.0f ? 0.0f : c->vx, 0.0f, cfg->max_speed, &limited);
            }
            yaw_rate = clamp(c->wz, -cfg->max_yaw_rate, cfg->max_yaw_rate, &limited);

            if (fabsf(c->vy) > EPS_VELOCITY) {
                limited = true;    /* no strafe yet */
            }
        }

        if (fabsf(c->tilt_r) > EPS_TILT || fabsf(c->tilt_p) > EPS_TILT || c->gait != CMD_GAIT_TRIPOD) {
            limited = true;
        }

        a->height = clamp(c->height, cfg->min_height, cfg->max_height, &limited);
        rep.limited = limited;
    }
    /* Without a link the twist is zero and the height and heading stay as they are */

    /* Speed follows the target at a limited rate, up and down: a start does not jerk
     * the robot, and a lost link brings it to a stop over a short distance. */
    if (cfg->max_accel > 0.0f) {
        const float step = cfg->max_accel * dt_s;
        if (target_speed > a->velocity + step) {
            a->velocity += step;
        } else if (target_speed < a->velocity - step) {
            a->velocity -= step;
        } else {
            a->velocity = target_speed;
        }
    } else {
        a->velocity = target_speed;
    }

    a->heading = wrap_two_pi(a->heading + yaw_rate * dt_s);

    out->velocity = a->velocity;
    out->heading = a->heading;
    out->height = a->height;
    *report = rep;
}
