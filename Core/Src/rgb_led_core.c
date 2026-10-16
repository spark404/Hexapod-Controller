/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rgb_led_core.h"

#define DEFAULT_PERIOD_MS 500U

static uint32_t on_ms(const rgb_led_core_t *c)
{
    const uint32_t period = c->period_ms != 0U ? c->period_ms : DEFAULT_PERIOD_MS;
    const uint32_t on = (uint32_t) ((float) period * c->duty + 0.5f);
    return on > period ? period : on;
}

static uint32_t off_ms(const rgb_led_core_t *c)
{
    const uint32_t period = c->period_ms != 0U ? c->period_ms : DEFAULT_PERIOD_MS;
    return period - on_ms(c);
}

/* A blink that flips: both phases have a length. Otherwise it is steady on or steady off. */
static bool flips(const rgb_led_core_t *c)
{
    return c->mode == RGB_LED_MODE_BLINK && on_ms(c) != 0U && off_ms(c) != 0U;
}

static void start_blink(rgb_led_core_t *c, uint32_t now_ms)
{
    c->lit = on_ms(c) != 0U;
    c->next_ms = now_ms + (c->lit ? on_ms(c) : off_ms(c));
}

void rgb_led_core_init(rgb_led_core_t *c)
{
    c->color = (rgb_led_color_t) {0, 0, 0};
    c->mode = RGB_LED_MODE_OFF;
    c->period_ms = DEFAULT_PERIOD_MS;
    c->duty = 0.5f;
    c->lit = false;
    c->next_ms = 0;
}

void rgb_led_core_apply(rgb_led_core_t *c, const rgb_led_cmd_t *cmd, uint32_t now_ms)
{
    switch (cmd->type) {
        case RGB_LED_CMD_SET_COLOR:
            c->color = cmd->data.color;
            break;
        case RGB_LED_CMD_SET_MODE:
            c->mode = cmd->data.mode;
            if (c->mode == RGB_LED_MODE_BLINK) {
                start_blink(c, now_ms);
            }
            break;
        case RGB_LED_CMD_SET_BLINK:
            c->period_ms = cmd->data.blink.period_ms;
            c->duty = cmd->data.blink.duty_cycle;
            if (c->duty < 0.0f) {
                c->duty = 0.0f;
            }
            if (c->duty > 1.0f) {
                c->duty = 1.0f;
            }
            if (c->mode == RGB_LED_MODE_BLINK) {
                start_blink(c, now_ms);
            }
            break;
        default:
            break;
    }
}

void rgb_led_core_tick(rgb_led_core_t *c, uint32_t now_ms)
{
    if (!flips(c)) {
        return;
    }
    if ((int32_t) (now_ms - c->next_ms) < 0) {
        return;
    }

    const uint32_t period = on_ms(c) + off_ms(c);
    c->lit = !c->lit;
    c->next_ms += c->lit ? on_ms(c) : off_ms(c);
    if ((uint32_t) (now_ms - c->next_ms) < 0x80000000U && (uint32_t) (now_ms - c->next_ms) >= period) {
        /* More than a whole cycle late: carry on from now */
        c->next_ms = now_ms + (c->lit ? on_ms(c) : off_ms(c));
    }
}

rgb_led_color_t rgb_led_core_output(const rgb_led_core_t *c)
{
    switch (c->mode) {
        case RGB_LED_MODE_ON:
            return c->color;
        case RGB_LED_MODE_BLINK:
            if (off_ms(c) == 0U) {
                return c->color;                    /* duty 1: steady */
            }
            return c->lit && on_ms(c) != 0U ? c->color : (rgb_led_color_t) {0, 0, 0};
        case RGB_LED_MODE_OFF:
        default:
            return (rgb_led_color_t) {0, 0, 0};
    }
}

uint32_t rgb_led_core_wait_ms(const rgb_led_core_t *c, uint32_t now_ms)
{
    if (!flips(c)) {
        return RGB_LED_WAIT_FOREVER;
    }
    const int32_t left = (int32_t) (c->next_ms - now_ms);
    return left > 0 ? (uint32_t) left : 0U;
}
