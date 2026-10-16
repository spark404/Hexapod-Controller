/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The logic of the RGB LED: which colour is shown now, and how long the task may sleep before the
 * LED has to change. No RTOS or HAL dependency, host test in tests/rgb_led_core_test.c.
 *
 * Used by the LED task (rgb_led.c) as:
 *
 *     wait   = rgb_led_core_wait_ms(&core, now);          sleep on the command queue for this long
 *     ...    a command arrives, or the wait ends ...
 *     rgb_led_core_apply(&core, &cmd, now);               for every command
 *     rgb_led_core_tick(&core, now);                      moves a blink on when it is due
 *     show(rgb_led_core_output(&core));
 *
 * Blink: the cycle starts with the ON phase when the mode is set to BLINK or the blink settings
 * change. The phases are on = period x duty, off = the rest. A duty of 0 shows nothing, a duty of 1
 * shows the colour steadily. Changing the colour keeps the phase. Times are milliseconds of a free
 * running 32 bit counter; wrap around is handled, and a late tick does not make up for missed
 * cycles in a burst.
 */

#ifndef RGB_LED_CORE_H
#define RGB_LED_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "rgb_led_types.h"

/* wait_ms() result: nothing changes by itself, wait for the next command */
#define RGB_LED_WAIT_FOREVER 0xFFFFFFFFU

typedef struct {
    rgb_led_color_t color;
    rgb_led_mode_t mode;
    uint32_t period_ms;
    float duty;
    bool lit;           /* BLINK: the ON phase is running */
    uint32_t next_ms;   /* BLINK: when the phase flips */
} rgb_led_core_t;

void rgb_led_core_init(rgb_led_core_t *c);
void rgb_led_core_apply(rgb_led_core_t *c, const rgb_led_cmd_t *cmd, uint32_t now_ms);
void rgb_led_core_tick(rgb_led_core_t *c, uint32_t now_ms);
rgb_led_color_t rgb_led_core_output(const rgb_led_core_t *c);
uint32_t rgb_led_core_wait_ms(const rgb_led_core_t *c, uint32_t now_ms);

#endif /* RGB_LED_CORE_H */
