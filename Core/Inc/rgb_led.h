/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RGB_LED_H
#define RGB_LED_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>

#include "rgb_led_types.h"

/* Initialize module, create task and queue */
void rgb_led_task(void *pvParameters);

/* Control functions (thread-safe, use queue internally) */
void rgb_led_set_color(rgb_led_color_t color);
void rgb_led_on(void);
void rgb_led_off(void);

/* Blink with given period (ms) and duty cycle 0.0-1.0 */
void rgb_led_blink(uint32_t period_ms, float duty_cycle);

#endif // RGB_LED_H
