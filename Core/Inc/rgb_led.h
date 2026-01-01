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

typedef enum {
    RGB_LED_MODE_OFF = 0,
    RGB_LED_MODE_ON,
    RGB_LED_MODE_BLINK
} rgb_led_mode_t;

typedef struct {
    uint8_t r; // 0-255
    uint8_t g; // 0-255
    uint8_t b; // 0-255
} rgb_led_color_t;

/* Predefined colors (RGB and CMY) */
static const rgb_led_color_t RGB_LED_COLOR_BLACK   = { 0x00, 0x00, 0x00 };
static const rgb_led_color_t RGB_LED_COLOR_RED     = { 0xFF, 0x00, 0x00 };
static const rgb_led_color_t RGB_LED_COLOR_GREEN   = { 0x00, 0xFF, 0x00 };
static const rgb_led_color_t RGB_LED_COLOR_BLUE    = { 0x00, 0x00, 0xFF };

static const rgb_led_color_t RGB_LED_COLOR_CYAN    = { 0x00, 0xFF, 0xFF };
static const rgb_led_color_t RGB_LED_COLOR_MAGENTA = { 0xFF, 0x00, 0xFF };
static const rgb_led_color_t RGB_LED_COLOR_YELLOW  = { 0xFF, 0xFF, 0x00 };
static const rgb_led_color_t RGB_LED_COLOR_WHITE   = { 0xFF, 0xFF, 0xFF };

/* Initialize module, create task and queue */
void rgb_led_init(void);

/* Control functions (thread-safe, use queue internally) */
void rgb_led_set_color(rgb_led_color_t color);
void rgb_led_on(void);
void rgb_led_off(void);

/* Blink with given period (ms) and duty cycle 0.0-1.0 */
void rgb_led_blink(uint32_t period_ms, float duty_cycle);

#endif // RGB_LED_H
