/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Colours, modes and commands of the RGB LED. No RTOS or HAL dependency, so the logic in
 * rgb_led_core.c builds on the host (tests/rgb_led_core_test.c). */

#ifndef RGB_LED_TYPES_H
#define RGB_LED_TYPES_H

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

/* Command queue */
typedef enum {
    RGB_LED_CMD_SET_COLOR,
    RGB_LED_CMD_SET_MODE,
    RGB_LED_CMD_SET_BLINK
} rgb_led_cmd_type_t;

typedef struct {
    rgb_led_cmd_type_t type;
    union {
        rgb_led_color_t color;
        rgb_led_mode_t  mode;
        struct {
            uint32_t period_ms;
            float    duty_cycle;
        } blink;
    } data;
} rgb_led_cmd_t;

#endif // RGB_LED_TYPES_H
