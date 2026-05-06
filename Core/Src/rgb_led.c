/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rgb_led.h"

#include "cmsis_os2.h"
#include "main.h"            // For ST_LED_R/G/B_Pin and _GPIO_Port (CubeMX generated)
#include "stm32f4xx_hal.h"   // For GPIO / HAL functions

/* ---- External Command Queue ---- */
extern osMessageQueueId_t rgb_led_queueHandle;

/* ---------- Hardware-specific section ---------- */
/* Track the CubeMX-generated pin labels so hardware revisions stay in sync. */

#define RGB_LED_R_GPIO_Port   ST_LED_R_GPIO_Port
#define RGB_LED_R_Pin         ST_LED_R_Pin

#define RGB_LED_G_GPIO_Port   ST_LED_G_GPIO_Port
#define RGB_LED_G_Pin         ST_LED_G_Pin

#define RGB_LED_B_GPIO_Port   ST_LED_B_GPIO_Port
#define RGB_LED_B_Pin         ST_LED_B_Pin

/* If your LED is active-high, set ON to GPIO_PIN_SET.
   If it is active-low, swap SET/RESET. */
#define LED_ON_LEVEL          GPIO_PIN_RESET
#define LED_OFF_LEVEL         GPIO_PIN_SET

static void rgb_led_hw_set_raw(uint8_t r_on, uint8_t g_on, uint8_t b_on)
{
    HAL_GPIO_WritePin(RGB_LED_R_GPIO_Port, RGB_LED_R_Pin,
                      r_on ? LED_ON_LEVEL : LED_OFF_LEVEL);
    HAL_GPIO_WritePin(RGB_LED_G_GPIO_Port, RGB_LED_G_Pin,
                      g_on ? LED_ON_LEVEL : LED_OFF_LEVEL);
    HAL_GPIO_WritePin(RGB_LED_B_GPIO_Port, RGB_LED_B_Pin,
                      b_on ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

/* Map 0-255 brightness to on/off; if you have PWM, replace this
   with a function that sets duty cycle. */
static void rgb_led_hw_set_color(rgb_led_color_t color)
{
    rgb_led_hw_set_raw(color.r > 0, color.g > 0, color.b > 0);
}

/* ---------- Internal command queue ---------- */

/* State owned by the task */
static rgb_led_color_t s_current_color = { 0, 0, 0 };
static rgb_led_mode_t  s_current_mode  = RGB_LED_MODE_OFF;
static uint32_t        s_blink_period_ms = 500;
static float           s_blink_duty      = 0.5f;

/* ---------- Task function ---------- */

void rgb_led_task(void *argument)
{
    (void)argument;

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t default_wait = pdMS_TO_TICKS(50); // base tick for blink

    for (;;) {
        rgb_led_cmd_t cmd;

        /* Block until a command arrives. osMessageQueueGet returns osOK (0)
           on success -- not pdPASS (1). The earlier comparison silently
           dropped every command. */
        if (osMessageQueueGet(rgb_led_queueHandle, &cmd, NULL, osWaitForever) == osOK) {
            switch (cmd.type) {
                case RGB_LED_CMD_SET_COLOR:
                    s_current_color = cmd.data.color;
                    break;
                case RGB_LED_CMD_SET_MODE:
                    s_current_mode = cmd.data.mode;
                    break;
                case RGB_LED_CMD_SET_BLINK:
                    s_blink_period_ms = cmd.data.blink.period_ms;
                    s_blink_duty      = cmd.data.blink.duty_cycle;
                    if (s_blink_duty < 0.0f) s_blink_duty = 0.0f;
                    if (s_blink_duty > 1.0f) s_blink_duty = 1.0f;
                    break;
                default:
                    break;
            }
        }

        /* Drive LED according to current mode and time */
        switch (s_current_mode) {
            case RGB_LED_MODE_OFF:
                rgb_led_hw_set_raw(0, 0, 0);
                vTaskDelayUntil(&last_wake_time, default_wait);
                break;

            case RGB_LED_MODE_ON:
                rgb_led_hw_set_color(s_current_color);
                vTaskDelayUntil(&last_wake_time, default_wait);
                break;

            case RGB_LED_MODE_BLINK: {
                uint32_t period = (s_blink_period_ms > 0) ?
                                  s_blink_period_ms : 500;
                TickType_t period_ticks = pdMS_TO_TICKS(period);
                TickType_t on_ticks  = (TickType_t)((float)period_ticks *
                                                    s_blink_duty);
                TickType_t off_ticks = (on_ticks < period_ticks)
                                       ? (period_ticks - on_ticks)
                                       : 0;

                /* ON phase */
                if (on_ticks > 0) {
                    rgb_led_hw_set_color(s_current_color);
                    vTaskDelayUntil(&last_wake_time, on_ticks);
                }

                /* OFF phase */
                if (off_ticks > 0) {
                    rgb_led_hw_set_raw(0, 0, 0);
                    vTaskDelayUntil(&last_wake_time, off_ticks);
                }
                break;
            }

            default:
                vTaskDelayUntil(&last_wake_time, default_wait);
                break;
        }
    }
}

/* ---------- Public API ---------- */

static void rgb_led_send_cmd(const rgb_led_cmd_t *cmd)
{
    if (rgb_led_queueHandle != NULL) {
        /* Don't block the caller (controller callback) if the LED task is
           wedged or behind -- the LED is cosmetic and a dropped state-change
           frame is far less harmful than stalling the controller. */
        (void)osMessageQueuePut(rgb_led_queueHandle, cmd, 0, 0);
    }
}

void rgb_led_set_color(rgb_led_color_t color)
{
    rgb_led_cmd_t cmd = {
        .type  = RGB_LED_CMD_SET_COLOR,
        .data.color = color
    };
    rgb_led_send_cmd(&cmd);
}


void rgb_led_on(void)
{
    rgb_led_cmd_t cmd;
    cmd.type = RGB_LED_CMD_SET_MODE;
    cmd.data.mode = RGB_LED_MODE_ON;
    rgb_led_send_cmd(&cmd);
}

void rgb_led_off(void)
{
    rgb_led_cmd_t cmd;
    cmd.type = RGB_LED_CMD_SET_MODE;
    cmd.data.mode = RGB_LED_MODE_OFF;
    rgb_led_send_cmd(&cmd);
}

void rgb_led_blink(uint32_t period_ms, float duty_cycle)
{
    rgb_led_cmd_t cmd;
    cmd.type = RGB_LED_CMD_SET_BLINK;
    cmd.data.blink.period_ms = period_ms;
    cmd.data.blink.duty_cycle = duty_cycle;
    rgb_led_send_cmd(&cmd);

    /* Also ensure mode is set to BLINK */
    cmd.type = RGB_LED_CMD_SET_MODE;
    cmd.data.mode = RGB_LED_MODE_BLINK;
    rgb_led_send_cmd(&cmd);
}