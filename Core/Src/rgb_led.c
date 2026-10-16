/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rgb_led.h"

#include "cmsis_os2.h"
#include "main.h"            // For ST_LED_R/G/B_Pin and _GPIO_Port (CubeMX generated)
#include "rgb_led_core.h"
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

static void rgb_led_hw_set_color(rgb_led_color_t color)
{
    /* On or off per colour: there is no PWM on these pins */
    HAL_GPIO_WritePin(RGB_LED_R_GPIO_Port, RGB_LED_R_Pin, color.r > 0 ? LED_ON_LEVEL : LED_OFF_LEVEL);
    HAL_GPIO_WritePin(RGB_LED_G_GPIO_Port, RGB_LED_G_Pin, color.g > 0 ? LED_ON_LEVEL : LED_OFF_LEVEL);
    HAL_GPIO_WritePin(RGB_LED_B_GPIO_Port, RGB_LED_B_Pin, color.b > 0 ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

static uint32_t now_ms(void)
{
    return (uint32_t) osKernelGetTickCount() * portTICK_PERIOD_MS;
}

/* ---------- Task function ---------- */

/*
 * Sleeps on the command queue until a command arrives or the LED has to change by itself (the
 * next phase of a blink), applies all commands that are waiting, and shows the result. The LED is
 * left alone until the first command: until then it shows what main() set at the end of the
 * start-up.
 */
void rgb_led_task(void *argument)
{
    (void)argument;

    rgb_led_core_t core;
    rgb_led_core_init(&core);

    for (;;) {
        const uint32_t wait = rgb_led_core_wait_ms(&core, now_ms());
        rgb_led_cmd_t cmd;

        /* osMessageQueueGet returns osOK (0) on success -- not pdPASS (1) */
        osStatus_t st = osMessageQueueGet(rgb_led_queueHandle, &cmd, NULL,
                                          wait == RGB_LED_WAIT_FOREVER ? osWaitForever : wait);
        while (st == osOK) {
            rgb_led_core_apply(&core, &cmd, now_ms());
            /* A colour and a blink arrive together: take them all before showing anything */
            st = osMessageQueueGet(rgb_led_queueHandle, &cmd, NULL, 0);
        }

        rgb_led_core_tick(&core, now_ms());
        rgb_led_hw_set_color(rgb_led_core_output(&core));
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
