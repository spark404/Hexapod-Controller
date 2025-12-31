#include "rgb_led.h"
#include "stm32f4xx_hal.h"   // For GPIO / HAL functions

/* ---------- Hardware-specific section ---------- */
/* Adjust these to your board: pins, ports, active level, etc. */

#define RGB_LED_R_GPIO_Port   GPIOA
#define RGB_LED_R_Pin         GPIO_PIN_15

#define RGB_LED_G_GPIO_Port   GPIOC
#define RGB_LED_G_Pin         GPIO_PIN_10

#define RGB_LED_B_GPIO_Port   GPIOA
#define RGB_LED_B_Pin         GPIO_PIN_14

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

static TaskHandle_t    s_rgb_led_task_handle = NULL;
static QueueHandle_t   s_rgb_led_queue       = NULL;

/* State owned by the task */
static rgb_led_color_t s_current_color = { 0, 0, 0 };
static rgb_led_mode_t  s_current_mode  = RGB_LED_MODE_OFF;
static uint32_t        s_blink_period_ms = 500;
static float           s_blink_duty      = 0.5f;

/* ---------- Task function ---------- */

static void rgb_led_task(void *argument)
{
    (void)argument;

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t default_wait = pdMS_TO_TICKS(50); // base tick for blink

    for (;;) {
        rgb_led_cmd_t cmd;

        /* Non-blocking check for new commands; we still want periodic timing.
           You can use xQueueReceive with timeout if you prefer. */
        if (xQueueReceive(s_rgb_led_queue, &cmd, 0) == pdPASS) {
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

void rgb_led_init(void)
{
    /* Create queue */
    s_rgb_led_queue = xQueueCreate(8, sizeof(rgb_led_cmd_t));
    if (s_rgb_led_queue == NULL) {
        /* handle error, e.g. assert */
        return;
    }

    /* Create task */
    BaseType_t rc = xTaskCreate(
        rgb_led_task,
        "RGB_LED",
        256,        /* stack size in words; tune as needed */
        NULL,
        tskIDLE_PRIORITY + 1,
        &s_rgb_led_task_handle
    );

    if (rc != pdPASS) {
        /* handle error, e.g. assert */
    }
}

static void rgb_led_send_cmd(const rgb_led_cmd_t *cmd)
{
    if (s_rgb_led_queue != NULL) {
        (void)xQueueSend(s_rgb_led_queue, cmd, portMAX_DELAY);
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