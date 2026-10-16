//
// Host test for the logic of the RGB LED, see Core/Src/rgb_led_core.c
//
// Build and run from the Hexapod-Controller directory:
//
//   cc -std=c99 -Wall -Wextra -pedantic -Werror -ICore/Inc \
//      tests/rgb_led_core_test.c Core/Src/rgb_led_core.c -o /tmp/rgb_led_core_test && /tmp/rgb_led_core_test
//
#include <assert.h>
#include <stdio.h>

#include "rgb_led_core.h"

static const rgb_led_color_t CYAN = {0x00, 0xFF, 0xFF};
static const rgb_led_color_t GREEN = {0x00, 0xFF, 0x00};

static bool same(rgb_led_color_t a, rgb_led_color_t b)
{
    return a.r == b.r && a.g == b.g && a.b == b.b;
}

static bool dark(const rgb_led_core_t *c)
{
    return same(rgb_led_core_output(c), (rgb_led_color_t) {0, 0, 0});
}

static void send(rgb_led_core_t *c, rgb_led_cmd_t cmd, uint32_t now)
{
    rgb_led_core_apply(c, &cmd, now);
}

static void set_color(rgb_led_core_t *c, rgb_led_color_t col, uint32_t now)
{
    send(c, (rgb_led_cmd_t) {.type = RGB_LED_CMD_SET_COLOR, .data.color = col}, now);
}

static void set_mode(rgb_led_core_t *c, rgb_led_mode_t m, uint32_t now)
{
    send(c, (rgb_led_cmd_t) {.type = RGB_LED_CMD_SET_MODE, .data.mode = m}, now);
}

static void set_blink(rgb_led_core_t *c, uint32_t period, float duty, uint32_t now)
{
    send(c, (rgb_led_cmd_t) {.type = RGB_LED_CMD_SET_BLINK, .data.blink = {period, duty}}, now);
}

/* What the controller callback sends on a state change: rgb_led_set_color() then rgb_led_blink() */
static void state_change(rgb_led_core_t *c, rgb_led_color_t col, uint32_t now)
{
    set_color(c, col, now);
    set_blink(c, 500, 0.5f, now);
    set_mode(c, RGB_LED_MODE_BLINK, now);
}

static void test_off_on_and_waiting(void)
{
    rgb_led_core_t c;
    rgb_led_core_init(&c);
    assert(dark(&c) && rgb_led_core_wait_ms(&c, 0) == RGB_LED_WAIT_FOREVER);

    set_color(&c, GREEN, 10);
    assert(dark(&c));                                          /* the mode is still off */
    set_mode(&c, RGB_LED_MODE_ON, 10);
    assert(same(rgb_led_core_output(&c), GREEN) && rgb_led_core_wait_ms(&c, 10) == RGB_LED_WAIT_FOREVER);
    rgb_led_core_tick(&c, 100000);
    assert(same(rgb_led_core_output(&c), GREEN));              /* steady, however long it takes */

    set_color(&c, CYAN, 20);
    assert(same(rgb_led_core_output(&c), CYAN));
    set_mode(&c, RGB_LED_MODE_OFF, 30);
    assert(dark(&c));
}

static void test_blink_cycle_and_deadlines(void)
{
    rgb_led_core_t c;
    rgb_led_core_init(&c);
    state_change(&c, CYAN, 1000);

    assert(same(rgb_led_core_output(&c), CYAN));               /* the cycle starts with the on phase */
    assert(rgb_led_core_wait_ms(&c, 1000) == 250);

    rgb_led_core_tick(&c, 1249);
    assert(same(rgb_led_core_output(&c), CYAN) && rgb_led_core_wait_ms(&c, 1249) == 1);
    rgb_led_core_tick(&c, 1250);
    assert(dark(&c) && rgb_led_core_wait_ms(&c, 1250) == 250);
    rgb_led_core_tick(&c, 1500);
    assert(same(rgb_led_core_output(&c), CYAN));

    /* ten cycles, polled late by up to 30 ms every time: the flips stay on the 250 ms grid */
    uint32_t flips = 0;
    bool last = true;
    for (uint32_t t = 1500; t <= 1500 + 5000; t += 30) {
        rgb_led_core_tick(&c, t);
        const bool lit = !dark(&c);
        if (lit != last) {
            flips++;
            last = lit;
        }
    }
    assert(flips >= 19 && flips <= 21);
    rgb_led_core_tick(&c, 6500);
    assert(rgb_led_core_wait_ms(&c, 6500) <= 250);
}

static void test_commands_of_a_state_change_do_not_flash_off(void)
{
    rgb_led_core_t c;
    rgb_led_core_init(&c);
    state_change(&c, GREEN, 0);
    state_change(&c, CYAN, 4000);                              /* the next state change: new colour at once */
    assert(same(rgb_led_core_output(&c), CYAN) && rgb_led_core_wait_ms(&c, 4000) == 250);

    /* a colour change on its own keeps the phase */
    rgb_led_core_tick(&c, 4250);
    assert(dark(&c));
    set_color(&c, GREEN, 4300);
    assert(dark(&c) && rgb_led_core_wait_ms(&c, 4300) == 200);
    rgb_led_core_tick(&c, 4500);
    assert(same(rgb_led_core_output(&c), GREEN));
}

static void test_duty_extremes_and_period_defaults(void)
{
    rgb_led_core_t c;
    rgb_led_core_init(&c);
    set_color(&c, CYAN, 0);
    set_mode(&c, RGB_LED_MODE_BLINK, 0);

    set_blink(&c, 400, 0.0f, 0);
    assert(dark(&c) && rgb_led_core_wait_ms(&c, 0) == RGB_LED_WAIT_FOREVER);   /* nothing to show */
    set_blink(&c, 400, 1.0f, 0);
    assert(same(rgb_led_core_output(&c), CYAN) && rgb_led_core_wait_ms(&c, 0) == RGB_LED_WAIT_FOREVER);
    set_blink(&c, 400, 7.0f, 0);                               /* clamped to 1 */
    assert(same(rgb_led_core_output(&c), CYAN) && rgb_led_core_wait_ms(&c, 0) == RGB_LED_WAIT_FOREVER);
    set_blink(&c, 400, -2.0f, 0);                              /* clamped to 0 */
    assert(dark(&c));
    set_blink(&c, 0, 0.5f, 100);                               /* period 0: the default of 500 ms */
    assert(same(rgb_led_core_output(&c), CYAN) && rgb_led_core_wait_ms(&c, 100) == 250);
    set_blink(&c, 100, 0.25f, 0);
    assert(rgb_led_core_wait_ms(&c, 0) == 25);
}

static void test_no_burst_after_a_stall_and_counter_wrap(void)
{
    rgb_led_core_t c;
    rgb_led_core_init(&c);
    state_change(&c, CYAN, 0);
    rgb_led_core_tick(&c, 100000);                             /* the task did not run for 100 s */
    const uint32_t wait = rgb_led_core_wait_ms(&c, 100000);
    assert(wait > 0 && wait <= 250);                           /* carries on from now, one phase at a time */

    rgb_led_core_init(&c);
    state_change(&c, CYAN, 0xFFFFFF00U);                       /* 256 ms before the wrap */
    assert(rgb_led_core_wait_ms(&c, 0xFFFFFF00U) == 250);
    rgb_led_core_tick(&c, 0xFFFFFFF9U);
    assert(!dark(&c));                                         /* 1 ms before the flip at 0xFFFFFFFA: still lit */
    rgb_led_core_tick(&c, 0x00000004U);                        /* 10 ms late, through the wrap */
    assert(dark(&c) && rgb_led_core_wait_ms(&c, 0x00000004U) == 240);   /* the next flip stays on the grid */
}

static void test_unknown_commands_are_ignored(void)
{
    rgb_led_core_t c;
    rgb_led_core_init(&c);
    state_change(&c, CYAN, 0);
    rgb_led_cmd_t bogus = {.type = (rgb_led_cmd_type_t) 99};
    rgb_led_core_apply(&c, &bogus, 10);
    assert(same(rgb_led_core_output(&c), CYAN) && rgb_led_core_wait_ms(&c, 10) == 240);
}

/* The loop of the LED task (rgb_led.c) on a made up clock and queue: sleep for the wait of the core or until the
 * next command, apply every command that is waiting, tick, show. Returns what the LED showed, as transitions. */
typedef struct { uint32_t t; rgb_led_cmd_t cmd; } timed_cmd_t;
typedef struct { uint32_t t; char c; } shown_t;   /* c: 'C' cyan, 'G' green, 'B' blue, '-' dark */

static char symbol(rgb_led_color_t col)
{
    if (col.r == 0 && col.g == 0 && col.b == 0) return '-';
    if (col.r == 0 && col.g != 0 && col.b != 0) return 'C';
    if (col.r == 0 && col.g != 0) return 'G';
    return 'B';
}

static unsigned run_task(const timed_cmd_t *cmds, unsigned n, uint32_t end, shown_t *out, unsigned max)
{
    rgb_led_core_t core;
    rgb_led_core_init(&core);
    uint32_t now = 0;
    unsigned next = 0, count = 0;
    char last = '?';                                           /* the boot colour is on until the first command */

    while (now < end) {
        const uint32_t wait = rgb_led_core_wait_ms(&core, now);
        uint32_t wake = wait == RGB_LED_WAIT_FOREVER ? end : now + wait;
        if (next < n && cmds[next].t < wake) {
            wake = cmds[next].t;
        }
        if (wake > end) {
            wake = end;
        }
        now = wake;
        while (next < n && cmds[next].t <= now) {
            rgb_led_core_apply(&core, &cmds[next++].cmd, now);
        }
        rgb_led_core_tick(&core, now);
        const char shown = symbol(rgb_led_core_output(&core));
        if (shown != last && (next > 0 || shown != '-')) {
            if (count < max) {
                out[count++] = (shown_t) {now, shown};
            }
            last = shown;
        }
    }
    return count;
}

static timed_cmd_t C(uint32_t t, rgb_led_cmd_t cmd)
{
    return (timed_cmd_t) {t, cmd};
}

static void test_task_loop_follows_the_state_changes(void)
{
    const rgb_led_cmd_t col_cyan = {.type = RGB_LED_CMD_SET_COLOR, .data.color = {0, 255, 255}};
    const rgb_led_cmd_t col_green = {.type = RGB_LED_CMD_SET_COLOR, .data.color = {0, 255, 0}};
    const rgb_led_cmd_t col_blue = {.type = RGB_LED_CMD_SET_COLOR, .data.color = {0, 0, 255}};
    const rgb_led_cmd_t blink = {.type = RGB_LED_CMD_SET_BLINK, .data.blink = {500, 0.5f}};
    const rgb_led_cmd_t mode = {.type = RGB_LED_CMD_SET_MODE, .data.mode = RGB_LED_MODE_BLINK};

    /* STANDUP at 1.2 s, STANDING at 8.0 s, WALKING at 20.0 s: three commands each, together */
    const timed_cmd_t cmds[] = {
        C(1200, col_cyan), C(1200, blink), C(1200, mode),
        C(8000, col_green), C(8000, blink), C(8000, mode),
        C(20000, col_blue), C(20000, blink), C(20000, mode),
    };
    shown_t seen[512];
    const unsigned n = run_task(cmds, 9, 21000, seen, 512);

    /* No command in the first second: the LED is not touched. Then cyan on at 1.2 s, flipping every 250 ms */
    assert(seen[0].t == 1200 && seen[0].c == 'C');
    assert(seen[1].t == 1450 && seen[1].c == '-' && seen[2].t == 1700 && seen[2].c == 'C');
    /* The state change at 8 s shows its colour immediately, with no dark flash in between */
    unsigned i = 0;
    while (i < n && seen[i].t < 8000) i++;
    assert(seen[i].t == 8000 && seen[i].c == 'G');
    assert(seen[i + 1].t == 8250 && seen[i + 1].c == '-');
    while (i < n && seen[i].t < 20000) i++;
    assert(seen[i].t == 20000 && seen[i].c == 'B');
    /* 250 ms phases all the way: 1.2 s to 21 s is about 79 flips */
    assert(n >= 75 && n <= 85);
    for (unsigned k = 1; k < n; k++) {
        const uint32_t d = seen[k].t - seen[k - 1].t;
        assert(d == 250 || seen[k].t == 8000 || seen[k].t == 20000 || d == 250 + 0);
    }
}

int main(void)
{
    test_off_on_and_waiting();
    test_blink_cycle_and_deadlines();
    test_commands_of_a_state_change_do_not_flash_off();
    test_duty_extremes_and_period_defaults();
    test_no_burst_after_a_stall_and_counter_wrap();
    test_unknown_commands_are_ignored();
    test_task_loop_follows_the_state_changes();
    printf("rgb_led_core_test: all tests passed\n");
    return 0;
}
