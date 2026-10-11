//
// Host test for the servo bus diagnostics, see Core/Src/bus_diag.c
//
// Build and run from the Hexapod-Controller directory:
//
//   DNM=cmake-build-debug/_deps/dynamixel-src/include
//   cc -std=c99 -Wall -Wextra -pedantic -Werror -ICore/Inc -I$DNM \
//      tests/bus_diag_test.c Core/Src/bus_diag.c -o /tmp/bus_diag_test && /tmp/bus_diag_test
//
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "bus_diag.h"
#include "dynamixel/dynamixel_error.h"

static void test_clean_window(void)
{
    bus_diag_init();
    bus_diag_snapshot_t prev, cur;
    bus_diag_report_t rep;
    bus_diag_snapshot(&prev, true);

    for (int i = 0; i < 100; i++) {
        bus_diag_record(BUS_OP_READ_STATE, DNM_OK, 2000 + (uint32_t) i);   /* 2000..2099 */
        bus_diag_record(BUS_OP_WRITE_GOAL, DNM_OK, 500);
    }
    bus_diag_snapshot(&cur, true);
    bus_diag_format(&prev, &cur, 30, &rep);

    assert(!rep.window_errors);
    assert(rep.protocol[0] == '\0' && rep.transport[0] == '\0' && rep.boot[0] == '\0');
    assert(strstr(rep.summary, "rd 100 ok 100 (us 2000/2049/2099)") != NULL);
    assert(strstr(rep.summary, "wr 100 ok 100 (us 500/500/500)") != NULL);
    assert(strstr(rep.summary, "max fail streak 0") != NULL);
}

static void test_failures_and_streak(void)
{
    bus_diag_init();
    bus_diag_snapshot_t prev, cur;
    bus_diag_report_t rep;
    bus_diag_snapshot(&prev, true);

    bus_diag_record(BUS_OP_READ_STATE, DNM_OK, 2000);
    bus_diag_record(BUS_OP_READ_STATE, DNM_LL_TIMEOUT, 10000);    /* streak 1 */
    bus_diag_record(BUS_OP_READ_STATE, DNM_RECV_CRC_FAIL, 2100);  /* streak 2 */
    bus_diag_record(BUS_OP_WRITE_GOAL, DNM_LL_ERR, 100);        /* streak 3 */
    bus_diag_record(BUS_OP_READ_STATE, DNM_OK, 3000);             /* streak reset */
    bus_diag_record(BUS_OP_READ_STATE, DNM_RECV_NOT_STATUS, 0);   /* streak 1 */
    bus_diag_record(BUS_OP_READ_STATE, 123, 0);                   /* unknown code -> other */
    bus_diag_event(BUS_EV_UART_FE, 2);
    bus_diag_event(BUS_EV_RESET_READ_FAIL, 3);
    bus_diag_event(BUS_EV_RX_DROP, 17);
    bus_diag_event(BUS_EV_LOOP_OVERRUN, 1);

    bus_diag_snapshot(&cur, true);
    bus_diag_format(&prev, &cur, 30, &rep);

    assert(rep.window_errors);
    /* Failed transactions do not count in the timing: only 2000 and 3000 */
    assert(strstr(rep.summary, "rd 6 ok 2 (us 2000/2500/3000)") != NULL);
    assert(strstr(rep.summary, "wr 1 ok 0 (us -)") != NULL);
    assert(strstr(rep.summary, "max fail streak 3") != NULL);
    assert(strstr(rep.summary, "loop overruns 1") != NULL);
    assert(strstr(rep.protocol, "rd err: crc 1 notstatus 1 ll 0 timeout 1 api 0 id 0 len 0 other 1") != NULL);
    assert(strstr(rep.protocol, "wr err: crc 0 notstatus 0 ll 1") != NULL);
    assert(strstr(rep.protocol, "last code 123") != NULL);
    assert(strstr(rep.transport, "uart ore 0 fe 2") != NULL);
    assert(strstr(rep.transport, "rx dropped 17 B") != NULL);
    assert(strstr(rep.transport, "after read 3") != NULL);
    assert(strstr(rep.boot, "rd failed 4/6") != NULL);
    assert(strstr(rep.boot, "wr failed 1/1") != NULL);
}

static void test_new_result_codes_and_servo_attribution(void)
{
    bus_diag_init();
    bus_diag_snapshot_t prev, cur;
    bus_diag_report_t rep;
    bus_diag_snapshot(&prev, true);

    bus_diag_record(BUS_OP_READ_STATE, DNM_RECV_ID_MISMATCH, 0);
    bus_diag_record(BUS_OP_READ_STATE, DNM_RECV_LENGTH_MISMATCH, 0);
    bus_diag_servo_failure(7);
    bus_diag_servo_failure(7);
    bus_diag_servo_failure(12);
    bus_diag_servo_failure(18);          /* out of range: ignored */
    bus_diag_event(BUS_EV_SERVO_ALERT, 4);

    bus_diag_snapshot(&cur, true);
    bus_diag_format(&prev, &cur, 30, &rep);

    assert(rep.window_errors);
    assert(strstr(rep.protocol, "id 1 len 1 other 0") != NULL);
    assert(strstr(rep.transport, "servo alerts 4") != NULL);
    assert(strstr(rep.servos, "7:2") != NULL);
    assert(strstr(rep.servos, "12:1") != NULL);
    assert(strstr(rep.servos, "18:") == NULL);

    /* a window without servo failures has no servo line, even though the totals remember */
    bus_diag_snapshot_t next;
    bus_diag_snapshot(&next, true);
    bus_diag_format(&cur, &next, 30, &rep);
    assert(rep.servos[0] == '\0');
    assert(!rep.window_errors);
}

static void test_window_reset_and_deltas(void)
{
    bus_diag_init();
    bus_diag_snapshot_t s0, s1, s2;
    bus_diag_report_t rep;
    bus_diag_snapshot(&s0, true);

    bus_diag_record(BUS_OP_READ_STATE, DNM_LL_TIMEOUT, 0);
    bus_diag_record(BUS_OP_READ_STATE, DNM_OK, 4000);
    bus_diag_snapshot(&s1, true);

    /* Second window is clean: deltas must not include the first window, and the
     * min/max/streak must have been reset by the snapshot */
    bus_diag_record(BUS_OP_READ_STATE, DNM_OK, 2000);
    bus_diag_record(BUS_OP_READ_STATE, DNM_OK, 2200);
    bus_diag_snapshot(&s2, true);
    bus_diag_format(&s1, &s2, 30, &rep);

    assert(!rep.window_errors);
    assert(strstr(rep.summary, "rd 2 ok 2 (us 2000/2100/2200)") != NULL);
    assert(strstr(rep.summary, "max fail streak 0") != NULL);
    /* but the totals since boot still remember the earlier failure */
    assert(strstr(rep.boot, "rd failed 1/4") != NULL);
}

static void test_counter_wrap(void)
{
    bus_diag_snapshot_t prev, cur;
    bus_diag_report_t rep;
    memset(&prev, 0, sizeof(prev));
    memset(&cur, 0, sizeof(cur));
    for (int op = 0; op < BUS_OP_COUNT; op++) {
        prev.time_min_us[op] = cur.time_min_us[op] = UINT32_MAX;
    }

    /* 3 reads recorded across a wrap of the OK counter and of the time sum */
    prev.res[BUS_OP_READ_STATE][BUS_RES_OK] = UINT32_MAX - 1;
    cur.res[BUS_OP_READ_STATE][BUS_RES_OK] = 1;
    prev.time_sum_us[BUS_OP_READ_STATE] = UINT32_MAX - 2999;
    cur.time_sum_us[BUS_OP_READ_STATE] = 6000;    /* +9000 us over 3 reads */
    cur.time_min_us[BUS_OP_READ_STATE] = 2900;
    cur.time_max_us[BUS_OP_READ_STATE] = 3100;

    bus_diag_format(&prev, &cur, 30, &rep);
    assert(strstr(rep.summary, "rd 3 ok 3 (us 2900/3000/3100)") != NULL);
}

int main(void)
{
    test_clean_window();
    test_failures_and_streak();
    test_new_result_codes_and_servo_attribution();
    test_window_reset_and_deltas();
    test_counter_wrap();
    printf("bus_diag: all tests passed\n");
    return 0;
}
