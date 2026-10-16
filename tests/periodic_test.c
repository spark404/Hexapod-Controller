//
// Host test for the periodic timer, see Core/Src/periodic.c
//
// Build and run from the Hexapod-Controller directory:
//
//   cc -std=c99 -Wall -Wextra -pedantic -Werror -ICore/Inc \
//      tests/periodic_test.c Core/Src/periodic.c -o /tmp/periodic_test && /tmp/periodic_test
//
#include <assert.h>
#include <stdio.h>

#include "periodic.h"

/* Poll every poll_ms from from_ms to to_ms (inclusive) and return the times it fired */
static unsigned run(periodic_t *p, uint32_t from_ms, uint32_t to_ms, uint32_t poll_ms, uint32_t *fired, unsigned max)
{
    unsigned n = 0;
    for (uint32_t t = from_ms; (uint32_t) (t - to_ms) > 0x7FFFFFFFU || t == to_ms; t += poll_ms) {
        if (periodic_due(p, t) && n < max) {
            fired[n++] = t;
        }
        if (t == to_ms) {
            break;
        }
    }
    return n;
}

static void test_fires_once_per_period(void)
{
    periodic_t p;
    uint32_t fired[16];
    periodic_init(&p, 30000, 5000);

    assert(!periodic_due(&p, 5000));      /* the first period starts at init */
    assert(!periodic_due(&p, 34999));
    unsigned n = run(&p, 5000, 125000, 1000, fired, 16);
    assert(n == 4);
    assert(fired[0] == 35000 && fired[1] == 65000 && fired[2] == 95000 && fired[3] == 125000);
}

static void test_does_not_drift_with_a_jittery_loop(void)
{
    periodic_t p;
    periodic_init(&p, 1000, 0);

    /* polled late by up to 90 ms every time: the due times stay on the 1 s grid */
    assert(periodic_due(&p, 1090));
    assert(!periodic_due(&p, 1999));      /* grid: 2000, not 1090 + 1000 */
    assert(periodic_due(&p, 2030));
    assert(periodic_due(&p, 3080));
    assert(!periodic_due(&p, 3999));
    assert(periodic_due(&p, 4000));
}

static void test_no_burst_after_a_stall(void)
{
    periodic_t p;
    periodic_init(&p, 1000, 0);

    assert(periodic_due(&p, 10500));      /* ten periods late: fires once */
    assert(!periodic_due(&p, 10501));
    assert(!periodic_due(&p, 11499));
    assert(periodic_due(&p, 11500));      /* and carries on a period after that */
}

static void test_timers_started_together_stay_in_step(void)
{
    periodic_t fast, slow;
    periodic_init(&fast, 30000, 777);
    periodic_init(&slow, 300000, 777);

    unsigned together = 0, fast_count = 0, slow_count = 0;
    for (uint32_t t = 777; t <= 777 + 900000; t += 1000) {
        bool f = periodic_due(&fast, t);
        bool s = periodic_due(&slow, t);
        fast_count += f;
        slow_count += s;
        together += f && s;
        assert(!s || f);                  /* the slow one always falls on a fast one */
    }
    assert(fast_count == 30 && slow_count == 3 && together == 3);
}

static void test_counter_wrap_around(void)
{
    periodic_t p;
    uint32_t fired[8];
    const uint32_t start = 0xFFFFFC00U;       /* 1024 ms before the wrap */
    periodic_init(&p, 1000, start);

    assert(!periodic_due(&p, start + 999));
    unsigned n = run(&p, start, start + 3500, 100, fired, 8);   /* runs through the wrap */
    assert(n == 3);
    assert(fired[0] == start + 1000U);        /* 24 ms before the wrap */
    assert(fired[1] == start + 2000U);        /* after the wrap */
    assert(fired[2] == start + 3000U);
    assert(fired[1] < fired[0]);              /* the counter did wrap in between */
}

static void test_zero_period_is_one_millisecond(void)
{
    periodic_t p;
    periodic_init(&p, 0, 100);
    assert(!periodic_due(&p, 100));
    assert(periodic_due(&p, 101));
    assert(periodic_due(&p, 102));
}

int main(void)
{
    test_fires_once_per_period();
    test_does_not_drift_with_a_jittery_loop();
    test_no_burst_after_a_stall();
    test_timers_started_together_stay_in_step();
    test_counter_wrap_around();
    test_zero_period_is_one_millisecond();
    printf("periodic_test: all tests passed\n");
    return 0;
}
