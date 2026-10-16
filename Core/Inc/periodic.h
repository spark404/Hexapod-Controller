/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * A timer for things that happen every so often (periodic reports) from a loop that
 * runs faster than the period.
 *
 * Times are milliseconds from a free running 32 bit counter; wrap around is handled.
 * Timers started with the same time stay in step: a report with a period of 30 s and one
 * of 300 s fall together every tenth time. A timer does not drift with the jitter of the
 * loop that polls it, and it does not make up for missed periods in a burst: after a long
 * stall it fires once and carries on a full period later.
 *
 * No HAL or RTOS dependency, host test in tests/periodic_test.c.
 */

#ifndef PERIODIC_H
#define PERIODIC_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t period_ms;
    uint32_t next_ms;
} periodic_t;

/* The first period starts at now_ms. A period of 0 is taken as 1 ms. */
void periodic_init(periodic_t *p, uint32_t period_ms, uint32_t now_ms);

/* True once per period, at the first call at or after the due time. */
bool periodic_due(periodic_t *p, uint32_t now_ms);

#endif /* PERIODIC_H */
