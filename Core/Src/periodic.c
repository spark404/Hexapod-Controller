/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "periodic.h"

void periodic_init(periodic_t *p, uint32_t period_ms, uint32_t now_ms)
{
    p->period_ms = period_ms != 0U ? period_ms : 1U;
    p->next_ms = now_ms + p->period_ms;
}

bool periodic_due(periodic_t *p, uint32_t now_ms)
{
    if ((int32_t) (now_ms - p->next_ms) < 0) {
        return false;
    }

    p->next_ms += p->period_ms;
    if ((int32_t) (now_ms - p->next_ms) >= 0) {
        /* More than a whole period late: do not fire again to catch up */
        p->next_ms = now_ms + p->period_ms;
    }
    return true;
}
