/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "accel_gate.h"

#include <math.h>
#include <stddef.h>

void accel_gate_init(accel_gate_t *g, float tolerance_ms2)
{
    g->tolerance = tolerance_ms2;
    g->accepted = 0;
    g->rejected = 0;
    g->last_rejected_raw[0] = g->last_rejected_raw[1] = g->last_rejected_raw[2] = 0;
    g->last_rejected_norm = 0.0f;
}

bool accel_gate_check(accel_gate_t *g, const float a[3], const int16_t raw[3])
{
    const float norm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);

    /* NaN fails the comparison, so it is rejected as well */
    if (isfinite(norm) && fabsf(norm - ACCEL_GATE_GRAVITY) <= g->tolerance) {
        g->accepted++;
        return true;
    }

    g->rejected++;
    g->last_rejected_norm = norm;
    if (raw != NULL) {
        g->last_rejected_raw[0] = raw[0];
        g->last_rejected_raw[1] = raw[1];
        g->last_rejected_raw[2] = raw[2];
    }
    return false;
}
