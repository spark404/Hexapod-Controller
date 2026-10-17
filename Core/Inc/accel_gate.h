/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Sanity check of accelerometer samples before they reach the attitude filter.
 *
 * Roll and pitch come from the direction of gravity in the accelerometer, which is only meaningful
 * when the sample is gravity: a still sensor measures a vector of length g, a corrupted read or an
 * impact does not. A sample whose length differs from g by more than the tolerance is not used.
 * That removes the garbage reads (one axis with a wrong high byte, seen at standstill as 20 to 45
 * m/s^2) and the impacts of the feet, and costs nothing when the sample is good. A sample that
 * passes with a smaller error moves the estimate by at most the tolerance, about 14 degrees at 25 %.
 *
 * No HAL or RTOS dependency, host test in tests/accel_gate_test.c.
 */

#ifndef ACCEL_GATE_H
#define ACCEL_GATE_H

#include <stdbool.h>
#include <stdint.h>

#define ACCEL_GATE_GRAVITY 9.80665f
#define ACCEL_GATE_DEFAULT_TOLERANCE (0.25f * ACCEL_GATE_GRAVITY)   /* m/s^2 around g */

typedef struct {
    float tolerance;               /* m/s^2 around g */
    uint32_t accepted;
    uint32_t rejected;
    int16_t last_rejected_raw[3];  /* raw sensor counts of the last rejected sample, for diagnosis */
    float last_rejected_norm;      /* its length in m/s^2 */
} accel_gate_t;

void accel_gate_init(accel_gate_t *g, float tolerance_ms2);

/* true when the sample (m/s^2, any frame) is usable; counts it either way. raw may be NULL. */
bool accel_gate_check(accel_gate_t *g, const float a[3], const int16_t raw[3]);

#endif /* ACCEL_GATE_H */
