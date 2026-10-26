/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * When to capture evidence of a servo bus that stays broken (backlog T7.17).
 *
 * A single failed read is normal background (one in several thousand) and the UART reset after
 * it fixes it. A run of failed reads is not: the walking test of 2026-10-26 lost all reads for
 * 130 s and the reset never cleared it. The capture (what the receiver holds, what an idle line
 * carries, whether servos 0 and 1 answer a ping) is taken once the run reaches a threshold, and
 * again every `repeat` reads while it lasts, so the log shows the state without flooding.
 *
 * No HAL or RTOS dependency, host test in tests/bus_capture_test.c.
 */

#ifndef BUS_CAPTURE_H
#define BUS_CAPTURE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t threshold;   /* failed reads in a row before the first capture */
    uint32_t repeat;      /* further failed reads between captures, 0 = capture once */
    uint32_t streak;      /* failed reads in a row now */
} bus_capture_t;

void bus_capture_init(bus_capture_t *c, uint32_t threshold, uint32_t repeat);

/* Call after every state read. Returns true when the evidence should be captured now. */
bool bus_capture_update(bus_capture_t *c, bool read_failed);

/* Write n bytes as "ff ff fd 00" into out (NUL terminated, cut off when out is too small).
 * Returns the number of bytes shown. */
size_t bus_capture_hex(char *out, size_t out_size, const uint8_t *data, size_t n);

#endif /* BUS_CAPTURE_H */
