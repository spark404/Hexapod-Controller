/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bus_capture.h"

void bus_capture_init(bus_capture_t *c, uint32_t threshold, uint32_t repeat)
{
    c->threshold = threshold;
    c->repeat = repeat;
    c->streak = 0;
}

bool bus_capture_update(bus_capture_t *c, bool read_failed)
{
    if (!read_failed) {
        c->streak = 0;
        return false;
    }
    if (c->streak < UINT32_MAX) {
        c->streak++;
    }
    if (c->streak < c->threshold) {
        return false;
    }
    if (c->streak == c->threshold) {
        return true;
    }
    return c->repeat != 0 && (c->streak - c->threshold) % c->repeat == 0;
}

size_t bus_capture_hex(char *out, size_t out_size, const uint8_t *data, size_t n)
{
    static const char digits[] = "0123456789abcdef";
    size_t shown = 0;

    if (out_size == 0) {
        return 0;
    }
    size_t pos = 0;
    while (shown < n && pos + 4 <= out_size) {   /* two digits, a space, the NUL */
        out[pos++] = digits[data[shown] >> 4];
        out[pos++] = digits[data[shown] & 0x0F];
        out[pos++] = ' ';
        shown++;
    }
    if (pos > 0) {
        pos--;   /* drop the last space */
    }
    out[pos] = '\0';
    return shown;
}
