/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "measure.h"

#include <string.h>

void running_avg_init(running_avg_t *avg)
{
    memset(avg, 0, sizeof(*avg));
}

void running_avg_add(running_avg_t *avg, uint32_t value)
{
    // Subtract the value that will be overwritten
    avg->sum -= avg->buffer[avg->index];

    // Store new value
    avg->buffer[avg->index] = value;
    avg->sum += value;

    // Advance index
    avg->index = (avg->index + 1) % AVG_WINDOW;

    // Grow count until full
    if (avg->count < AVG_WINDOW) {
        avg->count++;
    }
}

uint32_t running_avg_get(const running_avg_t *avg)
{
    if (avg->count == 0) {
        return 0;
    }
    return avg->sum / avg->count;
}
