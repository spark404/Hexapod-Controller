/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MEASURE_H
#define MEASURE_H

#define AVG_WINDOW 32   // number of samples
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t buffer[AVG_WINDOW];
    uint32_t sum;
    uint16_t index;
    uint16_t count;
} running_avg_t;

void running_avg_init(running_avg_t *avg);
void running_avg_add(running_avg_t *avg, uint32_t value);
uint32_t running_avg_get(const running_avg_t *avg);

#ifdef __cplusplus
}
#endif

#endif //MEASURE_H
