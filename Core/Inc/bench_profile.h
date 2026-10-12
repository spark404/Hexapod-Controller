/*
 * SPDX-FileCopyrightText: 2026 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Servo profile bench experiment (docs/servo-profile-bench-experiment.md).
 *
 * Only part of the bench builds (HEXAPOD_BENCH_PROFILE=ON, presets BenchDry and
 * Bench). The normal firmware does not contain it.
 */

#ifndef BENCH_PROFILE_H
#define BENCH_PROFILE_H

#include "dynamixel/dynamixel.h"

/**
 * Run the experiment on leg 0 (servos[0], servos[1], servos[2]) and park. Does
 * not return. Called from the servo task once all servos have been checked, in
 * place of the normal servo state machine, so it owns the servo bus.
 */
void bench_profile_run(dynamixel_servo_t *servos);

#endif /* BENCH_PROFILE_H */
