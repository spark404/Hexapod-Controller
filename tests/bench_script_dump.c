//
// Prints the bench script and its ideal paths so that tools/test_bench_profile_analyze.py can check that the
// Python mirror of the paths agrees with the C code. Not part of the firmware.
//
//   cc -std=c99 -ICore/Inc tests/bench_script_dump.c Core/Src/bench_script.c -lm -o /tmp/bench_script_dump
//
#include <math.h>
#include <stdio.h>

#include "bench_script.h"

int main(void)
{
    static bench_run_t runs[BENCH_MAX_RUNS];
    const size_t n = bench_script_build(BENCH_ALL | BENCH_E0, runs, BENCH_MAX_RUNS);
    for (size_t i = 0; i < n; i++) {
        const bench_run_t *r = &runs[i];
        printf("RUN,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%u,%u,%u\n", r->id, r->experiment, (unsigned) r->mode,
               (unsigned) r->path, r->joints, r->period_ms, r->time_ms, r->ramp_ms, r->velocity, r->accel,
               r->delta_mrad[0], r->delta_mrad[1], r->delta_mrad[2], r->path_param, r->sample_ms, r->duration_ms);
        for (unsigned j = 0; j < BENCH_JOINTS; j++) {
            if (!(r->joints & (1U << j))) continue;
            for (int t = -BENCH_PRE_TRIGGER_MS; t <= (int) r->duration_ms; t += 25) {
                printf("OFF,%u,%u,%d,%ld\n", r->id, j, t, lroundf(bench_path_offset_rad(r, j, t) * 1e6f));
            }
            for (unsigned k = 0; k < bench_goal_count(r); k++) {
                printf("GOAL,%u,%u,%u,%ld,%ld\n", r->id, j, k, (long) bench_goal_time_ms(r, k),
                       lroundf(bench_goal_offset_rad(r, j, k) * 1e6f));
            }
        }
    }
    return 0;
}
