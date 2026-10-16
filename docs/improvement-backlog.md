# Hexapod Controller — Improvement Backlog

Prioritized work items, re-planned around [`design.md`](design.md) (rev 2).
Each item is sized, sourced, and has a verification proposal so you can pick
work that fits the time you have.

## How to read this

**Phases** are ordered by dependency, not by size. Finish the foundations
(Phase 1) first; Phases 2 and 3 can run in parallel; Phase 4 builds on 2; Phase
5 (the rewrite of walking, rotating and the lifecycle) needs 3 and 4.

**IDs are stable labels, not priorities.** They were assigned before the
design and kept so that [`controller-audit.md`](controller-audit.md) and
[`design.md`](design.md) still resolve. The `T7.x` items are new, derived from
the design. Items that the design replaces are listed under *Superseded items*
at the end, with their original text.

**Effort** sizes (rough hours of focused work, excluding validation):
XS = ≤30 min, S = 1–2 h, M = ½ day, L = 1–2 days, XL = > 2 days.

**Verification** suggests a layered approach:
- **Static** — review / unit test in the [HexapodSim Controller
  project](../../HexapodSim/Controller), which links the same `controller.c`
  and friends (needs T7.7 once interfaces change).
- **Sim** — full simulator run (HexSpider + Controller) to verify behavior
  without risk to hardware.
- **Bench** — hardware test with the robot on a stand / legs free, e.g. for
  individual joint or single-leg behavior.
- **Walk** — full on-ground walking, joystick-driven.

Use the lowest layer that proves the change; promote to the next layer once it
passes.

**Sources** point at [`controller-audit.md`](controller-audit.md) (findings and
reasoning) and [`design.md`](design.md) (the design and its appendices). Servo
motion-profile work (T4.x) is specified in
[`servo-velocity-control-phase2.md`](servo-velocity-control-phase2.md).

---

## Quick index

| Phase | Theme | Active items |
|------:|-------|-------------:|
| 1 | Foundations and safety | 14 (1 done) |
| 2 | Servo bus and actuation | 9 (2 done) |
| 3 | Command interface | 1 |
| 4 | Frames, sensing and contact | 5 |
| 5 | Motion core | 4 |
| 6 | Extensions | 2 |
| — | Superseded items | 16 (reference only) |

### Dependencies

```
Phase 1 ──► Phase 2 (servo bus) ───────┐
   │                                   ├─► Phase 4 (sensing, contact) ─┐
   └──────► Phase 3 (command interface)┘                               ├─► Phase 5 (motion core) ─► Phase 6
                          (needs T1.6)                  T7.7 (sim) ────┘
Key chains:  T1.6 ─► T7.2 (25 Hz), T7.5 (command)
             T7.4 (load) ─► T7.6 (contact) ─► T5.3 (odometry), T7.9 (gait)
             T7.5 (command) ─► T7.8 (twist) ─► T7.9 (gait) ─► T7.10 (lifecycle), T7.11 (ripple)
             T1.2 (fault) ─► T4.3, T7.10, T5.6
             T2.13 (clock) ─► T7.3 (2 Mbps)
```

---

## Status

Checked against the code on 2026-10-10 (git history showed no commit implementing
any item other than the `dt_s` re-bound). "Open" means the code still shows the
problem the item describes. Updated 2026-10-11: T2.13 and T7.3 are partly done
and T7.12 is done, all verified on the robot (commits `2b0e064`, `d6c1be3`,
`34e582f`). Updated 2026-10-13 (re-checked against the code, nothing else was
implemented): T7.4 is done (units and the byte stuffing fix confirmed on the robot,
the bench experiment used the same registers), T7.14 is done and T7.15 was added.
Updated 2026-10-16: T4.1 is implemented (commit `cb73897`, not yet verified on the
robot), which makes T2.4 moot. T4.2 is implemented too (commit `2fca015`, not yet
verified on the robot).

| Item | Phase | Status | Note |
|------|:-----:|--------|------|
| T1.1 | 1 | Partial | Clamps re-bound in `517cb38`: control `[0.05, 0.2]` s, servo `[0.01, 0.04]` s. `CTRL_LOOP_DT_S` still defined and unused (T2.6). |
| T1.2 | 1 | Open | No `fault_code`, `CTRL_FAULT` or `SERVO_FAULT_HOLD`; `SERVO_ERROR` still calls `Error_Handler()`. |
| T1.5 | 1 | Open | No `target_seq` / `actual_seq`. |
| T1.6 | 1 | Open | `printf` busy-waits on USART1 at 115200; ~57 % CPU in WALKING. |
| T2.12 | 1 | Open | No run-time stats, stack or heap reporting; mutex leak in gyro/accel tasks. |
| T2.13 | 1 | Partial | Runs from the 16 MHz HSE with an HSI fallback (`2b0e064`); boots at 84 MHz. Open: power-cycle test, MCO measurement, load-cap decision (board has 15 pF caps, crystal wants 8 pF). |
| T2.3 | 1 | Open | `wrap_to_pi` still `fmodf`-based (`[-π, π)`). |
| T2.4 | 1 | Moot | `last_velocity` and `last_ticks` deleted with the software interpolation (T4.1, `cb73897`). |
| T2.5 | 1 | Open | `FEMUR_OFFSET_ANGLE_RAD` still in `servos.c`. |
| T2.6 | 1 | Open | `initialized`, `delta_yaw`, `pending_yaw`, `CTRL_LOOP_DT_S`, `CTRL_SERVO_Z_OFFSET` still present. |
| T2.9 | 1 | Open | Not re-verified; no related commits. |
| T4.3 | 1 | Open | Follows T1.2. |
| T7.1 | 1 | Partial | New. Overruns are now counted (T7.12) and reported; re-anchoring `tick_count` is not done. |
| T7.12 | 1 | Done | New. Servo bus diagnostics every 30 s (`34e582f`). |
| T3.5 | 2 | Open | No related code. |
| T4.1 | 2 | Partial | Implemented (`cb73897`): fixed profile with every goal, software interpolation removed. Not yet verified on the robot. |
| T4.2 | 2 | Partial | Implemented (`2fca015`): v and a per joint per goal, measured from the servo's Position Trajectory, acceleration floor 40 rad/s². `SERVO_PROFILE_MODE` switches back to the fixed profile. Not yet verified on the robot; T factor and ramp not yet chosen from the E4 data. |
| T7.2 | 2 | Open | New. |
| T7.3 | 2 | Partial | New. Servos and `USART6` are at 2 Mbps (`d6c1be3`), set up by hand; no migration routine, Return Delay Time margin not measured. |
| T7.4 | 2 | Done | New. Load, velocity and position come back in one 10-byte Sync Read and are published (`f1f0a2c`). Verified on the robot: values and timing as expected, units confirmed by the bench experiment, byte stuffing fixed (library `0a49c5c`, 0 failures in 2,950 reads). Left over: the rare read timeout (~1 per 3,000 cycles, first run) is not explained. |
| T7.16 | 2 | Open | New, next after T4.1/T4.2 are seen on the robot. Write the goals when the controller publishes, not on the next 50 Hz servo tick. |
| T7.15 | 2 | Open | New, from the first bench data. A loaded leg lags its servo profile by ~100 ms and stops short of the goal: tune the position controller. |
| T7.14 | 2 | Done | New. Bench experiment for the servo profile mode: all 103 runs on the robot, 2026-10-12. Result in [plan §6.3](servo-profile-bench-experiment.md): velocity-based computed profile if profiles are used, no sync benefit, servo lag is the issue (T7.15). |
| T7.13 | 2 | Open | New, low priority. Byte stuffing in the library costs ~0.12 ms per bus transaction in the Debug build; do it in one pass. |
| T7.5 | 3 | Partial | New. Slice 1 done: frame layout, CRC, status frame and seq handling in `cmd_frame.c` (host tested). Open: adapter and `T_LINK`, full-duplex SPI task, Pi `SpiSink`. Today's SPI task still takes 7-byte single-parameter frames, no CRC, no reply. |
| T5.1 | 4 | Open | Reach radius is a fixed 40 mm. |
| T5.2 | 4 | Open | `attitude` still discarded (`(void)attitude`). |
| T5.3 | 4 | Open | Open-loop body pose integration. |
| T7.6 | 4 | Open | New. |
| T7.7 | 4 | Open | New. |
| T5.6 | 5 | Open | No e-stop path. |
| T7.10 | 5 | Open | New. Today: `POWERDOWN` drops torque from standing. |
| T7.8 | 5 | Open | New. Today: separate WALKING / ROTATING, `(velocity, heading)` input. |
| T7.9 | 5 | Open | New. Today: event-driven tripod swap. |
| T6.3 | 6 | Open |  |
| T7.11 | 6 | Open | New. |
| T1.3 | — | Superseded | by T7.5, T7.8. |
| T1.4 | — | Superseded | by T7.10. |
| T2.1 | — | Superseded | by T7.8. |
| T2.10 | — | Superseded | by T7.9. |
| T2.11 | — | Superseded | by T7.5. |
| T2.2 | — | Superseded | by T7.8. |
| T2.7 | — | Superseded | by T7.10. |
| T2.8 | — | Superseded | by T7.8. |
| T3.1 | — | Superseded | by T7.8. |
| T3.2 | — | Superseded | by T7.8. |
| T3.3 | — | Superseded | by T7.9. |
| T3.4 | — | Superseded | by T7.9. |
| T5.4 | — | Superseded | by T7.9. |
| T5.5 | — | Superseded | by T7.10. |
| T6.1 | — | Superseded | by T7.8. |
| T6.2 | — | Superseded | by T7.9. |

---

## Phase 1 — Foundations and safety

Independent of the new design and a prerequisite for most of it. Do these first, in this order: logging and measurement come before anything that raises rates, and the fault path before anything that increases command activity.

### T1.6 — Make logging non-blocking and quiet by default

**Phase 1.** Prerequisite for the 25 Hz control rate (T7.2) and the command interface (T7.5).

**Source:** [`design.md`](design.md) Appendix B.3.
**Effort:** S.

**Do.**
1. Set the default `g_log_level` to `LOG_LEVEL_INFO` (or `WARN` for release
   builds).
2. Remove or gate the per-leg and summary `LOG_DEBUG` lines in WALKING
   (`controller.c:586-596`) and the per-frame `LOG_INFO` lines in
   `StartSpiSlaveTask` behind a compile-time flag (for example
   `CTRL_TRACE`).
3. Replace the busy-wait in `__io_putchar` with a ring buffer drained by a
   low-priority task (or UART DMA), dropping messages rather than spinning when
   full.

**Verify.**
- *Bench:* with the robot walking, per-task CPU (T2.12) shows the control task
  well below the logging-induced ~57 %.
- *Bench:* control tick period stays at its nominal value when the control rate
  is raised to 25 Hz.


### T2.12 — CPU, stack and heap instrumentation

**Phase 1.** Measure before relying on the CPU estimates in design Appendix B.

**Source:** [`design.md`](design.md) Appendix B.4–B.5.
**Effort:** S.

**Do.** Enable FreeRTOS run-time stats with the DWT cycle counter as the timer;
log per-task CPU %, `uxTaskGetStackHighWaterMark` per task and
`xPortGetMinimumEverFreeHeapSize()` once per second at `INFO`. Add a
`DWT->CYCCNT` measurement around `controller_update` and the servo loop body.
Also release `spiMutex` on the error paths in `StartGyroTask` and
`StartAccelTask`, and raise `configTOTAL_HEAP_SIZE` if free heap is under
~4 KB.

**Verify.**
- *Bench:* the figures in design Appendix B.2 are replaced with measurements.


### T1.1 — Fix `dt_s` upper clamp

**Phase 1.** Partly done (see Status). Finish by removing `CTRL_LOOP_DT_S` (T2.6) and re-banding for the new control rate (T7.2).

**Source:** [`controller-audit.md`](controller-audit.md) §1.1.
**Effort:** XS.

**Do.** In [`main.c:2375-2376`](../Core/Src/main.c:2375), raise the
clamp upper bound from `0.05f` to `0.15f` (i.e. allow up to 1.5×
the expected period). Remove the unused `CTRL_LOOP_DT_S` macro from
[`controller_config.h:10`](../Core/Inc/controller_config.h:10).

**Verify.**
- *Static:* add a one-shot log line for the first ten ticks
  printing `dt_s`; should see ~0.10 not 0.05.
- *Sim:* run a STANDUP cycle, compare time-to-standing before/after
  — should drop by ~half.
- *Walk:* command 50 mm/s for 5 s on a tape-measured line; physical
  distance should be ~250 mm not ~125 mm.


### T7.1 — Servo loop overrun handling

**Source:** [`design.md`](design.md) Appendix B.4.
**Effort:** XS.
**Status:** the overrun counter exists and is reported every 30 s (T7.12,
`BUS_EV_LOOP_OVERRUN`). Re-anchoring is still open.

**Do.** In `StartServoTask`, detect a missed deadline (`now` later than the
next `tick_count`) and re-anchor `tick_count = now` instead of letting
`osDelayUntil` run a catch-up burst. Count overruns and log at most once per
second at `WARN`.

**Verify.**
- *Bench:* inject a single `osDelay(30)` in the servo loop; confirm there is no
  burst of back-to-back iterations afterwards and the overrun counter reads 1.


### T7.12 — Servo bus diagnostics

**Phase 1.** **Done** in commit `34e582f`, verified on the robot.

**Source:** [`design.md`](design.md) Appendix A.8.
**Effort:** S.

**What it does.** `Core/Src/bus_diag.c` (no HAL or RTOS dependency, host test in
`tests/bus_diag_test.c`) counts Sync Read and Sync Write transactions by
outcome (ok, CRC failure, not a status packet, link-layer error, timeout, API
error), transport failures in `usart_read`/`usart_write`, UART overrun, framing,
noise, parity and DMA errors, bytes dropped when the RX ring is full, DMA
receiver resets by cause, and servo loop overruns. Successful transactions are
timed in µs with the DWT cycle counter (`htim1` is reset by the sensor delay
routines, so it cannot be used). The default task prints one summary line every
30 s, or three `WARN` lines when anything failed, plus totals since boot after
the first failure.

**Follow-ups.**
- T7.4 adds the load read: give it its own `bus_op_t` so it is timed and
  counted too.
- Per-servo attribution: **done** with the library's per-servo status (T7.4).
  A failed read is counted for the servo that gave no data; for a timeout only
  the first silent servo is counted, not the ones behind it.
- The per-failure `LOG_WARN` / `LOG_ERROR` lines in the servo task are still
  there and can flood the serial port on a bad bus; they could drop to `DEBUG`
  now that the summary exists.

**Result.** At 2 Mbps: Sync Read 3,178 µs and Sync Write 796 µs on average with
no failures in 4,500 transactions each.


### T1.2 — Servo fault back-channel + `CTRL_FAULT` state

**Phase 1.** Design role: the `FAULT` state of the lifecycle (design §8).

**Source:** [`controller-audit.md`](controller-audit.md) §2.1, §4.5 (P1).
**Effort:** M.

**Do.**
1. Add `servo_fault_t fault_code; uint8_t fault_leg, fault_joint;`
   to `servo_shared_state_t` in [`Core/Inc/servos.h`](../Core/Inc/servos.h).
2. Replace `Error_Handler()` in
   [`main.c:2616-2624`](../Core/Src/main.c:2616) with a state that
   disables torque and stays in a new `SERVO_FAULT_HOLD` state
   (loop still running, no torque, no GOAL_POSITION writes).
3. Add `CTRL_FAULT` to the controller FSM enum. On any tick, if
   `fault_code != SERVO_FAULT_NONE`, transition there. The state
   does nothing — just waits — and exits when the fault is
   externally cleared (e.g. via SPI or a button).
4. Drive a distinct LED state (red blink) from the existing
   `stm32_state_change_cb`.

**Verify.**
- *Sim:* (if the sim exposes joint limits) command a leg past its
  limit, confirm CTRL_FAULT transition and clean recovery.
- *Bench:* mechanically block one leg, command motion that requires
  it. Confirm: torque drops, LED goes red, no busy-loop.
- *Walk:* deliberately set a joint limit aggressive enough to fire
  during normal walking; confirm the robot stops cleanly and can
  be reset without power-cycling.


### T4.3 — Refactor `SERVO_ERROR` after T1.2

**Phase 1** (right after T1.2, so it is listed there).

**Source:** Implicit; cleanup after T1.2 lands.
**Effort:** S.
**Why:** Once `CTRL_FAULT` exists (T1.2), the servo task's old
`SERVO_ERROR → Error_Handler` path can be cleanly replaced with
"set fault_code, transition to SERVO_FAULT_HOLD, never busy-loop".
Worth a small cleanup pass once T1.2 is in.

**Do.** Remove `SERVO_ERROR` and `Error_Handler()` calls from
[`main.c:2616-2624`](../Core/Src/main.c:2616). Replace with the
`SERVO_FAULT_HOLD` state from T1.2.

**Verify.** Same as T1.2.


### T1.5 — Liveness sequence counters

**Phase 1.** Design role: `target_seq` / `actual_seq` in the shared state (design §9). Make thresholds time-based (T7.2).

**Source:** [`controller-audit.md`](controller-audit.md) §2.2 (P4).
**Effort:** S.

**Do.** Add `uint32_t target_seq` and `uint32_t actual_seq` to
`servo_shared_state_t`. The control task increments `target_seq`
on every published target; the servo task increments `actual_seq`
on every `SERVO_RUNNING` completion. Each side reads the other's
counter and asserts that it has incremented within the last N
ticks (e.g. 5 control ticks = 500 ms). If not, raise a fault.

**Verify.**
- *Sim:* artificially `osDelay()` one task for 1 s; confirm the
  other detects staleness and faults.
- *Bench:* run normal motion; confirm counters increment as
  expected (visible in log).
- *Walk:* no specific test; this is a watchdog that should never
  trigger in normal operation.


### T2.13 — Run from the HSE crystal on ControlHat

**Phase 1.** Recommended before raising the bus baud rate (T7.3).

**Source:** [`design.md`](design.md) Appendix A.7.
**Effort:** S.
**Status:** **partly done** in `2b0e064`: PLL source HSE, `PLLM = 8`, `PLLN =
168`, `PLLP = 4`, `PLLQ = 7` (2 MHz VCO input instead of 1 MHz), `HSE_VALUE =
16000000`, and an HSI fallback in `Error_Handler()` (`clock_hse_fallback()`)
that keeps the boot going if the crystal does not start. The first boot logged
`Clock: HSE, SYSCLK 84000000 Hz`. Open: the bench checks under *Verify* and the
decision on `C108` / `C109`.
**Notes:** Autogenerated clock code is involved (`CLAUDE.md`): the CubeMX
changes are for you to make. The crystal is 16 MHz (CL = 8 pF).

**Do.**
1. In CubeMX: HSE crystal/ceramic resonator mode, PLL source HSE,
   `PLLM = 8`, `PLLN = 168`, `PLLP = 4`, `PLLQ = 7`; set `HSE_VALUE = 16000000`.
   Do not copy the 25 MHz from ControlHat's `Hexapod.ioc`.
2. Keep a way to fall back to the HSI (done) or build for the Nucleo (8 MHz bypass, not done).
3. If startup is marginal, swap `C108` / `C109` (15 pF) for ~8.2 pF C0G and
   re-test; fix the values in the schematic for the next revision.

**Verify.**
- *Bench:* output the clock on MCO and measure SYSCLK with a frequency
  counter or scope; confirm 84 MHz within crystal tolerance.
- *Bench:* 20+ power cycles, including cold start, with reliable HSE startup
  (no fall-back to HSI).
- *Bench:* confirm the Dynamixel bus works at the chosen rate (done, 2 Mbps).
- *Bench:* force a start-up failure (for example hold the crystal pins) and
  confirm the HSI fallback boots and logs the warning.


### T2.3 — `wrap_to_pi` boundary

**Phase 1.** Still needed under the new model (heading and yaw wrapping).

**Source:** [`controller-audit.md`](controller-audit.md) §1.5.
**Effort:** XS.

**Do.** In [`controller_math.h:16`](../Core/Inc/controller_math.h:16),
replace with a stable single-branch version:

```c
static inline float32_t wrap_to_pi(float32_t a) {
    a = remainderf(a, 2.0f * (float)M_PI);
    return a;
}
```

`remainderf` returns the result in `(-π, +π]`.

**Verify.**
- *Static:* unit test in HexapodSim with `+π`, `-π`, `+3π`,
  `-3π`, `0`, small ε around ±π.


### T2.5 — Rename `FEMUR_OFFSET_ANGLE_RAD`

**Phase 1.** Independent hygiene.

**Source:** [`controller-audit.md`](controller-audit.md) §5.
**Effort:** XS.

**Do.** Rename to `TIBIA_OFFSET_ANGLE_RAD`. Add a one-line comment
explaining the convention (`compensate.femur = -src.femur`,
`compensate.tibia = src.tibia + TIBIA_OFFSET`).

**Verify.**
- *Static:* rebuild; pure rename should not change behavior.


### T2.6 — Remove dead fields and constants

**Phase 1.** Also removes the leftover `CTRL_LOOP_DT_S` from T1.1. Leave `attitude` alone: T5.2 will use it.

**Source:** [`controller-audit.md`](controller-audit.md) §5.
**Effort:** XS.

**Do.** Delete `servo_shared_state.initialized`,
`controller_ctx.delta_yaw`, `controller_ctx.pending_yaw`,
`CTRL_LOOP_DT_S`, `CTRL_SERVO_Z_OFFSET`. Drop the unused
`attitude` parameter from `controller_update` (or wire it up — see
T5.2).

**Verify.**
- *Static:* clean build with no warnings; grep for the removed
  names returns nothing.


### T2.9 — `calculate_motion_step` pace by 3D distance

**Phase 1.** Still needed: STANDUP and SITDOWN pace with `calculate_motion_step`.

**Source:** [`controller-audit.md`](controller-audit.md) §1.4.
**Effort:** XS.

**Do.** In [`calculator.c:25-26`](../Core/Src/calculator.c:25),
replace `remaining_distance_2d` with `fmaxf(2d_dist, fabsf(target.z -
current.z))` for the duration computation.

**Verify.**
- *Sim:* trigger STANDUP from a pose with one leg far below
  ground; confirm vertical motion is paced by `lift_velocity`,
  not by trivial XY distance.


### T2.4 — Reset `last_velocity` / `last_ticks` on RUNNING entry

**Phase 1.** Moot once T4.1 deletes the software interpolation; do only if T4.1 is far off.

**Source:** [`controller-audit.md`](controller-audit.md) §5.
**Effort:** XS.

**Do.** Zero `last_velocity` and set `last_ticks =
osKernelGetTickCount()` immediately before entering `SERVO_RUNNING`
(both at first entry and after each POWER_UP → SYNC → RUNNING
transition). Once T4.1 lands this is moot — `last_velocity` is
deleted entirely.

**Verify.**
- *Bench:* power-cycle the robot through POWERDOWN/POWERUP twice;
  observe smooth motion resumption (no first-tick jolt).


---

## Phase 2 — Servo bus and actuation

Move motion profiles into the servos, raise the bus speed, and read load so contact sensing becomes possible. Can run in parallel with Phase 3.

### T4.1 — Phase 1: XL430 profile, software interp removed

**Phase 2.** Design role: the servo takes over interpolation, which also removes the deadband stall (audit §4.6).

**Source:** [`servo-velocity-control-phase2.md`](servo-velocity-control-phase2.md) §1.
**Effort:** M.
**Depends on:** the bench experiment (T7.14) for the profile mode.
**Why:** Today the servo task does software trapezoidal
interpolation with deadband, vel-clamp, accel-clamp, min-step.
The XL430 can do this in hardware via `PROFILE_VELOCITY` /
`PROFILE_ACCELERATION`. Result: smoother motion, simpler servo
task, deadband-quantization issues (§4.4 interaction review) go
away.

**Status:** **implemented (2026-10-16), not yet verified on the robot.** Commits
`c93cd96` (units and packing in `Core/Src/servo_profile.c`, host test
`tests/servo_profile_test.c`) and `cb73897` (servo task). What was built, against
the plan below:
- Every goal is written with its profile as one 12-byte Sync Write at address 108
  (Profile Acceleration, Profile Velocity, Goal Position), not configured once in
  `PERIF_Dynamixel_Configure`: the servos keep the profile registers over a board
  reset (the bench once left values behind), so writing them with every goal means
  no stale value is used.
- The profile is fixed: `SERVO_PROFILE_VELOCITY` 4 rad/s and
  `SERVO_PROFILE_ACCELERATION` 40 rad/s² (the old software limit). The conversions
  never return 0 ("no limit" to the servo) and clamp at 4 rad/s and 150 rad/s²,
  because the Velocity Limit register does not cap the profile.
- Goals are written once per set of targets the controller publishes (10 Hz), not
  every servo tick: the shared state carries a sequence number and the duration of
  the targets (for T4.2). A failed write is repeated on the next servo tick. The
  limit check runs on the targets.
- The state Sync Read grows from 10 to 18 bytes (registers 126–143) and also returns
  Velocity Trajectory and Position Trajectory. The tracking error (Position
  Trajectory − Present Position) and the profile velocity are published in the
  shared state (`servo_copy_profile_tracking()`), to record the servo lag for T7.15
  and T7.6.
- Bus time per second: the write goes from 0.8 ms at 50 Hz to about 1.5 ms at
  10 Hz; the read grows by about 0.7 ms at 50 Hz (estimated, to be measured).

**Do.** Add `xl430_radps_to_profile_velocity()` and
`xl430_radps2_to_profile_acceleration()` helpers. Configure both
registers in `PERIF_Dynamixel_Configure`. In `SERVO_RUNNING`,
delete the per-joint clamp block and just convert
`target_joint_angles` → pulses → bulk write GOAL_POSITION. Keep
the joint-limit check.

**Verify.**
- *Static:* unit test the unit conversion against XL430
  datasheet examples.
- *Bench:* single joint, write GOAL_POSITION 1 rad away, time
  arrival under different `PROFILE_VELOCITY` values; confirm
  linear scaling.
- *Walk:* normal walking; foot motion should look visibly
  smoother (no audible servo "ticking" at low speeds — the
  former min-step quantization).


### T7.3 — Servo bus to 2 Mbps

**Source:** [`design.md`](design.md) Appendix A.
**Effort:** S.
**Depends on:** T2.13 recommended (accurate clock).
**Status:** **partly done** in `d6c1be3`: all 18 servos were set to 2 Mbps by
hand and `USART6` runs at 2000000 (the PC6 output speed went from medium to
high). Measured at 2 Mbps: Sync Read 3.18 ms, Sync Write 0.80 ms, no errors in
4,500 transactions each (design Appendix A.8). Open: step 1 below (a
replacement servo still has to be set up by hand, and a servo left at 1 Mbps
makes `PERIF_Dynamixel_Init` halt) and step 3.

**Do.**
1. Write a migration routine: try 2 Mbps, fall back to the current 1 Mbps, and
   write the Baud Rate register of each servo (torque off) so a half-migrated
   bus cannot strand any servo.
2. Set `USART6` to 2000000 (CubeMX region: the generated init is yours to
   change; the firmware side is the migration routine).
3. Re-verify Return Delay Time (currently 20 µs) against the STM32 TX→RX switch
   latency at the new rate.

**Verify.**
- *Bench:* logic analyzer on the bus; check the edge quality and the TX→RX
  switch margin against the 20 µs Return Delay Time. (The Sync Read time is
  measured: 3.18 ms against 2.16 ms in the first model. `tools/bus_budget.py`
  is now calibrated to it.)
- *Bench:* a few hundred cycles with zero CRC or timeout errors (done, the
  diagnostics report zero in 4,500 cycles).
- *Bench:* with a servo deliberately left at 1 Mbps, the routine finds it and
  moves it to 2 Mbps.


### T7.4 — Read load, velocity and position

**Source:** [`design.md`](design.md) §7.1, Appendix A.2.
**Effort:** S.
**Depends on:** register map confirmed against the XL430-W250-2 control table.
**Status:** **done (2026-10-13).** Verified on the robot: the load, timing and byte
stuffing checks below, and the units by the bench experiment (T7.14: velocity
0.229 rpm per unit, position and velocity registers read consistently with the
profile registers). Left over: the rare read timeouts, not explained. Since T4.1
(`cb73897`) the read is 18 bytes, registers 126–143, adding Velocity Trajectory and
Position Trajectory. Before that the servo task read
registers 126–135 for every servo with `dynamixel_get_bytes_multiple` (10 bytes:
load int16, velocity int32, position uint32) and publishes `load_pct[6][3]`
(0.1 % units converted to %) and `velocity_radps[6][3]` in `servo_shared_state`
through `servo_set_load_and_velocity()` / `servo_copy_load()` /
`servo_copy_velocity()`. The values are in the **servo frame**: the signs are not
the compensated joint frame (`compensate()` flips the femur sign). A
`[LOAD] f/t %: ...` debug line with the femur and tibia load of all legs is
printed every 5 s. The register addresses and units were confirmed on the robot.

**First run on the robot (log of 2026-10-11).** Read time 3.49/3.61–3.65/3.94 ms
(min/avg/max), as modelled (3.72 ms). No CRC errors, UART errors, dropped bytes
or alerts. Two findings:
- *Byte stuffing.* Dozens of `Failed to read all servos (error 71)` (length
  mismatch), worst while the robot moved or was lifted. Cause: the servo inserts
  an extra `0xFD` after every `FF FF FD` in a reply; a small negative velocity
  (`FF FF FF FF`) followed by a position whose low byte is `FD` triggers it, and
  the library did not remove it. Reproduced with a hand-built reply, fixed in
  the library (`0a49c5c`: stuffing of requests, un-stuffing of replies) and the
  firmware pin bumped. Confirmed: no `len` errors in 2,950 reads afterwards.
- *Rare timeouts.* About one per 3,000 cycles, always blamed on servo index 0,
  which means no reply at all (the request was lost or the first reply was
  missed). Each costs the 10 ms timeout and the 5 ms reset, and shows as one
  loop overrun. Not explained yet; the earlier 4,500-read window had none.

Load observations, servo frame, % of maximum (design §7.1): standing, femur
−11 to −20 and tibia −12 to +2; lifted, femur −3 to +3 and tibia −8 to 0. The
femur separates loaded from unloaded clearly, the tibia hardly. The values are
sticky: they hold for tens of seconds and drift slowly, depending on the last
movement.

Behavior changes that came with it:
- A servo that does not answer in a cycle now keeps its own last position; the
  other 17 are updated. Before, one failure meant no servo was updated.
- Only a timeout or bus error resets the UART receiver (with the 5 ms wait).
  A damaged reply is read in full, so the receiver stays in step and needs no
  reset.
- A servo reporting a hardware alert (flag `0x80`) no longer fails the whole
  read: its data is valid. A change in the set of alerting servos is logged and
  counted (`servo alerts`).
- The bus diagnostics gained a failed-reads-per-servo line (servo index =
  leg × 3 + joint), which answers the per-servo attribution follow-up in T7.12.

**Do.** Extend the per-tick Sync Read to start at Present Load and cover Present
Velocity and Present Position in one contiguous read (10 bytes per servo).
Publish `load[6][3]` and `velocity[6][3]` in `servo_shared_state` through
critical-section helpers. No behavior change yet.

**Verify.**
- *Bench:* log femur and tibia load with the robot standing and with a leg
  lifted; the loaded leg should show a clear difference.
- *Bench:* servo loop period unchanged with the larger read.


### T7.16 — Write the goals when the controller publishes

**Source:** review of the task rates after T4.1/T4.2, 2026-10-16.
**Effort:** S.
**Depends on:** T4.1 and T4.2 verified on the robot first.

**Why.** Since T4.1 the servos interpolate, and goals are written only when the
controller publishes new targets (10 Hz). The 50 Hz servo loop now writes on one tick
in five and otherwise only reads. A new target waits for the next servo tick: 0–20 ms,
fixed but arbitrary because the two tasks keep their phase. The Position Trajectory
that T4.2 measures Δ from is read on that tick, so it is up to 20 ms older than it could
be. Meanwhile the 50 Hz reads cost about 20 % of the bus and are hardly used yet (a
debug log of the load), but contact detection (T7.6) and odometry (T5.3) will need them.

**Do.** The control task sets an event flag after `servo_set_target_joint_angles()`.
The servo task waits for that flag or its 20 ms period, whichever comes first, and on
the flag does read → profile → write at once. Keep the 50 Hz reads for sensing. Measure
the time from publish to write with the DWT counter and add it to the bus diagnostics.

**Verify.**
- *Bench:* publish-to-write time about one read (~4 ms) instead of 0–20 ms; servo loop
  overruns unchanged; bus utilisation unchanged.
- *Walk:* no change in behaviour, or slightly less lag.

**Later.** With the faster control rate of [design.md](design.md) §7.5 (25 Hz), run the controller and
the servo cycle in lockstep at 25 Hz or more; the servo rate then becomes a sensing
choice, not a motion one.


### T7.15 — Servo position controller tuning

**Source:** first dry run of T7.14, 2026-10-12
([plan §6.1](servo-profile-bench-experiment.md)).
**Effort:** M, mostly bench time.

**Why.** The servos follow their profile late: the tibia, which follows it best, reaches
97 % of the profile's peak velocity but its position is a constant 43 to 44 ms behind the
profile (eight moves, full bench run); the loaded femur starts about 100 ms late and reaches
only 0.50 of 0.86 rad/s. A loaded joint also
stops short of the goal: the femur 21 pulses (32 mrad) at a load of 12 %, the tibia 10
pulses the other way at −5.7 %, about −1.7 pulses per % of load for both (the coxa,
without gravity load, is within 2 pulses). The position controller has no integral
action (gain 0), and the feedforward gains are 0 too. Any servo profile, time based or velocity based, is limited by this
lag on the loaded joints, and the walking controller works with positions that are off
by this much. The servos are also of two models (2XL430-W250 for coxa and femur, XL430
for the tibia), which may need different settings.

**What the manual says** ([XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/),
[2XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/)):
- In Position Control Mode only the position P, I, D gains and the two feedforward gains are
  used. The velocity P and I gains are used in Velocity Control Mode only, so they are left
  out of the tuning.
- The servo turns the goal into a trajectory with Profile Velocity and Acceleration, stores it
  in Velocity and Position Trajectory, then applies feedforward and PID, with Goal PWM as the
  limit. **Feedforward needs a profile:** with Profile Velocity 0 it has nothing to act on, so
  the feedforward gains land together with T4.1.
- The controller has an anti-windup gain, which the user cannot change.
- The XL430 has no current-based position mode (velocity, position, extended position and PWM
  only), so there is no goal current to limit the stiffness.
- The specifications are at 11.1 V (stall 1.4 N·m, no load 57 rev/min, about 6.0 rad/s): the
  5.9 rad/s measured at 11.0 V is the rated speed.
- The two axes of a 2XL430 share one communication module.
- A load near the maximum for some time sets the Overload error and switches the torque off;
  only a reboot clears it.

**Do.** The gains now (read by the bench identification): position P 640, I 0, D 2000
(servos 16 and 17) or 3600 (servo 18); feedforward 1st and 2nd 0 (velocity P 100, I 1800 or
1000, not used in position mode). On one leg, with the velocity-based computed profile:
- Step the feedforward 1st gain first (it is the one meant to remove the lag of a velocity
  profile), with more points between 100 and 300 (about 20–40 % of P), and expect different
  values for the femur and the tibia: the right value depends on the load.
- Then the position P and D gains (the tibia's higher D may be why it tracks best).
- Measure the lag and the shortfall with the bench harness (T7.14), at the standing load.
- For the shortfall that remains, prefer an offset of the goal by the measured load
  (about 1.7 pulses per %) in the firmware to integral action. Integral action, or a stiffer
  joint, adds overshoot and windup when the leg is blocked, and a stance leg that pushes for a
  long time moves towards the Overload shutdown. Decide with the data.

Settle on gains per joint and model, write them in `PERIF_Dynamixel_Configure` (they are RAM
and lost at power off).

**Verify.**
- *Bench:* with the new gains the loaded femur is within a few pulses of its goal at rest
  and follows a 100 ms profile within a small lag.
- *Loaded and walk:* no oscillation when standing and walking; the temperature of the servos
  stays normal; log the Hardware Error Status (overload, overheating) in these runs.

**Interaction.** The position minus profile difference is proportional to the load, so it
is a second indicator for contact detection (T7.6); changing the gains changes it.


### T7.14 — Servo profile bench experiment

**Plan:** [`servo-profile-bench-experiment.md`](servo-profile-bench-experiment.md).
**Effort:** M for the harness and analysis script (done), plus about an hour on the
bench.
**Status:** **done (2026-10-12), results in plan §6.3.** The code (bench firmware
`Core/Src/bench_profile.c` and `bench_script.c`, presets `BenchDry` and `Bench`, the
analysis script `tools/bench_profile_analyze.py` and their tests) is on the branch
`bench-test`, not on `main`. The dry runs and the full run of 103 runs on leg 0 went
through; the harness problems found on the way are in plan §6.1. The steps to prepare the
robot, flash and capture are in section 5 of the plan. A rerun with the fixed images is
possible (a control run without profile in the second half would settle the question of the
pose offset after the drive mode switch).
**Depends on:** nothing; T7.4 (done) gives the read it builds on.
**Needed by:** T4.1 and T4.2, which should not start before the result.

**Do.** Build the `Bench` firmware (leg 0 only, 1 kHz sampling of the servo's own
tick and trajectory registers), run the scripted sequence E1–E7 on the robot, and
analyse the log with `tools/bench_profile_analyze.py`.

**Decides.** Time-based or velocity-based servo profiles, or keeping the software
interpolation, from three open questions: does the motion chain smoothly across
goals, do joints with different travel arrive together in time-based mode, and
what happens when the requested time is too short.

**Verify.** Done: results in section 6.3 of the plan, T4.1/T4.2 and
`servo-velocity-control-phase2.md` updated to match.


### T7.13 — Single-pass byte stuffing in the dynamixel library

**Priority:** low, for later. **Effort:** S.
**Source:** log analysis of 2026-10-11, after the byte stuffing fix (library
`0a49c5c`, T7.4).

**Why.** With stuffing support the Sync Read went from 3.61–3.65 ms to
3.74–3.77 ms and the Sync Write from 0.83 ms to 0.96 ms on average. The wire
time did not change, so the extra ~0.12–0.13 ms per transaction is CPU time:
stuffing and un-stuffing run as byte-by-byte loops that call a helper per byte,
and `dynamixel_build_packet` scans the parameters twice (once to size, once to
copy). In the unoptimised Debug build that is about 1.2 % of one core at the
50 Hz servo loop. It is harmless today but it adds to every transaction, and it
grows with the number of servos and the payload (T4.2 writes 8–12 bytes per
servo).

**Do** (all in the `dynamixel` library):
1. `dynamixel_build_packet`: copy the parameters and stuff them in one pass,
   then patch the length field and compute the CRC. Check the buffer against the
   worst case up front (`param_length + param_length / 3`) so the size does not
   need a first pass, or check as it writes.
2. Fast path: if the parameters contain no `0xFF` (`memchr`), which is nearly
   always the case for goal data, use `memcpy` and skip the scan. Same for
   `dynamixel_unstuff`: no `0xFF` means nothing to remove.
3. Keep `dynamixel_stuffed_length` for callers that need the size, but do not
   call it from the packet builder.
4. Optionally add a Release build preset in the firmware and compare, since much
   of the cost is `-O0`.

**Verify.**
- *Static:* the library tests pass unchanged (143), including the randomized
  round trip against the reference implementation; add a test for the worst
  case buffer bound.
- *Bench:* the 30 s diagnostics (T7.12) show the Sync Read back at about
  3.6 ms and the Sync Write at about 0.83 ms.


### T4.2 — Phase 2: Per-joint synchronized profile

**Phase 2.** Write `PROFILE_ACCELERATION`, `PROFILE_VELOCITY` and `GOAL_POSITION` as one contiguous 12-byte Sync Write at address 108, only when a new control target is published (design Appendix A.2).

**Source:** [`servo-velocity-control-phase2.md`](servo-velocity-control-phase2.md) §1–§10.
**Effort:** M (after T4.1).
**Depends on:** the bench experiment (T7.14, done).
**Bench result (plan §6.3), to be weighed before starting.**
- **Variant:** velocity-based with `v = Δ/(T−Tr)`, `a = v/Tr` per goal, written with the goal. Time-based mode is
  not recommended: no tracking gain, 12 ms more lag, and an EEPROM drive mode.
- **Gain:** −5 to −33 % tracking error against plain goals at 25 Hz or faster, and a flatter speed at 10 Hz.
  At 25 Hz the modes are indistinguishable in the plots: the servo's own lag of 40–60 ms dominates, so T7.15
  (feedforward, gains) is probably worth more and comes first.
- **Not delivered:** joint synchronisation. The joints arrive 52–85 ms apart in every mode, because the servos
  follow their profiles differently (coxa slowest). The "finish together" motive below does not hold.
- **Cost:** 1.5 ms per write against 0.8 ms for the goal alone, at the control rate.
- **Also:** clamp the profile velocity in the firmware, the Velocity Limit register does not (joints reach about
  5.9 rad/s at most).
**Why:** Phase 1 leaves joints free to finish at different times,
so the foot's Cartesian path between control ticks is whatever
joint-space interpolation chooses. Phase 2 scales
`PROFILE_VELOCITY` per joint per tick so all joints in a leg
finish together, preserving the planned trajectory.

**Status:** **implemented (2026-10-16), not yet verified on the robot** (commit
`2fca015`). As built, replacing the implementation note's steps:
- The servo task, not the controller, computes the profile (`servo_computed_profiles()`
  in `main.c`). Per joint, Δ = the new goal − the servo's **Position Trajectory**, read
  in the same tick. Not the previous goal: with T longer than the period the profile
  has not quite reached it, and a small next move measured from it would crawl over
  the rest (25 mrad at 0.056 rad/s takes half a second). Not the measured position,
  which lags the profile by 40–60 ms. A servo that did not answer the read falls back
  to its previous goal.
- T = `SERVO_PROFILE_T_FACTOR` (1.2) × the published `target_duration_s`, Tr =
  `SERVO_PROFILE_TR_FRACTION` (0.25) × T, then `servo_profile_compute()`:
  v = |Δ|/(T−Tr), a = v/Tr.
- **Acceleration floor** `SERVO_PROFILE_MIN_ACCELERATION_RADPS2` (40 rad/s²): the
  servo also brakes with Profile Acceleration. A tiny move after a fast one (the end
  of a femur lift, the walk-to-stand transition) would otherwise give a ≈ 0 and the
  joint would take seconds to stop and run far past the goal. The bench paths (ramp,
  sine) change smoothly and do not show this; the gait's bursts do.
- `SERVO_PROFILE_MODE` (`main.c`) selects `COMPUTED` (default) or `FIXED` (T4.1) to
  compare on the robot. The first goals after a sync keep the fixed profile.
- Left: choose the T factor from the E4 log per factor (0.8, 1.2, 1.5); the docs only
  report the modes.

**Do (original).** Superseded by the status above.

**Verify.** Eight-step plan in the implementation note §8, except the
synchronisation, which the bench ruled out. Highlights:
- *Bench:* replay realistic gait sequences (walk 50/100 mm/s, turn) with `FIXED`
  and `COMPUTED`; the published profile velocity
  (`servo_copy_profile_tracking()`) should stay steady through a tick instead of
  peaking and falling to 0, with no run-on after a femur lift and no crawl into
  standing.
- *Walk:* high-speed walking; foot path should track the
  planned lift trapezoid closely (visible on a marked floor
  or via simulator overlay).


### T3.5 — Servo-bandwidth model in controller

**Phase 2.** Design role: provides `T_min` and the speed limits for the twist model (design §5.2, T7.8).

**Source:** [`controller-audit.md`](controller-audit.md) §4.4 (P2).
**Effort:** S.

**Do.** Compute:

```c
#define SERVO_MAX_DELTA_PER_CTRL_TICK_RAD  \
    (SERVO_MAX_VELOCITY * (CONTROL_LOOP_INTERVAL_MS * 1e-3f))
```

After IK in WALKING / ROTATING, check that every joint's
`|next - actual|` is below this; if not, scale `velocity` down
proportionally (or fault if it's already at minimum).

**Verify.**
- *Sim:* command unrealistic velocities; observe the controller
  clamps automatically, not the servo task silently.
- *Bench:* induce a large per-tick command (e.g. via a large
  yaw_error); confirm the lag visible in `actual_joint_angles`
  shrinks.


### T7.2 — Control loop at 25 Hz

**Source:** [`design.md`](design.md) §7.5, Appendix B.
**Effort:** S.
**Depends on:** T1.6 (logging), T2.12 (to measure), T1.1.

**Do.**
1. `CONTROL_LOOP_INTERVAL` to `HZ_TO_INTERVAL(25)` (40 ms) and re-band the
   control `dt_s` clamp around the new nominal tick (half to double:
   0.02–0.08 s).
2. Review every constant that is "per tick" (`CTRL_MAX_YAW_RATE`,
   `CTRL_MAX_ROT_FOOT_MM`, and similar in `controller_config.h`) and convert it
   to a per-second value multiplied by `dt_s`, otherwise the effective rates
   change by 2.5×.
3. Make the liveness thresholds (T1.5) and the idle timer time-based rather
   than counted in ticks.

**Verify.**
- *Bench:* T2.12 shows control task CPU and period jitter within budget.
- *Walk:* 50 mm/s on a tape-measured line gives the same distance as at 10 Hz.


---

## Phase 3 — Command interface

The Pi-facing protocol: one atomic command, CRC, status reply and link supervision. Needs only Phase 1; can run in parallel with Phase 2.

### T7.5 — Command interface: twist frame, CRC, status reply, link supervision

**Source:** [`design.md`](design.md) §3.
**Effort:** M.
**Depends on:** T1.6.
**Status:** in progress, in slices. Slice 1 (the frame, CRC, status frame and sequence
handling: `Core/Src/cmd_frame.c`, host test `tests/cmd_frame_test.c`) is done and
fixes the layout (31 bytes, design §3.2). To do: the adapter from the twist to the
current controller input with `T_LINK` (slice 2), the full-duplex SPI task (slice 3),
the Pi side `SpiSink` in `poom-remote` (slice 4). The controller logic is not changed
until T7.8.

**Do.**
1. Define the single fixed-size command frame (`magic`, `version`, `seq`,
   `flags`, `vx`, `vy`, `wz`, `height`, tilt, `gait`, `crc16`).
2. In the SPI task: receive, validate magic, version and CRC, drop duplicates by
   `seq`, and write a `cmd_shared` struct through critical-section helpers
   (atomic command, replacing the three `updated_*` globals).
3. Fill a status frame as the reply on the same transfer. Give it a version
   byte and a length (decision D7): v1 is state, fault, yaw and per-leg contact
   bits; odometry pose, roll and pitch, battery and servo health are added later
   as the Pi needs them (design §3.2). The Pi ignores unknown fields.
4. Link supervision: no valid frame for `T_LINK` (300 ms) forces a zero twist,
   so the robot stops and stands; there is no separate dead-link behavior
   (decision D5). Keep the old `0x01`–`0x03` commands only during the transition.
5. Add a Pi-side test sender under `tools/`.

**Verify.**
- *Bench:* Pi test sender; corrupt CRC, drop frames and stop sending, and
  confirm the twist goes to zero after `T_LINK`.
- *Bench:* sustained 25–50 Hz command rate with no per-frame logging.


---

## Phase 4 — Frames, sensing and contact

The sensing the motion core depends on: level frame from the IMU, ground contact, odometry, and simulator support so later items can be verified without hardware.

### T5.2 — IMU integration (horizon hold)

**Phase 4.** Design role: this is the **level frame** (design §4): gravity-aligned, body yaw, from the EKF.

**Source:** [`controller-audit.md`](controller-audit.md) §3.4 D10.
**Effort:** L.

**Do.** Pre-multiply the body-to-world transform by the EKF's
estimated roll/pitch before computing leg targets. The simplest
first cut: rotate the foot target so `z = -body_z` *in world
frame*, not body frame.

**Verify.**
- *Sim:* (if sim supports inclines) tilt the ground 15° and walk
  across; legs should remain at constant load.
- *Walk:* on a ramp or wedge, walk up/across; body should stay
  level relative to gravity, not relative to the ramp.


### T7.6 — Ground contact detection

**Source:** [`design.md`](design.md) §7.
**Effort:** M.
**Depends on:** T7.4.

**Do.** In the servo task: maintain an unloaded-load baseline per leg,
calibrated by a per-leg lift test once after each MCU reset, during the first
STANDUP (decision D8, design §7.6: one leg at a time, load recorded at a few
poses between stance and swing height; STANDING is not reported until it is
done; an incomplete test leaves the kinematic fallback active and is retried at
the next STANDUP). Compute `excess` over femur and tibia,
declare contact after `N` consecutive samples above `T_on` and release below
`T_off`. Publish `contact[6]` and its source. When load is unavailable use the
kinematic fallback (`tip_world_z ≤ ε_z`, flat ground). Log load/kinematic
disagreements.

**Verify.**
- *Bench:* press and release each foot; contact flips cleanly with no chatter.
- *Bench:* disable the load read to confirm the fallback path engages.
- *Bench:* after a reset the lift test runs on all six legs and the baseline is
  not recalibrated at a later standup; interrupt the test (drop the command
  link) and confirm the fallback is used and the test is retried.


### T7.7 — Simulator support for the new design

**Source:** [`design.md`](design.md) §4, §7.
**Effort:** M.

**Do.** The HexapodSim Controller project links `controller.c`, so every
interface change in this backlog (command type, contact, level frame) must keep
building there. Add simulated ground contact flags and a twist input to the
simulator, and the status/contact plumbing the controller expects, so the
"Sim" verification steps in later items are possible.

**Verify.**
- *Sim:* a scripted run (stand up, walk, strafe, rotate, stop) completes with
  contact flags reflecting the simulated ground.


### T5.1 — Body-Z-aware reach disk

**Phase 4.** Design role: `R(body_z)` is the reach radius used by the stride calculation (T7.8).

**Source:** [`controller-audit.md`](controller-audit.md) §3.2 M1, §3.4 D8.
**Effort:** M.

**Do.** Derive `R_disk(body_z)` from the leg's femur/tibia
lengths and the home pose. Either compute it once per height
change and cache, or recompute every WALKING tick (cheap — one
sqrt + one min).

**Verify.**
- *Static:* spreadsheet the disk radius vs body Z; confirm
  monotonic and matches geometric expectation.
- *Sim:* walk at body Z = 60, 100, 140 mm; confirm reach disk
  changes accordingly.
- *Walk:* command extreme heights and confirm no IK failures
  during walking.


### T5.3 — Leg odometry / close the body-pose loop

**Phase 4.** Design role: odometry from confirmed-contact stance feet (design §5.5). Needs T7.6.

**Source:** [`controller-audit.md`](controller-audit.md) §4.3 (P3).
**Effort:** XL.

**Do.** On every control tick, before publishing targets:

1. For each grounded leg, compute its current world position
   from `forward_kinematics(actual_joint_angles)` plus the
   *previous* body world pose.
2. Estimate the body's *physical* world pose as the rigid
   transform that best maps `tip_world_coordinates[stance_legs]`
   to those current world positions (least-squares).
3. Set `ctx->robot.hexapod` to the estimated pose.
4. Continue with WALKING as before.

This naturally combines with T5.2 (the leg odometry gives x/y/yaw;
the IMU gives roll/pitch — they're orthogonal). Together they're
a full odometry-EKF stack.

**Verify.**
- *Sim:* with simulated servo lag, confirm grounded feet stop
  slipping (their world positions stay fixed).
- *Walk:* mark a starting position on the floor, walk a 1 m
  square, return; measure final offset from start. Should be
  closer to zero after this change.


---

## Phase 5 — Motion core

The rewrite of walking and rotating: twist model, gait clock, support rules and lifecycle. Needs the command type (Phase 3) and contact (Phase 4).

### T7.8 — Unified twist motion model

**Source:** [`design.md`](design.md) §5.
**Effort:** L.
**Depends on:** T7.5 (command type), T1.1; level frame (T5.2) can start as the
identity.

**Do.**
1. Take `(vx, vy, wz)` in the level frame and compute each foot's velocity
   `u_i = v + wz × h_i`; replace the summed `effective_velocity` and the
   separate WALKING / ROTATING math with this one formula and remove the
   ROTATING state.
2. Stride and cycle time from the fastest foot: `T_stance = 2R / max|u_i|`,
   `T_cycle` clamped to `[T_min, T_max]`; constant stride above
   `v_crit = 2R / T_max`, proportional below it (decision D2).
3. Command shaping in order: validate, clamp magnitudes, scale the twist so the
   fastest foot is feasible (keep direction), then slew-limit.
4. Speed limits from the servo bandwidth model (T3.5).

**Verify.**
- *Sim:* forward, backward, strafe at several angles, rotate in place, arcs,
  direction reversal, and very low speed.
- *Walk:* tape-measured line and a marked 360° rotation.


### T7.9 — Gait scheduler (clock, support rules, contact gating)

**Source:** [`design.md`](design.md) §6, §7.4.
**Effort:** L.
**Depends on:** T7.6 (contact), T7.8.

**Do.**
1. Represent a gait as `{duty factor, phase offset[6]}`; add the tripod table.
2. A single gait clock advances at a rate set by the cycle time; each leg's
   local phase decides stance or swing.
3. A swing ends on the earlier of nominal landing plus descent, confirmed
   contact, or a maximum descent depth; the clock holds until the leg due to
   land has landed or timed out.
4. A leg may only lift if the remaining stance legs are in confirmed contact and
   their support polygon contains the COM projection (level frame).
5. Choose the initial phase from the actual foot heights; keep a reach-disk
   check as a safety net.
6. Replace `swap_legs` and the swing-arrival trigger.

**Verify.**
- *Sim:* normal walking, direction reversal, noisy command input, a foot that
  lands early (step) or late (hole).
- *Bench:* lift one tripod's contact signal; the gait must hold rather than
  lift the other tripod.


### T7.10 — Lifecycle: STOPPING, SITDOWN, RESTING, idle timers

**Source:** [`design.md`](design.md) §8.
**Effort:** M.
**Depends on:** T1.2 (FAULT), T7.9 (STOPPING).

**Do.**
1. States `RESTING`, `STANDUP`, `STANDING`, `MOVING`, `STOPPING`, `SITDOWN`,
   `FAULT` as in design §8.1; STANDUP sequences legs out one at a time if the
   starting pose is unknown (rest on the ground, decision D6).
2. `STOPPING`: finish swings, recenter feet to home in alternating groups,
   body level; `STANDING` handles height changes.
3. Timers: `T_LINK` forces zero twist, so a lost link stops the robot and
   leaves it standing (decision D5); `T_IDLE` (120 s) is the time spent standing
   and triggers `SITDOWN`, which lowers the body to the ground and then drops
   torque. It is the same timer whether the Pi sends zero twist or is gone. A
   lost link does not interrupt `STANDUP`; it completes into `STANDING`.
4. `RESTING`: servo bus quiet (torque off, no reads); wake on the first
   non-idle valid command.
5. After an MCU reset the first `STANDUP` includes the leg lift test (T7.6,
   decision D8) before it reports `STANDING`.

**Verify.**
- *Sim:* walk, stop, wait for `T_IDLE`; the body sits and torque goes off, then
  wakes on a command.
- *Bench:* repeat on the robot; legs must not flop on power-down.
- *Bench:* stop the command stream while walking: the robot stops, ends in
  `STANDING`, and sits down after `T_IDLE`.


### T5.6 — Emergency stop input

**Phase 5.** Design role: bypasses `SITDOWN` and drops torque; needs the lifecycle (T7.10).

**Source:** [`controller-audit.md`](controller-audit.md) §2.6.
**Effort:** S (plumbing) or M (with a real GPIO + debounce).

**Do.** Wire an external GPIO (or a dedicated SPI command) to
directly drive `CTRL_FAULT` (or a new `CTRL_ESTOP`). On trigger:
disable torque immediately via `servo_set_request_powerdown`.

**Verify.**
- *Bench:* press the e-stop while walking; confirm immediate
  torque drop; controller goes to fault state.
- *Walk:* same test while in motion; robot collapses
  controllably (legs go limp, not crash).


---

## Phase 6 — Extensions

Only after the motion core is solid.

### T7.11 — Ripple gait

**Source:** [`design.md`](design.md) §6.1.
**Effort:** M.
**Depends on:** T7.9.

**Do.** Add a ripple table (duty factor about 2/3, six evenly spaced phase
offsets; confirm values on the robot) and the speed limits that follow from the
different duty factor. The support rule is already generic.

**Verify.**
- *Sim:* ripple walking with the support rule never violated.
- *Walk:* compare speed and stability with tripod.


### T6.3 — Degraded gait if a leg faults

**Phase 6.** Extension after the gait scheduler (T7.9).

**Source:** [`controller-audit.md`](controller-audit.md) §3.2 M7.
**Effort:** XL.

**Do.** Introduce a per-leg `enabled` flag. On servo fault,
disable the faulted leg, fold it up, and switch gait planner to a
5-leg variant (wave or modified tripod with the fault-side legs
re-grouped).

**Verify.**
- *Sim:* artificially disable one leg; confirm walking continues
  with reduced speed.
- *Walk:* same — but only after extensive sim validation; this
  is a high-risk feature.


---

## Suggested rhythm

| Time available | Pick from |
|----------------|-----------|
| 30 min on the couch | Phase 1 hygiene (T2.3, T2.5, T2.6, T2.9), T7.1 |
| One evening | A single Phase 1 item (T1.6, T2.12, T1.2, T2.13) |
| A weekend | One Phase 2 item (T7.15, T4.1, T7.3) or T7.5 |
| A focused week | T4.1 + T4.2, or T7.6 (contact), or T7.5 + the Pi test sender |
| A long break | The motion core: T7.8 + T7.9 + T7.10, with T7.7 first so it can be verified in sim |

Do **all of Phase 1 before anything else**: T1.6 and T2.12 give you the
measurements the rest of the plan assumes, and T1.2 and T1.5 build the safety net
that the extra command activity of the new motion model will exercise. The
highest-leverage single hour is **T1.6**, since it unblocks both the 25 Hz
control rate and a usable command rate from the Pi.

---

## Superseded items

These were replaced by the design. Their original text is kept for reference:
the fix ideas and verification steps are still useful inputs to the replacing
item.

| Item | Replaced by | Why |
|------|-------------|-----|
| T1.3 | T7.5, T7.8 | Validation moves to the command interface (T7.5); clamping, scaling and slew are command shaping in the twist model (T7.8). The SPI handler already range-checks today. |
| T1.4 | T7.10 | STANDING height handling is part of the lifecycle rewrite. Only worth an interim fix if height changes are needed before Phase 5. |
| T2.1 | T7.8 | Pacing comes from the fastest foot, not `fminf(stance, swing)`. |
| T2.2 | T7.8 | Per-foot velocity `u_i = v + wz × h_i` uses the home position `h_i`. |
| T2.7 | T7.10 | Re-anchoring on entering a motion state is part of the new state entry rules. |
| T2.8 | T7.8 | Each foot has its own `|u_i|` by construction. |
| T2.10 | T7.9 | Foot-outside-disk is a safety net inside the gait scheduler. |
| T2.11 | T7.5 | The single command frame replaces the per-parameter globals. |
| T3.1 | T7.8 | Cycle-time bounds are part of the stride and cycle model. |
| T3.2 | T7.8 | Slew is the last step of command shaping. |
| T3.3 | T7.9 | Swing path is already computed once per swing (`9d57934`) and the new swing ends on contact. Re-evaluate whether a commit point is still needed after T7.9. |
| T3.4 | T7.9 | Support rules plus the gait clock replace the swap triggers. |
| T5.4 | T7.9 | The gait's initial phase is chosen from actual foot heights. |
| T5.5 | T7.10 | STANDUP and SITDOWN sequence the legs; unknown-pose checks live there. |
| T6.1 | T7.8 | Strafing is core to the unified twist, not a late feature. |
| T6.2 | T7.9 | The COM-in-support-polygon check is part of the gait's lift rule. |

### T1.3 — Clamp and slew-limit command inputs

**Superseded by T7.5, T7.8.** Validation moves to the command interface (T7.5); clamping, scaling and slew are command shaping in the twist model (T7.8). The SPI handler already range-checks today.

**Source:** [`controller-audit.md`](controller-audit.md) §2.3, §3.4 D6, §3.2 M6.
**Effort:** S.

**Do.** At the top of `controller_update`:

```c
velocity = clampf(velocity, -CTRL_MAX_VEL_MMS, +CTRL_MAX_VEL_MMS);
height   = clampf(height,    CTRL_MIN_HEIGHT, CTRL_MAX_HEIGHT);
heading  = wrap_to_pi(heading);

float32_t v_slew = clampf(velocity - ctx->filtered_velocity,
                          -CTRL_VEL_SLEW * dt_s, +CTRL_VEL_SLEW * dt_s);
ctx->filtered_velocity += v_slew;
velocity = ctx->filtered_velocity;
```

…and feed `velocity` into the rest of the function. Pick
`CTRL_MAX_VEL_MMS ≈ 150 mm/s`, `CTRL_VEL_SLEW ≈ 200 mm/s²`.

**Verify.**
- *Static:* unit test the clamp/slew in HexapodSim with a few
  pathological inputs (huge, NaN, step from +max to −max).
- *Sim:* command step changes; observe smooth ramp in
  `filtered_velocity`.
- *Walk:* slam the joystick from +full forward to +full reverse;
  body should decelerate-and-reaccelerate, not lurch.


### T1.4 — STANDING-state height handling

**Superseded by T7.10.** STANDING height handling is part of the lifecycle rewrite. Only worth an interim fix if height changes are needed before Phase 5.

**Source:** [`controller-audit.md`](controller-audit.md) §1.2.
**Effort:** S.

**Do.** Pick one of:
- (preferred) Run a STANDUP-style motion step in STANDING whenever
  `|cmd.height - body.translation[2]| > ε`. Reuse
  `calculate_motion_step`; target is the current `tip_home` with
  the new body Z.
- (alternative) Refuse height changes in STANDING; only allow them
  in STANDUP or WALKING. Update the height-mirror condition in
  [`controller.c:175`](../Core/Src/controller.c:175) to exclude
  STANDING.

**Verify.**
- *Sim:* command a height change while STANDING; confirm legs
  re-pose smoothly (preferred fix) or the change is rejected and
  applied at the next motion state (alternative fix).
- *Walk:* on the bench, command +20 mm height while STANDING;
  confirm body lifts (or the height takes effect on the next walk
  command without snapping).


### T2.1 — `longest_path = longest_stance` explicit

**Superseded by T7.8.** Pacing comes from the fastest foot, not `fminf(stance, swing)`.

**Source:** [`controller-audit.md`](controller-audit.md) §3.4 D1.
**Effort:** XS.

**Do.** In [`controller.c:447`](../Core/Src/controller.c:447) and
[`controller.c:619`](../Core/Src/controller.c:619), replace the
`fminf` line with `longest_path = longest_stance;` (and rename
the accumulator to match).

**Verify.**
- *Static:* read the code; the change should be obviously
  equivalent in steady state.
- *Sim:* compare commanded-vs-actual body velocity over a
  multi-cycle WALKING run before/after. Should be identical in
  steady state; edge-case overshoot disappears at transients.


### T2.2 — `rot_disp` moment arm from `tip_home`

**Superseded by T7.8.** Per-foot velocity `u_i = v + wz × h_i` uses the home position `h_i`.

**Source:** [`controller-audit.md`](controller-audit.md) §3.4 D5.
**Effort:** XS.

**Do.** In [`controller.c:385-400`](../Core/Src/controller.c:385)
and the parallel block in ROTATING
([`controller.c:556-571`](../Core/Src/controller.c:556)), build
`r_perp` from `leg->tip_home`, not `p_current`. Use `p_current`
only for the path start.

**Verify.**
- *Sim:* command a slow constant yaw rate for 30 s; track
  heading drift before/after. Should be flatter.
- *Walk:* on a marked floor, rotate in place 360° forward then
  360° back. Should return closer to start.


### T2.7 — Re-anchor `next_joint_angles` on motion-state entry

**Superseded by T7.10.** Re-anchoring on entering a motion state is part of the new state entry rules.

**Source:** [`controller-audit.md`](controller-audit.md) §5.
**Effort:** XS.

**Do.** Extend the transition handler in
[`controller.c:243-250`](../Core/Src/controller.c:243) to also reset
on entry to WALKING and ROTATING:

```c
if (to == CTRL_STANDUP || to == CTRL_WALKING || to == CTRL_ROTATING) {
    for (int i = 0; i < 6; i++) {
        memcpy(ctx->robot.leg_state[i].next_joint_angles,
               ctx->robot.leg_state[i].actual_joint_angles,
               sizeof(float32_t) * 3);
    }
}
```

**Verify.**
- *Sim:* alternate WALKING ↔ STANDING for 30 cycles; observe no
  cumulative drift in commanded-vs-actual at each transition.


### T2.8 — `effective_velocity` per-leg

**Superseded by T7.8.** Each foot has its own `|u_i|` by construction.

**Source:** [`controller-audit.md`](controller-audit.md) §3.4 D7.
**Effort:** S.

**Do.** Compute `effective_velocity` per leg as `|arc_disp|` of
that leg, not as a global sum. Substeps then become a per-leg
quantity; `step_length = path_length / per_leg_substeps`.

**Verify.**
- *Sim:* command translation + yaw simultaneously; trace the
  per-leg trajectory and confirm landing-together property
  improves.


### T2.10 — Detect foot outside reach disk

**Superseded by T7.9.** Foot-outside-disk is a safety net inside the gait scheduler.

**Source:** [`controller-audit.md`](controller-audit.md) §3.4 D4.
**Effort:** S.

**Do.** In the WALKING leg loop, before path generation:

```c
float32_t in_disk = arm_euclidean_distance_f32(p_current,
                                                leg->tip_home, 2);
if (in_disk > CTRL_LEG_RADIUS_MM + 2.0f && leg->grounded) {
    leg->grounded = 0;   /* force this leg to swing next */
}
```

**Verify.**
- *Sim:* nudge a grounded leg's `actual_joint_angles` to be
  outside the disk; confirm the next WALKING tick lifts that leg.
- *Walk:* push the robot gently mid-walk so one foot slides;
  controller should compensate by lifting that leg.


### T2.11 — Atomic command frame

**Superseded by T7.5.** The single command frame replaces the per-parameter globals.

**Source:** [`controller-audit.md`](controller-audit.md) §4.7 (P5).
**Effort:** S.

**Do.** Replace the three `updated_velocity / heading / height` globals
(`main.c:299-301`) with a `cmd_shared_state` struct (superseded by the single
command frame in [`design.md`](design.md) §3 if that design is adopted) guarded by the same
critical-section helpers as `servo_shared_state`. The SPI handler writes all
fields in one call; the control task snapshots them in one call.

**Verify.**
- *Static:* grep shows no direct access to the old globals.
- *Bench:* send alternating command frames with distinct
  `(velocity, heading)` pairs; log what the control task reads and confirm it
  never sees a mixed pair.


### T3.1 — Cycle-time bounds

**Superseded by T7.8.** Cycle-time bounds are part of the stride and cycle model.

**Source:** [`controller-audit.md`](controller-audit.md) §3.2 M5, §3.4 D9.
**Effort:** M.

**Do.** Compute `T_cycle` at the top of WALKING:

```c
const float32_t T_cycle = 2.0f * CTRL_LEG_RADIUS_MM
                       / fmaxf(fabsf(velocity), CTRL_EPS_VEL);
if (T_cycle < CTRL_T_CYCLE_MIN) {
    velocity = copysignf(2.0f * CTRL_LEG_RADIUS_MM / CTRL_T_CYCLE_MIN,
                         velocity);  /* clamp v */
}
if (T_cycle > CTRL_T_CYCLE_MAX) {
    ctx->next_state = CTRL_STANDING; /* refuse to walk; fall back */
    return;
}
```

`CTRL_T_CYCLE_MIN` derived from servo bandwidth (initially ~0.25 s
empirically; refine after T3.5). `CTRL_T_CYCLE_MAX` set by feel
(~4 s).

**Verify.**
- *Sim:* command sweep v from 0 → 300 mm/s; observe velocity is
  clamped at the cycle-time-min boundary.
- *Walk:* command `velocity = 1000 mm/s`; robot should walk
  smoothly at the max sustainable speed, not lurch.


### T3.2 — Controller-level slew on velocity & heading

**Superseded by T7.8.** Slew is the last step of command shaping.

**Source:** [`controller-audit.md`](controller-audit.md) §3.2 M6, §3.4 D6.
**Effort:** S (already part of T1.3 if combined).

**Do.** Already covered by T1.3 (which also adds the velocity
slew). Add a matching heading slew if needed — currently the
yaw-rate clamp inside the controller already serves this purpose
implicitly.

**Verify.** Same as T1.3.


### T3.3 — Swing-leg commit point

**Superseded by T7.9.** Swing path is already computed once per swing (`9d57934`) and the new swing ends on contact. Re-evaluate whether a commit point is still needed after T7.9.

**Source:** [`controller-audit.md`](controller-audit.md) §3.2 M2, §3.4 D2.
**Effort:** M.

**Do.** Add to `leg_state`:

```c
float32_t swing_phase;        /* [0..1] along the swing arc */
float32_t committed_target[3]; /* frozen once swing_phase ≥ commit */
uint8_t   committed;
```

In the WALKING swing branch:

```c
if (!leg->committed) {
    /* existing re-projection */
    if (leg->swing_phase > 0.6f) {
        memcpy(leg->committed_target, p_target_body, 3 * sizeof(float32_t));
        leg->committed = 1;
    }
} else {
    /* plan against committed_target */
}
```

Reset `committed = 0; swing_phase = 0;` on swap (in `swap_legs`).

**Verify.**
- *Sim:* inject noise on `cmd.heading` while walking; before the
  fix the swing legs flutter and miss touchdown; after, they land
  cleanly.
- *Walk:* slowly walk on rough-ish floor; swings should look
  decisive once past mid-arc.


### T3.4 — Stance-edge swap safety net

**Superseded by T7.9.** Support rules plus the gait clock replace the swap triggers.

**Source:** [`controller-audit.md`](controller-audit.md) §3.2 M3, §3.4 D3.
**Effort:** S.

**Do.** Track stance disk margin in the WALKING leg loop:

```c
if (leg->grounded) {
    float32_t r = arm_euclidean_distance_f32(p_current,
                                              leg->tip_home, 2);
    stance_min_margin = fminf(stance_min_margin,
                              CTRL_LEG_RADIUS_MM - r);
}
```

After the loop:

```c
if (stance_min_margin < CTRL_DISK_SWAP_MARGIN ||
    remaining_path_length < CTRL_CLOSE_THRESH) {
    swap_legs(ctx);
}
```

`CTRL_DISK_SWAP_MARGIN ≈ 2 mm`.

**Verify.**
- *Sim:* execute a 180° direction reversal mid-cycle; confirm a
  swap fires when stance legs reach the disk edge, not when
  swing legs are still mid-flight.
- *Walk:* reverse direction at moderate speed; gait should
  recover within one extra swap, no scrape.


### T5.4 — Tripod selection from actual foot heights

**Superseded by T7.9.** The gait's initial phase is chosen from actual foot heights.

**Source:** [`controller-audit.md`](controller-audit.md) §2.5.
**Effort:** XS.

**Do.** In `tripod_init_gait`, pick the three legs with the
highest current z (per `forward_kinematics(actual)` →
`Thexapod_body` transform) as the next swing tripod, instead of
hard-coding the indices.

**Verify.**
- *Bench:* manually pose three random legs above ground; start
  WALKING; confirm those legs swing first, not the hard-coded
  set.


### T5.5 — Powerdown pose verification

**Superseded by T7.10.** STANDUP and SITDOWN sequence the legs; unknown-pose checks live there.

**Source:** [`controller-audit.md`](controller-audit.md) §2.4.
**Effort:** S.

**Do.** Add a pre-STANDUP step that lifts each leg individually
(or in pairs) before global STANDUP. Or: refuse to power back on
if any joint angle is outside `r.leg[i].limits[j]` by more than
a margin.

**Verify.**
- *Bench:* manually move legs into "bad" poses while powered
  down; power back up; confirm safe motion sequence.


### T6.1 — Strafing mode (body-frame velocity)

**Superseded by T7.8.** Strafing is core to the unified twist, not a late feature.

**Source:** [`controller-audit.md`](controller-audit.md) §1.3; [`control-model.md`](control-model.md).
**Effort:** L.

**Do.** Add `v_body_x` and `v_body_y` to `controller_command_t`.
Use them directly as the `movement_vector` in body frame
(skipping the heading rotation). Add an SPI / joystick toggle to
select arc vs strafe mode. While in strafe mode, `heading` is
controlled independently.

**Verify.**
- *Sim:* command pure sideways motion; body translates without
  rotating.
- *Walk:* sidestep along a marked line; confirm minimal
  forward drift.


### T6.2 — COM stability check

**Superseded by T7.9.** The COM-in-support-polygon check is part of the gait's lift rule.

**Source:** [`controller-audit.md`](controller-audit.md) §3.2 M4.
**Effort:** L.

**Do.** On every potential swap, compute the projected body-COM
position in body frame (approximately `(0, 0)`) and check that
the would-be stance triangle (legs 0,2,4 or 1,3,5) contains it.
If not, defer the swap by one tick or refuse the commanded yaw
rate.

**Verify.**
- *Sim:* command aggressive yaw + translation; confirm the
  controller hesitates rather than tipping.
