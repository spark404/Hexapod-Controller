> **ARCHIVED (2026-07-20).** Superseded by [controller-redesign-requirements.md](../controller-redesign-requirements.md) (leading plan) and [improvement-backlog.md](../improvement-backlog.md). Unique findings from this document have been folded into those. Code line references reflect the May 2026 state of the sources.

# Hexapod Controller — Improvement Backlog

A consolidated, prioritized list of everything discussed across the
review documents. Each task is sized, sourced, and has a verification
proposal so you can pick work that fits the time you have without
losing track of where it came from or how to validate it.

## How to read this

**Tiers** are priority bands. Tier 1 fixes correctness or stability
issues that bite under normal use. Tier 2 is quick-win hygiene. Tier 3+
adds robustness features and structural improvements. Within a tier,
items are in suggested execution order.

**Effort** sizes (rough hours of focused work, excluding validation):
XS = ≤30 min, S = 1–2 h, M = ½ day, L = 1–2 days, XL = > 2 days.

**Verification** suggests a layered approach:
- **Static** — review / unit test in the [HexapodSim Controller
  project](/Users/htrippaers/Projects/Personal/HexapodSim/Controller),
  which links the same `controller.c` and friends.
- **Sim** — full simulator run (HexSpider + Controller) to verify
  behavior without risk to hardware.
- **Bench** — hardware test with the robot on a stand / legs free,
  e.g. for individual joint or single-leg behavior.
- **Walk** — full on-ground walking, joystick-driven.

Use the lowest layer that proves the change; promote to the next layer
once it passes.

**Sources** point back to:
- [`controller-gaps.md`](controller-gaps.md) — §1, §2, §3, §4 (full
  audit, severity-tagged).
- [`walking-model-review.md`](walking-model-review.md) — M-items
  (model gaps), D-items (code-vs-model discrepancies).
- [`controller-servo-interaction-review.md`](controller-servo-interaction-review.md) —
  §4 (interaction gaps), P-items (proposed protocol additions).
- [`servo-velocity-control-phase2.md`](servo-velocity-control-phase2.md) —
  XL430 profile control plan.

---

## Quick index

| Tier | Theme                            | Item count |
|------|----------------------------------|-----------:|
| 1    | Critical correctness & stability |          5 |
| 2    | Quick-win hygiene                |         10 |
| 3    | Walking-model robustness         |          5 |
| 4    | Servo control upgrades           |          3 |
| 5    | Structural improvements          |          6 |
| 6    | New features                     |          3 |

---

## Tier 1 — Critical correctness & stability

These are bugs in the existing code paths or holes that make the robot
unrecoverable from normal failures. Do these before adding any new
features.

### T1.1 — Fix `dt_s` upper clamp

**Source:** [`controller-gaps.md`](controller-gaps.md) §1.1.
**Effort:** XS.
**Why:** The control task fires every 100 ms but clamps `dt_s` at
50 ms. Every motion integration is silently running at half the
intended speed.

**Do.** In [`main.c:2375-2376`](Core/Src/main.c:2375), raise the
clamp upper bound from `0.05f` to `0.15f` (i.e. allow up to 1.5×
the expected period). Remove the unused `CTRL_LOOP_DT_S` macro from
[`controller_config.h:10`](Core/Inc/controller_config.h:10).

**Verify.**
- *Static:* add a one-shot log line for the first ten ticks
  printing `dt_s`; should see ~0.10 not 0.05.
- *Sim:* run a STANDUP cycle, compare time-to-standing before/after
  — should drop by ~half.
- *Walk:* command 50 mm/s for 5 s on a tape-measured line; physical
  distance should be ~250 mm not ~125 mm.

### T1.2 — Servo fault back-channel + `CTRL_FAULT` state

**Source:** [`controller-gaps.md`](controller-gaps.md) §1.2 + §2.1;
[`controller-servo-interaction-review.md`](controller-servo-interaction-review.md)
§4.3 + P1.
**Effort:** M.
**Why:** Today any servo-side fault (joint limit, bus failure)
ends in `Error_Handler()` — a busy loop. The controller keeps
publishing targets forever; the LED stays whatever color the last
state set; recovery requires a full reset.

**Do.**
1. Add `servo_fault_t fault_code; uint8_t fault_leg, fault_joint;`
   to `servo_shared_state_t` in [`Core/Inc/servos.h`](Core/Inc/servos.h).
2. Replace `Error_Handler()` in
   [`main.c:2616-2624`](Core/Src/main.c:2616) with a state that
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

### T1.3 — Clamp and slew-limit command inputs

**Source:** [`controller-gaps.md`](controller-gaps.md) §3.3;
[`walking-model-review.md`](walking-model-review.md) M6 / D6.
**Effort:** S.
**Why:** `updated_velocity` / `updated_heading` / `updated_height`
flow from SPI into the controller with no bounds check. A noisy or
malformed frame can produce e.g. `velocity = 1e6 mm/s` and the
controller will honor it.

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

**Source:** [`controller-gaps.md`](controller-gaps.md) §1.3.
**Effort:** S.
**Why:** In STANDING, the controller mutates
`body.translation[2]` to match `cmd.height` but never runs IK —
legs don't move. On the next state transition the body model and
the actual legs are out of sync.

**Do.** Pick one of:
- (preferred) Run a STANDUP-style motion step in STANDING whenever
  `|cmd.height - body.translation[2]| > ε`. Reuse
  `calculate_motion_step`; target is the current `tip_home` with
  the new body Z.
- (alternative) Refuse height changes in STANDING; only allow them
  in STANDUP or WALKING. Update the height-mirror condition in
  [`controller.c:175`](Core/Src/controller.c:175) to exclude
  STANDING.

**Verify.**
- *Sim:* command a height change while STANDING; confirm legs
  re-pose smoothly (preferred fix) or the change is rejected and
  applied at the next motion state (alternative fix).
- *Walk:* on the bench, command +20 mm height while STANDING;
  confirm body lifts (or the height takes effect on the next walk
  command without snapping).

### T1.5 — Liveness sequence counters

**Source:** [`controller-servo-interaction-review.md`](controller-servo-interaction-review.md) §4.5, P4.
**Effort:** S.
**Why:** Neither task can detect that the other is hung. With
T1.2 in place, the servo task can fault — but a *silent* hang
(deadlock, RTOS scheduler stall) still goes unnoticed.

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

---

## Tier 2 — Quick-win hygiene

Small, low-risk fixes. Each is 1–3 lines or a single function-local
change. Do them in any order, in any free 30 minutes.

### T2.1 — `longest_path = longest_stance` explicit

**Source:** [`walking-model-review.md`](walking-model-review.md) D1.
**Effort:** XS.
**Why:** The current `fminf(stance, swing)` works in steady state
only because of a contingent geometry fact. Making the intent
explicit removes edge-case overshoot.

**Do.** In [`controller.c:447`](Core/Src/controller.c:447) and
[`controller.c:619`](Core/Src/controller.c:619), replace the
`fminf` line with `longest_path = longest_stance;` (and rename
the accumulator to match).

**Verify.**
- *Static:* read the code; the change should be obviously
  equivalent in steady state.
- *Sim:* compare commanded-vs-actual body velocity over a
  multi-cycle WALKING run before/after. Should be identical in
  steady state; edge-case overshoot disappears at transients.

### T2.2 — `rot_disp` moment arm from `tip_home`

**Source:** [`walking-model-review.md`](walking-model-review.md) D5.
**Effort:** XS.
**Why:** Using `p_current` as the moment arm makes the rotation
target depend on accumulated foot drift, producing a slow yaw
bias over many cycles.

**Do.** In [`controller.c:385-400`](Core/Src/controller.c:385)
and the parallel block in ROTATING
([`controller.c:556-571`](Core/Src/controller.c:556)), build
`r_perp` from `leg->tip_home`, not `p_current`. Use `p_current`
only for the path start.

**Verify.**
- *Sim:* command a slow constant yaw rate for 30 s; track
  heading drift before/after. Should be flatter.
- *Walk:* on a marked floor, rotate in place 360° forward then
  360° back. Should return closer to start.

### T2.3 — `wrap_to_pi` boundary

**Source:** [`controller-gaps.md`](controller-gaps.md) §3.2.
**Effort:** XS.
**Why:** Returns `-π` for input `+π`, so a `heading - yaw = π`
input oscillates sign every tick.

**Do.** In [`controller_math.h:16`](Core/Inc/controller_math.h:16),
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

### T2.4 — Reset `last_velocity` / `last_ticks` on RUNNING entry

**Source:** [`controller-gaps.md`](controller-gaps.md) §4.2 + §4.3.
**Effort:** XS.
**Why:** `last_velocity[6][3]` retains values across IDLE→RUNNING.
`last_ticks = 0` makes the first tick after boot use a huge `dt`
(saved only by the clamp).

**Do.** Zero `last_velocity` and set `last_ticks =
osKernelGetTickCount()` immediately before entering `SERVO_RUNNING`
(both at first entry and after each POWER_UP → SYNC → RUNNING
transition). Once T4.1 lands this is moot — `last_velocity` is
deleted entirely.

**Verify.**
- *Bench:* power-cycle the robot through POWERDOWN/POWERUP twice;
  observe smooth motion resumption (no first-tick jolt).

### T2.5 — Rename `FEMUR_OFFSET_ANGLE_RAD`

**Source:** [`controller-gaps.md`](controller-gaps.md) §4.5.
**Effort:** XS.
**Why:** In [`servos.c:15-45`](Core/Src/servos.c:15) the constant
is named `FEMUR_OFFSET` but applied to index 2 (tibia). Misleading.

**Do.** Rename to `TIBIA_OFFSET_ANGLE_RAD`. Add a one-line comment
explaining the convention (`compensate.femur = -src.femur`,
`compensate.tibia = src.tibia + TIBIA_OFFSET`).

**Verify.**
- *Static:* rebuild; pure rename should not change behavior.

### T2.6 — Remove dead fields and constants

**Source:** [`controller-gaps.md`](controller-gaps.md) §4.1.
**Effort:** XS.
**Why:** Dead code lies about what the system does.

**Do.** Delete `servo_shared_state.initialized`,
`controller_ctx.delta_yaw`, `controller_ctx.pending_yaw`,
`CTRL_LOOP_DT_S`, `CTRL_SERVO_Z_OFFSET`. Drop the unused
`attitude` parameter from `controller_update` (or wire it up — see
T5.2).

**Verify.**
- *Static:* clean build with no warnings; grep for the removed
  names returns nothing.

### T2.7 — Re-anchor `next_joint_angles` on motion-state entry

**Source:** [`controller-gaps.md`](controller-gaps.md) §4.6.
**Effort:** XS.
**Why:** Currently `next_joint_angles` is only reset on entry to
STANDUP. On any other transition (e.g. WALKING → STANDING →
WALKING), accumulated drift between commanded and actual is
locked in.

**Do.** Extend the transition handler in
[`controller.c:243-250`](Core/Src/controller.c:243) to also reset
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

**Source:** [`walking-model-review.md`](walking-model-review.md) D7.
**Effort:** S.
**Why:** Summing the magnitudes of translation and rotation
overestimates effective velocity for legs whose translation and
rotation components point opposite ways, making them lag.

**Do.** Compute `effective_velocity` per leg as `|arc_disp|` of
that leg, not as a global sum. Substeps then become a per-leg
quantity; `step_length = path_length / per_leg_substeps`.

**Verify.**
- *Sim:* command translation + yaw simultaneously; trace the
  per-leg trajectory and confirm landing-together property
  improves.

### T2.9 — `calculate_motion_step` pace by 3D distance

**Source:** [`controller-gaps.md`](controller-gaps.md) §3.5.
**Effort:** XS.
**Why:** STANDUP pacing uses 2D distance, so a leg far below
ground but close in XY crawls vertically.

**Do.** In [`calculator.c:25-26`](Core/Src/calculator.c:25),
replace `remaining_distance_2d` with `fmaxf(2d_dist, fabsf(target.z -
current.z))` for the duration computation.

**Verify.**
- *Sim:* trigger STANDUP from a pose with one leg far below
  ground; confirm vertical motion is paced by `lift_velocity`,
  not by trivial XY distance.

### T2.10 — Detect foot outside reach disk

**Source:** [`walking-model-review.md`](walking-model-review.md) D4.
**Effort:** S.
**Why:** If a leg drifts outside its 40 mm disk (post-STANDUP,
post-disturbance), nothing forces it back. The disk constraint is
enforced on targets only.

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

---

## Tier 3 — Walking-model robustness

Medium-sized features that close the gaps between the WALKING design
intent and the implementation. None are urgent in isolation, but
together they deliver C4 ("robust to mid-cycle command changes") in
practice rather than just on paper.

### T3.1 — Cycle-time bounds

**Source:** [`walking-model-review.md`](walking-model-review.md) M5 / D9.
**Effort:** M.
**Why:** With constant 40 mm stride, `T_cycle = S/v`. There's no
upper bound on commanded `v` (servos can't keep up) and no lower
bound (legs hang aloft for minutes). Need both.

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

**Source:** [`walking-model-review.md`](walking-model-review.md) M6 / D6.
**Effort:** S (already part of T1.3 if combined).
**Why:** Step changes in command translate to step changes in
per-foot path length. Constant stride amplifies this — a
direction reversal makes a swing target jump by up to 80 mm in one
tick.

**Do.** Already covered by T1.3 (which also adds the velocity
slew). Add a matching heading slew if needed — currently the
yaw-rate clamp inside the controller already serves this purpose
implicitly.

**Verify.** Same as T1.3.

### T3.3 — Swing-leg commit point

**Source:** [`walking-model-review.md`](walking-model-review.md) M2 / D2.
**Effort:** M.
**Why:** Without a commit point, a swing leg whose target is
re-projected every tick from a noisy command will never land. This
is the single biggest hole in C4 robustness.

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

**Source:** [`walking-model-review.md`](walking-model-review.md) M3 / D3.
**Effort:** S.
**Why:** Today's swap fires only on swing-leg arrival. During
transients (direction reversal, large yaw change), stance legs
can run out of disk before swing legs are close. Need a fallback.

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

### T3.5 — Servo-bandwidth model in controller

**Source:** [`controller-servo-interaction-review.md`](controller-servo-interaction-review.md) P2.
**Effort:** S.
**Why:** Controller can ask for joint deltas the servo can't
deliver in one control tick. The bandwidth model lets it refuse
in advance — and it pins down `CTRL_T_CYCLE_MIN` from T3.1.

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

---

## Tier 4 — Servo control upgrades

Move motion-profile generation from software (in the servo task) to
the XL430 hardware. Substantial smoothness improvement; full plan in
[`servo-velocity-control-phase2.md`](servo-velocity-control-phase2.md).

### T4.1 — Phase 1: XL430 profile, software interp removed

**Source:** [`servo-velocity-control-phase2.md`](servo-velocity-control-phase2.md) §1.
**Effort:** M.
**Why:** Today the servo task does software trapezoidal
interpolation with deadband, vel-clamp, accel-clamp, min-step.
The XL430 can do this in hardware via `PROFILE_VELOCITY` /
`PROFILE_ACCELERATION`. Result: smoother motion, simpler servo
task, deadband-quantization issues (§4.4 interaction review) go
away.

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

### T4.2 — Phase 2: Per-joint synchronized profile

**Source:** [`servo-velocity-control-phase2.md`](servo-velocity-control-phase2.md) §1–§10.
**Effort:** M (after T4.1).
**Why:** Phase 1 leaves joints free to finish at different times,
so the foot's Cartesian path between control ticks is whatever
joint-space interpolation chooses. Phase 2 scales
`PROFILE_VELOCITY` per joint per tick so all joints in a leg
finish together, preserving the planned trajectory.

**Do.** Follow the implementation note step-by-step. Adds
`target_joint_velocities[6][3]` to shared state; controller
publishes them; servo task bulk-writes `PROFILE_VELOCITY` before
`GOAL_POSITION` each tick.

**Verify.** Eight-step plan in the implementation note §8.
Highlights:
- *Bench:* single leg with large unequal joint deltas; confirm
  joints arrive simultaneously.
- *Walk:* high-speed walking; foot path should track the
  planned lift trapezoid closely (visible on a marked floor
  or via simulator overlay).

### T4.3 — Refactor `SERVO_ERROR` after T1.2

**Source:** Implicit; cleanup after T1.2 lands.
**Effort:** S.
**Why:** Once `CTRL_FAULT` exists (T1.2), the servo task's old
`SERVO_ERROR → Error_Handler` path can be cleanly replaced with
"set fault_code, transition to SERVO_FAULT_HOLD, never busy-loop".
Worth a small cleanup pass once T1.2 is in.

**Do.** Remove `SERVO_ERROR` and `Error_Handler()` calls from
[`main.c:2616-2624`](Core/Src/main.c:2616). Replace with the
`SERVO_FAULT_HOLD` state from T1.2.

**Verify.** Same as T1.2.

---

## Tier 5 — Structural improvements

Multi-day changes that improve the underlying model rather than
patching gaps. Tackle once Tiers 1–3 are solid.

### T5.1 — Body-Z-aware reach disk

**Source:** [`walking-model-review.md`](walking-model-review.md) M1 / D8.
**Effort:** M.
**Why:** The 40 mm disk is geometrically wrong at extreme body
heights. At low Z, leaving stride length on the table; at high Z,
exceeding IK envelope.

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

### T5.2 — IMU integration (horizon hold)

**Source:** [`controller-gaps.md`](controller-gaps.md) §3.4.
**Effort:** L.
**Why:** Today `controller_update` casts `attitude` to `(void)`.
The EKF is running but its output is wasted. Result: feet are
commanded to `z = -body_z` in body frame regardless of body
tilt; on any incline, load goes unequal.

**Do.** Pre-multiply the body-to-world transform by the EKF's
estimated roll/pitch before computing leg targets. The simplest
first cut: rotate the foot target so `z = -body_z` *in world
frame*, not body frame.

**Verify.**
- *Sim:* (if sim supports inclines) tilt the ground 15° and walk
  across; legs should remain at constant load.
- *Walk:* on a ramp or wedge, walk up/across; body should stay
  level relative to gravity, not relative to the ramp.

### T5.3 — Leg odometry / close the body-pose loop

**Source:** [`controller-servo-interaction-review.md`](controller-servo-interaction-review.md) §4.1 + P3.
**Effort:** XL.
**Why:** This is the structural fix for the deepest interaction
issue: `hexapod.translation` is integrated open-loop at the
commanded velocity, so any servo lag silently makes grounded legs
slip in world frame. Three or more stance legs uniquely
determine body world pose; using that closes the loop.

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

### T5.4 — Tripod selection from actual foot heights

**Source:** [`controller-gaps.md`](controller-gaps.md) §2.4.
**Effort:** XS.
**Why:** `tripod_init_gait` hard-codes legs 1, 3, 5 as the first
swing tripod. If the robot booted with those legs already on the
ground and 0, 2, 4 lifted, the swap fights reality.

**Do.** In `tripod_init_gait`, pick the three legs with the
highest current z (per `forward_kinematics(actual)` →
`Thexapod_body` transform) as the next swing tripod, instead of
hard-coding the indices.

**Verify.**
- *Bench:* manually pose three random legs above ground; start
  WALKING; confirm those legs swing first, not the hard-coded
  set.

### T5.5 — Powerdown pose verification

**Source:** [`controller-gaps.md`](controller-gaps.md) §2.3.
**Effort:** S.
**Why:** After POWERDOWN, legs flop. On the next user command,
STANDUP drives them to `tip_home` — potentially through unsafe
poses (e.g. a leg hanging below the body trying to fold under).

**Do.** Add a pre-STANDUP step that lifts each leg individually
(or in pairs) before global STANDUP. Or: refuse to power back on
if any joint angle is outside `r.leg[i].limits[j]` by more than
a margin.

**Verify.**
- *Bench:* manually move legs into "bad" poses while powered
  down; power back up; confirm safe motion sequence.

### T5.6 — Emergency stop input

**Source:** [`controller-gaps.md`](controller-gaps.md) §2.5.
**Effort:** S (plumbing) or M (with a real GPIO + debounce).
**Why:** No external panic button. `servo_set_request_powerdown`
exists but only fires via the 120 s inactivity timer.

**Do.** Wire an external GPIO (or a dedicated SPI command) to
directly drive `CTRL_FAULT` (or a new `CTRL_ESTOP`). On trigger:
disable torque immediately via `servo_set_request_powerdown`.

**Verify.**
- *Bench:* press the e-stop while walking; confirm immediate
  torque drop; controller goes to fault state.
- *Walk:* same test while in motion; robot collapses
  controllably (legs go limp, not crash).

---

## Tier 6 — New features

Beyond robustness. Only after the rest is solid.

### T6.1 — Strafing mode (body-frame velocity)

**Source:** [`controller-gaps.md`](controller-gaps.md) §5;
[`control-model.md`](control-model.md);
[`walking-model-review.md`](walking-model-review.md) §5.
**Effort:** L.
**Why:** The control-model design calls for two modes; only
arc-based is implemented. The dead world→body→world conversion in
WALKING (gaps §1.4) is a relic of the missing strafing path.

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

**Source:** [`walking-model-review.md`](walking-model-review.md) M4.
**Effort:** L.
**Why:** Tripod gait is stable only if body COM projects inside
the stance triangle. During large yaw transients with translation,
this can fail. Adding a check refuses unsafe swaps.

**Do.** On every potential swap, compute the projected body-COM
position in body frame (approximately `(0, 0)`) and check that
the would-be stance triangle (legs 0,2,4 or 1,3,5) contains it.
If not, defer the swap by one tick or refuse the commanded yaw
rate.

**Verify.**
- *Sim:* command aggressive yaw + translation; confirm the
  controller hesitates rather than tipping.

### T6.3 — Degraded gait if a leg faults

**Source:** [`walking-model-review.md`](walking-model-review.md) M7.
**Effort:** XL.
**Why:** Today, a single fault kills the whole robot. A more
robust controller would drop to a 5-leg wave gait or fold the
faulted leg up out of the way.

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

| Time available           | Pick from                                                 |
|--------------------------|-----------------------------------------------------------|
| 30 min on the couch      | Tier 2 (any item)                                         |
| One evening              | A single Tier 1 item, or 2-3 Tier 2 items                 |
| A weekend                | One Tier 3 item end-to-end                                |
| A focused week           | T4.1 + T4.2 together, or T5.3 alone                       |
| A long break             | T5.2 + T5.3 together (the full odometry-EKF stack)        |

Recommend doing **all of Tier 1 before any Tier 3+**. The reason: the
Tier 3 features (commit point, stance-edge swap, cycle-time bounds)
amplify command activity, which exposes any fault-handling and
input-validation weaknesses. Tier 1 builds the safety net that Tier 3
depends on.

The single highest-leverage hour-long fix is **T1.1** (the `dt_s`
clamp) — it makes every motion calculation honest and is one line of
code. After that, **T1.2** + **T1.3** turn the robot from "fragile
single-fault" into "fault-tolerant within a session". From there the
ordering is flexible.
