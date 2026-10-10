# Controller Audit

Findings from a critical read of the controller (`controller.c`), the servo
task (`main.c`), and the boundary between them. This is the *evidence*
document: what is wrong or missing, and why. The actionable work items, with
fixes and verification, live in [`improvement-backlog.md`](improvement-backlog.md);
every finding below carries the backlog ID that tracks it.

Audit baseline: May 2026 (line numbers cited here are from that snapshot and
have drifted). Current implementation status is tracked in the backlog, not here.

Related documents:
- [`control-model.md`](control-model.md) — design intent the findings are measured against.
- [`controller-overview.md`](controller-overview.md) — as-built description of the code.

Source under inspection:
[`controller.c`](../Core/Src/controller.c),
[`main.c`](../Core/Src/main.c) (control + servo tasks),
[`calculator.c`](../Core/Src/calculator.c),
[`servos.c`](../Core/Src/servos.c),
[`controller_config.h`](../Core/Inc/controller_config.h).

Severity: 🔴 wrong in normal use, 🟠 wrong under specific conditions,
🟡 hygiene.

---

## Contents

1. [Bugs and correctness](#1-bugs-and-correctness)
2. [Safety and lifecycle](#2-safety-and-lifecycle)
3. [Walking model gaps](#3-walking-model-gaps)
4. [Controller ↔ servo interaction](#4-controller--servo-interaction)
5. [State and data hygiene](#5-state-and-data-hygiene)
6. [What is already fine](#6-what-is-already-fine)
7. [Finding → backlog map](#7-finding--backlog-map)

---

## 1. Bugs and correctness

### 1.1 🔴 `dt_s` upper clamp halves the control-loop step → T1.1

The control task fires at 10 Hz (`dt_s ≈ 0.1`) but the clamp upper bound was
0.05 s (`main.c:2375`):

```c
dt_s = clampf(dt_s, 0.0005f, 0.05f);
```

Every integration (`STANDUP` `velocity · dt_s`, `WALKING` `movement_vector`,
`rotation_step = ω · dt_s`) therefore ran at half the commanded speed.
`CTRL_LOOP_DT_S = 0.2f` in `controller_config.h` is a third, unused,
disagreeing "intended dt".

### 1.2 🔴 STANDING accepts `height` changes but never moves the legs → T1.4

`controller.c:175-179` mirrors `cmd.height` into `body.translation[2]` in both
`WALKING` and `STANDING`. In `STANDING` no IK runs and `next_joint_angles` is
never updated, so the model says "body is at height H" while the legs hold the
old pose. The command silently does nothing, and the next `STANDING → WALKING`
transition runs `tripod_init_gait` against a body Z the legs never reached, so
the first swing starts from a wrong world anchor.

### 1.3 🟠 Dead world→body→world conversion in WALKING → T6.1

`controller.c:332-348` rotates `(v·cosψ, v·sinψ)` by `-ψ`, which is an identity:
`v_body` is always `(velocity, 0)`. It costs two cos/sin pairs per tick and,
more importantly, *disguises the missing strafing feature* — it reads like
preparation for a body-frame velocity input that does not exist. Resolution is
part of adding strafing (T6.1) or, if arc-only is final, deleting the block.

### 1.4 🟠 `calculate_motion_step` paces by 2D distance only → T2.9

`calculator.c:25`: a foot far below ground but close in XY has
`remaining_distance_2d ≈ 0`, so `remaining_duration_s ≈ 0` and the lift-height
auto-shrink drives the lift to ~0. The leg's vertical motion is paced only by
`lift_velocity · dt_s`. Should pace by 3D distance or `max(2D, |Δz|)`.

### 1.5 🟠 `wrap_to_pi` returns `-π` for `+π` → T2.3

`controller_math.h:16` yields `[-π, +π)` instead of `(-π, +π]`. A
`heading - yaw = π` input flips sign every tick, nudging yaw direction
unpredictably.

### 1.6 🟠 Servo min-step dithers against the deadband → T4.1

`main.c:2552-2559`: with `error ∈ [0.002, 0.003]` rad the step is rounded *up*
to ±0.003 (overshoot); noisy reads can rebound across the deadband and the joint
twitches forever (audible as buzzing). The software deadband/min-step logic is
removed entirely by moving profile generation onto the XL430 (T4.1), so this
needs no standalone fix. Related: §4.4 (low-velocity deadband stall).

### 1.7 🟡 `leg_generate_path` always steps to the reach-circle boundary — by design

`project_point_on_circle(CTRL_LEG_RADIUS_MM, …)` places every target on the
40 mm circle regardless of `arc_disp` magnitude, so foot placement is not
proportional to velocity. An earlier pass flagged this as a start/stop jerk
bug and proposed scaling the step. That conflicts with the deliberate
constant-stride design (§3.1), under which small velocity should still give a
full stride traversed slowly. **Decision: keep constant stride.** The
legitimate residue — the stride must actually be reachable at the current body
Z, and very low velocity must not make legs hang aloft — is covered by
§3.2 (M1) and §3.5 (M5).

---

## 2. Safety and lifecycle

### 2.1 🔴 `SERVO_ERROR` is a one-way door; no fault state in the controller → T1.2, T4.3

`main.c:2616-2624`: any servo-side fault (a single joint-limit alert, true or
noise) disables torque on all 18 servos and enters `Error_Handler()`, a busy
loop. The controller is never told: it keeps publishing targets every 100 ms
into a task that will never read them. The state-change callback only reports
transitions, and the fault path bypasses it. The FSM has no `CTRL_FAULT` /
`CTRL_ESTOP`, so there is also no way to tell the controller the bus is dead or
to request a clean external shutdown (only the 120 s timeout to `POWERDOWN`).

Needed: a fault back-channel in `servo_shared_state` (code, leg, joint), a
`SERVO_FAULT_HOLD` state instead of `Error_Handler`, and a `CTRL_FAULT` state
that holds a safe pose and exits only on explicit external clear. Protocol shape
(P1):

```c
typedef enum { SERVO_FAULT_NONE, SERVO_FAULT_BUS,
               SERVO_FAULT_LIMIT, SERVO_FAULT_BANDWIDTH } servo_fault_t;
servo_fault_t fault_code;  uint8_t fault_leg, fault_joint;
```

### 2.2 🟠 No liveness check between the tasks → T1.5

`servo_set_target_joint_angles` writes regardless of whether the servo task is
keeping up, and `servo_copy_actual_joint_angles` returns whatever is in the
buffer however stale. If either task hangs the other keeps stepping on
fossilized data — combined with §2.1, "robot looks alive at the SPI level but
the legs do nothing". Needed: monotonic `target_seq` / `actual_seq` counters
(P4).

### 2.3 🟠 No input validation in the controller; none on link loss → T1.3

The SPI handler (`StartSpiSlaveTask`, `main.c`) range-checks each parameter
and *rejects* out-of-range values (velocity 0–100 mm/s, heading 0–2π rad,
height 50–170 mm), so a corrupted payload is mostly caught at the edge. Gaps
that remain:

- `controller_update` itself trusts its inputs; any other caller (the
  simulator, a future command source) bypasses the SPI checks.
- Frames carry no checksum or sequence number, so a corrupted frame whose
  header and range still look valid is accepted.
- Nothing supervises the link: if the Raspberry Pi stops sending, the last
  velocity stays in force and the robot keeps walking.
- No filtering or slew (§3.4 D6).

### 2.4 🟠 No pose verification after `POWERDOWN` → T5.5

`POWERDOWN` disables torque and legs flop. The next `STANDUP` seeds
`next_joint_angles = actual_joint_angles` and drives to `tip_home`. If a leg has
collapsed under the body, the XY-directed lift may not clear the body and the
motion scrapes. Needed: a safe-pose check (joint outside limits by a margin →
refuse power-up) or per-leg sequenced extraction before global `STANDUP`.

### 2.5 🟠 Tripod choice is hard-coded → T5.4

`controller_init` marks all legs `grounded`; `tripod_init_gait` then hard-codes
legs 1, 3, 5 as the first swing tripod. If the robot booted with 0, 2, 4 already
lifted, the first swap fights reality. Pick the three highest feet from
`actual_joint_angles → FK` instead.

### 2.6 🟠 No emergency stop input → T5.6

`servo_set_request_powerdown(true)` fires only from the controller's transition
to `POWERDOWN`. The servo task can drop torque instantly but no button, SPI
command or GPIO is wired to it.

---

## 3. Walking model gaps

### 3.1 The design under audit

> The WALKING state moves each leg with a **fixed 40 mm stride** in the X/Y
> plane around its neutral position (`tip_home`), regardless of body height. The
> robot moves at the commanded speed by running a tripod-gait lift/stand cycle;
> lower commanded speed simply means a longer cycle time. New movement
> instructions are incorporated immediately, taking current leg positions into
> account.

The contracts C1–C4 this decomposes into are defined in
[`control-model.md`](control-model.md#walking-gait-contracts). The constant
stride is deliberate: it keeps gait geometry invariant across speeds, gives a
predictable landing impact, and avoids the "shuffle" failure mode of
proportional-stride walkers.

**Strengths:** reach disk as the primary constraint (decouples *where the foot
can go* from *how fast we move*); constant stride (stance and swing finish
together in steady state); tripod gait (statically stable, fast); reactive
re-planning every tick from the current foot position.

### 3.2 Gaps in the model itself

| ID | Gap | Backlog |
|----|-----|---------|
| M1 | **Reach disk ignores body height.** 40 mm "regardless of height" is geometrically untrue: at low Z it leaves stride on the table, at high Z it may exceed the IK-feasible annulus. `CTRL_LEG_RADIUS_MM` has no derivation from femur/tibia lengths or home-pose IK margin and is never asserted feasible (D8). | T5.1 |
| M2 | **No commit point for swing legs.** A controller that re-targets the swing foot from the latest command every tick never lets a swing leg land under noisy input. Sharper under constant stride: at low velocity the swing is long in time, giving noise many ticks to keep nudging the target. C4 fights C1. | T3.3 |
| M3 | **Stance-edge swap trigger missing.** The swap fires only on swing arrival. Stance and swing finish together in steady state, so it works — but it diverges during reversals and yaw transients, exactly when robustness matters. Should fire on swing arrival *or* any stance leg past the trailing edge of its disk. | T3.4 |
| M4 | **No static-stability check.** Tripod is stable only if the COM projects inside the stance triangle; a moving COM with a fixed swap schedule can violate that. | T6.2 |
| M5 | **No cycle-time bounds.** `T_cycle ≈ S/v`. Upper bound on `v`: swing legs cannot complete 40 mm inside servo bandwidth (`v_max = S/T_min`). Lower bound: 1 mm/s means a 40 s cycle with legs aloft ~20 s. The model is silent on both. | T3.1 |
| M6 | **No slew on the command.** A step change in commanded velocity is a step change in per-foot path length inside one tick; a reversal moves a swing target by up to 80 mm. Needs a controller-level rate limit separate from the servo accel cap. | T1.3 (T3.2) |
| M7 | **No degraded gait.** The model assumes the tripod always works; one faulted leg kills the robot. | T6.3 |

M2, M3, M5 and M6 are the high-impact ones for the robustness claim of C4.

### 3.3 Code vs. contracts

**C1 (constant 40 mm stride) — implemented.** The target is forced onto the disk
boundary by `project_point_on_circle`. Residue: the disk is enforced on the
*target*, never on the *state* (§3.4 D4); and the target `z = -body_z` is
body-frame, so a tilted body (IMU ignored, §3.4 D10) puts the disk plane in the
wrong place.

**C2 (commanded speed, cycle-averaged) — implemented, fragile.** Per-tick body
displacement is `velocity · dt_s` regardless of pacing; pacing only sets how many
ticks the cycle takes (`substeps = longest_path / effective_velocity`). The
intent is "pace by stance" because stance legs, anchored in world, set body
velocity; swing legs move faster in 3D to cover the lift trapezoid. The encoding
is `fminf(stance, swing)`, which resolves to stance only because swing 3D paths
happen to be longer in steady state (see D1).

**C3 (tripod gait) — mostly.** `swap_legs` toggles all six on swap; the trigger
is `remaining_path_length < CTRL_CLOSE_THRESH`, computed over swing legs only.
There is no enforcement that all three swing legs land together; early finishers
idle until the worst one arrives (small effect under constant stride).

**C4 (mid-cycle re-incorporation) — geometrically yes, dynamically no.** Works:
paths regenerate every tick from `actual_joint_angles`; grounded legs keep their
world anchor, so a direction change doesn't slip stance feet. Doesn't: no commit
point (M2), no cycle-time bounds (M5), no slew (M6).

### 3.4 Concrete discrepancies

| ID | Discrepancy | Backlog |
|----|-------------|---------|
| D1 | **`longest_path = fminf(stance, swing)` encodes "pace by stance" implicitly** (`controller.c:447, 619`). If a swing path ends up shorter than stance (leg started near target, mid-cycle retarget toward current position), `fminf` picks swing → `substeps` shrinks → stance over-steps → body moves *faster* than commanded for that cycle. Replace with `longest_path = longest_stance`. Note that `fminf` does not and should not fix servo-infeasible velocities — that is M5. | T2.1 |
| D2 | **Swing leg has no commit point** (= M2). Workable rule: freeze `committed_target` once `swing_phase` passes a fraction of the arc. | T3.3 |
| D3 | **Swap gated on swing legs only** (= M3). Add per-stance `disk_margin = R − dist(p_current.xy, tip_home.xy)` and swap when any margin < `SWAP_MARGIN`. A stance leg that lags its commanded position also never gates the swap, so new stance legs can inherit positions that no longer match reality. | T3.4 |
| D4 | **No detection of a foot outside its disk.** After STANDUP or a disturbance nothing forces it back. | T2.10 |
| D5 | **`rot_disp` uses `p_current` as the moment arm** (`controller.c:385-400`). The target direction should use `tip_home`; using `p_current` couples the target to accumulated foot drift (slow yaw bias, and the projection can fold back across `tip_home` for a leg outside its disk). | T2.2 |
| D6 | **`velocity` and `heading` unfiltered** (`controller.c:171-173`): no deadband beyond `CTRL_EPS_VEL`, no slew, no limit (= M6, §2.3). | T1.3 |
| D7 | **`effective_velocity` sums magnitudes with no direction** (`controller.c:469-473`). For legs whose translation and rotation oppose, the sum overestimates and under-budgets time, so they lag. Use per-leg `|arc_disp|`. | T2.8 |
| D8 | **40 mm is not derived from leg mechanics** (= M1). | T5.1 |
| D9 | **No cycle-time bounds** (= M5). | T3.1 |
| D10 | **IMU `attitude` is plumbed but discarded.** `controller_update` casts it to `(void)`; body Z is `-body_z` in the body frame, so on any incline the legs no longer share a horizontal plane and load goes unequal. The EKF runs but its output never reaches the controller. | T5.2 |

---

## 4. Controller ↔ servo interaction

### 4.1 The protocol as built

A flat shared-memory pipeline: one struct, `servo_shared_state`, guarded by
`taskENTER_CRITICAL` helpers in `servos.c`.

```
  Control task (10 Hz)                     Servo task (50 Hz)
  1. copy actual joint angles   <-------   5. publish actual joint angles
  2. controller_update                     1. read PRESENT_POSITION
  3. publish target joint angles ------>   2. read target joint angles
                                           3. per-joint deadband / vel / accel clamp
                                           4. write GOAL_POSITION
```

Sideband: `request_powerdown` and `limit_alert_enabled` (controller → servo
only). There is **no contract on time**: targets carry no deadline, and the servo
task does not report whether the previous target was reached. See
[`controller-overview.md`](controller-overview.md) §6 for the full as-built
description.

### 4.2 What the design implicitly assumes

| ID | Assumption |
|----|------------|
| A1 | The servo task can reach `target_joint_angles` within one control tick (100 ms). |
| A2 | Therefore `actual_joint_angles` ≈ previously commanded target at every read. |
| A3 | Therefore the open-loop body-pose integration (`hexapod.translation += velocity · dt_s`) stays consistent with where the legs physically are. |
| A4 | Servo faults can be reflected back so the controller can re-plan, abort or hold. |

**Where they hold:** A1/A2 at the configured operating point (50 mm/s, 40 mm
stride): per-tick joint deltas are 0.05–0.15 rad against a servo ceiling of
`SERVO_MAX_VELOCITY · 0.02 = 0.16 rad` per servo tick × 5 servo ticks per
control tick = 0.8 rad. The servo task acts as a smooth low-pass filter and the
controller never notices the limits.

### 4.3 🔴 A3 fails — body-pose integration is open-loop → T5.3

`controller.c:362-364` integrates `hexapod.translation` and `rotation[2]` at the
**commanded** velocity, whether or not the legs carried the body that far.
Landing a swing leg stores `tip_world_coordinates` from the *model* body pose.
If the physical body lags the model by Δ mm, the anchor sits Δ mm ahead of the
foot, and every later tick drags the foot forward in body frame to maintain that
fiction: **the grounded leg slips Δ mm in world**.

Worked example: the model integrates 5 mm/tick, the servos deliver 4 mm/tick. At
touchdown the model body is at x = 50, the physical body at x = 40, so the
anchor is 10 mm off, and the next 10 ticks drag the foot another 1 mm each.

This violates C1 (the foot drifts off its disk in world though body-frame
planning keeps it on the disk) and undermines C4 (we *use* the actual leg
position but *anchor* against a fictional body pose). It is the deepest issue in
the audit.

Fix options (P3): a cheap fix that anchors against the physical pose inferred
from `actual_joint_angles + tip_world_coordinates`; or proper leg odometry,
estimating body (x, y, yaw) from the stance legs by least squares, which
composes naturally with the IMU for roll/pitch (full state estimator).

### 4.4 🟠 A1 fails — no servo bandwidth model in the controller → T3.5 (feeds T3.1)

`substeps` is computed purely from kinematic path length and commanded velocity;
the controller knows nothing of `SERVO_MAX_VELOCITY`, `SERVO_MAX_ACCELERATION`,
deadband or min-step. At high velocity it publishes targets the servo task
cannot reach in 100 ms, lag accumulates, A2 fails and §4.3 kicks in. This is the
missing input behind M5/D9: `T_min` must be derived from a bandwidth model
(`max_per_tick_joint_delta = SERVO_MAX_VELOCITY · 0.1 s = 0.8 rad`), not guessed.
Options (P2): static (clamp to a constant per-tick delta) or dynamic (publish
`last_velocity` from the servo task so the controller sees real headroom).

### 4.5 🟠 A4 fails — servo errors do not reach the controller → T1.2

Same defect as §2.1, seen from the interaction side: "robust" in C4 has no story
for "the actuators cannot do what was asked".

### 4.6 🟡 A2 softens at low velocity — deadband stall → T4.1

A 0.0015 rad per-tick delta is inside the 0.002 rad deadband, so the servo task
emits nothing and the next tick's `actual_joint_angles` are unchanged. The
controller's model of the leg freezes against the plan; if it is a grounded leg
the body model still integrates (§4.3), so slip can accumulate purely from
deadband-stalled stance legs. Removed by the XL430 hardware profile (T4.1);
if that slips, P6: raise `CTRL_EPS_VEL` so sub-deadband commands fall back to
STANDING, or accumulate small deltas in the controller until they cross the
deadband.

### 4.7 🟡 Command parameters are not delivered atomically → T2.11

The SPI protocol sends one parameter per 7-byte frame (`0x01` speed, `0x02`
heading, `0x03` height), each written to its own global
(`main.c:299-301`) and copied field by field into `cmd` (`main.c:2379-2381`)
without a critical section. Single-float reads are atomic on Cortex-M4, but
there is no such thing as a coherent command: the controller can see the new
speed with the old heading for a tick or more. Fix (P5): a single command
frame carrying the whole command, written under the same critical-section
pattern as `servo_shared_state`. See [`design.md`](design.md) §3 for the
proposed protocol.

---

## 5. State and data hygiene

| Finding | Detail | Backlog |
|---------|--------|---------|
| Dead fields/constants | `servo_shared_state_t.initialized`, `CTRL_LOOP_DT_S`, `CTRL_SERVO_Z_OFFSET`, `ctx->delta_yaw`, `ctx->pending_yaw` (initialized never read), unused `attitude` parameter. | T2.6 |
| `last_velocity` not reset | `IDLE → RUNNING` resumes mid accel-ramp because the servo task's `last_velocity[][]` survives `POWERDOWN`. | T2.4 (moot after T4.1) |
| `last_ticks = 0` | First `RUNNING` tick computes `dt` against boot time; bounded only by the clamp. | T2.4 |
| Misleading names | `FEMUR_OFFSET_ANGLE_RAD` is applied to index 2 (tibia); the sign flip on `[1]` is undocumented (`servos.c:35-45`). | T2.5 |
| `next_joint_angles` reset only on `STANDUP` entry | After `WALKING → STANDING → ROTATING` the plan starts from the commanded position, not the real one, locking in drift if the servos lagged. | T2.7 |

---

## 6. What is already fine

- The 5:1 servo:control loop rate ratio gives the servo task room to smooth
  per-tick steps.
- Critical-section discipline around `servo_shared_state` is consistent; no torn
  reads of the angle arrays.
- `compensate` / `uncompensate` isolate the controller from hardware geometry.
- Startup ordering (`SYNC_FROM_HW` → `SYNC_TO_HW` → `EVT_SERVO_READY` →
  `controller_init`) guarantees the controller never publishes into an
  unprepared servo task.
- `request_powerdown` is correctly latched and acted on.
- The reach-disk, constant-stride, tripod, replan-every-tick design is sound
  (§3.1); the issues are in what surrounds it.

The interaction shape is right. What it lacks is the bidirectional honesty
needed to keep the WALKING contracts intact when the world doesn't cooperate.

---

## 7. Finding → backlog map

Backlog IDs are stable labels. Some items were replaced by the design (see the
*Superseded items* section of [`improvement-backlog.md`](improvement-backlog.md));
the table below still names the original item, and the backlog says what
replaces it.

| Finding | Backlog | Finding | Backlog |
|---------|---------|---------|---------|
| §1.1 `dt_s` clamp | T1.1 | §3.4 D4 foot outside disk | T2.10 |
| §1.2 STANDING height | T1.4 | §3.4 D5 moment arm | T2.2 |
| §1.3 dead conversion | T6.1 | §3.4 D6 unfiltered cmd | T1.3 |
| §1.4 2D pacing | T2.9 | §3.4 D7 effective velocity | T2.8 |
| §1.5 `wrap_to_pi` | T2.3 | §3.4 D8/D9 | T5.1 / T3.1 |
| §1.6 min-step dither | T4.1 | §3.4 D10 IMU | T5.2 |
| §1.7 boundary step | n/a (by design) | §4.3 open-loop body pose | T5.3 |
| §2.1 / §4.5 fault path | T1.2, T4.3 | §4.4 bandwidth model | T3.5 |
| §2.2 liveness | T1.5 | §4.6 deadband stall | T4.1 |
| §2.3 input validation | T1.3 | §4.7 atomic command frame | T2.11 |
| §2.4 powerdown pose | T5.5 | §5 hygiene | T2.4–T2.7 |
| §2.5 tripod choice | T5.4 | M1 / M2 / M3 | T5.1 / T3.3 / T3.4 |
| §2.6 e-stop | T5.6 | M4 / M5 / M6 / M7 | T6.2 / T3.1 / T1.3 / T6.3 |
| §3.4 D1 `fminf` | T2.1 | | |
