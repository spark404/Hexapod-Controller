> **ARCHIVED (2026-07-20).** Superseded by [controller-redesign-requirements.md](../controller-redesign-requirements.md) (leading plan) and [improvement-backlog.md](../improvement-backlog.md). Unique findings from this document have been folded into those. Code line references reflect the May 2026 state of the sources.

# Controller Robustness — Gaps and Suggested Fixes

A critical re-read of the controller and servo task with a focus on
robustness. Findings are grouped by category and tagged with a severity
hint (🔴 high — likely to cause incorrect motion or hangs in normal use;
🟠 medium — wrong under specific conditions or future scaling; 🟡 low —
correctness/maintainability nits worth fixing eventually).

Source under inspection:
[`Core/Src/controller.c`](Core/Src/controller.c),
[`Core/Src/main.c`](Core/Src/main.c) (control + servo tasks),
[`Core/Src/calculator.c`](Core/Src/calculator.c),
[`Core/Inc/controller_config.h`](Core/Inc/controller_config.h).

---

## 1. Real bugs (🔴 fix before next test run)

### 1.1 `dt_s` upper-clamp halves the control-loop step

[`main.c:2375-2376`](Core/Src/main.c:2375):

```c
float dt_s = (float)dt_ticks * (float)portTICK_PERIOD_MS * 1e-3f;
dt_s = clampf(dt_s, 0.0005f, 0.05f);
```

The control task fires at `CONTROL_LOOP_INTERVAL = HZ_TO_INTERVAL(10) = 100 ms`,
so a healthy tick has `dt_s = 0.1`. The clamp **upper bound is 0.05 s**,
which is half the actual loop period. Every motion integration therefore
runs at half the intended velocity:

- `STANDUP` advances `velocity · dt_s = 50 · 0.05 = 2.5 mm/tick` → 25 mm/s
  (config says `CTRL_DEFAULT_VEL = 50 mm/s`).
- `WALKING` `movement_vector` is half magnitude → commanded `velocity` is
  silently halved.
- `CTRL_MAX_YAW_RATE` enforcement still works (it caps `desired_omega`
  before multiplication), but `rotation_step = ω · dt_s` is halved as well.

Also note `CTRL_LOOP_DT_S = 0.2f` is defined in
[`controller_config.h:10`](Core/Inc/controller_config.h:10) and never used —
two different "intended dt" values disagree with the actual loop period.

**Fix.** Pick one truth. Either raise the clamp to `0.15` (i.e. allow up to
1.5× expected period) and accept measured dt, *or* pass a fixed
`CTRL_LOOP_DT_S` and verify the actual period under load. Remove the unused
macro. Same review for the servo task — its 20 ms loop with `0.05` upper
clamp is fine, just keep them consistent.

### 1.2 `SERVO_ERROR` is a one-way door

[`main.c:2616-2624`](Core/Src/main.c:2616):

```c
case SERVO_ERROR:
    for (int i=0; i<18; i++) {
        dynamixel_set_torque_enable(&dynamixel_servos[i], 0);
        dynamixel_set_led(&dynamixel_servos[i], 0);
    }
    Error_Handler();
    break;
```

A single joint-limit alert (legitimate or false-positive from servo noise
while `limit_alert_enabled = true`) drops torque on every servo and jumps
into `Error_Handler()`, which on STM32 HAL is a busy loop in red. There is
no recovery path and no notification back to the controller, which keeps
publishing targets into a dead task.

The TODO in the source already flags this. Concretely the missing pieces:

1. A back-channel flag (e.g. `servo_shared_state.fault_code`) the controller
   can read, plus a callback or event flag to wake it.
2. A controller-side state — e.g. `CTRL_FAULT` — that puts the robot in a
   safe pose (legs folded down, no torque), holds there, and only allows
   exit via an explicit external clear.
3. A `SERVO_FAULT_HOLD` instead of `Error_Handler` — torque off but loop
   still running so we can re-arm.

### 1.3 `STANDING` accepts `height` changes but never moves the legs

[`controller.c:175-179`](Core/Src/controller.c:175):

```c
if (ctx->state == CTRL_WALKING || ctx->state == CTRL_STANDING) {
    if (height != ctx->robot.body.translation[2]) {
        ctx->robot.body.translation[2] = height;
    }
}
```

In `STANDING` the controller body branch (lines 322 onward) is never entered;
no IK runs, `next_joint_angles` is never updated. Yet `body.translation[2]`
is mutated, so the model says "body is now at height H" while the legs are
still commanded to the previous targets.

Visible consequences:

- The user's height command silently does nothing in `STANDING`.
- The next time `STANDING → WALKING` triggers, `tripod_init_gait` uses the
  new body Z to compute world coordinates from `forward_kinematics(actual)`
  — i.e. it assumes the body just teleported up — and the very first swing
  step starts from a wrong world anchor.

**Fix.** Either:
- (preferred) Drive a STANDUP-style motion step in `STANDING` whenever
  `height` differs from `body.translation[2]`, or
- Reject the height change in `STANDING` and only let the user adjust
  height in `STANDUP`.

### 1.4 World→body→world conversion in `WALKING` is dead math

[`controller.c:332-348`](Core/Src/controller.c:332):

```c
float32_t v_world[2] = { velocity*cos(yaw),  velocity*sin(yaw) };
float32_t v_body[2]  = {
    v_world[0]*cos(-yaw) - v_world[1]*sin(-yaw),
    v_world[0]*sin(-yaw) + v_world[1]*cos(-yaw)
};
```

Substituting: `v_body[0] = velocity*cos²(yaw)+velocity*sin²(yaw) = velocity`,
`v_body[1] = 0`. The roundtrip is an identity — `v_body` is always
`(velocity, 0)`. Combined with the subsequent body-to-world conversion using
`yaw_mid`, the net displacement is `velocity·dt·[cos(yaw_mid), sin(yaw_mid)]`.

This works for arc-mode (`velocity` = forward speed in current heading) but:
- The intent reads like preparation for strafing (per
  [`control-model.md`](control-model.md) Mode 2), but there is no path for
  a true body-frame velocity command — the controller has no way to express
  "step sideways". The dead code disguises the missing feature.
- The cos/sin pair is computed twice per tick for nothing.

**Fix.** Either commit to arc-mode and delete lines 332-348 (replace with a
direct `movement_vector = (velocity·cos(yaw_mid), velocity·sin(yaw_mid))·dt`),
or introduce a real body-frame velocity input
(`controller_command_t.v_body_x/y`) so the roundtrip becomes meaningful.

### 1.5 `leg_generate_path` always steps to the boundary of the reach circle

[`controller.c:140-157`](Core/Src/controller.c:140):

```c
project_point_on_circle(step_size, origin, arc_disp, point);
```

`step_size = CTRL_LEG_RADIUS_MM = 40 mm`. The target is the point on the
circle of radius 40 around `tip_home`, in the direction of `arc_disp`. Its
magnitude is fixed — even a tiny `arc_disp` (low velocity, near-zero yaw
error) puts the target at a full 40 mm step from home.

Two consequences:

- Foot placement is *not* proportional to commanded velocity. Velocity only
  changes how fast we traverse the path, not its length. Small inputs still
  result in maximum-reach steps.
- At the WALKING ↔ STANDING boundary (velocity just above
  `CTRL_EPS_VEL = 1e-3`), feet still place at 40 mm from home, then drift
  back when the controller returns to STANDING. Jerk on every start/stop.

**Fix.** Scale `step_size` by either the magnitude of `arc_disp` (clamped to
`CTRL_LEG_RADIUS_MM`) or by the desired walking cadence:

```c
float32_t step = fminf(arm_vec_magnitude_f32(arc_disp, 2) * STEP_GAIN,
                       CTRL_LEG_RADIUS_MM);
project_point_on_circle(step, origin, arc_disp, point);
```

### 1.6 Tripod swap uses only swing-leg progress

WALKING [`controller.c:500-520`](Core/Src/controller.c:500) and ROTATING
[`controller.c:653-670`](Core/Src/controller.c:653): `remaining_path_length`
is only updated inside the `else` (swing) branch. Stance legs never gate the
swap.

If a stance leg falls behind its commanded position (servo velocity-limited,
or temporarily blocked), the world anchor (`tip_world_coordinates`) is
held constant, so the foot stays "planted" in world but commanded joints
keep marching. When the swap fires (driven only by swing-leg progress), the
new stance legs inherit positions that no longer match reality, and the new
swing legs lift from wherever they happen to be.

**Fix.** When evaluating the swap, also check whether stance legs are within
some tolerance of their predicted body-frame target (or simply require all
six legs to be within `CTRL_CLOSE_THRESH` of their planned step end).

---

## 2. Missing safety / lifecycle (🟠)

### 2.1 No fault state in the controller FSM

The FSM models steady-state motion but has no entry for "something went
wrong". There is no way to:

- Tell the controller the servo bus is dead.
- Tell the controller a joint limit was tripped.
- Request a clean shutdown from outside (only the 120 s timeout to
  `POWERDOWN` exists).

**Suggest** adding `CTRL_FAULT` and `CTRL_ESTOP` states plus
`controller_request_state(ctx, CTRL_POWERDOWN | CTRL_FAULT)`.

### 2.2 No watchdog between control task and servo task

`servo_set_target_joint_angles` writes regardless of whether the servo task
is keeping up; `servo_copy_actual_joint_angles` returns whatever's in the
buffer regardless of how stale it is. If either task hangs, the other will
happily keep stepping using fossilized data.

**Suggest** a monotonic `target_seq` / `actual_seq` counter in
`servo_shared_state` so each side can detect a stuck peer and degrade
safely.

### 2.3 No verification of pose after `POWERDOWN`

`POWERDOWN` disables torque. Legs can flop. On the next user command,
controller enters `STANDUP`, seeds `next_joint_angles = actual_joint_angles`,
and drives the legs to `tip_home` via `calculate_motion_step`. That step
generator does include a lift, but only along the XY direction toward target
— if a leg has collapsed *underneath* the body (z way below ground level),
the planned lift may not clear the body, and the motion will scrape.

**Suggest** an explicit "safe pose" check: if any actual joint angle is
outside `r.leg[i].limits[j]` by more than a margin, refuse to power up; if
any foot is forward-kinematics-implied to be inside the body envelope,
sequence each leg out one at a time before global STANDUP.

### 2.4 `controller_init` declares all legs `grounded`, then `STANDUP` runs without re-checking

[`controller.c:66`](Core/Src/controller.c:66) — `grounded = 1` for all six
on init. `STANDUP` itself doesn't toggle `grounded` (it just drives toward
`tip_home`). The first `WALKING` entry calls `tripod_init_gait`, which sees
all grounded and re-initializes the tripod from `actual_joint_angles`. OK
but fragile — the "tripod" choice is hard-coded to legs 1, 3, 5; if the
robot booted in a state where legs 0, 2, 4 are already lifted, the swap will
fight reality on the first step.

**Suggest** in `tripod_init_gait`, pick whichever set of three has the
*highest* feet (per `actual_joint_angles` → fk → z), not always 1, 3, 5.

### 2.5 No emergency stop input

`servo_set_request_powerdown(true)` is only ever called from
`stm32_state_change_cb` when the controller transitions to `POWERDOWN`. The
servo task has the capability to drop torque instantly, but there is no
user-facing path (button, SPI command, external GPIO) wired to it. An
external e-stop on the chassis would have nothing to talk to.

---

## 3. Numerical / math issues (🟠)

### 3.1 Servo "min-step" can dither against deadband

[`main.c:2552-2559`](Core/Src/main.c:2552):

```c
if (fabsf(error) >= SERVO_DEADBAND_RAD) {     // ≥ 0.002
    step = clampf(error, -max_step, max_step);
    if (fabsf(step) < SERVO_MIN_STEP_RAD) {   // < 0.003
        step = copysignf(SERVO_MIN_STEP_RAD, step);
    }
}
```

When `error ∈ [deadband, min_step]` (i.e. 0.002–0.003 rad), the step is
rounded up to ±0.003 → guaranteed overshoot. The overshoot puts the next
error inside the deadband, so it stays put. But on a noisy servo read, the
error can rebound across the deadband and the joint twitches ±0.003 rad
forever. Visible as a quiet buzzing.

**Fix.** Use hysteresis: a wider "release" deadband than "engage" deadband,
or simply require `|error| ≥ MIN_STEP` before commanding any motion. The
XL430's own pulse quantization is ~0.00153 rad, so a 0.003 rad threshold is
two ticks — fine to demand that as the minimum motion.

### 3.2 `wrap_to_pi` returns `-π` for input `+π`

[`controller_math.h:16`](Core/Inc/controller_math.h:16). The result is in
the half-open range `(-π, +π]` per the comment intent, but actually it's
`[-π, +π)` because `fmodf` produces 0 at `2π` and then 0 - π = -π. Minor —
nothing today depends on the sign at exactly π, but a `heading - yaw = π`
input flips between +π and -π on every dt, producing a yaw_error sign flip
that nudges yaw direction unpredictably.

### 3.3 No clamp on `cmd.velocity` or `cmd.height`

The control task reads global floats (`updated_velocity`, `updated_heading`,
`updated_height` from `main.c:299-301`) populated by the SPI handler. There
is no validation: a stuck/corrupted SPI frame can deliver `velocity = 1e6
mm/s` or `height = -50 mm` and the controller will run them through the
math.

**Fix.** Apply input clamps in `controller_update` *before* using the
command:

```c
velocity = clampf(velocity, -CTRL_MAX_VELOCITY, +CTRL_MAX_VELOCITY);
height   = clampf(height,    CTRL_MIN_HEIGHT,   CTRL_MAX_HEIGHT);
```

### 3.4 IMU `attitude` is plumbed but discarded

`controller_update(ctx, attitude, cmd, dt_s)` casts `attitude` to `(void)`.
The body Z is always commanded as `-body_z` in the body frame, which assumes
the body is perfectly level. On any incline the foot targets are tilted
relative to gravity, the legs no longer share a horizontal plane, and load
goes unequal.

If the goal is a robust controller, the body pose should compensate for
measured roll/pitch (servo-level horizon hold). The EKF
([`Core/Src/attitude_ekf.c`](Core/Src/attitude_ekf.c)) is wired up and
running but its output never reaches the controller.

### 3.5 `calculate_motion_step` paces by 2D distance only

[`calculator.c:25`](Core/Src/calculator.c:25):

```c
const float32_t remaining_distance_2d = arm_euclidean_distance_f32(current, target, 2);
const float32_t remaining_duration_s = remaining_distance_2d / motion->velocity;
```

If a leg starts close in XY but far in Z (e.g. from a powered-down floppy
state), `remaining_distance_2d ≈ 0`, `remaining_duration_s ≈ 0`, and the
lift-height auto-shrink in lines 42-51 then drives the calculated lift to
~0. The leg's vertical motion is paced only by `lift_velocity * dt_s`. In
practice this means a foot that's hanging far below ground level lifts
slowly and never gets a proper "approach to target XY".

**Fix.** Pace by 3D distance or by `max(2D_distance, |z_error|)` so the
slower axis governs.

### 3.6 `longest_path = fminf(longest_path, longest_lifted_path)`

[`controller.c:447, 619`](Core/Src/controller.c:447). Reads "use the
*shorter* of stance and swing path lengths". If a tripod-half has very
little motion to do (just touched down), the other half is paced down to
match. This is probably wrong: pacing should be by the *longer* phase so
the slow half completes; the fast half can wait.

Worth double-checking against the original intent — the comment-light area
of the code makes the design choice ambiguous.

---

## 4. State / data hygiene (🟡)

### 4.1 Dead/unused fields and constants

- `servo_shared_state_t.initialized` ([`servos.h:18`](Core/Inc/servos.h:18))
  is never read or written. Either wire it up (signal first
  `SYNC_TO_HW` complete) or delete it.
- `CTRL_LOOP_DT_S` and `CTRL_SERVO_Z_OFFSET` in `controller_config.h` are
  never used.
- `controller_attitude_t` parameter on `controller_update` is unused; either
  use it (§3.4) or drop it from the signature.
- `ctx->delta_yaw` and `ctx->pending_yaw` in `controller_types.h:54-55` —
  `pending_yaw` is initialized but never read; `delta_yaw` is never touched.

### 4.2 `last_velocity` in servo task is not reset across `IDLE → RUNNING`

`last_velocity[i][j]` is a function-local static-equivalent (declared in the
task body, but the task is an infinite loop). On `POWERDOWN → IDLE →
POWER_UP → RUNNING`, `last_velocity` retains its values from before the
powerdown — the accel ramp resumes mid-ramp. Combined with the fact that on
re-entry, `error` is reset by `SYNC_FROM_HW` (target = actual, so error ≈
0), the first few ticks will be commanding near-zero step but the accel
clamp will allow up to `|dv| ≤ SERVO_MAX_ACCELERATION * dt_s` change per
tick.

**Fix.** Zero `last_velocity` whenever entering `SERVO_RUNNING` from a
non-RUNNING state, or when `request_powerdown` flips.

### 4.3 `last_ticks = 0` on first `RUNNING` entry

[`main.c:2438`](Core/Src/main.c:2438) initializes `last_ticks = 0` outside
the loop. The first `RUNNING` tick computes `dt_ticks = osKernelGetTickCount() - 0`,
which is "boot time so far". `dt_s` is clamped to 0.05, so it's bounded —
but it's still a wrong dt that gets used for one step.

**Fix.** Initialize `last_ticks = osKernelGetTickCount()` right before the
infinite loop, and re-initialize when transitioning *into* `RUNNING`.

### 4.4 Globals `updated_velocity / heading / height` are unsynchronized

Read by the control task without a critical section. On Cortex-M4 the
individual float reads are aligned-32-bit atomic, so single-field reads are
safe, but if the SPI handler ever decides to update them together as a
"command frame", the control task can see a torn snapshot.

**Fix.** Move them into a small `cmd_shared_state` struct guarded by the
same critical-section helpers as `servo_shared_state`.

### 4.5 `compensate`/`uncompensate` names are misleading

[`servos.c:35-45`](Core/Src/servos.c:35) — applies `FEMUR_OFFSET_ANGLE_RAD`
to `[2]`, which is the **tibia** index. The constant name lies. Rename to
`COXA_FEMUR_TIBIA_OFFSET[3]` array or `TIBIA_OFFSET_ANGLE_RAD`. The sign
flip on `[1]` is also undocumented — add a one-line comment about the
mechanical convention.

### 4.6 `next_joint_angles` reset only on entry to `STANDUP`

[`controller.c:243-250`](Core/Src/controller.c:243): the reset happens only
when entering `STANDUP`. On every other state entry (e.g. `WALKING →
STANDING → ROTATING`), the previous state's `next_joint_angles` carries
over. This is mostly fine because the angles represent commanded positions,
but if `actual_joint_angles` has diverged from `next_joint_angles` (e.g.
servo couldn't keep up at high yaw rate), the next state will plan from the
commanded position, not the real one — locking in the drift.

**Fix.** On any state transition into a motion state, optionally re-anchor
`next_joint_angles` to `actual_joint_angles` if the difference exceeds some
threshold.

---

## 5. Missing functionality vs. the design document

[`control-model.md`](control-model.md) calls for two locomotion modes
(arc-based and strafing). The implementation supports only arc-based —
`velocity` is interpreted as "forward in current heading". Specifically
missing:

- **Strafing**: no body-frame translation input. §1.4 makes this clear.
- **Independent yaw during translation**: present implicitly (heading
  command can change while velocity is non-zero), but no separate yaw-rate
  input — only the absolute `heading` target.
- **Mode toggle button / external mode flag**: no plumbing.

These are out of scope of pure robustness, but if strafing is on the
near-term roadmap, fixing §1.4 first is the natural place to start.

---

## 6. Suggested priority

If the goal is "make the controller robust for everyday hardware testing"
without rewriting it, I'd take them in this order:

1. **§1.1** dt clamp (correctness across the board) — one-line fix.
2. **§1.2** `SERVO_ERROR` recovery + `CTRL_FAULT` state (avoid hard hangs).
3. **§1.3** STANDING height behavior (currently silently broken).
4. **§3.3** Clamp/validate `cmd` inputs (cheap insurance against SPI noise).
5. **§1.5** Scale step size to commanded velocity (fixes start/stop jerk).
6. **§3.1** Min-step hysteresis (eliminates idle servo buzz).
7. **§2.2** Add seq counters and degrade gracefully on staleness.
8. **§3.4** Wire IMU back into body-pose compensation.
9. **§1.4 + §5** Decide arc-only vs. strafing and clean up dead math.

The rest are mostly hygiene that becomes valuable as the codebase grows.
