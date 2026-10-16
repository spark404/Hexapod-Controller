> **ARCHIVED (2026-07-20).** Superseded by [controller-redesign-requirements.md](../controller-redesign-requirements.md) (leading plan) and [improvement-backlog.md](../improvement-backlog.md). Unique findings from this document have been folded into those. Code line references reflect the May 2026 state of the sources.

# Controller ↔ Servo Task Interaction Review

This doc evaluates the boundary between the control task and the servo task
against the WALKING-state design contracts from
[`walking-model-review.md`](walking-model-review.md):

- **C1** — every leg performs a full 40 mm stride around `tip_home`.
- **C2** — body moves at the commanded speed (cycle-averaged).
- **C3** — tripod-gait swing/stance cycle.
- **C4** — new commands are absorbed mid-cycle, using current leg
  positions to decide what's feasible.

The question: does the interaction actually deliver these, or does the
controller's plan get silently broken at the seam?

---

## 1. The protocol as built

The interaction is a flat shared-memory pipeline (one struct,
`servo_shared_state`, guarded by `taskENTER_CRITICAL` helpers in
[`servos.c`](Core/Src/servos.c)):

```
  ┌────────────────── Control task (10 Hz) ──────────────────┐
  │  1. servo_copy_actual_joint_angles()      [read sensor]   │
  │  2. controller_update(...)                [plan one tick] │
  │  3. servo_set_target_joint_angles()       [publish goal]  │
  └──────────────────────────────────────────────────────────┘
                              ↓
        servo_shared_state.target_joint_angles[6][3]
                              ↓
  ┌────────────────── Servo task (50 Hz) ───────────────────┐
  │  1. dynamixel_get(... PRESENT_POSITION)   [read sensor]  │
  │  2. servo_copy_target_joint_angles()      [read goal]    │
  │  3. per-joint deadband / vel / accel clamp [interpolate] │
  │  4. dynamixel_set(... GOAL_POSITION)      [drive]        │
  │  5. servo_set_actual_joint_angles()       [publish pos]  │
  └─────────────────────────────────────────────────────────┘
```

**Direction of information flow:**

- *Controller → Servo:* `target_joint_angles` (a stepping stone — the
  position the joint should reach by some unspecified future moment).
- *Servo → Controller:* `actual_joint_angles` (where the joint is now).
- *Sideband:* `request_powerdown` and `limit_alert_enabled` (controller →
  servo only). No counterpart in the other direction.

There is **no contract on time**: the controller does not tag its
targets with a deadline; the servo task does not report whether the
previous target was reached. Each side asynchronously reads the latest
value the other published.

---

## 2. What the design implicitly assumes

For the WALKING model to deliver C1–C4, the interaction has to satisfy
four implicit assumptions:

| #  | Assumption                                                                |
|----|---------------------------------------------------------------------------|
| A1 | The servo task can reach `target_joint_angles` within one control tick (100 ms). |
| A2 | Therefore `actual_joint_angles` ≈ previously-commanded target on every control-tick read. |
| A3 | Therefore the controller's open-loop body-pose integration (`hexapod.translation += velocity·dt_s`) stays consistent with where the legs physically are. |
| A4 | Faults at the servo level can be reflected back to the controller so it can re-plan, abort, or hold. |

If any of these fail, one or more of C1–C4 silently breaks. Let's check
each.

---

## 3. Where the assumptions hold

**A1, A2 — Normal-velocity walking.** With `CTRL_DEFAULT_VEL = 50 mm/s`
and a 40 mm stride, per-control-tick joint deltas are typically
0.05–0.15 rad. Servo bandwidth = `SERVO_MAX_VELOCITY * 0.02 = 0.16 rad`
per servo tick, ×5 servo ticks per control tick = 0.8 rad/control-tick
ceiling. Plenty of margin. **A1/A2 hold comfortably for the configured
operating point.** The servo task acts as a smooth low-pass filter; the
controller never notices the limits.

**Mid-cycle command absorption (C4) at the protocol level.** The control
task re-reads `actual_joint_angles` at the top of every tick (line
2384), passes them into the path planner (`leg_current_position_body`
uses `actual_joint_angles`), and publishes fresh targets. So the
"replan every tick from where we actually are" property is correctly
plumbed through the shared state.

**Critical-section discipline.** Every read or write of
`servo_shared_state` goes through a helper in
[`servos.c`](Core/Src/servos.c) that wraps the access in
`taskENTER_CRITICAL` / `taskEXIT_CRITICAL`. No torn reads. ✓

**Startup ordering.** The control task waits on `EVT_SERVO_READY`
([`main.c:2341-2344`](Core/Src/main.c:2341)). The servo task only sets
that flag after `SYNC_FROM_HW` (seed target = actual from physical
servos) and `SYNC_TO_HW` (first goal write). So when `controller_init`
runs, the shared state is already coherent.

---

## 4. Where the assumptions break

### 4.1 A3 fails: body-pose integration is open-loop

This is the deepest interaction issue and it directly compromises the
WALKING contracts.

In [`controller.c:362-364`](Core/Src/controller.c:362):

```c
ctx->robot.hexapod.translation[0] += dx_world;
ctx->robot.hexapod.translation[1] += dy_world;
ctx->robot.hexapod.rotation[2]    += rotation_step;
```

This integrates body pose at the **commanded** velocity every tick,
regardless of whether the legs have actually carried the body that far.
The controller's belief about world position is "what the legs would
have done if they tracked perfectly". Nothing checks back against the
actual leg positions.

**The grounded-leg slip mechanism.** A swing leg landing publishes:

```c
arm_vec_copy_f32(p_next_world, leg->tip_world_coordinates, 3);   // controller.c:508
```

i.e. `tip_world_coordinates` is anchored using the *current model body
pose*. Subsequent ticks treat that anchor as ground truth — they
transform the anchor back to coxa frame using `Tinv` (also derived from
the current model body pose) and command the leg to stay there.

If the physical body is lagging the model by Δ mm (because servos
couldn't push it that fast), the world-frame anchor was set Δ mm ahead
of where the foot physically is. Every subsequent tick the controller
believes the foot is planted at the leading anchor and drags it forward
in body frame to maintain that fiction. **The grounded leg slips Δ mm
in world.**

Worked example. Body model integrates 5 mm/tick (50 mm/s × 0.1 s).
Servo task can only deliver 4 mm/tick of physical body motion (because
the joint angles it can produce in 100 ms fall short of the
controller's commanded delta). At the moment a swing leg lands:

- Model body at world (50, 0). Physical body at world (40, 0).
- Leg lands; `tip_world_coordinates` is set using model body =
  somewhere around (50 + foot_offset, 0).
- Physically the foot is at (40 + foot_offset, 0).
- Discrepancy: 10 mm. The next 10 control ticks each drag the foot
  another 1 mm in body frame to maintain the wrong anchor.

This violates **C1** (the foot drifts off its disk in world frame even
though body-frame planning keeps it on the disk) and undermines the
"taking current leg positions into account" half of **C4** (we *use*
the actual leg position, but we *anchor* against a fictional body
pose).

### 4.2 A1 fails: controller has no model of servo bandwidth

The path planner in [`controller.c:481-488`](Core/Src/controller.c:481)
computes `substeps` purely from kinematic path length and commanded
velocity — it has no awareness of `SERVO_MAX_VELOCITY`,
`SERVO_MAX_ACCELERATION`, the deadband, or the min-step.

So the controller can — and at high velocity will — publish targets
that the servo task can't physically reach in the 100 ms before the
next target arrives. The servo task silently does its best, lag
accumulates, A2 fails, and §4.1 kicks in.

This is the missing piece behind **M5/D9** in
[`walking-model-review.md`](walking-model-review.md): the cycle-time
floor `T_min` *must* be derived from the servo bandwidth model, not
guessed. The controller needs to know:

```
max_per_tick_joint_delta = SERVO_MAX_VELOCITY * CONTROL_LOOP_INTERVAL_S
                         = 8 rad/s * 0.1 s = 0.8 rad
```

…and refuse commanded velocities that would require larger joint
deltas. Currently nothing does this.

### 4.3 A4 fails: servo errors are a one-way doorway

[`main.c:2616-2624`](Core/Src/main.c:2616):

```c
case SERVO_ERROR:
    for (int i=0; i<18; i++) {
        dynamixel_set_torque_enable(&dynamixel_servos[i], 0);
        dynamixel_set_led(&dynamixel_servos[i], 0);
    }
    Error_Handler();
```

When a joint limit alert fires (or, in principle, any other servo-side
fault), the servo task drops torque and busy-loops in
`Error_Handler()`. The controller is not notified. It keeps publishing
new `target_joint_angles` every 100 ms forever, into a task that is
never going to read them again. The state callback into `main.c`
doesn't see a fault either — only state *transitions* are reported, and
the fault path bypasses that.

In terms of the contracts: there's no failure-handling at all. The
"robust" claim of C4 has no story for "what happens when the actuators
can't do what was asked".

### 4.4 A2 has a softer failure mode under low velocity

Even when A1 holds, the servo task's per-joint **deadband** (0.002 rad)
and **min-step** (0.003 rad) interact awkwardly with very slow,
finely-sampled trajectories.

The controller can compute, e.g., a 0.0015 rad delta for a joint over
one control tick. The servo task sees this on every read, decides it's
inside the deadband, and emits no motion. The next control tick's path
planner sees `actual_joint_angles` unchanged from last tick — the
robot's leg position model "freezes" with respect to the commanded
plan.

If this happens to a grounded leg, the body model still integrates as
if motion occurred (open-loop, §4.1). The slip in §4.1 can accumulate
purely from deadband-stalled stance legs at low velocity.

### 4.5 No liveness check in either direction

The shared state has no sequence number, no timestamp, no "did you
actually process my last target?" Either task could hang and the other
would keep happily reading the last stable value. Combined with §4.3,
this means a servo-task fault produces an indefinite period of "robot
looks alive at the SPI level (controller still running) but the legs
do nothing."

### 4.6 Command snapshot torn against the controller's frame

The shared SPI command (`updated_velocity`, `updated_heading`,
`updated_height` — globals in
[`main.c:299-301`](Core/Src/main.c:299)) is copied into the local
`cmd` struct field-by-field at
[`main.c:2379-2381`](Core/Src/main.c:2379), without a critical section.

For C4, this means: if the SPI handler updates all three fields as a
coherent "command frame", the controller can see, e.g., the new
velocity but the old heading on one tick, and absorb that as the new
target. One tick later it gets the new heading. The robot wobbles for
100 ms.

Float reads are atomic on Cortex-M4, so within a single field there's
no tearing — but the *across-field* atomicity that the SPI protocol
likely intends isn't enforced anywhere.

---

## 5. Summary: does the interaction match the design?

**Locally yes; globally no.**

- The protocol shape (10 Hz planner publishes, 50 Hz tracker consumes;
  feedback via `actual_joint_angles`; bounded by critical sections) is
  correct and matches the design's intent.
- The implementation faithfully translates each contract into shared
  state — C3 (tripod gait) is fully in the controller and works; C4's
  "replan every tick from actuals" is plumbed through.
- But the design implicitly assumes perfect servo tracking, and the
  implementation doesn't enforce that assumption or detect when it
  fails. The result is that A1–A4 form a load-bearing wall that the
  controller has no view of.

The most consequential break is **§4.1 (open-loop body-pose
integration)**, which silently violates **C1** (legs drift off-disk in
world) under any leg lag. The most actionable breaks are **§4.2** (no
servo bandwidth model in the controller) and **§4.3** (servo errors
have no path back).

---

## 6. Concrete additions to close the gaps

In rough priority order. These are protocol additions — the existing
shared-memory layout would just need a few new fields and an extra
helper or two.

### P1 — Servo fault back-channel (§4.3)

Add to `servo_shared_state_t`:

```c
typedef enum {
    SERVO_FAULT_NONE,
    SERVO_FAULT_BUS,          // can't talk to bus
    SERVO_FAULT_LIMIT,        // joint limit alert
    SERVO_FAULT_BANDWIDTH,    // sustained lag > threshold
} servo_fault_t;

servo_fault_t fault_code;
uint8_t       fault_leg;
uint8_t       fault_joint;
```

Servo task sets these instead of jumping into `Error_Handler`. Add a
`CTRL_FAULT` state in the controller that reads the fault on every tick
and drives to a safe pose (or commands the servo task to disable
torque cleanly). The existing `controller_state_cb_t` callback can
relay the fault to whatever owns the LEDs / SPI / external e-stop.

### P2 — Servo bandwidth model in the controller (§4.2)

Two options, pick one:

- **Static** — define `SERVO_MAX_JOINT_DELTA_PER_TICK = SERVO_MAX_VELOCITY
  * CONTROL_LOOP_INTERVAL_S` and refuse to plan a step that exceeds it.
  Clamp `velocity` and `yaw_rate` accordingly. Cheap, conservative.
- **Dynamic** — publish `last_velocity[i][j]` from the servo task into
  `servo_shared_state` and let the controller see the actual headroom
  per joint. More accurate, more shared state.

Either way, the controller becomes able to refuse impossible commands
(M5 / D9 from `walking-model-review.md` becomes implementable).

### P3 — Close the body-pose loop (§4.1)

The grounded-leg world anchors *are* a measurement of body world pose.
With three grounded legs and known leg geometry, the body's (x, y, yaw)
in world is determined up to the leg-to-anchor errors. Two ways to
exploit this:

- **Cheap fix** — anchor `tip_world_coordinates` against the *physical*
  body pose, not the model. That requires inferring physical body pose
  from `actual_joint_angles + tip_world_coordinates`, then using *that*
  for `Thexapod_body` in the kinematics. Equivalent to: integrate
  `hexapod.translation` from the median per-leg-implied body motion
  rather than from `velocity * dt_s`.
- **Proper fix** — leg-odometry: every control tick, compute body
  motion from the three stance legs' actual joint angles and update
  `hexapod.translation` from that, not from the commanded velocity.
  Yaw too. This makes the world frame physically grounded.

The cheap fix removes the slip mechanism with minimal surgery. The
proper fix is the right long-term path and integrates naturally with
the IMU (attitude EKF + leg odometry = full state estimator).

### P4 — Liveness counters (§4.5)

Add `target_seq` (written by controller, incremented every tick) and
`actual_seq` (written by servo task, incremented on every `RUNNING`
cycle). Each side can detect peer staleness as `seq_diff > N` and react
(controller throttles / faults; servo task assumes "hold current
target" if controller is silent).

### P5 — Atomic command frame (§4.6)

Wrap the SPI command in the same shared-state pattern: a
`controller_command_t` field inside `servo_shared_state` (or a parallel
struct), guarded by the same critical-section helpers. SPI handler
writes all three fields atomically; controller reads them atomically.

### P6 — Deadband-aware path quantization (§4.4)

This is the smallest fix. Either:

- Raise `CTRL_EPS_VEL` so commands that produce sub-deadband joint
  deltas just fall back to STANDING, or
- In the controller, accumulate small per-tick deltas and only publish
  them once they cross the deadband — i.e. do the deadband
  pre-quantization at the controller level so the model and the servo
  task agree on when motion happens.

---

## 7. What's actually fine

To be balanced: not every aspect of the interaction is broken. These
parts of the design are correctly realized:

- The 5:1 servo:control loop rate ratio gives plenty of headroom for
  the servo task to smooth out per-control-tick steps.
- `compensate` / `uncompensate` correctly isolate the controller from
  hardware geometry.
- The seed-on-boot ordering (`SYNC_FROM_HW` → `SYNC_TO_HW` →
  `EVT_SERVO_READY` → `controller_init`) ensures the controller never
  publishes targets into an unprepared servo task.
- The critical-section discipline is consistent.
- `request_powerdown` is correctly latched and acted on (within the
  servo task's tick cadence).

The interaction shape is right. What it lacks is the bidirectional
honesty needed to keep the WALKING contracts intact when the world
doesn't cooperate.
