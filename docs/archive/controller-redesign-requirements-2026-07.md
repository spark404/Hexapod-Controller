# WALKING/ROTATING Analysis and Redesign Requirements

This document analyses the WALKING and ROTATING states in `Core/Src/controller.c` in
detail, explains the mechanisms that were previously undocumented, lists the defects
found, and derives a numbered set of requirements for the controller redesign.

The desired control model is defined in [control-model.md](control-model.md)
(arc-based locomotion as default, strafing as alternate mode, proportional joystick
input, 100 Hz control / 200 Hz servo split). Requirements below reference its
sections where applicable.

---

## 1. How WALKING actually works (reconstructed)

Per control tick (nominally 10 Hz), `controller_update()` in the WALKING state does
the following. Line references are to `Core/Src/controller.c`.

### 1.1 Steering (lines 326–329)

```
desired_omega = yaw_error / dt   →  clamp(±CTRL_MAX_YAW_RATE)  →  rotation_step
```

This is a proportional heading controller with gain `1/dt`, saturated at the maximum
yaw rate. In effect: *rotate at max yaw rate toward the commanded heading until
within one tick of it*. `heading` is an absolute setpoint compared against the
dead-reckoned yaw — not a rate command.

### 1.2 Velocity (lines 332–348) — the identity round trip explained

`v_world = velocity·(cos yaw, sin yaw)` followed by a rotation by `−yaw` is an exact
identity: `v_body` is always `{velocity, 0}`. This is a vestige of an earlier design
where `heading` defined a world-frame travel direction. The *net behaviour* is:

> The robot always walks along its own +X (forward) axis; heading is achieved by
> yawing while walking.

That is arc-based locomotion — the current WALKING state is already behaviourally an
arc mode, just driven by a heading *setpoint* instead of a yaw-*rate* command. This
is good news for the redesign: the gait mechanics underneath do not need to change
conceptually to implement control-model.md §1.

### 1.3 Dead-reckoned pose integration (lines 351–369)

The body pose is integrated open-loop from the *commanded* motion using midpoint yaw
(`yaw_mid = yaw + rotation_step/2` — a reasonable numerical choice). Nothing ever
corrects this pose: the EKF output is explicitly discarded (`(void)attitude`,
line 160). Note `hexapod.rotation[2]` is never wrapped and grows without bound
(float precision degrades slowly; benign in practice, sloppy in principle).

Important: integration happens unconditionally every tick, *regardless of whether
the legs can realize the motion*. Nothing couples body speed to what the gait can
deliver (see D13).

### 1.4 Per-leg path generation (lines 377–445)

For each leg:

- `p_current` = forward kinematics of the **measured** joint angles → current foot
  position in the body frame.
- `rot_disp = rotation_step·(ẑ × r)` — the per-tick displacement of a point at the
  foot's position due to body yaw, clamped to `CTRL_MAX_ROT_FOOT_MM` (6 mm — at
  0.005 rad/tick and r ≈ 250 mm this clamp essentially never binds; the config
  comment suggesting 15–25 mm doesn't match the value).
- A **direction** vector is formed:
  - stance legs: `−movement_vector + rot_disp`
  - swing legs: `+movement_vector + rot_disp`
- The leg's **target** is the point on a fixed circle of radius
  `CTRL_LEG_RADIUS_MM` (40 mm) centred on the leg's home position, in that
  direction, at `z = −body_z` (`leg_generate_path` →
  `project_point_on_circle`). So stride amplitude is a **constant 40 mm from home**,
  independent of speed.
- A 4-waypoint path is built from `p_current` to the target: degenerate/straight
  for stance legs, a lift trapezoid for swing legs (`calculate_path` with
  **hardcoded** 25 mm lift height and 2.0 incline factor at line 147 —
  `CTRL_LIFT_Z = 50` is a different value used only by STANDUP).

The path is **regenerated from the measured foot position every tick** — this is
receding-horizon replanning, not trajectory tracking. Because the servo layer lags
the command, planning from measured positions couples servo lag back into the
planner (STANDUP was deliberately changed to plan from commanded angles for exactly
this reason — see the comment at line 271 — but WALKING/ROTATING were not).

### 1.5 Pacing and synchronization (lines 447, 469–488) — the `fminf` explained

```c
longest_path = fminf(longest_path, longest_lifted_path);   // "Some form of clamping?"
...
substeps    = longest_path / effective_velocity;           // ticks to complete
step_length = path_length[i] / substeps;                    // per-leg advance per tick
```

`effective_velocity` is the per-tick body displacement (|movement_vector| in mm/tick
plus yaw-rate rim speed `|rotation_step|·max_r`). So `substeps` is *the number of
ticks the longest path would take at body ground speed*, and each leg advances
`1/substeps` of **its own** path per tick — i.e. **all legs are scheduled to
complete their paths simultaneously, synchronized to body speed**.

The `fminf` is the synchronization horizon, and it is load-bearing, not clamping:

- If the **stance** group's remaining stroke is smaller (feet approaching the
  trailing edge of their 40 mm circles), the horizon shrinks → `substeps` drops →
  swing legs speed up to land **before the stance stroke runs out**.
- If the **swing** group's remaining path is smaller (about to land), the horizon
  shrinks → the tripod swap (§1.7) triggers promptly.

The mechanism is clever but implicit and fragile: it depends on stance-leg
path-length bookkeeping whose rotation term has a sign bug (D2), and pacing feeds
off distances to *fictitious* stance targets that are never actually tracked.

`max_r` (lines 450–458) computes the largest home-foot radius. It "works" only
because `body.translation` XY is always zero — it mixes a body-frame point with a
pose translation vector and happens to compute distance-from-origin.

### 1.6 Execution (lines 460–516)

- **Stance legs:** the interpolated point is computed and then **discarded**; the
  foot is pinned to its stored `tip_world_coordinates` and the joint angles come
  from transforming that fixed world point through the freshly integrated body pose
  (world → coxa → IK). This gives exact no-slip *in the model* by construction.
  Stance paths therefore exist **only** to feed the pacing/swap bookkeeping in §1.5.
- **Swing legs:** the interpolated point (one `step_length` along this tick's fresh
  path) becomes the new commanded position, and `tip_world_coordinates` is updated
  to it. `remaining_path_length` tracks the largest swing distance-to-touchdown.
- If `longest_path < CTRL_CLOSE_THRESH` (3 mm), the loop `continue`s for **all**
  legs: no joint targets are updated *and* `remaining_path_length` stays at its
  initial 0.0 — see D3.

### 1.7 Tripod swap (lines 518–520)

Swap fires when `remaining_path_length < 3 mm`, i.e. when the swing legs are within
3 mm of touchdown — **or** when the execute loop was skipped entirely, because 0.0
is the default (D3). `swap_legs()` toggles all six grounded flags. The newly-stance
feet are pinned wherever they were at swap time — up to ~3 mm short of their
touchdown point, possibly 1–2 mm **above the ground** (D6).

`tripod_init_gait` re-seeds `tip_world_coordinates` from measured angles only when
all six legs are grounded — effectively only on the first WALKING entry after
STANDUP. Subsequent STANDING↔WALKING transitions keep the 3/3 split and the stored
world coordinates, which is correct as long as nothing moves during STANDING.

---

## 2. How ROTATING differs

ROTATING (lines 521–671) is a copy of WALKING with `movement_vector = 0` and three
deviations:

1. **Stance rotation sign is negated relative to WALKING** — `arc_disp = −rot_disp`
   with the (physically correct) comment "Grounded legs move opposite of body
   rotation". WALKING uses `+rot_disp` for stance legs. One of the two is wrong,
   and ROTATING's is the one that matches the physics (body yaw +ψ moves a
   world-fixed point by −ψ·(ẑ×r) in the body frame). See D2.
2. **Pacing basis differs:** `distance_per_tick = CTRL_DEFAULT_VEL · dt` (a fixed
   50 mm/s) instead of the actual commanded displacement. Two states, two pacing
   conventions.
3. **The close-threshold guard is commented out** (lines 630–632). Combined with
   the `substeps = max(…, 1.0)` floor, when `longest_path` is small each leg gets
   `step_len = its entire remaining path in one tick`. The only thing preventing a
   foot from being commanded across its whole path instantaneously is the servo
   task's 8 rad/s velocity limiter. See D4.

Entry/exit thresholds: STANDING → ROTATING at `|yaw_error| > CTRL_EPS_ANG = 1e-3`
rad (0.057°). With dead-reckoned yaw this converges exactly and terminates; the
moment real sensor feedback closes this loop, a 0.057° threshold will chatter (D9).

---

## 3. Previously-unexplained code, now explained

| Location | Code | Explanation |
|---|---|---|
| controller.c:447 | `longest_path = fminf(longest_path, longest_lifted_path)` + "Some form of clamping?" | Synchronization horizon — paces all legs to complete simultaneously, keyed to whichever tripod group has less remaining stroke (§1.5). Load-bearing. |
| controller.c:332–341 | world→body velocity round trip | Exact identity; vestige of heading-as-travel-direction design. Net behaviour: walk along body +X, steer by yawing = arc locomotion (§1.2). Delete in redesign. |
| controller.c:450–458 | `max_r` loop, "Some form of clamping?" region | Max home-foot radius, used to convert yaw rate to rim speed for pacing. Correct value by accident of `body.translation` XY = 0. |
| controller.c:496–498 | stance interpolation result discarded | Intentional: stance feet are world-pinned; the path exists only for pacing/swap bookkeeping. The wasted interpolation/IK-prep work is real, though. |
| additional_functions.c:84 | `calculate_path` "I don't get this calculation yet" branch | When the remaining XY distance is shorter than the lift+lower run at the configured incline, the full lift trapezoid does not fit; the formula shrinks the apex height so ascent and descent (at the incline slope) meet within the available distance — the trapezoid degenerates toward a triangle. Explainable, but should be an explicit, tested function in the redesign. |
| additional_functions.c:147 | `interpolate` FIXME with captured path | The captured case is "step size exceeds total remaining path" (waypoints 0–2 identical, last segment 9.5 mm < step 10.0 mm). The function clamps to the path end, which is acceptable behaviour; the FIXME is resolvable with a unit test asserting exactly that. |

## 4. Defects and hazards (WALKING/ROTATING)

Ordered roughly by severity. D-numbers are referenced by the requirements in §6.

| # | Defect | Location | Effect |
|---|---|---|---|
| D1 | Control-loop dt clamped to 0.05 s while the loop runs at 0.1 s — dt is wrong **every tick** | main.c:2376 vs `CONTROL_LOOP_INTERVAL` | All velocities, yaw rates and pose integration run at half the commanded rate. Also: three inconsistent loop-period definitions (`CTRL_LOOP_DT_S = 0.2` is unused). |
| D2 | Stance-leg rotation term sign inconsistent: WALKING `+rot_disp`, ROTATING `−rot_disp`; physics says minus | controller.c:411–414 vs 582–585 | Stance path lengths (→ pacing horizon and swap timing) are skewed while walking an arc. Doesn't move feet directly (stance targets are fictitious) but corrupts synchronization. |
| D3 | `remaining_path_length` defaults to 0.0 and is only written when the execute loop runs; skip path (`longest_path < 3 mm`) → immediate swap | controller.c:374, 477–479, 518 | "Nothing to do" is conflated with "swing complete". Can cause spurious tripod swaps every tick at near-zero commanded motion. |
| D4 | ROTATING: close-threshold guard commented out + `substeps ≥ 1` floor → `step_len` = entire remaining path in one tick | controller.c:630–641 | Foot commanded across its whole path instantaneously; only the servo-task rate limiter prevents a physical jump. WALKING has the same `substeps ≥ 1` floor hazard at high effective velocity. |
| D5 | Stopping mid-cycle freezes swing legs in the air: WALKING→STANDING has no settle phase, and STANDING computes nothing at all | controller.c:221–223, no STANDING block | Robot stands on 3 legs indefinitely. Also: height commands latched in STANDING (line 176–179) produce no motion until the next WALKING/ROTATING tick, then apply as a step. |
| D6 | Swap triggers up to 3 mm before touchdown; newly-stance foot is pinned at its current (possibly airborne) position | controller.c:518, CTRL_CLOSE_THRESH | Feet land short/above ground; load transfer relies on mechanical compliance. |
| D7 | No reachability/workspace guard before IK; `inverse_kinematics` silently returns stale angles when too far and produces NaN (acos domain) when geometry is infeasible; NaN passes the servo limit check and hits UB in float→pulse conversion | inverse_kinematics.c:16, 71–72; servos.c:24 | A stance foot dragged past the workspace edge (nothing prevents this, see D13) can slam a servo to pulse 0. Highest-severity hazard in the motion path. |
| D8 | Direction degeneracy: `arc_disp` near zero (velocity and rotation nearly cancelling for a given leg) → normalized direction is noise → 40 mm target flips around the circle tick-to-tick | controller.c:143, hexapod.c:14–29 | Path-length noise corrupts pacing; only exact zero is handled. |
| D9 | `CTRL_EPS_ANG = 1e-3` rad (0.057°) entry threshold for ROTATING | controller_config.h:15 | Converges only because yaw is self-integrated fiction; will chatter with real feedback. |
| D10 | Config rot: `CTRL_MAX_YAW_RATE = 0.1` but its own comment says 0.52; swing lift hardcoded 25 mm/2.0 while `CTRL_LIFT_Z = 50`; unused: `CTRL_ROT_STEP_RAD`, `CTRL_SERVO_Z_OFFSET`, `robot.step_size`, `ctx->pending_yaw`, `ctx->delta_yaw` | controller_config.h, controller.c:147, robot.h:27 | Config cannot be trusted as documentation of behaviour. |
| D11 | WALKING/ROTATING plan from measured angles each tick while STANDUP plans from commanded angles (deliberately, per its comment) | controller.c:271 vs 383, 553 | Servo lag feeds back into the planner: softened steps, possible limit cycles. Inconsistent position-source policy. |
| D12 | `hexapod.rotation[2]` accumulates unbounded; wrap only applied to the error | controller.c:364 | Slow float-precision degradation on long runs. Minor. |
| D13 | No coupling between commanded body velocity and gait capability (fixed 40 mm stride, swap frequency emergent) | design-level | Roughly self-balancing because pacing scales with velocity, but nothing *enforces* that stance feet stay in the reachable workspace → feeds D7. |
| D14 | Per-leg loop recomputes loop-invariant `effective_velocity`; stance interpolation+transform work discarded | controller.c:469–475, 487–494 | Wasted cycles. Irrelevant at 10 Hz, relevant at the 100 Hz target of control-model.md. |

### Items needing hardware investigation (not decidable statically)

- **I1:** Measure actual ground speed vs commanded (expect ~½, confirming D1).
- **I2:** Log swap intervals while walking a tight arc, with and without the D2 sign
  fixed, to quantify the pacing skew.
- **I3:** Measure foot-touchdown gap at swap (D6) — is compliance actually
  absorbing 1–3 mm, or do newly-stance legs unload?
- **I4:** Drive velocity to the SPI cap (100 mm/s) and observe stance feet
  approaching the workspace edge (D7/D13) — determine the real safe velocity bound.

---

## 5. What is worth keeping conceptually

The redesign should not discard these ideas — they are sound:

- **World-pinned stance feet** with body motion realized through the inverse
  transform: exact no-slip by construction, and directly satisfies control-model.md
  "grounded legs do not slip".
- **Proportional completion pacing** (all legs finish together, keyed to body
  speed) — but made explicit (a gait phase variable) instead of emergent from
  path-length min/max bookkeeping.
- **Midpoint yaw integration** for the pose update.
- **Target projection on a circle around the home foot position** as the foothold
  selection rule — simple and predictable; make the radius speed-dependent.
- The existing behaviour is already arc-like (§1.2), so control-model.md's arc mode
  is a command-interface change, not a gait rewrite.

---

## 6. Requirements for the redesign

Grouped; each requirement cites the motivating defect (D#) and/or the section of
[control-model.md](control-model.md) (CM) it implements.

### Command interface

- **R1** — The controller SHALL accept a proportional command struct
  `{v_forward, v_lateral, yaw_rate, height, mode}` replacing the current
  `{velocity, heading, height}`. Arc mode uses `{v_forward, yaw_rate}`; strafe mode
  uses `{v_forward, v_lateral}` plus independent `yaw_rate`. There SHALL be no
  world-frame heading setpoint in the motion path. *(CM §1, §2, §Controller
  Responsibilities; removes the dead-reckoning dependency and the D9 threshold
  entirely; supersedes the identity round trip of §1.2.)*
- **R2** — Input shaping (dead zone, saturation of max linear velocity, max yaw
  rate, max per-foot displacement, and slew/acceleration limits on the command)
  SHALL happen in exactly one place, at the command boundary, with limits defined
  in one config header whose comments match the values. Commands SHALL be
  transported as a single coherent frame (one guarded struct, written and read
  atomically), not as independent globals. *(CM §Control Scaling; D10;
  interaction-review §4.6/P5.)*
- **R3** — Switching between arc and strafe mode SHALL NOT reset gait state:
  grounded flags, per-leg phase, and pinned world coordinates persist across the
  switch. *(CM §Mode Switching.)*
- **R4** — Negative `v_forward` (walking backward) SHALL be supported. *(CM §1
  joystick Y; currently excluded by SPI validation.)*
- **R25** — Motion SHALL follow user input with minimum delay: a change in the
  shaped command SHALL be reflected in commanded *body* motion (stance-leg IK)
  within one control period, and in swing trajectories within one control period
  via R7's event-driven refresh — the sole exception being a swing leg past its
  commit point, which lands first and reflects the command on the next step.
  Responsiveness comes from the body-velocity channel, which is always live;
  never from per-tick re-derivation of swing targets. *(User requirement,
  2026-07-20; CM §Control Scaling "proportional input".)*

### Gait engine

- **R5** — Gait state SHALL be explicit: a per-cycle phase variable (0..1) and
  per-leg role (stance/swing), advanced by dt. Tripod swap, liftoff, and touchdown
  SHALL be events derived from phase — not inferred from residual path lengths.
  *(Replaces §1.5/§1.7 emergent logic; D2, D3.)*
- **R6** — Stride length and cycle frequency SHALL be derived from commanded body
  velocity such that stance feet provably remain inside a configured workspace
  margin for the whole stance phase; commanded velocity SHALL be saturated to the
  value the gait can deliver. Both bounds SHALL be derived from measurable limits,
  not guessed constants: the velocity ceiling from a servo bandwidth model (max
  joint delta per control period), and the workspace/stride radius from the
  kinematic chain as a function of body height — not a fixed 40 mm. *(D13, feeds
  D7; interaction-review P2; walking-model-review M1/M5/D8.)*
- **R7** — Swing trajectories SHALL be persistent, phase-parameterized objects
  rather than re-derived every tick from measured joint angles: generated at
  liftoff from the *commanded* foot position to the selected foothold, then
  tracked by phase. Measured positions are used for supervision (divergence/fault
  detection) and touchdown registration, consistent with the feedback role in
  CM's task diagram. Responsiveness is preserved by **event-driven refresh**:
  when the shaped command (R2) changes beyond its dead zone, affected swing
  trajectories SHALL be regenerated within one control period, from the current
  commanded position to the new foothold — so reaction latency to user input is
  one control tick, the same as per-tick replanning, without coupling servo lag
  (D11) and input noise (D8) into the trajectory. Past a commit threshold late in
  swing (~60–70 % phase), the touchdown target SHALL freeze so the foot lands
  decisively (walking-model-review M2); the command continues to act on body
  motion throughout and is fully reflected in foot placement from the next step.
  *(D8, D11; R25.)*
- **R8** — Swing touchdown SHALL complete to the ground plane: a foot is pinned as
  stance at its planned touchdown point (z = ground), not at wherever it was when a
  threshold fired. *(D6, D3.)*
- **R9** — On command → zero, the controller SHALL execute a settle sequence:
  finish the active swing (shortened landing), ground all six legs, then enter
  STANDING. Swing legs SHALL never freeze airborne. *(D5; CM "swing legs complete
  their paths before phase swap".)*
- **R10** — Height changes SHALL be executed (rate-limited) in every standing and
  walking state, not latched until the next WALKING tick. *(D5 second half.)*
- **R11** — Per-tick foot displacement SHALL be bounded by the controller itself
  (max foot velocity × dt); the servo-layer rate limiter is a backstop, not the
  mechanism. No code path may command a full remaining path in one tick. *(D4.)*

### Timing

- **R12** — There SHALL be a single source of truth for the control period. The
  measured dt SHALL be sanity-checked against the nominal period (reject/flag
  outliers); it SHALL NOT be silently clamped to a value that rescales motion.
  *(D1.)*
- **R13** — Loop rates SHALL follow the CM task model: control task ≈ 100 Hz
  producing timestamped targets (T + one control period), servo task ≈ 200 Hz
  consuming the latest target through an overwrite-safe buffer. The gait math
  SHALL be efficient enough for 100 Hz (hoist loop invariants, no discarded
  stance-path work). *(CM task diagram; D14.)*
- **R24** — Joint targets published to the servo layer SHALL carry both a
  position setpoint and a per-joint velocity derived from the planned trajectory
  (today only position is commanded; the servo layer guesses the interpolation).
  The intended execution mechanism is the XL430's hardware profile control —
  `PROFILE_VELOCITY` written alongside `GOAL_POSITION` each tick so all joints of
  a leg complete together and the foot tracks the planned path between control
  ticks — as specified in
  [servo-velocity-control-phase2.md](servo-velocity-control-phase2.md).
  *(CM task diagram "interpolates short trajectory"; backlog Track 2.)*

### Safety and fault handling

- **R14** — IK SHALL return a status. Every IK call site SHALL handle failure by
  clamping the target into the reachable workspace and raising a recoverable
  fault — never by silently keeping stale angles. A NaN/inf barrier SHALL reject
  values at the shared-state write so nothing non-finite ever reaches the servo
  task. *(D7.)*
- **R15** — Joint targets SHALL be clamped to per-joint limits in the controller
  before publication. The servo-task limit alert SHALL become a recoverable event
  reported back to the controller (degrade to STANDING/settle), replacing the
  current `Error_Handler()` halt. *(D7; existing servo-task TODO.)*
- **R16** — There SHALL be a servo→controller status channel (bus read failures,
  limit events, tracking-divergence beyond threshold) so the controller can react;
  today this information dead-ends in logs. *(CM feedback loop "used by BOTH
  loops".)*

### Estimation and frames

- **R17** — Arc and strafe modes SHALL operate purely in the body frame (CM allows
  body-aligned strafing), making world-frame dead reckoning unnecessary for
  locomotion. If a world pose is still maintained (telemetry, future navigation),
  it SHALL be an isolated estimator module with a defined interface to the EKF —
  not state mutated inside the gait code. Leg odometry (body pose inferred from
  the stance legs' measured joint angles) is the preferred estimator for world
  x/y/yaw and combines naturally with the EKF's roll/pitch. Accumulated yaw SHALL
  be wrapped. *(D12; removes the open-loop-heading fiction of §1.3;
  interaction-review §4.1/P3.)*

### Quality and verification

- **R18** — The gait core SHALL be host-buildable (no HAL/RTOS includes) and
  unit-tested. Minimum regression scenarios: straight walk, arc left/right, rotate
  in place, walk backward, stop mid-swing (settle), mode switch mid-cycle,
  command at workspace-edge velocity, degenerate near-zero command. The
  `calculate_path` short-distance branch and `interpolate` step-exceeds-path case
  (§3) SHALL each get an explicit test. *(hexapodmath already has a test harness;
  the controller has none.)*
- **R19** — Dead code and vestigial config SHALL be removed as part of the
  redesign: the velocity round trip, `pending_yaw`, `delta_yaw`, `robot.step_size`,
  `CTRL_ROT_STEP_RAD`, `CTRL_SERVO_Z_OFFSET`, and the duplicated/contradicting lift
  and yaw-rate constants. *(D10.)*

### Adopted from earlier reviews

The May 2026 review documents (now in [archive/](archive/)) contribute four
requirements the clean review did not surface, plus two design notes:

- **R20** — Before executing a tripod swap, the controller SHALL verify that the
  would-be stance triangle contains the projected body centre of mass; if not,
  defer the swap or reduce the commanded rate.
  *(walking-model-review M4.)*
- **R21** *(stretch)* — A per-leg `enabled` flag and a degraded gait (five-leg wave
  or folded-leg variant) SHOULD allow continued, slower operation after a single
  leg fault instead of a full stop. *(walking-model-review M7.)*
- **R22** — The controller↔servo shared state SHALL carry liveness sequence
  counters (`target_seq`/`actual_seq`) so either task can detect a hung peer and
  degrade safely instead of consuming stale data indefinitely.
  *(interaction-review §4.5/P4.)*
- **R23** — An external emergency-stop input (GPIO or bus command) SHALL drive an
  immediate torque-off through a controller fault state; today
  `servo_set_request_powerdown` is reachable only via the 120 s idle timer.
  *(controller-gaps §2.5.)*

Design notes carried over:

- For the rotational component of foothold selection, use `tip_home` as the moment
  arm rather than the current foot position, so accumulated foot drift does not
  bias the yaw response. *(walking-model-review D5.)*
- The swing-leg commit point requested by walking-model-review M2/D2 is subsumed by
  R7: planning the swing once per phase and tracking it *is* the commit.
- STANDUP survivors for whichever parts of that state the redesign keeps: pace
  `calculate_motion_step` by 3D (not 2D) distance, verify the pose after a
  powerdown before standing up, and select the first swing tripod from actual foot
  heights instead of hard-coding legs 1/3/5. *(controller-gaps §3.5, §2.3, §2.4.)*

---

## 7. Traceability summary

| control-model.md element | Covered by |
|---|---|
| Arc-based locomotion (§1) | R1, R4, R17 |
| Strafing locomotion (§2) | R1, R17 |
| Proportional control & saturation (§Control Scaling) | R2 |
| Mode switching without gait reset (§Mode Switching) | R3, R5 |
| No stance slip / swing completes before swap (§Responsibilities) | R5–R9 (kept mechanism: world-pinned stance feet, §5) |
| 100 Hz control / 200 Hz servo / timestamped targets (task diagram) | R12, R13 |
| Feedback used by both loops (task diagram) | R7, R16 |

| Defect | Addressed by |
|---|---|
| D1 dt clamp | R12 |
| D2 stance sign inconsistency | R5 (mechanism replaced) |
| D3 swap-by-skip | R5, R8 |
| D4 one-tick jump | R11 |
| D5 airborne freeze / latent height | R9, R10 |
| D6 short touchdown | R8 |
| D7 IK NaN/stale cascade | R14, R15, R6 |
| D8 direction degeneracy | R7 |
| D9 chatter threshold | R1 (heading setpoint removed) |
| D10 config rot | R2, R19 |
| D11 measured-vs-commanded planning | R7 |
| D12 unbounded yaw | R17 |
| D13 velocity/gait decoupling | R6 |
| D14 wasted work | R13 |
