> **ARCHIVED (2026-07-20).** Superseded by [controller-redesign-requirements.md](../controller-redesign-requirements.md) (leading plan) and [improvement-backlog.md](../improvement-backlog.md). Unique findings from this document have been folded into those. Code line references reflect the May 2026 state of the sources.

# WALKING Model — Design Review and Code Audit

## 1. The stated design

> The WALKING state moves each leg with a **fixed 40 mm stride** in the
> X/Y plane around its neutral position (`tip_home`), regardless of body
> height. The robot moves at the commanded speed by running a tripod-gait
> lift/stand cycle; lower commanded speed simply means a longer cycle
> time — the swing always covers the full stride, it just takes longer.
> New movement instructions are incorporated immediately — without waiting
> for the current cycle to complete — taking the current leg positions
> into account to decide what motion is feasible.

That decomposes into four contracts the controller must keep:

| #  | Contract                                                              |
|----|-----------------------------------------------------------------------|
| C1 | Each cycle, every leg performs a full 40 mm stride around `tip_home` in body-frame XY. |
| C2 | Body moves at the commanded linear (and angular) speed *averaged over a cycle* — at lower speeds, the cycle stretches in time, not in stride. |
| C3 | Movement is a tripod-gait swing/stance cycle.                          |
| C4 | New `(velocity, heading)` commands are absorbed without finishing the current cycle, and the controller respects the current leg geometry when doing so. |

The constant-stride choice is deliberate: it keeps gait geometry invariant
across speeds (every cycle is the same, only timing changes), gives a
predictable landing impact profile, and avoids the "shuffle" failure mode
of proportional-stride walkers at low speed.

The rest of this document evaluates the model first, then checks each
contract against [`Core/Src/controller.c`](Core/Src/controller.c).

---

## 2. Is the model itself good?

### What's strong

- **Reach disk as the primary constraint.** Decoupling "where can my foot
  go" from "how fast do I want to move" is the right abstraction. It maps
  cleanly to hardware: a per-leg reachability volume and a per-cycle stride
  length.
- **Constant stride.** Every cycle is the same shape — only the rate
  changes. The gait is symmetric (stance and swing both cover ~40 mm), so
  in steady state the two phases finish together, no idle dwell.
- **Tripod gait.** Statically stable at any instant (3 stance feet form a
  triangle), and twice as fast per cycle as a wave gait. The right default
  for a flat-floor hexapod.
- **Reactive (no committed cycle).** Re-planning every tick from the
  current foot position is the modern way; pre-baked gait pattern
  generators are brittle to disturbances.

### What's missing from the model

Each of these is a real gap — they all show up later as concrete code
issues:

**M1 — Reach disk depends on body height.** "40 mm regardless of body
height" is geometrically untrue. With longer effective leg projection (low
body Z), 40 mm is conservative — the mechanical reach is far larger and
you're leaving stride length on the table. With shorter projection (high
body Z), 40 mm may exceed the IK-feasible XY annulus. The model should
either treat `CTRL_LEG_RADIUS_MM` as `f(body_z)` (e.g. ≤ half the
horizontal projection of the femur+tibia at the commanded height), or
declare a body-Z operating range over which a fixed 40 mm is safe and
refuse heights outside it.

**M2 — Commit threshold for swing legs.** A "re-plan every tick"
controller that retargets the swing-leg destination based on the latest
command will, under noisy or rapidly-changing input, never let a swing leg
land. **This is sharper under constant stride** than under proportional
stride: at low velocity the swing arc is long *in time*, so joystick noise
has many ticks to keep nudging the target. The model needs an explicit
*commit point* — once the swing foot is past apex and within some fraction
of the projected touch-down, freeze the target. Without it, C4 fights C1.

**M3 — Stance-edge swap as a safety net.** With constant stride and
symmetric phases, the current "swap when swing legs near target" trigger
*usually* works because stance and swing finish at the same time. It only
diverges during direction reversals or yaw transients — which is exactly
when robustness matters. A robust swap rule fires when **either** all
swing legs have landed **or** any stance leg is past the trailing edge of
its reach disk, whichever comes first.

**M4 — Static-stability check.** Tripod gait is stable only if body COM
projects inside the stance triangle. With a moving COM (e.g. rapid yaw
change while translating) and a fixed swap schedule, this can fail.
Add a "no swap if the would-be stance triangle doesn't contain the COM
projection" guard.

**M5 — Cycle-time bounds (replaces "speed clamp").** With constant stride,
`T_cycle ≈ S/v`. That has two limits:

- *Upper bound on v* — at high commanded speed, `T_cycle` shrinks until
  the swing legs can't physically complete a 40 mm arc inside the servo
  bandwidth. Need `v_max = S / T_min`, where `T_min` is the
  servo-bandwidth-limited cycle time.
- *Lower bound on v* — at low commanded speed, `T_cycle` grows; legs
  spend a long time aloft (1 mm/s → 40 s cycle, with the swing tripod off
  the ground for ~20 s). Need a `T_cycle_max` cap below which the
  controller refuses to walk and falls back to STANDING (or, if you want
  smooth slow walking, breaks into shorter sub-strides).

The model as stated is silent on both bounds. Pick them and document them.

**M6 — Acceleration / jerk limit on the command itself.** Even with C4
(reactive replanning), a step change in commanded velocity translates to a
step change in per-foot path length within a single tick. The path planner
will happily emit it; the servos can't follow it. **Constant stride
amplifies this**: a direction reversal makes a swing leg's target jump by
up to 80 mm in one tick. The model should include a controller-level slew
rate on `velocity` and `heading`, separate from the servo-side accel cap.

**M7 — Degraded gait if a leg fails.** The model assumes the tripod
always works. A robust model nominates a degraded gait (4+1 wave, or "drop
to quadruped" if a single leg is faulted) so a single mechanical fault
doesn't immediately cascade.

Items M2, M3, M5, M6 are the high-impact ones for the robustness claim
of C4.

---

## 3. Does the implementation match the model?

### C1 — Constant 40 mm stride

**Yes — this is correctly implemented.** [`controller.c:143`](Core/Src/controller.c:143):

```c
project_point_on_circle(step_size /* = 40 */, origin /* = tip_home */, arc_disp, point);
```

The target is forced onto the boundary of the disk regardless of
`arc_disp` magnitude. Per the design, that's the right behavior — small
velocity should still produce a full-stride target, just traversed
slowly.

Two related issues remain:

- **The current foot position is never tested against the disk.** If a leg
  drifts outside its disk (post-STANDUP, post-disturbance, prior swap
  ending late), nothing forces it back. The disk constraint is enforced
  only on *target*, never on *state*.
- **The `z` of the target is `-body_z`** ([`controller.c:144`](Core/Src/controller.c:144)),
  measured in body frame; if the body is tilted (which the controller
  doesn't know — IMU is ignored), the disk plane is wrong.

### C2 — Body moves at the commanded speed (cycle-averaged)

**Yes — and the pacing math is correctly intended, but fragilely encoded.**

Body velocity is set by the stance legs (which are anchored in world).
The per-tick body displacement is `velocity · dt_s` regardless of any
pacing logic. What pacing actually determines is how many ticks the
cycle takes:

```c
substeps    = longest_path / effective_velocity;     // controller.c:481
step_length = path_length / substeps;
```

If `longest_path = longest_stance_path`, then over `substeps` ticks the
stance legs traverse their full path (= body translates by that path
length in world). Body velocity averages to commanded. Swing legs get
`step_length_swing = swing_3d_path / substeps`, which is larger than
the body's per-tick displacement — i.e. swing legs move *faster than
the body* in 3D, which they need to because their path includes the
lift trapezoid.

The current code achieves this via:

```c
longest_path = fminf(longest_stance, longest_swing);
```

In steady-state symmetric tripod, swing 3D path is always longer than
stance (lift adds ~25 mm of height plus the diagonal lift/lower
segments), so `fminf` resolves to stance. The intent is fulfilled in
the normal case. The fragility (D1 below) is that it's an *implicit*
encoding — any condition that makes swing shorter than stance flips the
pacing target.

### C3 — Tripod gait cycle

**Mostly.** Initial tripod is hard-coded `grounded = {1,0,1,0,1,0}` after
`tripod_init_gait` (legs 1, 3, 5 lift on entry —
[`controller.c:97-99`](Core/Src/controller.c:97)). `swap_legs` toggles all
six on each swap. The swap trigger is:

```c
if (remaining_path_length < CTRL_CLOSE_THRESH) swap_legs(ctx);    // controller.c:518
```

where `remaining_path_length` is computed only across **swing** legs
([`controller.c:500-507`](Core/Src/controller.c:500)). Under constant
stride this is OK in steady state because stance and swing finish
together. It only fails during transients — see M3.

Also: there's no enforcement that all three swing legs land *together*.
Whichever swing leg's path is shortest reaches `paths[i][3]` first; the
swap waits until the *worst* one is close. Until then, the early
finishers idle at their target. With constant stride the swing arcs are
all ~40 mm so this is a small effect, but it's there.

### C4 — Mid-cycle command re-incorporation

**Geometrically yes, dynamically no.**

What works:

- The path is fully regenerated every tick from `actual_joint_angles`
  (via `leg_current_position_body`, line 383). New `arc_disp` is computed
  from the *current* velocity / yaw_error, so commands really do apply on
  the next tick.
- Grounded legs keep their world-frame anchor (`tip_world_coordinates`,
  line 498), so a direction change doesn't make stance feet slip — they
  stay planted, and the body twists around them until the swap fires.

What doesn't:

- **No commit point** (M2). Every tick the swing target is projected
  *fresh* from the current `arc_disp`. With noisy input the foot
  oscillates in mid-air.
- **No cycle-time bounds** (M5). The commanded velocity passes straight
  through to `movement_vector` and `substeps`; nothing measures whether
  the resulting cycle time is inside `[T_min, T_max]`. So "I asked for
  200 mm/s" produces a swing time the servos can't deliver, and "I asked
  for 0.5 mm/s" produces an 80-second cycle where legs hang aloft for
  half a minute.
- **No controller-level slew** (M6). The path planner sees the new
  command in full magnitude on the first tick after a change. Under
  constant stride, a direction reversal flips the swing target by up to
  80 mm in a single tick; the servos can't follow.
- **The `arc_disp` direction is a function of the current foot position**
  for the rotation component:
  ```c
  r_perp = (-p_current.y, p_current.x);
  rot_disp = r_perp * rotation_step;
  ```
  ([`controller.c:391-400`](Core/Src/controller.c:391)). This is correct
  geometry — the per-foot tangent of the rotation — but it means a stance
  leg that has drifted outside the reach disk gets `r_perp` pointing
  inconsistently relative to `tip_home`, and the projection onto the 40 mm
  circle can fold back across `tip_home`. The fix is to compute `r_perp`
  from `tip_home`, not from `p_current`, for the *target* direction. (The
  current position is fine as the path's start.)

### What's *not* in the code at all

These pieces of the design are absent — they need to be added if the
"robust to mid-cycle change" claim is to hold:

| Tag | Missing piece |
|-----|---------------|
| M1  | Body-Z-dependent reach disk. `CTRL_LEG_RADIUS_MM` is a single constant. |
| M2  | Commit point on swing legs — currently every tick re-targets. |
| M3  | Stance-edge swap as a fallback alongside swing-arrival. |
| M4  | COM-in-stance-triangle check before allowing a swap. |
| M5  | Cycle-time bounds (clamp `v` to `[v_min, v_max] = [S/T_max, S/T_min]`). |
| M6  | Slew rate on `velocity` and `heading` at the controller layer. |
| M7  | Degraded gait if a leg is faulted. |
| —   | Drag-zone check: if `p_current` is already outside the disk, force a swap. |
| —   | Sanity gate on `velocity`: an SPI glitch can produce a 1 m/s command that the controller will obey. |

---

## 4. Concrete code-vs-model discrepancies

Tightly-scoped issues that follow from the design audit above:

### D1 — `fminf(stance, swing)` is a fragile encoding of "pace by stance"

[`controller.c:447`](Core/Src/controller.c:447) and
[`controller.c:619`](Core/Src/controller.c:619):

```c
longest_path = fminf(longest_path, longest_lifted_path);
```

The intent is correct: pace cycle time by the **stance** phase, because
stance legs physically determine body velocity (they're anchored in
world). Swing legs adapt their per-tick step to keep up — they need to
move faster in 3D because their path includes the lift trapezoid.

The encoding works *because* of a contingent fact: in steady-state
tripod gait, the swing leg's 3D path is always longer than the stance
leg's path in body frame. So `fminf` picks stance — the right answer.

But it's fragile. Any condition where swing path ends up shorter than
stance path flips the pacing target:

- A swing leg that started its arc already close to its target (prior
  cycle's edge case, swap-during-transient, manual nudge).
- A swing leg whose path is shortened by a mid-cycle re-targeting toward
  its current position (e.g. small joystick deflection after a large
  one).

When that happens, `fminf` picks swing → `substeps` shrinks → stance
leg's `step_length = stance_path / substeps` exceeds the per-tick body
displacement → stance over-steps → body actually translates *faster*
than commanded velocity for that cycle.

**Fix.** Make the intent explicit. Drop the `fminf` line and use:

```c
longest_path = longest_stance;   // pace by stance — stance legs set body velocity
```

Same behavior in steady state, no edge-case overshoot.

**What `fminf` does *not* fix.** If commanded velocity is high enough that
the swing leg physically can't keep up (per-tick `step_length_swing`
exceeds servo bandwidth), the right answer is not to slow the cycle
down — it's to refuse the commanded velocity in the first place. That's
the cycle-time bounds check in **M5** / **D9**: clamp `v ≤ S / T_min`
where `T_min` is the bandwidth-limited cycle floor. Together D1 (pace
by stance) and D9 (clamp `v` to swing-feasible) cleanly separate the
two concerns: D1 makes velocity tracking honest, D9 makes the
commanded velocity physically achievable.

### D2 — Swing leg has no commit point

Every tick, the swing leg's `arc_disp` is rebuilt from the latest
`movement_vector + rot_disp`, and its target is freshly projected. If the
joystick is noisy or the user is making continuous adjustments, the
target chases. Under constant stride this is more important than under
proportional stride: at low velocity the swing leg spends many ticks
aloft, all of them re-targeting.

A workable commit rule:

```c
if (leg->swing_phase >= COMMIT_PHASE_FRACTION) {
    // freeze leg->committed_target, plan against it
} else {
    // existing re-projection
}
```

with `swing_phase` tracked in `leg_state` as fraction of swing arc
completed.

### D3 — Swap is gated on swing legs only

[`controller.c:500-520`](Core/Src/controller.c:500). `remaining_path_length`
loops only inside the `else` (swing) branch. Under constant stride this
works in steady state but breaks down during transients. A robust
controller also asks: are stance legs near the trailing edge of their
reach disks? Concretely, for each stance leg compute

```c
disk_margin = CTRL_LEG_RADIUS_MM
            − arm_euclidean_distance_f32(p_current.xy, tip_home.xy, 2);
```

and force a swap when *any* stance leg's `disk_margin < SWAP_MARGIN`.

### D4 — No detection of "foot outside disk"

If a leg is somehow outside its 40 mm disk (post-STANDUP, post-disturbance,
or because the prior swing phase ended late), nothing forces it back. A
guard at the top of the WALKING leg loop:

```c
float32_t in_disk = arm_euclidean_distance_f32(p_current, leg->tip_home, 2);
if (in_disk > CTRL_LEG_RADIUS_MM + DISK_HYSTERESIS && leg->grounded) {
    // mark this leg as the next to swing
}
```

### D5 — `rot_disp` uses `p_current`, not `tip_home`, as the moment arm

[`controller.c:385-400`](Core/Src/controller.c:385). For the *target*
direction, the moment arm should be `tip_home` (the nominal foot position
about which the body rotates). Using `p_current` makes the projected
target depend on accumulated foot drift, which compounds into a slow yaw
bias. Use `tip_home` for `r_perp`; use `p_current` only as the path's
start.

### D6 — `velocity` and `heading` are unfiltered

[`controller.c:171-173`](Core/Src/controller.c:171). The command is
copied straight from `cmd` into the math. No deadband (beyond
`CTRL_EPS_VEL`), no slew, no upper limit. A `velocity` of `1e6` is
honored. Add:

```c
velocity = clampf(velocity, -CTRL_MAX_VEL, +CTRL_MAX_VEL);
ctx->filtered_velocity += clampf(velocity - ctx->filtered_velocity,
                                 -CTRL_VEL_SLEW * dt_s,
                                 +CTRL_VEL_SLEW * dt_s);
```

and feed `filtered_velocity` into the kinematics. Same treatment for
`heading`.

### D7 — Effective velocity mixes magnitudes with no direction

[`controller.c:469-473`](Core/Src/controller.c:469):

```c
movement_velocity   = arm_vec_magnitude_f32(movement_vector, 3);
rotational_velocity = fabsf(rotation_step) * max_r;
effective_velocity  = movement_velocity + rotational_velocity;
```

These are scalar magnitudes that get summed. For a leg whose linear and
rotational displacements point opposite ways (which happens for the legs
on the rotation side opposite the translation), the actual per-tick
displacement is much smaller than the sum suggests. The sum
overestimates `effective_velocity` and under-budgets time, making those
legs lag. Use per-leg `|arc_disp|` for pacing instead.

### D8 — The 40 mm constant has no relation to actual leg mechanics

`CTRL_LEG_RADIUS_MM = 40` is set in a config file, with no derivation
from the femur/tibia lengths or the home-pose IK margin. With the
documented `tip_home_angles = {0, 0, 90°}` for every leg
([`robot_config.h:18, 28, 38, ...`](Core/Inc/robot_config.h:18)), the IK
margin in body-frame XY is some specific number — and 40 mm may be
inside or outside it. **Under constant stride this matters more**: every
cycle commits to a full 40 mm step. If 40 mm is outside the IK envelope
at the current body Z, every single cycle fails. Worth deriving once
from the kinematic chain and asserting at startup.

### D9 — No cycle-time bounds

Nowhere does the controller compute `T_cycle = S / v` and check it
against bandwidth. Commanded velocity flows through to per-tick path
length unchecked. Add at the top of WALKING:

```c
const float32_t T_cycle = CTRL_LEG_RADIUS_MM * 2.0f / fmaxf(fabsf(velocity), CTRL_EPS_VEL);
if (T_cycle < T_CYCLE_MIN)  velocity = copysignf(CTRL_LEG_RADIUS_MM * 2.0f / T_CYCLE_MIN, velocity);
if (T_cycle > T_CYCLE_MAX)  { /* refuse — fall back to STANDING */ }
```

with `T_CYCLE_MIN` derived from servo bandwidth (~0.25 s today) and
`T_CYCLE_MAX` chosen by feel (a few seconds).

---

## 5. Summary recommendation

The four contracts state a coherent walking model. The pieces that need
to be added to the model itself — before any code change — are:

1. **Body-Z-aware reach disk** (M1).
2. **Swing-leg commit point** (M2). Without it, the "react every tick"
   property eats itself — and constant stride makes this sharper, not
   softer.
3. **Stance-edge swap trigger as safety net** (M3) — alongside, not
   replacing, the existing swing-arrival trigger.
4. **Cycle-time bounds** (M5) and **controller-level slew** (M6) —
   together these are what makes "incorporate new commands directly"
   actually safe.

In the code, the priority order:

| Order | Fix  | Effort  | Pay-off                                          |
|-------|------|---------|--------------------------------------------------|
| 1     | D9   | small   | Refuse impossible / nonsensical speeds (M5).     |
| 2     | D6   | small   | Filter input (M6); eliminates SPI-glitch chaos.  |
| 3     | D1   | 1 line  | Make "pace by stance" explicit; eliminates edge-case overshoot. |
| 4     | D2   | medium  | Stable swing legs under noisy commands.          |
| 5     | D3   | medium  | Reliable transient behavior (reversals, yaw).    |
| 6     | D4   | small   | Self-correction if a leg ends up outside disk.   |
| 7     | D5   | 1 line  | Removes yaw bias from rotation pacing.           |
| 8     | D7   | small   | Better pacing under mixed translation+rotation.  |
| 9     | D8   | small   | Asserts 40 mm is feasible at current body Z.     |

D9 + D6 are the cheapest defensive fixes — they protect the controller
from inputs it cannot honor. D1 is a one-line clean-up that removes a
fragility but isn't urgent (the current code does the right thing in
steady state). D2 + D3 are the medium-effort additions that actually
deliver on C4's "robust to mid-cycle changes" property.
