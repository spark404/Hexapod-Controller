# Servo Velocity Control — Phase 2 Implementation Note

Implementation note for moving the per-joint motion profile from software
(in the servo task) onto the XL430 servos, with per-tick **synchronized**
joint timing so the foot's Cartesian trajectory is preserved between
control ticks.

This builds on Phase 1 (one-time `PROFILE_VELOCITY` / `PROFILE_ACCELERATION`
configured at boot, software interpolation removed). Phase 2 publishes a
**per-joint** profile velocity every control tick so all three joints in
a leg finish their motion simultaneously.

---

## 1. Goal

Each XL430 generates its own trapezoidal velocity profile from current to
goal position. With a single global `PROFILE_VELOCITY`:

```
joint A (Δ = 0.5 rad)  ──peak_vel──▶ ████████████████░░░░░░░  arrives at 100 ms
joint B (Δ = 0.1 rad)  ──peak_vel──▶ ████░░░░░░░░░░░░░░░░░░░  arrives at  20 ms
```

…the foot's Cartesian path is whatever joint-space interpolation falls
out — not the lift trapezoid the controller planned. Phase 2 fixes that
by scaling `PROFILE_VELOCITY` per joint so all three joints finish
together:

```
joint A (Δ = 0.5 rad)  ──vel_A──▶ ████████████████░░░░░░░  arrives at 80 ms
joint B (Δ = 0.1 rad)  ──vel_B──▶ █░░░░░░░░░░░░░░░░░░░░░░  arrives at 80 ms
```

`vel_A / vel_B = ΔA / ΔB`, both arriving at the configured tick-time
budget (e.g. 80 ms inside the 100 ms control period, leaving 20 ms of
slack for jitter / bus latency).

---

## 2. XL430 background

XL430 has two profile modes, selected by `DRIVE_MODE` bit 2:

| Bit 2 | Mode | `PROFILE_VELOCITY` means | `PROFILE_ACCELERATION` means |
|-------|------|---------------------------|------------------------------|
| 0 (default) | Velocity-based | peak velocity (0.229 rev/min units) | acceleration (214.577 rev/min² units) |
| 1 | Time-based     | total motion time (ms)            | acceleration-ramp time (ms)            |

**Velocity-based** is the existing convention and what this note targets.
Time-based is mentioned in §9 as an alternative.

Relevant control-table addresses (RAM area):

| Macro                              | Addr | Size | Notes                          |
|------------------------------------|------|------|--------------------------------|
| `XL430_CT_RAM_DRIVE_MODE`          | 10   | 1    | Set to 0 (Velocity-based)      |
| `XL430_CT_RAM_PROFILE_ACCELERATION`| 108  | 4    | Per-joint, write each tick     |
| `XL430_CT_RAM_PROFILE_VELOCITY`    | 112  | 4    | Per-joint, write each tick     |
| `XL430_CT_RAM_GOAL_POSITION`       | 116  | 4    | Per-joint, write each tick     |
| `XL430_CT_RAM_PRESENT_POSITION`    | 132  | 4    | Read each tick (unchanged)     |

Verify these against the dynamixel header in this codebase — adding any
missing macros to it is part of the work.

Both `PROFILE_VELOCITY` and `PROFILE_ACCELERATION` must be non-zero for
profile generation to engage; a value of 0 means "infinite", i.e.
immediate jump to goal (the Phase 0 behavior we're moving away from).

---

## 3. Shared state

Extend `servo_shared_state_t` in
[`Core/Inc/servos.h`](../Core/Inc/servos.h):

```c
typedef struct {
    float32_t actual_joint_angles[6][3];
    float32_t target_joint_angles[6][3];
    float32_t target_joint_velocities[6][3];   // NEW: per-joint profile velocity, rad/s
    bool initialized;
    bool request_powerdown;
    bool limit_alert_enabled;
} servo_shared_state_t;
```

Add critical-section-guarded helpers in
[`Core/Src/servos.c`](../Core/Src/servos.c), matching the existing pattern:

```c
void servo_copy_target_joint_velocities(float32_t dst[6][3],
                                        const servo_shared_state_t *state);
void servo_set_target_joint_velocities(servo_shared_state_t *state,
                                       float32_t src[6][3]);
```

If `request_powerdown` cycles through, also extend
`servo_set_actual_and_target_joint_angles` to zero the velocities (so the
servo task doesn't read stale values from the previous powered-up
session).

---

## 4. Controller side

Two small additions to `controller_update` in
[`Core/Src/controller.c`](../Core/Src/controller.c):

### 4.1 — Compute the profile time budget

Pick a target completion fraction of the control period:

```c
#define CTRL_PROFILE_BUDGET_S  0.08f   // 80 ms inside the 100 ms tick
```

This is the time the joint should take to traverse its commanded delta.
Less than `CTRL_LOOP_DT_S` so jitter / bus latency don't cause the next
target to arrive before the previous one finishes.

### 4.2 — Per-joint velocity from per-joint delta

Wherever the controller emits `next_joint_angles` (end of each WALKING /
ROTATING / STANDUP branch), also compute and store
`next_joint_velocities`:

```c
for (int j = 0; j < 3; j++) {
    float32_t delta = leg->next_joint_angles[j] - leg->actual_joint_angles[j];
    leg->next_joint_velocities[j] = fabsf(delta) / CTRL_PROFILE_BUDGET_S;
}
```

Then in the `StartControlTask` wrap-up
([`Core/Src/main.c:2395-2403`](../Core/Src/main.c:2395)), publish the new
field alongside the angles:

```c
for (int i = 0; i < 6; i++) {
    struct leg_state *s = &controller_ctx.robot.leg_state[i];
    target_joint_angles[i][0] = s->next_joint_angles[0];
    target_joint_angles[i][1] = s->next_joint_angles[1];
    target_joint_angles[i][2] = s->next_joint_angles[2];

    target_joint_velocities[i][0] = s->next_joint_velocities[0];
    target_joint_velocities[i][1] = s->next_joint_velocities[1];
    target_joint_velocities[i][2] = s->next_joint_velocities[2];
}
servo_set_target_joint_angles(&servo_shared_state,     target_joint_angles);
servo_set_target_joint_velocities(&servo_shared_state, target_joint_velocities);
```

Add `next_joint_velocities[3]` to `struct leg_state` in
[`Core/Inc/robot.h`](../Core/Inc/robot.h) alongside `next_joint_angles[3]`.

### 4.3 — Velocity floor

If a joint's delta is below the controller-level deadband, the computed
velocity is near zero. The XL430 treats `PROFILE_VELOCITY = 0` as
"infinite" — which would *defeat* the smoothing. Floor the published
velocity:

```c
#define CTRL_MIN_PROFILE_VEL_RADPS  0.05f   // sanity floor

leg->next_joint_velocities[j] = fmaxf(fabsf(delta) / CTRL_PROFILE_BUDGET_S,
                                       CTRL_MIN_PROFILE_VEL_RADPS);
```

Anything below the floor is treated as "go gently at the minimum
profile" rather than "instant teleport".

### 4.4 — Velocity ceiling

Likewise, clamp the upper end to the XL430's physical capability:

```c
#define CTRL_MAX_PROFILE_VEL_RADPS  6.0f    // ≈ XL430 no-load speed margin

leg->next_joint_velocities[j] = fminf(
    fmaxf(fabsf(delta) / CTRL_PROFILE_BUDGET_S, CTRL_MIN_PROFILE_VEL_RADPS),
    CTRL_MAX_PROFILE_VEL_RADPS);
```

XL430 no-load speed at 12 V is ~61 rpm ≈ 6.4 rad/s. Saturating at 6.0
keeps you below that with margin.

---

## 5. Servo task

The `SERVO_RUNNING` body becomes (changes from the Phase 1 simplified
version are minimal — one extra snapshot, one extra bulk write):

```c
case SERVO_RUNNING: {
    servo_copy_target_joint_angles    (target_joint_angles,     &servo_shared_state);
    servo_copy_target_joint_velocities(target_joint_velocities, &servo_shared_state);

    /* A) Read present positions (unchanged) */
    if (dynamixel_get_long_parameter_multiple(... PRESENT_POSITION ...) == DNM_OK) {
        for (int i = 0; i < 6; i++) {
            angles[i][0] = xl430_pulse_to_rad_centered(values[i][0]);
            angles[i][1] = xl430_pulse_to_rad_centered(values[i][1]);
            angles[i][2] = xl430_pulse_to_rad_centered(values[i][2]);
            compensate(angles[i], joint_angles[i]);
        }
    } /* else: existing recovery path */

    /* B) Build PROFILE_VELOCITY and GOAL_POSITION buffers */
    uint32_t vel_pulses[6][3];
    for (int i = 0; i < 6; i++) {
        uncompensate(target_joint_angles[i], cmd_angles[i]);
        values[i][0] = xl430_rad_centered_to_pulse(cmd_angles[i][0]);
        values[i][1] = xl430_rad_centered_to_pulse(cmd_angles[i][1]);
        values[i][2] = xl430_rad_centered_to_pulse(cmd_angles[i][2]);

        vel_pulses[i][0] = xl430_radps_to_profile_velocity(target_joint_velocities[i][0]);
        vel_pulses[i][1] = xl430_radps_to_profile_velocity(target_joint_velocities[i][1]);
        vel_pulses[i][2] = xl430_radps_to_profile_velocity(target_joint_velocities[i][2]);
    }

    /* C) Limit check before writing anything (existing, unchanged) */
    if (limit_alert_enabled && limit_violated(cmd_angles)) {
        state = SERVO_ERROR;
        break;
    }

    /* D) Write profile then position. ORDER MATTERS — see §7. */
    if (dynamixel_set_long_parameter_multiple(... PROFILE_VELOCITY, &vel_pulses[0][0]) != DNM_OK) {
        LOG_ERROR("[ServoTask] PROFILE_VELOCITY write failed");
    }
    if (dynamixel_set_long_parameter_multiple(... GOAL_POSITION,   &values[0][0])    != DNM_OK) {
        LOG_ERROR("[ServoTask] GOAL_POSITION write failed");
    }

    servo_set_actual_joint_angles(&servo_shared_state, joint_angles);
    break;
}
```

The deadband / velocity-clamp / accel-clamp / min-step block from
[`Core/Src/main.c:2546-2588`](../Core/Src/main.c:2546) is **removed**.
`last_velocity` and `commanded_position` arrays are no longer needed.

---

## 6. Unit conversion helper

Add to [`Core/Src/servos.c`](../Core/Src/servos.c) alongside
`xl430_rad_centered_to_pulse`:

```c
#define XL430_PROFILE_VEL_UNIT_RADPS  0.0239808f  /* 0.229 rev/min in rad/s */
#define XL430_PROFILE_ACC_UNIT_RADPS2 0.374508f   /* 214.577 rev/min² in rad/s² */

uint32_t xl430_radps_to_profile_velocity(float32_t radps) {
    float32_t units = radps / XL430_PROFILE_VEL_UNIT_RADPS;
    if (units < 1.0f)        units = 1.0f;          /* avoid 0 = "infinite" */
    if (units > 1023.0f)     units = 1023.0f;       /* XL430 max */
    return (uint32_t)(units + 0.5f);
}

uint32_t xl430_radps2_to_profile_acceleration(float32_t radps2) {
    float32_t units = radps2 / XL430_PROFILE_ACC_UNIT_RADPS2;
    if (units < 1.0f)        units = 1.0f;
    if (units > 32767.0f)    units = 32767.0f;
    return (uint32_t)(units + 0.5f);
}
```

Constants: `0.229 rev/min × 2π/60 = 0.0239808 rad/s` per unit;
`214.577 rev/min² × 2π/3600 = 0.374508 rad/s²` per unit. Cross-check
against the XL430 e-Manual for your firmware revision.

---

## 7. Configuration sequence (one-time)

In `PERIF_Dynamixel_Configure` (or a new `xl430_enable_profile_motion`):

1. **Torque off.** Required to change `DRIVE_MODE`.
2. **Set `DRIVE_MODE` = 0** for velocity-based profile (default — usually
   already correct, but make it explicit so a previous experiment doesn't
   leave it at 1).
3. **Set a default `PROFILE_ACCELERATION`** for every servo, e.g.
   corresponding to ~20 rad/s² — enough to ramp up and down within
   `CTRL_PROFILE_BUDGET_S`. This stays constant per tick; only
   `PROFILE_VELOCITY` is rewritten dynamically.
4. **Initial `PROFILE_VELOCITY`** — set to the floor value
   (`CTRL_MIN_PROFILE_VEL_RADPS` converted to units). It gets overwritten
   on the first `SERVO_RUNNING` tick anyway.
5. **Torque on.**

**Write order matters.** Every tick, write `PROFILE_VELOCITY` *before*
`GOAL_POSITION`. If you write the position first, the servo starts
generating its profile using the *old* velocity, then has to re-plan
when the new velocity arrives — visible as a brief jolt.

If the dynamixel library exposes a multi-register write (one packet
containing `PROFILE_VELOCITY` + reserved + `GOAL_POSITION`, contiguous
in the control table from 112 to 119), use that for atomicity. If not,
two back-to-back bulk writes are acceptable.

---

## 8. Validation

In rough order — each step should pass before moving to the next.

1. **Bench test, single joint.** Configure profile mode, write
   `PROFILE_VELOCITY = X`, `GOAL_POSITION` 1 rad away. Confirm the
   joint reaches the goal in approximately `X / 1 rad/s` seconds.
   Sweep `PROFILE_VELOCITY` and verify linear scaling.
2. **Single leg, all three joints, large equal delta.** Confirm all
   three joints reach their goals in roughly the same wall-clock time.
3. **Single leg, large unequal deltas.** With per-joint
   `PROFILE_VELOCITY` scaled by `|Δ| / max(|Δ|) * vel_target`, confirm
   joints arrive simultaneously. Without scaling (Phase 1 baseline),
   confirm they arrive at different times — establishes that the
   scaling is in fact what's doing the work.
4. **STANDING → STANDUP → STANDING.** No motion command, no swap.
   Verify legs move smoothly to home pose.
5. **WALKING, low velocity.** With small per-tick deltas, the new
   path should look indistinguishable from Phase 1 (deltas are inside
   the profile envelope at all times).
6. **WALKING, high velocity.** This is where Phase 2 should visibly
   beat Phase 1 — foot trajectory should be close to the planned lift
   trapezoid even on the long post-swap stance/swing legs.
7. **Direction reversal mid-cycle.** Stress test for the profile
   re-planning behavior. Foot should redirect smoothly, not snap.
8. **Joint-limit alert.** Verify the existing
   `SERVO_ERROR → torque off` path still triggers and behaves the same
   way (the limit check still runs on `cmd_angles`).

---

## 9. Pitfalls and alternatives

### `PROFILE_VELOCITY = 0` is "infinite", not "stopped"

The most likely source of silent regressions. Anywhere a delta could
round to zero — first tick of a state, joint sitting exactly at goal,
floating-point underflow — the unit conversion will produce 0 and
defeat the profile. The `CTRL_MIN_PROFILE_VEL_RADPS` floor in §4.3 is
the protection; double-check that it survives unit conversion (i.e.
its converted unit value is `≥ 1`, not `0`).

### `PROFILE_ACCELERATION` interaction

In velocity-based mode, if `PROFILE_VELOCITY / PROFILE_ACCELERATION >
total_distance`, the trapezoidal profile collapses into a triangle and
the joint never reaches `PROFILE_VELOCITY`. That's geometrically
correct — but it means the actual joint timing is shorter than your
budget assumes for tiny motions. Usually fine; if it matters, derive
`PROFILE_ACCELERATION` per tick the same way you derive
`PROFILE_VELOCITY`.

### Time-based profile (alternative)

XL430's Time-based profile (DRIVE_MODE bit 2 = 1) lets you write the
*duration* directly:

- `PROFILE_VELOCITY` (in this mode) = total motion time in ms.
- `PROFILE_ACCELERATION` (in this mode) = accel-ramp time in ms.

For Phase 2's synchronization goal, this is more natural: write
`PROFILE_VELOCITY = 80` to every joint and they all arrive in 80 ms
regardless of their individual deltas. The XL430 picks the peak velocity
internally.

Trade-off: it's a different mental model from the rest of the codebase
(everything else thinks in rad/s), and it requires a `DRIVE_MODE`
change which is sticky in the servo's EEPROM if you write the EEPROM
variant of the register. Keep it in RAM if you experiment.

### Bus bandwidth

Two bulk-writes per servo tick (PROFILE_VELOCITY + GOAL_POSITION),
each touching 18 servos. With 4 byte payload per servo and
SYNC_WRITE overhead, that's ~120 bytes per write. At 4 Mbps that's
~250 µs total per tick — comfortably inside the 20 ms servo period.

### `SYNC_FROM_HW` does not need to read PROFILE_VELOCITY

On startup, the seeded target = actual position has zero delta, so
no profile motion is needed for the first tick. The first computed
profile velocities take effect on the first `SERVO_RUNNING` tick.

### Body-pose drift is not fixed by this

Phase 2 reduces lag and therefore the magnitude of drift, but the open-loop
body-pose integration remains; see
[`controller-audit.md`](controller-audit.md) §4.3 (leg odometry, backlog T5.3).

Effort and sequencing are tracked in the backlog (T4.1, T4.2).
