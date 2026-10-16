# Hexapod Controller — Improvement Backlog

Consolidated work plan, reorganized around the controller redesign.
The leading document is
[controller-redesign-requirements.md](controller-redesign-requirements.md)
(requirements R1–R23); the desired control model is
[control-model.md](control-model.md). This backlog orders the work into tracks
and maps the surviving items from the May 2026 backlog
([archive/improvement-backlog-2026-05.md](archive/improvement-backlog-2026-05.md))
to their new home.

## Verification method (unchanged)

Use the lowest layer that proves a change, then promote:

- **Static** — review / unit test in the HexapodSim Controller project
  (`~/Projects/Personal/HexapodSim/Controller`), which links the same
  `controller.c` and friends.
- **Sim** — full simulator run (HexSpider + Controller).
- **Bench** — hardware on a stand, legs free.
- **Walk** — on-ground, joystick-driven.

---

## Track 0 — Immediate fixes to the current firmware

Small, high-leverage fixes worth landing *before* the redesign so hardware
testing in the meantime is honest and safe. None of them conflict with the
redesign.

| # | Item | Requirement | Effort |
|---|------|-------------|--------|
| 0.1 | Fix the control-task `dt_s` upper clamp (0.05 s vs the real 0.1 s period) — every motion currently runs at half speed. Also remove the unused `CTRL_LOOP_DT_S`. | R12 (D1) | XS |
| 0.2 | NaN/inf barrier + joint-limit clamp at the shared-state write (`servo_set_target_joint_angles`), so an IK failure can never slam a servo to pulse 0. | R14 (D7) | S |
| 0.3 | Clamp and slew-limit command inputs (`velocity`, `heading`, `height`) at the top of `controller_update`. | R2 | S |
| 0.4 | Servo fault back-channel: replace `SERVO_ERROR → Error_Handler()` with a `fault_code` in shared state and a recoverable hold state. | R15/R16 | M |

Verification for these is in the archived backlog (T1.1, T1.3, T1.2 recipes
still apply verbatim).

## Track 1 — Controller redesign (the leading effort)

Implement [controller-redesign-requirements.md](controller-redesign-requirements.md).
Suggested phasing; each phase is independently testable in sim:

1. **Test harness first** (R18): make the gait core host-buildable, port the
   regression scenarios, pin down current behaviour where it's worth keeping
   (world-pinned stance feet, completion pacing — requirements doc §5).
2. **Command interface & modes** (R1–R4): proportional
   `{v_forward, v_lateral, yaw_rate, height, mode}` frame, atomic transport,
   input shaping, arc + strafe modes per the control model.
3. **Gait engine** (R5–R11): explicit per-leg phase, stride/cycle derived from
   velocity and workspace (body-Z-aware), plan-once-track-by-phase swing
   trajectories, proper touchdown, settle-on-stop, bounded per-tick motion.
4. **Timing & task split** (R12–R13, R24): single dt source, move toward the
   100 Hz control / 200 Hz servo model with timestamped targets carrying
   position + per-joint velocity (executed via Track 2's hardware profiles).
5. **Safety** (R14–R16, R20, R22, R23): IK status + workspace clamp, recoverable
   faults, COM-guarded swaps, liveness counters, e-stop input.
6. **Estimation** (R17): body-frame locomotion; leg odometry + EKF as an
   isolated estimator module (can trail the rest).

Stretch: R21 (degraded five-leg gait after a leg fault).

## Track 2 — Servo-layer upgrade (independent of Track 1)

Move motion profiling onto the XL430s per
[servo-velocity-control-phase2.md](servo-velocity-control-phase2.md). This is the
implementation of **R24** (command position *and* velocity per joint):

- **Phase 1** — hardware `PROFILE_VELOCITY`/`PROFILE_ACCELERATION`, delete the
  software deadband/min-step/accel block (also removes the min-step dither and
  deadband-quantization issues from the May reviews).
- **Phase 2** — per-joint synchronized profiles so all joints in a leg finish
  together.

This complements R13 (the servo task gets simpler and faster) and can be done
before, during, or after Track 1. The validation plan in the phase-2 note stands.

## Track 3 — Post-redesign hygiene

Leftovers from the May backlog that neither Track 0 nor the redesign absorbs:

- Rename `FEMUR_OFFSET_ANGLE_RAD` → `TIBIA_OFFSET_ANGLE_RAD` in `servos.c` and
  document the compensate/uncompensate sign convention.
- Reset `last_velocity`/`last_ticks` on `SERVO_RUNNING` entry (moot once Track 2
  Phase 1 deletes the software interpolator).
- `wrap_to_pi` ±π boundary (use `remainderf`).
- Delete remaining dead fields once the redesign lands
  (`servo_shared_state.initialized`, plus whatever R19 hasn't already removed).

## Disposition of the May 2026 backlog

| Old item | Disposition |
|---|---|
| T1.1 dt clamp | **Track 0.1** |
| T1.2 fault channel + CTRL_FAULT | **Track 0.4**, completed by R15/R16 in Track 1 |
| T1.3 input clamp/slew | **Track 0.3**, subsumed by R2 |
| T1.4 STANDING height | Subsumed by R10 (redesign) |
| T1.5 liveness counters | Adopted as **R22** |
| T2.1 pace-by-stance (`fminf`) | Superseded — mechanism replaced by R5 explicit phase |
| T2.2 `rot_disp` from `tip_home` | Carried as design note in requirements §6 |
| T2.3 `wrap_to_pi` | **Track 3** |
| T2.4 servo-task resets | **Track 3** (moot after Track 2) |
| T2.5 rename offset constant | **Track 3** |
| T2.6 dead fields/constants | Subsumed by R19 |
| T2.7 re-anchor on state entry | Superseded by R7 (plan from commanded state) |
| T2.8 per-leg effective velocity | Superseded by R5 |
| T2.9 3D pacing in `calculate_motion_step` | Carried as STANDUP design note in requirements §6 |
| T2.10 foot-outside-disk detection | Subsumed by R6 (workspace enforcement) |
| T3.1 cycle-time bounds | Subsumed by R6 |
| T3.2 command slew | Subsumed by R2 |
| T3.3 swing commit point | Subsumed by R7 |
| T3.4 stance-edge swap | Subsumed by R5 |
| T3.5 servo bandwidth model | Folded into R6 |
| T4.1/T4.2 XL430 profiles | **Track 2** |
| T4.3 SERVO_ERROR refactor | **Track 0.4** / R15 |
| T5.1 body-Z reach disk | Folded into R6 |
| T5.2 IMU horizon hold | Track 1 phase 6 (R17 scope) |
| T5.3 leg odometry | Folded into R17 |
| T5.4 tripod from foot heights | Carried as STANDUP design note in requirements §6 |
| T5.5 powerdown pose check | Carried as STANDUP design note in requirements §6 |
| T5.6 e-stop | Adopted as **R23** |
| T6.1 strafing mode | Core of R1 (no longer a "later feature") |
| T6.2 COM stability check | Adopted as **R20** |
| T6.3 degraded gait | Adopted as **R21** (stretch) |
