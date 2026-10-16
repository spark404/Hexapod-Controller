# Reference frames in the controller: as built, and the way to the planned frames

Analysis of how `controller.c` uses coordinate frames today, written as input for the
refactor to the frames of [`design.md`](design.md) §4: **world, level, body, leg (hip)**,
plus the joint/servo space below them. Read from the code on `main` at `c677613`
(2026-10-16); nothing was run for this analysis. Line numbers refer to
`Core/Src/controller.c` at that commit.

The simulator keeps a second copy of `controller.c`
(`HexapodSim/Controller/controller.c`). Since the sync of 2026-10-16 the only difference is
the source of the current foot position in `leg_current_position_body()` (commanded angles in
the simulator, measured angles in the firmware); everything in this document holds for both.

## 1. Summary

- The code has a chain `world ← hexapod ← body ← coxa`, built from `struct pose` values and
  4×4 matrices. The *body* frame, as it is used, is the design's **level** frame; there is no
  separate level frame, and no frame that tilts.
- Attitude is not used anywhere. The controller receives roll, pitch and yaw from the caller
  and discards them; the caller passes zeros. Yaw is dead-reckoned from the commanded yaw rate.
- All planning (swing paths, standup targets) is done in the body frame with the ground
  assumed flat and parallel to the body. Stance feet are anchored in the world frame, but the
  anchors are commanded positions, not measurements.
- The leg frame (coxa) and the joint space are in good shape; the joint/servo frame needs a
  name and an explicit conversion for the load and velocity readings (T7.4, T7.6).

## 2. The frames today

| Frame | In the code | Defined by | Holds |
|-------|-------------|-----------|-------|
| **World** | implicit | origin and yaw are those of the `hexapod` pose at `controller_init()` (boot), not reset at standup | `tip_world_coordinates` (stance anchors), the command `heading` |
| **Hexapod** | `robot.hexapod`, a `struct pose` | pose of a point on the ground under the body centre, in world: `(x, y, 0, 0, 0, yaw)`; x, y and yaw are integrated open loop from the commanded speed and yaw rate (lines 383–386, 584) | position and heading of the robot |
| **Body** | `robot.body`, a `struct pose` | pose of the body centre in hexapod: `(0, 0, height, roll, pitch, 0)`; roll and pitch are never set | height; the frame all paths are planned in |
| **Coxa (hip)** | `leg_state.coxa_mat` (and `coxa_mat_inv`) | pose of the coxa axis in body: the mount point from polar coordinates, yaw = mount angle (+x points radially outward) | the frame of forward and inverse kinematics |
| **Joint / servo** | `servos.c`: `compensate()`, `uncompensate()` | joint angles after the femur sign flip and the tibia offset; servo pulses are centred on 2048 | goal positions; load and velocity readings (T7.4) |

Conventions: `pose_get_transformation()` builds `T = Tr · Rz · Ry · Rx` from
`rotation = (roll, pitch, yaw)`. The composite `Thexapod_body = Thexapod · Tbody` is
world←body. Per leg, `T = Thexapod_body · coxa_mat` is world←coxa and its inverse is the
world→coxa transform used for IK.

## 3. How each state uses them

**STANDUP** (lines 289–343): body and coxa frames only. The target is
`(tip_home x, tip_home y, −CTRL_BODY_Z)` in body. The current position comes from the *last
commanded* angles (`next_joint_angles`, FK in coxa, then `coxa_mat` to body), not from the
measured ones (since `f1bbf00`). The height is the constant `CTRL_BODY_Z`, not the commanded
one. `body.translation[2]` is set to `CTRL_BODY_Z` when the legs are close enough.

**STANDING**: no motion code. A height command only writes `body.translation[2]`
(line 185); the legs do not move until WALKING starts.

**WALKING** (lines 344–571):
1. `yaw_error = wrap(heading − yaw)`; the rotation step is that error divided by `dt`, clamped
   to `CTRL_MAX_YAW_RATE`.
2. The speed is rotated into world with `yaw` and straight back into body with `−yaw`
   (lines 354–363). That is an identity: the movement vector is always `(speed·dt, 0)` along
   body x. `heading` only influences the yaw error.
3. The hexapod pose is advanced in world (`dx_world`, `dy_world`, `yaw += rotation_step`) and
   `Thexapod_body` is rebuilt.
4. Per leg, in **body**: the current position (FK of the *measured* angles, then `coxa_mat`);
   the rotation displacement `rot_disp = (ẑ × r) · rotation_step`, clamped; and for a swing
   leg a path to a landing point on a circle of radius `CTRL_LEG_RADIUS_MM` around `tip_home`,
   in the direction of travel, at `z = −body_z`. The swing path is computed once at lift-off
   and stored in the leg state, in the body frame (`9d57934`), and advanced along its arc
   length (`e804445`).
5. Per leg, to joint angles: a swing leg's body-frame point goes body→world with the *current*
   `Thexapod_body` (and is stored as the new `tip_world_coordinates`), then world→coxa with
   the inverse of `Thexapod_body · coxa_mat`. A grounded leg ignores its planned path: its
   target is simply the stored world anchor, mapped world→coxa. Both end in `inverse_kinematics`
   in the coxa frame with origin `(0, 0, 0)`.
6. When the swing legs are within `CTRL_CLOSE_THRESH` of their landing point the groups swap.

**ROTATING** (lines 572–717): the same structure, with the rotation displacement only. It does
**not** use the stored swing trajectory: the swing path is regenerated on every tick from the
measured foot position, the swing legs are paced with `CTRL_DEFAULT_VEL` instead of the foot
velocity, and the landing direction follows the instantaneous foot position.

**SYNCING, POWERDOWN**: no frame work (SYNCING copies the measured angles to the commanded
ones).

## 4. Findings

Each finding says what it means for the refactor.

1. **No level frame; the body frame plays its part.** The body frame has its origin at the body
   centre and takes no tilt, which is the design's level frame. The hexapod frame (origin on
   the ground) is a duplicate that the design does not have: there the height is part of the
   world←level translation. *Refactor:* name the frame that exists, move planning and the
   command into it explicitly, and add the level→body rotation (§4.2 of the design) as a new
   frame that tilts.

2. **Attitude is dead.** `controller_update(…, attitude, …)` starts with `(void) attitude`;
   `main.c:2700` passes zeros and never updates them. The EKF output (`ekf_out`) goes only to the
   log. The yaw the controller steers on is its own integral of the commanded rate, which drifts
   with servo lag and slip, and it is also what the SPI status reports as yaw. *Refactor:* the
   level frame's yaw and the body frame's roll and pitch come from the EKF (backlog T5.2); decide
   how the dead-reckoned and the measured yaw are combined.

3. **Flat ground, open loop.** Swing targets use `z = −body_z` in the body frame: a flat ground
   parallel to the body. Touchdown is path completion, with no contact, and the world anchors are
   commanded positions. *Refactor:* swing apex and touchdown in the level frame (gravity
   aligned), touchdown by contact (T7.6, T7.9), anchors from the measured landing position.

4. **The world frame starts at boot and has no z.** The `hexapod` pose is zeroed in
   `controller_init()` only, so the world origin and X axis are those at boot, and the pose
   keeps accumulating through power down and standup. The design defines the world at standup
   with `Z = 0` the ground plane. *Refactor:* reset the world pose when STANDUP starts and give
   the world pose a z (the body height over the ground).

5. **The command is mixed between frames.** `heading` is a world angle, `velocity` is a speed
   along the robot's own x axis (the round trip in WALKING step 2 is an identity). *Refactor:*
   the twist `(vx, vy, wz)` in the level frame replaces both; the `v_world`/`v_body` code goes.

6. **Swing paths are stored in body, evaluated against a moving body.** That is the same as
   level while there is no tilt. With tilt it would put the apex and the landing point in a
   tilted plane. *Refactor:* store the swing path in level.

7. **Dead work for grounded legs.** The path computed for a grounded leg is discarded (the world
   anchor overrides it), but its length still enters `longest_path`, which sets the pacing
   (`substeps`) of the swing legs. The pacing therefore depends on the distance from a foot to
   a target it is not moving to. *Refactor:* pace the swing from the swing legs only, from the
   foot velocity `u_i` (design §5).

8. **`tip_home` is computed through the world transform.** It is documented as "in the body
   frame" but computed as `Thexapod_body · coxa_mat · FK(home angles)` (lines 60–65). That is
   correct only because the pose is identity at init, and the z includes the initial body offset
   of 150 mm (`controller_init` sets 150, `CTRL_BODY_Z` is 100). Only x and y are used, so
   nothing breaks. *Refactor:* compute it in body (or level) directly: `coxa_mat · FK(home)`.

9. **Height is handled inconsistently.** STANDUP uses the constant `CTRL_BODY_Z`; STANDING
   stores a new height without moving the legs; WALKING jumps to the stored height on its first
   tick (`body_z` changes the target z at once). *Refactor:* the body height is a pose
   component with a rate limit, applied in every state that stands (see also T1.4).

10. **ROTATING differs from WALKING in the swing handling** (§3 above). It is the state with the
    known fatal joint limit alert (leg 0, coxa, ±0.79 rad). The cause has not been traced; the
    difference in how the swing path and its pacing are computed is a lead. In the refactor
    rotation is not a state: it falls out of the twist (design §5.1).

11. **Cost.** Six generic 4×4 inversions per tick (`matrix_3d_invert` on
    `Thexapod_body · coxa_mat`, lines 506–507 and 678–679) and up to three rebuilds of
    `Thexapod`, `Tbody` and their product per tick. The transforms are rigid, so the inverse is a
    transposed rotation plus a translation. *Refactor:* build the chain once per tick and
    invert analytically.

12. **The world detour for swing legs is an identity.** For a swing leg
    body→world→coxa equals body→coxa (`Tinv · Thexapod_body = coxa_mat_inv`); the world position
    is only needed to store the anchor for the landing. For grounded legs the world step is the
    point. *Refactor:* keep the anchor logic in world, plan swing legs without the detour.

13. **The joint/servo frame is implicit.** `compensate()` flips the femur sign and adds a
    tibia offset; the offset constant is called `FEMUR_OFFSET_ANGLE_RAD` (T2.5). Load and
    velocity readings from the servos (T7.4) are in servo frame, so the sign of the femur load
    is the opposite of the kinematic joint's. Contact detection (T7.6) has to convert. *Refactor:*
    one place that converts between joint and servo frame, for goals and for readings.

14. **Yaw is never wrapped in the state** (`hexapod.rotation[2] += …`), only inside the error
    (`wrap_to_pi`). Harmless, but a long run accumulates; the wrap helper itself is the
    `fmodf` version (T2.3).

## 5. Mapping to the planned frames

| Planned frame | Today | What changes |
|---------------|-------|--------------|
| **World** | `hexapod` pose in world, boot origin, no z | defined at standup, ground plane `Z = 0`, z = body height, updated from odometry (T5.3); holds the stance anchors |
| **Level** | the body frame, by accident (yaw only, origin at body centre) | explicit; holds the command twist, the reach disks (`tip_home`), swing apex and touchdown, and the stability check |
| **Body** | the same frame as level, tilt unused | level→body rotation from EKF roll and pitch plus the commanded tilt; the frame the hips are fixed in |
| **Leg (hip)** | `coxa_mat` | unchanged; cache the inverse, document that +x is radial |
| **Joint / servo** | `compensate()` / `uncompensate()` | name it, fix the misnamed offset (T2.5), use it for the readings too |

A chain per tick, in the design's order: read attitude and joint angles; feet in hip →
body (`coxa_mat`) → level (attitude rotation); command twist in level gives each foot's
velocity; stance feet hold their world anchor and go world → level → body → hip; swing feet are
planned in level; IK in hip; `compensate` to servo.

## 6. Decisions the refactor has to make

- **Where the level origin is.** The design puts it at the body centre (same as body). The code's
  hexapod frame has it on the ground. Putting it at the body centre makes level→body a pure
  rotation, as the design says; the ground plane then lives in the world frame only.
- **Where the height lives.** In the world←level translation (design) rather than in a separate
  body pose; STANDING and WALKING then share one code path for it.
- **Which yaw is authoritative.** Measured (EKF, with its drift) or integrated from the commanded
  rate (smooth, drifting): the stance anchors need a yaw that is consistent over a swing cycle,
  the heading command needs one that matches the world.
- **Planning source for the current foot position.** The firmware's WALKING and ROTATING use the
  measured angles, STANDUP and the simulator use the commanded ones. With the hardware motion
  profile (T4.1) and the lag measured in the bench experiment (T7.14, T7.15) the commanded
  position is the better planning source for swing and standup, and the measured one is the input
  for contact and slip.
- **Whether rotation stays a state.** The twist model removes it (design §5.1); the fault above is
  then moot, but it is worth reproducing in the simulator first (the ROTATING code is identical
  there, `tests/controller_equivalence` keeps the two copies in step).

## 7. Related

[`design.md`](design.md) §4 (frames) and §5 (motion model), [`controller-audit.md`](controller-audit.md)
§3.4 D10 (attitude discarded), backlog T5.2 (level frame / horizon hold), T5.3 (odometry), T7.6
(contact), T7.8 (twist model), T2.5 (offset rename), T1.4 (height in STANDING).
