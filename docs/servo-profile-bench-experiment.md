# Servo profile bench experiment — plan

**Status: done, run on the robot on 2026-10-12; results in section 6.** Backlog item T7.14.
The result decides how T4.1 and T4.2
([`servo-velocity-control-phase2.md`](servo-velocity-control-phase2.md)) are done.

**Where the code is.** The bench firmware, the analysis script and their tests are **not on
`main`**: they are on the branch **`bench-test`** (`Core/Src/bench_*.c`, the `BenchDry` and `Bench`
presets, `tools/bench_profile_analyze.py`, `tests/bench_script_test.c`, `tests/run_host_tests.sh`). This
document, with the findings, is on `main`. The file names and commands below refer to that branch.

## 1. What it decides

The phase 2 note assumes that a servo-side motion profile makes all joints of a leg
arrive together and keeps the motion smooth across control ticks. The review of that note
left three questions that only the hardware can answer:

1. **Chaining.** When a new goal arrives while the servo is still moving, does the motion
   stay continuous, or does the servo stop and start at every goal? At 10 Hz (and 25 Hz
   after T7.2) a stop-and-go would be visible and audible.
2. **Time-based mode.** Does time-based mode (Drive Mode bit 2) really make joints with
   different travel arrive together, for any distance?
3. **Saturation.** What does the servo do when the requested time is shorter than it can
   move in?

It also checks what the phase 2 note took from memory: the unit constants (0.229 rpm per
Profile Velocity unit, 214.577 rpm² per Profile Acceleration unit), the register addresses
and the value ranges.

**What to expect.** A simulation of a servo that plans again from where it is at every goal
(used to test the analysis) gives stop-and-go when the profile time T is at most the
control period P, borderline at T = 1.2 P, and continuous from T = 1.5 P. If the real
servo behaves like that, time-based mode needs T of about 1.5 P, which costs lag but not
bus time. The experiment tests T = 0.8, 1.2 and 1.5 P to find where the real servo
changes over.

## 2. What was built

A separate **bench firmware**, selected by a CMake option and two presets, each with its
own build directory. The normal firmware is not changed (137,120 bytes of flash and
55,488 bytes of RAM before and after, no bench symbols in it).

| Preset | Runs | Use |
|--------|------|-----|
| `BenchDry` | E1 (identification, direction check, unit check: 4 runs) and E0 (safety self test) | First, about 1.5 minutes including the countdowns |
| `Bench` | E1 to E6, 103 runs | The full experiment, estimated 16 minutes including the countdowns |

Both are 154,320 bytes of flash and 92,048 bytes (93.6 %) of RAM, because of the 29 KB
sample buffer; they link with room to spare.

- **Boot.** The bench firmware does the normal boot, including the ping of all 18 servos.
  Then the servo task, instead of the normal state machine, runs the script once and parks
  (`Core/Src/bench_profile.c`, hooked in at `SERVO_INIT` behind `#ifdef BENCH_PROFILE`).
- **Which servos.** Only leg 0 (servos 16, 17, 18) gets torque. The other 15 stay limp.
  The body must be supported by a stand.
- **Sampling.** One Sync Read of registers 120–143 (24 bytes) at 1 kHz for single-joint
  runs and 500 Hz for the three-joint runs: the servo's own millisecond tick, `Moving`,
  `Present Load`, `Present Velocity`, `Present Position`, and the profile generator's own
  `Velocity Trajectory` and `Position Trajectory`. The servo's tick gives accurate
  timestamps, free of host jitter. Samples go to a static buffer (1,600 samples) and are
  dumped as CSV lines starting with `BP,` after each run; printing during a run would
  disturb the timing. Other log output is switched off while the experiment runs.
- **Range guard.** Goals are limited to ±0.5 rad around the centre pose (the pose the leg is
  in when the experiment takes over). A run is aborted, torque off, when a joint leaves
  ±0.5 rad plus a margin, on any hardware alert, and after 5 failed bus reads in a row.
  The E0 self test checks that the guard works: it narrows the guard so that a small move
  leaves it, and the run has to abort.
- **Drive mode and profile.** Time-based mode is Drive Mode 4 (EEPROM, needs the torque
  off). The script runs all runs that do not need it first, then asks you to hold the leg
  while the torque goes off for the change. At the end, and after an abort, it restores the
  drive mode and sets Profile Velocity and Acceleration back to 0 (the normal firmware does
  not use profiles). The normal firmware also resets Drive Mode to 0 at boot
  (`PERIF_Dynamixel_Configure`). The profile registers are RAM and survive a reset of the
  board, so power-cycle the servos anyway (section 5.7).
- **LEDs.** The LEDs of the three servos of leg 0 are switched on while the experiment
  runs, to show which leg is under test (section 5.1).
- **Start pose.** The pose the leg is in when the torque comes on becomes the centre of
  the experiment; the firmware refuses a pose within 350 pulses of the end of the servo
  range, so that ±0.5 rad is always possible for the servo (section 5.2 for the leg).
- **Host tests.** `sh tests/run_host_tests.sh` runs, without hardware or the ARM toolchain:
  the bus diagnostics test, the C test of the script (103 runs; every ideal path and goal
  stays within ±0.5 rad; every run is possible for the servo, see below; the computed
  profile arrives in the time asked; the start pose check) and 26 Python tests of the
  analysis against a simulated servo, including the plots and a check that the Python
  mirror of the paths agrees with the C code.

Files: `Core/Inc/bench_script.h`, `Core/Src/bench_script.c` (the runs, no hardware),
`Core/Inc/bench_profile.h`, `Core/Src/bench_profile.c` (the runner), `CMakeLists.txt` and
`CMakePresets.json`, `tools/bench_profile_analyze.py`, `tools/test_bench_profile_analyze.py`,
`tests/bench_script_test.c`, `tests/bench_script_dump.c`, `tests/run_host_tests.sh`.

### Things the preparation caught

Writing the tests exposed design mistakes in the first version of the script, which would
have wasted bench time:
- The three-joint runs (E5) asked the tibia for 0.8 rad in 80 ms, 13 rad/s, twice what the
  servo can do (Velocity Limit 265 units = 6.35 rad/s). Now 40, 100 and 150 mrad.
- The servo-side stop and reversal (E6) were triggered 150 ms into a time-based move of
  100 ms, after it had finished. The time-based version now takes 300 ms.
- The profile duration was measured with a 1-pulse threshold, which makes short moves look
  short; it is now the time the profile's velocity is above 5 % of its peak.
- Chained runs now include T = 1.5 P (see section 1).

The C test now checks that every run is possible for the servo, except the runs of E2 and
E3 that deliberately ask for too much to see what the servo does.

## 3. The runs

| # | What | Varied | Measured |
|---|------|--------|----------|
| E1 | Identification, direction check (each joint 100 mrad), unit check | — | Model number (expect 1060), firmware, drive and operating mode, velocity limit, defaults; direction of each joint; measured peak velocity and move time against the unit constants |
| E2 | Velocity-based profile, tibia, 0.5 rad | Profile velocity {50, 100, 200, 400} × acceleration {20, 100, 500} | Arrival time against Δ/v + v/a, peak velocity, trapezoid shape; what happens above the velocity limit (400 > 265) |
| E3 | Time-based mode, tibia | T {30, 60, 100, 200} ms × ramp {T/4, T/2} × Δ {0.05, 0.2, 0.5, 1.0} rad, plus T {40, 80} | Duration against T for each Δ; where it saturates and what happens then |
| E4 | **Chained goals**, tibia | Mode (none as today; velocity-based computed; time-based); control period P {100, 40, 20} ms; T {0.8, 1.2, 1.5} × P; path (0.6 rad/s ramp, 1 Hz sine of 0.25 rad) | Velocity dips per segment (stop-and-go), tracking error and lag, velocity ripple |
| E5 | Three joints in sync (coxa, femur, tibia), 40/100/150 mrad | Time-based T 80 and 40 ms; velocity-based computed; velocity-based with one velocity for all | Spread of the arrival times |
| E6 | Servo-side stop and reversal in the middle of a move, tibia 0.8 rad | Velocity-based and time-based (T 300 ms) | Stop time and distance (the D4 option); velocity continuity at the reversal |
| E7 | Latency | taken from the step runs of E2 and E3 | Goal written to first movement |

For the velocity-based computed variant, each goal gets `v = Δ / (T − Tr)` and
`a = v / Tr` (the corrected formula from the review). The phase 2 note's `v = Δ / T`
ignores the ramps and makes the joint arrive late.

## 4. Decision criteria (proposed)

- **Time-based mode** wins if:
  - the duration is within ±10 % of T for every move the servo can make in that time (E3),
  - in E4 at P = 40 ms the velocity never dips below about 50 % of the segment mean (ramp)
    and the ripple stays below 0.3 (sine), and
  - the tracking error is no worse than with no profile.
- **Velocity-based with computed v and a** wins if time-based stops and starts or
  saturates badly but velocity-based chains smoothly.
- **Keep the software interpolation** at 50 Hz if neither chains smoothly. In that case
  synchronise the joints in software (scale the per-joint steps so all finish together)
  and use the servo profile only as a speed limit.

The analysis script applies these and prints the recommendation. The result changes
T4.1/T4.2 as follows: time-based mode needs one profile configuration at boot and only
`GOAL_POSITION` per control tick (the 0.8 ms write that is done today); velocity-based
needs an 8- or 12-byte write per tick (about 1.2–1.5 ms) with per-joint values.

## 5. Preparing the robot and running it

### 5.1 The leg under test

The experiment uses **leg 0: servos 16 (coxa), 17 (femur) and 18 (tibia)**
(`Core/Inc/robot_config.h`). The firmware **switches on the LEDs of these three servos**
as soon as it takes over, so the leg is recognisable at a glance, and it prints
`BP,NOTE,testing leg 0, servos 16 17 18: their LEDs are on`. The LEDs stay on until the run
is finished (`BP,DONE,OK`); after an abort they stay on, so you can see which leg was being
tested. The normal firmware switches them off again when it starts.

Check on the robot that the leg with the LEDs on is the one you prepared. The first
countdown is 30 s for this. If it is the wrong leg, switch the power off.

### 5.2 Mechanical set-up

1. **Robot on a stand with the body lifted**, so that **no leg carries weight or touches
   anything**: the other 15 servos are limp, and leg 0 must hold only its own weight. The
   body must not rest on leg 0.
2. **Choose the start pose of leg 0 and hold it there during the first countdown.** The
   experiment moves every joint up to **±0.5 rad (±28.6°) from this pose**, in both
   directions, one joint at a time (the three-joint runs use at most ±0.075 rad). So the
   start pose has to leave 0.5 rad free on both sides for each joint:
   - coxa: the leg swings sideways ±29° without touching a neighbouring leg or the body;
   - femur: ±29° up and down;
   - tibia: ±29°, the largest moves are 1.0 rad from end to end (from −0.5 to +0.5 rad).
   A pose near the middle of each joint's range works. Check it by hand with the torque
   off (the robot is off, or in the countdown before the torque goes on): move each joint
   0.5 rad each way from the pose, with the other two joints in the pose.
3. **Keep the neighbouring legs clear of the sweep.** They are limp; a leg that is hit is
   pushed away, and a leg that hangs in the way is a collision. Support or tie them back if
   they hang into the area of leg 0.
4. The firmware refuses a start pose within 350 pulses (about 30°) of the end of a servo's
   range (0 to 4095) and aborts with `ABORT:start pose of joint N near range end`.

### 5.3 Electrical and safety

- Battery charged or a bench supply that gives the servo voltage. Only one leg moves, so
  the current is low.
- **A hand on the power switch** for the dry run and the first minutes of the full run.
  The firmware stops itself on a hardware alert, on a joint leaving the range and on
  repeated bus errors, but the power switch is the real emergency stop.
- The firmware drops the torque of leg 0 when it aborts and after the last run. **The leg
  then goes limp: support it** when you see `BP,NOTE,...LEG 0 GOES LIMP` or `BP,DONE`.
- The other tasks keep running but their console output is switched off during the
  experiment, so the console shows only `BP,` lines. A Pi gamepad service that sends
  commands over SPI does no harm, but it is better off.

### 5.4 Flashing and capturing (the Pi as before)

The images are built: `build/BenchDry/Hexapod.hex` and `build/Bench/Hexapod.hex`. They
are separate from `~/Hexapod.hex`, which is the normal firmware: copy them under their own
names so that nothing is overwritten.

    # on the Mac
    scp build/BenchDry/Hexapod.hex hexapod-controller.local:Hexapod-bench-dry.hex
    scp build/Bench/Hexapod.hex    hexapod-controller.local:Hexapod-bench.hex

    # on the Pi, terminal 1: start the capture first, the experiment starts at once after the flash
    tio -b 115200 -l --log-file bench-dry.log /dev/ttyS0

    # on the Pi, terminal 2: flash (as in flash-capture.txt), the application starts by itself
    cd ~/Documents/Projects/Stm32Loader
    ./target/debug/Stm32Loader --type SPI --port spidev0.0 write ~/Hexapod-bench-dry.hex --erase --verify --go

Use a log file per session (`bench-dry.log`, `bench-full.log`) so that the analysis does
not have to pick between sessions. The experiment does not start until the countdown ends:
the first lines are the identification of the servos, then the countdown.

### 5.5 Dry run (BenchDry, about 1.5 minutes including the countdowns)

On the console:
- `BP,BEGIN,65,5,...`, the identification lines, and the note that the LEDs of 16, 17, 18
  are on;
- `HOLD LEG 0 in a neutral pose with a hand: the torque goes off`, with a 30 s countdown.
  **Hold the leg in the start pose.** Then the torque goes off, the pose is captured, the
  torque comes on and `torque is on, the leg holds this pose: RELEASE IT NOW` appears.
  Let go within 10 s;
- four small runs (100 mrad on each joint, then 0.5 rad on the tibia), each dumped as
  `BP,RUN` / `BP,START` / `BP,G` / `BP,S` lines ending in `BP,END,<id>,<samples>,OK`;
- the safety self test: `BP,NOTE,self test: ...`, then `BP,END,5,...,ABORT:out of range`
  and `BP,DONE,ABORT`. **This abort is the expected end of the dry run.** The torque is
  off afterwards: support the leg.

Then, on the Mac (copy the log: `scp hexapod-controller.local:bench-dry.log .`):

    python3 tools/bench_profile_analyze.py bench-dry.log

and check:
- the model number is 1060 and the three drive modes are 0;
- the direction check says `ok` for all three joints. If one says `NOT AS COMMANDED`, the
  direction or the identifier is not what the script assumes: **do not run the full
  experiment**, tell me;
- the unit checks say `confirmed` (or give the implied unit);
- the self test says `PASS`.

### 5.6 Full run (Bench, about 16 minutes including the countdowns)

Flash `Hexapod-bench.hex` **with `--erase --verify`** and capture to `bench-full.log` the same way.
- Start as in the dry run: hold the leg for the first countdown (30 s), release when asked.
- After the 45 runs that do not use time-based mode (about 7 minutes in) the script asks you to
  support the leg: `SUPPORT LEG 0: the torque goes off in 10 s`. Then **the torque is off and the leg is
  limp: hold it in the pose of the start** (the same pose as at the beginning, within 2°). The console
  prints how far each joint is from that pose and asks again every 10 s until it is close enough (it
  aborts after 4 tries). Then the drive mode is written, the torque comes on, and you release the leg.
  (In the first full run the torque only went off for a moment after the countdown, so the leg could not
  be put back: see section 6.3.)
- At the end: `done: LEG 0 GOES LIMP, hold it` (15 s), then `BP,DONE,OK`, and the LEDs
  go off.
- An abort prints `BP,END,<id>,<n>,ABORT:<reason>` and `BP,DONE,ABORT` and drops the
  torque. The data of the aborted run is still dumped. Reset the board to start over.

### 5.7 Afterwards

**Switch the robot's power off and on (the servos) before running the normal firmware.**
The harness restores the drive mode and the profile registers at the end and after an
abort, but if a run was interrupted by a reset or a power problem, the servos keep what was
written until they lose power. Then flash the normal firmware again (`./update.sh` on the
Pi; it resets Drive Mode to 0 at boot anyway). Then:

    scp hexapod-controller.local:bench-full.log .
    python3 tools/bench_profile_analyze.py bench-full.log                       # the report
    python3 tools/bench_profile_analyze.py bench-full.log --csv metrics.csv
    python3 tools/bench_profile_analyze.py bench-full.log --run 17              # details of a run
    tools/.venv/bin/python tools/bench_profile_analyze.py bench-full.log --plots plots/

The plots (matplotlib is installed in `tools/.venv`, versions in `tools/requirements.txt`)
are one PNG per run (position with the profile and the ideal path, velocity, load), plus
`e3_profile_time.png`, `e5_sync.png` and `e4_<path>_P<period>.png` with the modes on top of
each other. Fill in section 6.

## 6. Results

### 6.1 First dry run, 2026-10-12 (two sessions, same result)

Runs 1 (coxa) and 2 (femur) completed; run 3 (tibia) aborted with `cannot reach the start
pose`. Flashed: the first version of the bench firmware. What the log shows:

- **The servos are not all the same model.** Servos 16 (coxa) and 17 (femur) are
  `2XL430-W250` (model 1090, firmware 49), servo 18 (tibia) is an `XL430-W250` (model 1060,
  firmware 50). Velocity limit 250 against 265 units (the harness uses the lowest). Return
  delay 10 (20 µs), baud 4 (2 Mbps), operating mode 3, drive mode 0, supply 11.0 V,
  temperatures 28–32 °C.
- **Directions and units are right.** Coxa and femur moved the way they were commanded. The
  profile peaks match the trapezoid and triangle formulas (0.86 rad/s against 0.87 and 0.88
  predicted), which confirms the acceleration unit (214.577 rpm²).
- **A loaded joint does not follow its profile.** The femur, carrying its leg, starts to
  move about 100 ms after its profile does, peaks at 0.53 rad/s against the profile's 0.86,
  and **stops 21 pulses (32 mrad) short of the goal at a load of 12.3 %** (the coxa, with no
  gravity load, ends within 1 pulse). The profile itself finishes at the goal (`moving` 0).
  The position controller has no integral action, so the shortfall follows the load. This
  limits what any profile mode can do for a loaded joint (backlog T7.15).
- **Why run 3 aborted.** The start pose move required the position to be within 3 pulses
  of the goal, which a loaded joint can never meet. Fixed: arrival is now judged on the
  profile (trajectory at the goal, no movement), and only a position more than 80 pulses
  from the profile counts as blocked. The firmware also prints, on a failure, the position,
  profile and goal of each joint.
- **The experiment left its profile in the servos.** At the start of the first session
  Profile Velocity and Acceleration were 0 and 0; at the start of the second they were 50
  and 20, written by the first session. These registers are RAM, and the servos stay
  powered when the board is reset, so they survive. The normal firmware does not write
  them, so it would have run with a velocity limit of 1.2 rad/s. Fixed: the harness now
  restores drive mode and profile (0, 0) at the end and after an abort. **Power-cycle the
  servos (switch the robot's power off and on) before running the normal firmware again;
  the servos of leg 0 currently hold velocity 50 and acceleration 20.**
- The identification now also lists the position controller gains, to interpret the lag.

### 6.2 Second dry run, 2026-10-12: passed

All five runs as designed: runs 1 to 4 completed, then the self test aborted as it has to.
The servos had been power-cycled (profile registers 0 and 0 at the start).

| Check | Result |
|-------|--------|
| Direction of coxa, femur, tibia | all `ok` (moved +64/+66, +46/+68, +59/+59 pulses; the femur falls short, see below) |
| Velocity unit (0.229 rpm) | confirmed: the 0.5 rad tibia profile peaks at 2.40 rad/s, as asked (ratio 1.00) |
| Acceleration unit (214.577 rpm²) | confirmed by the three short profiles (ratios 0.99, 0.98, 1.05) |
| Profile duration against the trapezoid | 0.95, 0.94, 1.00, 0.98 of the formula (the 5 % velocity threshold costs about 5 % on short ones) |
| E0 self test | `PASS`: the guard stopped the run after 107 samples and the torque went off |

Position controller gains read from the servos (nothing was changed in them):

| | servo 16, 17 (2XL430) | servo 18 (XL430) |
|---|---|---|
| Position P / I / D | 640 / 0 / 2000 | 640 / 0 / 3600 |
| Velocity P / I | 100 / 1800 | 100 / 1000 |
| Feedforward 1st / 2nd | 0 / 0 | 0 / 0 |

Findings for the design:
- **The servo follows its profile about 43 ms late.** The tibia (run 4, 0.5 rad) starts to
  move 72 ms after the goal is written (its profile starts after 10 ms); its position reaches
  half-way a constant 43 to 44 ms after the profile does (eight moves in the full run), with 97 % of the peak
  velocity (2.33 against 2.40 rad/s). The coxa and the femur show the same delay at lower peak
  velocity (0.62 and 0.50 against 0.86 rad/s). With the feedforward gains at 0 this is the
  response of the position loop. A lag longer than the 40 ms control period at 25 Hz will
  dominate the chained runs (E4), where the analysis reports it as the lag of the best fit.
- **Position error follows the load, with the same factor on both joints:** about −1.7
  pulses per % of load (femur −22 pulses at 12.9 %, tibia +10 pulses at −5.7 %). The
  difference between the position and the profile is therefore a second measure of the
  load (backlog T7.6, T7.15). The coxa has no gravity load and stays within 2 pulses.
- **Dynamic load.** The tibia load peaks at 21 % during the acceleration of the 0.5 rad
  move and is 0 at rest: the load includes the inertia, so contact detection has to be
  done at low speed or against a speed-dependent baseline.

Not tested on the hardware yet: the second drive mode group. The dry run only has runs
in group 0. The switch to time-based mode (hold the leg, Drive Mode 4 written to the
EEPROM, profile in ms) happens for the first time about 7 minutes into the full run. If
something is wrong there, the run aborts safely (torque off, drive mode and profile
restored).

### 6.3 Full run, 2026-10-12: all 103 runs completed

Log `bench-full3.log`, first successful run. (Two earlier attempts ran only E1, 4 runs, because the full-run
image was written over the dry-run image without an erase: the two differ in the experiment mask, 65 and 63,
and flash can only clear bits, so the board saw 65 AND 63 = 1. Flash with `--erase --verify`.)

**Units and the profile registers (E1, E2).** All 16 velocity-based profiles match the trapezoid formula
(duration 0.94–1.03, peak 0.99–1.05). The velocity unit (0.229 rpm) and the acceleration unit (214.577 rpm²)
are confirmed. **The Velocity Limit register does not cap the profile velocity** in position mode: a profile
of 400 units ran at 9.5 rad/s with a limit of 250. The highest speed a joint actually reached was **5.9 rad/s
(tibia)**; a faster profile is followed late, not faster. The coxa and femur were only run slowly.

**Time-based mode (E3).** The profile takes T for every distance, 0.04 to 1.0 rad and T 30 to 200 ms, within
about 5 % (29, 57, 95, 195 ms for T = 30, 60, 100, 200), also for moves no motor can make (0.99 rad in 30 ms:
the profile runs at 43 rad/s). The joint then follows late: arrival 170 to 260 ms for those. For moves it can
make, the position arrives about 30 to 70 ms after T.

**Chained goals (E4), tibia, 42 runs.** The real servo is smoother than the simple model used to test the
analysis: **no mode stops and goes**, not even at T = 0.8 P. Tracking error at the best lag, relative to no
profile:

| Path, P | velocity-computed | time-based | no profile |
|---------|-------------------|------------|------------|
| ramp 100 ms | ×1.07, ripple 0.18 | ×1.20, ripple 0.10 | 4.9 mrad, ripple 0.24 |
| ramp 40 ms | ×0.95, ripple 0.06 | ×1.40, ripple 0.06 | 4.0 mrad, ripple 0.07 |
| ramp 20 ms | ×0.97, ripple 0.03 | ×1.47, ripple 0.04 | 3.8 mrad, ripple 0.03 |
| sine 100 ms | ×0.77 | ×0.99 | 11.2 mrad |
| sine 40 ms | ×0.67 | ×1.02 | 7.5 mrad |
| sine 20 ms | ×0.74 | ×1.01 | 7.1 mrad |

The lag of the best fit is 35–45 ms without a profile or with the velocity-computed one, 52–66 ms time-based.
At **25 Hz or faster the modes cannot be told apart in the plots** (`e4_sine_P40.png`): the servo's own
response, about 50 ms behind the ideal path, dominates. At today's 10 Hz the profiles even out the speed
ripple (0.24 without, 0.10–0.18 with), and the joint never stops between goals in any mode.

**Three joints together (E5).** The arrival times of coxa, femur and tibia differ by **52–85 ms in every
mode**, time-based included (T = 80: 151, 135, 187 ms). The profile makes the servos' internal references
arrive together, but the joints follow them differently: the coxa (2XL430) is the slowest although it travels
least. A profile does not synchronise the joints; the servo dynamics have to (backlog T7.15).

**Servo-side stop and reversal (E6).** Setting the goal to the present position with a hard profile, while a
joint moves at 2.65 rad/s, does **not** stop it at once: its velocity reaches zero 79 to 89 ms after the
command, and it **runs on 149 mrad** past the command point with the velocity-based stop (speed 400 units,
acceleration 500) and **55 mrad** with the time-based one (20 ms, ramps of 5 ms), then comes back to the goal
(it settles within 7 pulses of it). *(An earlier version of this section said 5–6 mrad: that was the distance
between the end point and the goal, which are the same point by construction.)* The run-on grows with the
square of the speed, so at the 0.1 to 0.4 rad/s of a slow foot descent it would be a few mrad at most
(extrapolated, not measured). A reversal in the middle of a move is smooth (largest step in velocity
0.07 rad/s).

**Latency (E7).** Goal written to the start of the profile: 3.5 ms time-based, 7 ms velocity-based (gentler
acceleration); to the start of the position moving: 30 and 51 ms.

**Position error under load.** Tibia −1.76 pulses per % of load over 100 rest points, femur about −1.7: the same
factor, so position minus profile is a second measure of the load (T7.6, T7.15). Coxa 0.

**What it means** (the analysis script's rule gives *velocity-based with computed v and a*: it passes all three
criteria; time-based passes the first two and fails the tracking error):
1. **Smoothness does not need a profile at 25 Hz.** The control rate of T7.2 gives it; a profile adds
   little (−5 to −33 % tracking error with the velocity-computed variant, none for time-based).
2. **The synchronisation motive for T4.2 does not hold.** No mode brings the joints in together.
3. If profiles are used, **velocity-based with `v = Δ/(T−Tr)`, `a = v/Tr` per goal**, written together with
   the goal (12 bytes at 108, about 1.5 ms per write), at the control rate. Time-based needs the EEPROM
   drive mode, has more lag and no tracking gain: **not recommended.**
4. **The servo-side stop is available but not instant** (D4): from 2.65 rad/s it takes about 80 ms and runs on
   55 to 149 mrad (time-based stop the shorter); at slow final-approach speeds it would be small
   (extrapolated), so it suits a slow descent, not a fast stride.
5. The larger gain is probably in the servo itself: the lag of 40–60 ms and the shortfall under load come
   from the controller gains (feedforward 0, no integral action): **T7.15 first.**
6. Clamp profile velocities in the firmware (about 5.9 rad/s on the tibia): the register does not.

**Caveat found in the run.** At the switch to the time-based group the torque only went off briefly (to write the
EEPROM) after the 45 s countdown, so the leg stayed stiff while it was held and could not be moved back to the
pose of the start: the second half (all 58 time-based runs) was centred on the pose the previous run left the
tibia in, 255 pulses (0.39 rad) lower. The loading was nearly the same (tibia at rest −4.5 % against −5.1 %, 0 %
at the end of runs in both), so the comparison is not badly distorted, but a small effect cannot be excluded.
Fixed in the harness: the torque now goes off after a warning, at the start of the hold; the leg has to be back
within 30 pulses of the pose of the start before the drive mode is written; the centre of the start is kept. A
control run with no profile in the second half would settle the point, if the time-based result matters.

## 7. Open points

1. Leg 0 can move ±0.5 rad if it starts from a suitable pose (confirmed). Another leg is a
   change of the servo indices in `bench_profile_run`'s caller (one line in `main.c`).
2. The time-based runs write Drive Mode (EEPROM) on three servos and restore it. EEPROM
   endurance is not a concern at this volume.
3. Plots: matplotlib is installed in the venv `tools/.venv` (not in the system Python, which
   Homebrew manages); the analysis prints text tables without it.
4. The thresholds in section 4 are proposals.
5. The 29 KB sample buffer leaves about 6 KB of static RAM free in the bench builds. That
   is enough (the normal firmware is not affected), but adding more to the bench builds
   needs the buffer to shrink.
