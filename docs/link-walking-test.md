# Walking test with the bus diagnostics

A saved test for later: does the servo bus stay clean while the robot walks with the SPI link
running? It is the open item of backlog T7.5 ("walking test with the bus diagnostics") and the
follow-up of the interrupt priority change `327a4d3`. Nothing here has been run on the robot yet; the
tools were checked against the logs captured on 2026-10-16.

## What it answers

1. Do the servo reads still fail while walking? Before the priority change the 30 s bus
   diagnostics showed 5 and 2 failed reads (of 1,500) in windows with the link running, always on servo 0
   (the first to reply). After it, three standing windows had 0 failed reads, and the boot window 1.
   Standing is not walking: the servo bus carries the same traffic, but the control task and the
   profile writes are busier and the legs load the servos.
2. Does the SPI link stay clean while walking: no bursts of damaged replies, no rejected commands?
3. Does walking, stopping and the walk-to-stand transition run without a fault?

## The test

`tools/link_walk_test.sh` runs on the Pi and drives the robot with the fake Poom of `poom-remote`
(`--source stdin`, no Bluetooth). Defaults, all settable through environment variables:

| Step | Default | Why |
|------|---------|-----|
| wake | hold `up` until the state is `walking`, release | a robot in power down stands up first; a few centimetres of walking |
| settle | 6 s | |
| standing baseline | 90 s (3 windows) | the bus numbers without walking, link running; under the 120 s power down timer |
| walks | 9 × (3 s walking, 7 s standing) | 27 s of walking, about 1.35 m forward at 50 mm/s; each 30 s window gets 6 to 12 s of walking |
| standing after | 35 s | one closing window |

About 4 minutes. Before every step it checks that the robot answers over the link and that the console
shows no fatal error; if not, it releases the buttons, ends the daemon and stops (`ABORT: ...` in
`marks.txt`).

The test never turns the robot: the ROTATING state has a fault (backlog T7.5, T7.8).

## Before you start

- The firmware is running, the battery is attached, the legs have room: **1.35 m ahead**, or the robot hangs free
  (lower `CYCLES` or `WALK_S` for less: `CYCLES=4 tools/link_walk_test.sh --go` is 0.6 m).
- `[spi] enabled = true` and the daemon is installed (`/usr/local/bin/poom-remote`). The script stops
  the service `poom-remote` (it would use `/dev/spidev0.0` as well) and starts it again at the end, also
  when it aborts.
- Copy the two files to the Pi: `scp tools/link_walk_test.sh tools/console_ts.py hexapod-controller.local:`
- Look at the plan first, it does nothing: `./link_walk_test.sh`

## Run and read it

```bash
ssh hexapod-controller.local
./link_walk_test.sh --go                       # on the Pi

scp -r hexapod-controller.local:link-walk-<stamp> .   # on the Mac
python3 tools/link_walk_report.py --console link-walk-<stamp>/console.log \
        --marks link-walk-<stamp>/marks.txt --events link-walk-<stamp>/events.jsonl
```

The output is one row per 30 s window of the firmware's bus diagnostics, with the share of the window
spent walking, the failed servo reads and which servo they were on, read and write times, the longest
run of failed reads, loop overruns, and the SPI link errors and rejected commands of the daemon in
that window. Then totals for standing and walking and a verdict:

- **PASS**: in every window with at least 5 s of walking: at most 1 failed read (the background
  rate is about 1 in 3,000), never two in a row, no failed write, at most 3 SPI link errors; and no fatal
  error in the console.
- **FAIL**: any of those is violated. The reasons are listed per window.
- **INCONCLUSIVE** (exit code 2): fewer than 2 windows with enough walking (change `--min-walk-s` and
  `--min-windows`, or walk more).

Limits are options (`--max-failures`, `--max-spi-errors`). Exit code 0, 1 or 2.

## Reference numbers (2026-10-16)

| Situation | Windows | Failed reads |
|-----------|---------|--------------|
| link running, interrupt priorities all 5, short walks | 2 | 5 and 2 of 1,500, servo 0 |
| link not running, standing | 1 | 0 of 1,450 |
| link running, SPI1 and USART1 at priority 6, standing | 4 | 1, 0, 0, 0 (5,950 reads) |

Read time 4.5 ms (18 bytes per servo), write time 1.95 ms, no loop overruns in the clean windows.

## Files

- `tools/link_walk_test.sh`: the test, on the Pi. Dry run by default.
- `tools/console_ts.py`: timestamped capture of the firmware console (`tio` does not run detached, `cat` loses the time).
- `tools/link_walk_report.py`: the report; `tools/test_link_walk_report.py`: its tests
  (`python3 -m unittest tools/test_link_walk_report.py`).
- Collected per run, in `~/link-walk-<stamp>/`: `console.log`, `marks.txt` (what the test did and when),
  `events.jsonl` (the daemon's events), `daemon.out`.

## Not covered

Rotation (fault), walking backward and strafing (the controller cannot yet), the real Poom over Bluetooth,
and the quality of the motion itself (smoothness, lag of the profile): this test is about the buses and the link.
