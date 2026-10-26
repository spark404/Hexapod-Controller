#!/usr/bin/env python3
"""Report of the walking test with the bus diagnostics (docs/link-walking-test.md).

Reads what tools/link_walk_test.sh collected on the Pi:

  console.log   the firmware console, one line per line: "<unix time> <text>"
                (tools/console_ts.py)
  marks.txt     what the test did and when: "<unix time> <label>", with the labels
                "walk start" and "walk end" around every walk
  events.jsonl  the event log of poom-remote (robot_status, spi_error, ...), optional

and puts every 30 s window of the servo bus diagnostics ("[Bus] 30s: ...", see
Core/Src/bus_diag.c) next to what the robot was doing in that window: how much of it was
spent walking, how many servo reads failed, how many SPI link errors there were. A second table
shows what the firmware reports in the same block: the accelerometer samples it rejected
("[Accel]"), the roll and pitch of the filter ("[Sensors]"), and the servo input voltage and
temperature ("[Servos]"). The attitude is information (a note when the robot stands still and
roll or pitch is off); a servo supply or temperature out of range fails the run.

Exit code: 0 pass, 1 fail, 2 inconclusive (not enough walking in the windows).

    python3 tools/link_walk_report.py --console console.log --marks marks.txt --events events.jsonl
"""
import argparse
import json
import math
import re
import sys

WINDOW_S = 30.0

BUS = re.compile(
    r"\[Bus\] 30s: rd (\d+) ok (\d+) \(us (?:(\d+)/(\d+)/(\d+)|-)\) \| wr (\d+) ok (\d+) \(us (\d+)/(\d+)/(\d+)\)"
    r" \| max fail streak (\d+) \| loop overruns (\d+)")
SERVOS = re.compile(r"\[Bus\] reads without data per servo \(leg\*3\+joint\): (.*)$")
HEALTH = re.compile(r"\[Servos\] input (\d+)\.(\d) to (\d+)\.(\d) V \(lowest: servo (\d+)\), "
                    r"temperature up to (\d+) C \(servo (\d+)\)")
SENSORS = re.compile(r"\[Sensors\].*ekf roll\s+(-?\d+(?:\.\d+)?) pitch\s+(-?\d+(?:\.\d+)?)")
ACCEL_NONE = re.compile(r"\[Accel\] (\d+) samples, none rejected")
ACCEL_BAD = re.compile(r"\[Accel\] (\d+) of (\d+) samples rejected")
FATAL = re.compile(r"FATAL ERROR|HARD FAULT|Limit alert triggered")
BLOCK_S = 3.0          # the reports of one block follow the [Bus] line within this many seconds
VOLTAGE_LOW_DV = 95    # as in the firmware (SERVO_VOLTAGE_LOW_DV, units of 0.1 V)
TEMPERATURE_HIGH_C = 65
REST_TILT_DEG = 3.0    # roll or pitch beyond this at standstill is worth a note
LINE = re.compile(r"^(\d+(?:\.\d+)?) (.*)$")


def read(path):
    with open(path, errors="replace") as f:
        return f.read()


def parse_console(text):
    """-> (windows, fatal) : windows are dicts with the numbers of one [Bus] 30s line"""
    windows, fatal, last = [], [], None
    for raw in text.splitlines():
        m = LINE.match(raw.rstrip("\r"))
        if not m:
            continue
        t, body = float(m.group(1)), m.group(2)
        b = BUS.search(body)
        if b:
            g = [int(x) if x is not None else 0 for x in b.groups()]
            last = {"t_end": t, "rd": g[0], "rd_ok": g[1], "rd_us": g[2:5], "wr": g[5], "wr_ok": g[6],
                    "wr_us": g[7:10], "streak": g[10], "overruns": g[11], "servos": {},
                    "volt": None, "temp": None, "roll": None, "pitch": None, "acc_rej": None, "acc_total": None}
            windows.append(last)
            continue
        if last is not None and t - last["t_end"] < BLOCK_S:
            h = HEALTH.search(body)
            if h:
                v = [int(x) for x in h.groups()]
                last["volt"] = (v[0] * 10 + v[1], v[2] * 10 + v[3], v[4])
                last["temp"] = (v[5], v[6])
            m2 = SENSORS.search(body)
            if m2:
                last["roll"], last["pitch"] = float(m2.group(1)), float(m2.group(2))
            a0 = ACCEL_NONE.search(body)
            if a0:
                last["acc_rej"], last["acc_total"] = 0, int(a0.group(1))
            a1 = ACCEL_BAD.search(body)
            if a1:
                last["acc_rej"], last["acc_total"] = int(a1.group(1)), int(a1.group(2))
        s = SERVOS.search(body)
        if s and last is not None and t - last["t_end"] < 2.0:
            for part in s.group(1).split():
                servo, _, count = part.partition(":")
                if servo.isdigit() and count.isdigit():
                    last["servos"][int(servo)] = int(count)
        if FATAL.search(body):
            fatal.append((t, body.strip()))
    return windows, fatal


def parse_marks(text):
    """-> (walk intervals [(start, end)], start of the test or None)"""
    marks = []
    for raw in text.splitlines():
        m = LINE.match(raw.strip())
        if m:
            marks.append((float(m.group(1)), m.group(2).strip()))
    walks, open_at = [], None
    for t, label in marks:
        if label == "walk start" and open_at is None:
            open_at = t
        elif label == "walk end" and open_at is not None:
            walks.append((open_at, t))
            open_at = None
    if open_at is not None and marks:          # the test ended while walking
        walks.append((open_at, marks[-1][0]))
    return walks, (marks[0][0] if marks else None)


def parse_events(text):
    out = []
    for raw in text.splitlines():
        try:
            out.append(json.loads(raw))
        except ValueError:
            pass
    return out


def overlap(a0, a1, b0, b1):
    return max(0.0, min(a1, b1) - max(a0, b0))


def build(windows, walks, events):
    """Attach the walking share and the link events to every window."""
    for w in windows:
        t0, t1 = w["t_end"] - WINDOW_S, w["t_end"]
        w["walk_s"] = sum(overlap(a, b, t0, t1) for a, b in walks)
        w["bouts"] = sum(1 for a, b in walks if overlap(a, b, t0, t1) > 0)
        inside = [e for e in events if t0 < e.get("t", 0) <= t1]
        w["spi_errors"] = sum(1 for e in inside if e.get("event") == "spi_error")
        w["rejected"] = sum(1 for e in inside if e.get("event") == "robot_status" and e.get("rejected"))
        w["link_lost"] = sum(1 for e in inside if e.get("event") == "robot_status" and e.get("link_ok") is False)
        w["failures"] = w["rd"] - w["rd_ok"]
    return windows


def judge(windows, fatal, max_failures, max_spi_errors, min_walk_s, min_windows):
    """-> (verdict, reasons): 'PASS', 'FAIL' or 'INCONCLUSIVE'"""
    reasons = []
    if fatal:
        reasons.append(f"fatal error in the console: {fatal[0][1]}")
    for w in windows:
        if w["volt"] is not None and w["volt"][0] < VOLTAGE_LOW_DV:
            reasons.append(f"window ending {w['t_end']:.0f}: servo input down to {w['volt'][0] / 10:.1f} V "
                           f"(servo {w['volt'][2]}), limit {VOLTAGE_LOW_DV / 10:.1f} V")
        if w["temp"] is not None and w["temp"][0] >= TEMPERATURE_HIGH_C:
            reasons.append(f"window ending {w['t_end']:.0f}: servo {w['temp'][1]} at {w['temp'][0]} C, "
                           f"limit {TEMPERATURE_HIGH_C} C")
    walking = [w for w in windows if w["walk_s"] >= min_walk_s]
    for w in walking:
        if w["failures"] > max_failures:
            reasons.append(f"window ending {w['t_end']:.0f}: {w['failures']} failed servo reads (limit {max_failures})")
        if w["streak"] > 1:
            reasons.append(f"window ending {w['t_end']:.0f}: {w['streak']} failed reads in a row")
        if w["wr"] != w["wr_ok"]:
            reasons.append(f"window ending {w['t_end']:.0f}: {w['wr'] - w['wr_ok']} failed servo writes")
        if w["spi_errors"] > max_spi_errors:
            reasons.append(f"window ending {w['t_end']:.0f}: {w['spi_errors']} SPI link errors (limit {max_spi_errors})")
    if reasons:
        return "FAIL", reasons
    if len(walking) < min_windows:
        return "INCONCLUSIVE", [f"only {len(walking)} window(s) with at least {min_walk_s:.0f} s of walking, "
                                f"{min_windows} needed"]
    return "PASS", []


def fmt_table(windows, t_ref):
    head = ("end s", "phase", "reads", "fail", "rd us avg/max", "wr us avg", "streak", "overrun", "spi err", "rejected", "servos")
    rows = [head]
    for w in windows:
        phase = "standing" if w["walk_s"] == 0 else f"walking {100 * w['walk_s'] / WINDOW_S:.0f}% ({w['bouts']}x)"
        servos = " ".join(f"{k}:{v}" for k, v in sorted(w["servos"].items())) or "-"
        rows.append((f"{w['t_end'] - t_ref:.0f}", phase, f"{w['rd']}", f"{w['failures']}",
                     f"{w['rd_us'][1]}/{w['rd_us'][2]}", f"{w['wr_us'][1]}", f"{w['streak']}", f"{w['overruns']}",
                     f"{w['spi_errors']}", f"{w['rejected']}", servos))
    width = [max(len(r[i]) for r in rows) for i in range(len(head))]
    return "\n".join("  ".join(c.ljust(width[i]) for i, c in enumerate(r)).rstrip() for r in rows)


def phase_of(w):
    return "standing" if w["walk_s"] == 0 else f"walking {100 * w['walk_s'] / WINDOW_S:.0f}%"


def fmt_attitude_table(windows, t_ref):
    head = ("end s", "phase", "roll deg", "pitch deg", "acc rejected", "servo V", "servo C")
    rows = [head]
    for w in windows:
        roll = "-" if w["roll"] is None else f"{math.degrees(w['roll']):.1f}"
        pitch = "-" if w["pitch"] is None else f"{math.degrees(w['pitch']):.1f}"
        acc = "-" if w["acc_rej"] is None else f"{w['acc_rej']} of {w['acc_total']}"
        volt = "-" if w["volt"] is None else f"{w['volt'][0] / 10:.1f}-{w['volt'][1] / 10:.1f}"
        temp = "-" if w["temp"] is None else f"{w['temp'][0]}"
        rows.append((f"{w['t_end'] - t_ref:.0f}", phase_of(w), roll, pitch, acc, volt, temp))
    width = [max(len(r[i]) for r in rows) for i in range(len(head))]
    return "\n".join("  ".join(c.ljust(width[i]) for i, c in enumerate(r)).rstrip() for r in rows)


def attitude_notes(windows):
    """Observations about the attitude, not part of the verdict."""
    notes = []
    still = [w for w in windows if w["walk_s"] == 0 and w["roll"] is not None]
    for w in still:
        r, p = math.degrees(w["roll"]), math.degrees(w["pitch"])
        if abs(r) > REST_TILT_DEG or abs(p) > REST_TILT_DEG:
            notes.append(f"standing, window ending {w['t_end']:.0f}: roll {r:.1f} / pitch {p:.1f} deg, "
                         f"more than {REST_TILT_DEG:.0f} deg from level at rest: check the accelerometer")
    rej = sum(w["acc_rej"] or 0 for w in windows)
    tot = sum(w["acc_total"] or 0 for w in windows)
    if tot and rej:
        at_rest = sum(w["acc_rej"] or 0 for w in windows if w["walk_s"] == 0)
        notes.append(f"accelerometer samples rejected: {rej} of {tot} ({100 * rej / tot:.1f} %), "
                     f"{at_rest} of them in windows without walking")
    return notes


def summary(windows):
    def tot(sel):
        ws = [w for w in windows if sel(w)]
        reads, fails = sum(w["rd"] for w in ws), sum(w["failures"] for w in ws)
        return len(ws), reads, fails
    lines = []
    for name, sel in (("standing", lambda w: w["walk_s"] == 0), ("walking", lambda w: w["walk_s"] > 0)):
        n, reads, fails = tot(sel)
        rate = f"{fails / reads * 1000:.2f} per 1000 reads" if reads else "-"
        att = [w for w in windows if sel(w) and w["roll"] is not None]
        tilt = ""
        if att:
            tilt = (f"; roll up to {max(abs(math.degrees(w['roll'])) for w in att):.1f} deg, "
                    f"pitch up to {max(abs(math.degrees(w['pitch'])) for w in att):.1f} deg")
        lines.append(f"{name:9s} {n} window(s), {reads} reads, {fails} failed ({rate}){tilt}")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--console", required=True)
    ap.add_argument("--marks", required=True)
    ap.add_argument("--events", action="append", default=[])
    ap.add_argument("--max-failures", type=int, default=1, help="failed servo reads allowed per window with walking")
    ap.add_argument("--max-spi-errors", type=int, default=3, help="SPI link errors allowed per window with walking")
    ap.add_argument("--min-walk-s", type=float, default=5.0, help="seconds of walking for a window to count")
    ap.add_argument("--min-windows", type=int, default=2, help="windows with walking needed for a verdict")
    a = ap.parse_args(argv)

    windows, fatal = parse_console(read(a.console))
    walks, t_start = parse_marks(read(a.marks))
    events = [e for f in a.events for e in parse_events(read(f))]
    if not windows:
        print("no [Bus] 30s lines in the console log: was the capture running, did the firmware print them?")
        return 2
    windows = build(windows, walks, events)
    t_ref = t_start if t_start is not None else windows[0]["t_end"] - WINDOW_S

    print(fmt_table(windows, t_ref))
    print()
    if any(w["roll"] is not None or w["volt"] is not None or w["acc_rej"] is not None for w in windows):
        print(fmt_attitude_table(windows, t_ref))
        print()
    print(summary(windows))
    for note in attitude_notes(windows):
        print("note:", note)
    if events:
        states = sorted({e["state"] for e in events if e.get("event") == "robot_status" and "state" in e})
        print("robot states reported over the link:", ", ".join(states) or "-")
    verdict, reasons = judge(windows, fatal, a.max_failures, a.max_spi_errors, a.min_walk_s, a.min_windows)
    print(f"\n{verdict}" + ("" if not reasons else ":"))
    for r in reasons:
        print("  -", r)
    return {"PASS": 0, "FAIL": 1, "INCONCLUSIVE": 2}[verdict]


if __name__ == "__main__":
    sys.exit(main())
