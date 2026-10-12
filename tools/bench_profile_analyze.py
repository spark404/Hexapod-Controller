#!/usr/bin/env python3
"""Analysis of the servo profile bench experiment.

Reads the console log of the Bench firmware (the lines that start with "BP,", see
Core/Src/bench_profile.c and docs/servo-profile-bench-experiment.md), computes the
metrics of every run and prints the tables and the recommendation of the
experiment. Standard library only.

    python3 tools/bench_profile_analyze.py tio-output.log
    python3 tools/bench_profile_analyze.py tio-output.log --csv metrics.csv
    python3 tools/bench_profile_analyze.py tio-output.log --run 17     # details of one run
    python3 tools/bench_profile_analyze.py tio-output.log --session 1  # an earlier session in the log
    tools/.venv/bin/python tools/bench_profile_analyze.py tio-output.log --plots plots/   # PNG plots

The plots need matplotlib, which is not needed for the rest:
    python3 -m venv tools/.venv && tools/.venv/bin/pip install -r tools/requirements.txt

Times are in the servo's own millisecond tick (Realtime Tick), so host jitter does
not matter. A goal that was written at host time h is placed on that scale by
interpolating between the samples around it.
"""
import argparse
import bisect
import csv
import math
import os
import statistics
import sys

PULSE_RAD = 2 * math.pi / 4096
VEL_UNIT = 0.229 * 2 * math.pi / 60            # rad/s per Present/Profile Velocity unit
ACC_UNIT = 214.577 * 2 * math.pi / 3600        # rad/s^2 per Profile Acceleration unit
TICK_WRAP = 32768                              # the Realtime Tick counts 0..32767 ms
PRE_TRIGGER_MS = 50

MODE_NAMES = {0: "none", 1: "velocity", 2: "velocity-computed", 3: "time"}
PATH_NAMES = {0: "step", 1: "ramp", 2: "sine", 3: "stop", 4: "reverse"}
JOINT_NAMES = {0: "coxa", 1: "femur", 2: "tibia"}
MODEL_NAMES = {"1060": "XL430-W250", "1090": "2XL430-W250"}

# Decision thresholds of the plan (section 4)
ARRIVAL_TOLERANCE = 0.10        # time based: arrival within +-10 % of T
CONTINUOUS_MIN_RATIO = 0.5      # chained: velocity never below 50 % of the segment mean
RIPPLE_LIMIT = 0.30             # chained sine: relative velocity ripple


# ---------------------------------------------------------------- the ideal path (mirror of bench_script.c)

def joint_delta_rad(run, joint):
    if not run["joints"] & (1 << joint):
        return 0.0
    return run["delta"][joint] / 1000.0


def path_offset_rad(run, joint, t_ms):
    """Ideal offset in rad of a joint at path time t_ms; before 0 the start pose."""
    d = joint_delta_rad(run, joint)
    if d == 0.0:
        return 0.0
    t = max(t_ms, 0)
    path = PATH_NAMES[run["path"]]
    if path in ("step", "stop"):
        return -d / 2 if t_ms < 0 else d / 2
    if path == "reverse":
        if t_ms < 0:
            return -d / 2
        return -d / 2 if t >= run["path_param"] else d / 2
    if path == "ramp":
        travel = min(abs(d), run["path_param"] / 1000.0 * t / 1000.0)
        return -d / 2 + (-travel if d < 0 else travel)
    if path == "sine":
        return d / 2 * math.sin(2 * math.pi * run["path_param"] / 1000.0 * t / 1000.0)
    raise ValueError(path)


# ---------------------------------------------------------------- parsing

def new_run(fields):
    keys = ["id", "exp", "mode", "path", "joints", "period", "time", "ramp", "velocity", "accel",
            "d0", "d1", "d2", "path_param", "sample_ms", "duration"]
    values = [int(x) for x in fields]
    run = dict(zip(keys, values))
    run["delta"] = [run.pop("d0"), run.pop("d1"), run.pop("d2")]
    run.update(park=None, centre=None, goals=[], samples={0: [], 1: [], 2: []}, status=None, sample_count=0)
    return run


def parse(lines):
    """Split the log into sessions (one per BP,BEGIN); returns a list of sessions."""
    sessions = []
    session = None
    bad = 0
    for raw in lines:
        i = raw.find("BP,")
        if i < 0:
            continue
        parts = raw[i:].strip().split(",")
        try:
            kind = parts[1]
            if kind == "BEGIN":
                session = {"mask": int(parts[2]), "runs_planned": int(parts[3]), "estimate": float(parts[4]),
                           "ids": {}, "notes": [], "runs": {}, "done": None, "bad_lines": 0}
                sessions.append(session)
                continue
            if session is None:
                continue
            if kind == "ID":
                session["ids"].setdefault(int(parts[2]), {})[parts[3]] = parts[4]
            elif kind == "NOTE":
                session["notes"].append(",".join(parts[2:]))
            elif kind == "RUN":
                run = new_run(parts[2:])
                session["runs"][run["id"]] = run
            elif kind == "START":
                run = session["runs"][int(parts[2])]
                run["park"] = [int(x) for x in parts[3:6]]
                run["centre"] = [int(x) for x in parts[6:9]]
            elif kind == "G":
                run = session["runs"][int(parts[2])]
                run["goals"].append({"host_us": int(parts[3]), "joint": int(parts[4]), "goal": int(parts[5]),
                                     "velocity": int(parts[6]), "accel": int(parts[7])})
            elif kind == "S":
                run = session["runs"][int(parts[2])]
                joint = int(parts[4])
                run["samples"][joint].append({"host_us": int(parts[3]), "tick": int(parts[5]),
                                              "moving": int(parts[6]), "load": int(parts[7]),
                                              "vel": int(parts[8]), "pos": int(parts[9]),
                                              "vtraj": int(parts[10]), "ptraj": int(parts[11])})
            elif kind == "END":
                run = session["runs"][int(parts[2])]
                run["sample_count"] = int(parts[3])
                run["status"] = parts[4]
            elif kind == "DONE":
                session["done"] = parts[2]
        except (IndexError, ValueError, KeyError):
            bad += 1
            if session is not None:
                session["bad_lines"] += 1
    return sessions


# ---------------------------------------------------------------- time base

def unwrap_ticks(samples):
    """Servo time in ms of every sample (the tick wraps at 32768)."""
    out = []
    offset = 0
    last = None
    for s in samples:
        if last is not None and s["tick"] < last - TICK_WRAP // 2:
            offset += TICK_WRAP
        last = s["tick"]
        out.append(s["tick"] + offset)
    return out


class Trace:
    """The samples of one joint of one run on the servo's time scale (ms), positions in pulses."""

    def __init__(self, run, joint):
        self.joint = joint
        samples = run["samples"][joint]
        self.n = len(samples)
        self.host_us = [s["host_us"] for s in samples]
        t = unwrap_ticks(samples)
        self.t = [float(x) for x in t]
        self.pos = [s["pos"] for s in samples]
        self.ptraj = [s["ptraj"] for s in samples]
        self.vel = [s["vel"] * VEL_UNIT for s in samples]          # rad/s
        self.vtraj = [s["vtraj"] * VEL_UNIT for s in samples]      # rad/s
        self.load = [s["load"] * 0.1 for s in samples]             # % of maximum
        self.moving = [s["moving"] for s in samples]

    def servo_time_of_host(self, host_us):
        """Place a host time on the servo's scale by interpolating between the samples around it."""
        if self.n == 0:
            return None
        i = bisect.bisect_left(self.host_us, host_us)
        if i == 0:
            return self.t[0] - (self.host_us[0] - host_us) / 1000.0
        if i >= self.n:
            return self.t[-1] + (host_us - self.host_us[-1]) / 1000.0
        h0, h1 = self.host_us[i - 1], self.host_us[i]
        if h1 == h0:
            return self.t[i]
        f = (host_us - h0) / (h1 - h0)
        return self.t[i - 1] + f * (self.t[i] - self.t[i - 1])


def goals_of(run, joint):
    return sorted((g for g in run["goals"] if g["joint"] == joint), key=lambda g: g["host_us"])


# ---------------------------------------------------------------- metrics of a step

def settle_time(t, values, target, tol, t_from):
    """First time at or after t_from from which values stay within tol of target until the end."""
    last_bad = None
    for k in range(len(t) - 1, -1, -1):
        if t[k] < t_from:
            break
        if abs(values[k] - target) > tol:
            last_bad = k
            break
    if last_bad is None:
        idx = bisect.bisect_left(t, t_from)
        return t[idx] if idx < len(t) else None
    if last_bad + 1 >= len(t):
        return None
    return t[last_bad + 1]


def step_metrics(run, joint):
    """Metrics of the first goal of a joint: latency, arrival, profile duration and peak velocity."""
    trace = Trace(run, joint)
    goals = goals_of(run, joint)
    if trace.n < 5 or not goals:
        return None
    goal = goals[0]
    tg = trace.servo_time_of_host(goal["host_us"])
    before = [trace.pos[k] for k in range(trace.n) if trace.t[k] < tg]
    if not before:
        return None
    p0 = statistics.median(before)
    target = goal["goal"]
    delta = target - p0
    if delta == 0:
        return None
    tol = max(3.0, 0.02 * abs(delta))

    # A joint that carries the leg stops short of its goal (no integral action in the position controller):
    # arrival is then judged against where it settles, and the shortfall is reported.
    tail = max(5, trace.n // 20)
    final = statistics.median(trace.pos[-tail:])
    static_error = final - target
    settle_target = target if abs(static_error) <= tol else final

    def first_after(values, threshold):
        for k in range(trace.n):
            if trace.t[k] >= tg and abs(values[k] - p0) >= threshold:
                return trace.t[k]
        return None

    t_pos = first_after(trace.pos, 3)
    t_traj = first_after(trace.ptraj, 1)
    arrival = settle_time(trace.t, trace.pos, settle_target, tol, tg)

    peak_vel = 0.0
    peak_time = None
    peak_traj = 0.0
    for k in range(trace.n):
        if trace.t[k] < tg:
            continue
        if abs(trace.vel[k]) > peak_vel:
            peak_vel, peak_time = abs(trace.vel[k]), trace.t[k] - tg
        peak_traj = max(peak_traj, abs(trace.vtraj[k]))

    # The duration of the profile: how long the profile generator's velocity is above 5 % of its
    # peak. Thresholds on the position would make short moves look shorter (the first and last
    # pulse of a ramp take several ms).
    profile_ms = None
    if peak_traj > 0:
        above = [trace.t[k] for k in range(trace.n) if trace.t[k] >= tg and abs(trace.vtraj[k]) >= 0.05 * peak_traj]
        if above:
            profile_ms = above[-1] - above[0]
            t_traj = above[0]       # the profile starts when its velocity does, not when the first pulse moves

    return {
        "joint": joint, "delta_pulses": delta, "delta_rad": abs(delta) * PULSE_RAD,
        "latency_traj_ms": None if t_traj is None else t_traj - tg,
        "latency_pos_ms": None if t_pos is None else t_pos - tg,
        "arrival_ms": None if arrival is None else arrival - tg,
        "profile_ms": profile_ms,
        "peak_vel": peak_vel, "peak_vel_time_ms": peak_time, "peak_vtraj": peak_traj,
        "static_error_pulses": static_error,
    }


def rest_points(run, joint):
    """(load %, position - profile in pulses) of the moments the joint stands still: before the first goal
    and at the end of the run."""
    trace = Trace(run, joint)
    goals = goals_of(run, joint)
    if trace.n < 20 or not goals:
        return []
    tg = trace.servo_time_of_host(goals[0]["host_us"])
    points = []
    for k_range in ([k for k in range(trace.n) if trace.t[k] < tg][-20:], list(range(trace.n))[-20:]):
        still = [k for k in k_range if trace.moving[k] == 0 and abs(trace.vtraj[k]) < 2 * VEL_UNIT]
        if len(still) >= 5:
            points.append((statistics.median(trace.load[k] for k in still),
                           statistics.median(trace.pos[k] - trace.ptraj[k] for k in still)))
    return points


def expected_velocity_move(run, delta_rad, limit_units=None):
    """Duration and peak velocity (rad/s) of a trapezoid with the profile of a velocity run.
    The Velocity Limit register does NOT cap the profile velocity in position mode (measured: a profile of 400
    units ran at 9.5 rad/s with a limit of 250), so limit_units is not used; the argument is kept for callers."""
    v = run["velocity"] * VEL_UNIT
    a = run["accel"] * ACC_UNIT
    if v <= 0 or a <= 0:
        return None, None
    if v * v / a >= delta_rad:       # triangle: never reaches the velocity
        return 2 * math.sqrt(delta_rad / a), math.sqrt(delta_rad * a)
    return delta_rad / v + v / a, v


# ---------------------------------------------------------------- metrics of a chained path

def moving_average(values, width):
    if width <= 1:
        return list(values)
    out = []
    acc = 0.0
    for k, v in enumerate(values):
        acc += v
        if k >= width:
            acc -= values[k - width]
        out.append(acc / min(k + 1, width))
    return out


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


def chained_metrics(run, joint):
    """Tracking error, lag and velocity ripple of a ramp or sine path."""
    trace = Trace(run, joint)
    goals = goals_of(run, joint)
    if trace.n < 20 or len(goals) < 2:
        return None
    t0 = trace.servo_time_of_host(goals[0]["host_us"])
    centre = run["centre"][joint]
    end = run["duration"]
    idx = [k for k in range(trace.n) if 0 <= trace.t[k] - t0 <= end]
    if len(idx) < 20:
        return None
    rel_t = [trace.t[k] - t0 for k in idx]
    pos_rad = [(trace.pos[k] - centre) * PULSE_RAD for k in idx]
    vel = [trace.vel[k] for k in idx]

    def error_at(lag):
        return rms([p - path_offset_rad(run, joint, t - lag) for t, p in zip(rel_t, pos_rad)])

    best_lag, best_err = 0, error_at(0)
    err_zero = best_err
    for lag in range(-100, 251):
        e = error_at(lag)
        if e < best_err:
            best_lag, best_err = lag, e

    # velocity ripple: what is left after averaging over one goal period
    width = max(1, int(round(run["period"] / run["sample_ms"])))
    smooth = moving_average(vel, width)
    lo, hi = len(vel) // 6, len(vel) - len(vel) // 6
    ripple = rms([v - s for v, s in zip(vel[lo:hi], smooth[lo:hi])]) / max(rms(vel[lo:hi]), 1e-9)

    metrics = {"joint": joint, "rms_err_rad": best_err, "lag_ms": best_lag, "rms_err_zero_lag_rad": err_zero,
               "ripple": ripple, "dip_fraction": None, "min_segment_ratio": None, "median_segment_ratio": None}

    if PATH_NAMES[run["path"]] == "ramp":
        v_ideal = run["path_param"] / 1000.0
        mid = [abs(v) for t, v in zip(rel_t, vel) if 200 <= t <= 900]
        if mid:
            metrics["dip_fraction"] = sum(1 for v in mid if v < 0.5 * v_ideal) / len(mid)
        ratios = []
        times = [trace.servo_time_of_host(g["host_us"]) - t0 for g in goals]
        for a, b in zip(times, times[1:]):
            if a < 200 or b > 900:
                continue
            seg = [abs(v) for t, v in zip(rel_t, vel) if a + 0.1 * (b - a) <= t <= b]
            if seg and statistics.mean(seg) > 0:
                ratios.append(min(seg) / statistics.mean(seg))
        if ratios:
            metrics["min_segment_ratio"] = min(ratios)
            metrics["median_segment_ratio"] = statistics.median(ratios)
    return metrics


# ---------------------------------------------------------------- stop and reversal

def stop_metrics(run, joint):
    trace = Trace(run, joint)
    goals = goals_of(run, joint)
    if trace.n < 20 or len(goals) < 2:
        return None
    t1 = trace.servo_time_of_host(goals[1]["host_us"])
    k1 = bisect.bisect_left(trace.t, t1)
    if k1 >= trace.n - 3:
        return None
    pos_at = trace.pos[k1]
    result = {"joint": joint, "event_ms": t1 - trace.servo_time_of_host(goals[0]["host_us"]),
              "speed_at_event": abs(trace.vel[k1])}

    if PATH_NAMES[run["path"]] == "stop":
        stopped = None
        for k in range(k1, trace.n):
            if all(abs(v) <= 0.05 for v in trace.vel[k:]):
                stopped = trace.t[k]
                break
        result["stop_ms"] = None if stopped is None else stopped - t1
        result["stop_distance_rad"] = abs(trace.pos[-1] - pos_at) * PULSE_RAD
        result["stop_distance_free_rad"] = None
        return result

    # reversal: when does the velocity change sign, how smooth is it, when is the start pose reached
    direction = 1 if trace.vel[k1] >= 0 else -1
    flip = None
    for k in range(k1, trace.n):
        if trace.vel[k] * direction < 0:
            flip = k
            break
    if flip is None:
        result.update(flip_ms=None, min_speed_at_flip=None, max_dv=None, back_ms=None)
        return result
    window = [k for k in range(max(k1, flip - 20), min(trace.n, flip + 20))]
    result["flip_ms"] = trace.t[flip] - t1
    result["min_speed_at_flip"] = min(abs(trace.vel[k]) for k in window)
    result["max_dv"] = max(abs(trace.vel[k + 1] - trace.vel[k]) for k in window if k + 1 < trace.n)
    back = settle_time(trace.t, trace.pos, goals[1]["goal"], max(3.0, 0.02 * abs(goals[0]["goal"] - goals[1]["goal"])), t1)
    result["back_ms"] = None if back is None else back - t1
    return result


# ---------------------------------------------------------------- per run summary

def describe(run):
    mode = MODE_NAMES[run["mode"]]
    if run["mode"] == 1:
        mode += f" v={run['velocity']} a={run['accel']}"
    elif run["mode"] in (2, 3):
        mode += f" T={run['time']} Tr={run['ramp']}"
    path = PATH_NAMES[run["path"]]
    if run["period"]:
        path += f" P={run['period']}"
    joints = "+".join(JOINT_NAMES[j] for j in range(3) if run["joints"] & (1 << j))
    return f"{mode:<28} {path:<12} {joints}"


def analyze_run(run):
    """All metrics of a run, per joint, depending on its path."""
    result = {"run": run, "joints": {}}
    path = PATH_NAMES[run["path"]]
    for joint in range(3):
        if not run["joints"] & (1 << joint) or not run["samples"][joint]:
            continue
        if path == "step":
            result["joints"][joint] = step_metrics(run, joint)
        elif path in ("ramp", "sine"):
            result["joints"][joint] = chained_metrics(run, joint)
        else:
            result["joints"][joint] = stop_metrics(run, joint)
    return result


def fmt(value, spec="{:.1f}", none="-"):
    return none if value is None else spec.format(value)


# ---------------------------------------------------------------- the report

def velocity_limit_units(session):
    limits = []
    for registers in session["ids"].values():
        try:
            limits.append(int(registers["velocity_limit"]))
        except (KeyError, ValueError):
            pass
    return min(limits) if limits else 265


def velocity_limit_radps(session):
    return velocity_limit_units(session) * VEL_UNIT


def report_ids(session, out):
    out.append("== E1: identification of the servos of leg 0")
    if not session["ids"]:
        out.append("   (no identification lines in the log)")
        return
    names = []
    for registers in session["ids"].values():
        for name in registers:
            if name not in names:
                names.append(name)
    servo_ids = sorted(session["ids"])
    out.append("   " + f"{'register':<24}" + "".join(f"{('servo ' + str(i)):>12}" for i in servo_ids))
    for name in names:
        out.append("   " + f"{name:<24}" + "".join(f"{session['ids'][i].get(name, '-'):>12}" for i in servo_ids))
    models = {i: MODEL_NAMES.get(session["ids"][i].get("model_number"), "unknown model") for i in servo_ids}
    out.append("   models: " + ", ".join(f"servo {i} = {m}" for i, m in models.items()))
    if len(set(models.values())) > 1:
        out.append("   NOTE: the servos of the leg are not all the same model; velocity limits and behaviour can differ")


def report_selftest(runs, out):
    out.append("== E0: safety self test (the range guard has to abort the run)")
    tests = [r for r in runs if r["exp"] == 0]
    if not tests:
        out.append("   (not in this log)")
        return None
    ok = True
    for r in tests:
        passed = bool(r["status"]) and r["status"].startswith("ABORT:out of range")
        ok = ok and passed
        out.append(f"   run {r['id']}: {r['status']} -> " + ("PASS, the guard stopped the run and the torque went off"
                                                            if passed else "FAIL, the guard did not abort"))
    return ok


def report_directions(analyses, out):
    out.append("== E1: direction check (each joint 100 mrad, did it move as commanded?)")
    rows = [a for a in analyses if a["run"]["exp"] == 1 and a["run"]["path"] == 0 and a["run"]["mode"] == 1
            and bin(a["run"]["joints"]).count("1") == 1 and abs(max(a["run"]["delta"], key=abs)) <= 100]
    if not rows:
        out.append("   (no runs)")
        return
    for a in rows:
        run = a["run"]
        for joint, m in a["joints"].items():
            if not m:
                continue
            trace = Trace(run, joint)
            moved = trace.pos[-1] - statistics.median(trace.pos[:max(3, PRE_TRIGGER_MS // run["sample_ms"] // 2)])
            wanted = m["delta_pulses"]
            ok = moved * wanted > 0 and abs(moved) >= 0.5 * abs(wanted)
            out.append(f"   run {run['id']:>3} {JOINT_NAMES[joint]:<6} commanded {wanted:+.0f} pulses, moved {moved:+.0f}"
                       f" -> {'ok' if ok else 'NOT AS COMMANDED'}")


def report_static_error(analyses, out):
    out.append("== Position error under load (position minus profile, joint standing still)")
    by_joint = {}
    for a in analyses:
        run = a["run"]
        for joint in a["joints"]:
            if run["path"] == 0 and run["exp"] in (1, 2, 3):
                by_joint.setdefault(joint, []).extend(rest_points(run, joint))
    if not any(by_joint.values()):
        out.append("   (no data)")
        return
    for joint in sorted(by_joint):
        pts = by_joint[joint]
        if not pts:
            continue
        sll = sum(l * l for l, _ in pts)
        slope = sum(l * e for l, e in pts) / sll if sll > 0 else None
        biggest = max(pts, key=lambda p: abs(p[1]))
        out.append(f"   {JOINT_NAMES[joint]:<6} {len(pts):>3} rest points, largest error {biggest[1]:+.0f} pulses "
                   f"({biggest[1] * PULSE_RAD * 1000:+.0f} mrad) at load {biggest[0]:+.1f} %"
                   + (f", about {slope:+.2f} pulses per % of load" if slope is not None and len(pts) >= 3 else ""))
    out.append("   (the profile's own position is the goal the servo is working towards; the difference is what the"
               " position controller leaves, so it follows the load)")


def report_units(analyses, out, limit_units=None):
    out.append("== E1/E2: units and velocity based profile (measured against the trapezoid of the profile)")
    rows = [a for a in analyses if a["run"]["exp"] in (1, 2) and a["run"]["mode"] == 1 and a["run"]["path"] == 0]
    if not rows:
        out.append("   (no runs)")
        return None
    out.append(f"   {'run':>4} {'joint':<6} {'v':>5} {'a':>5} {'delta rad':>9} {'exp s':>7} {'profile s':>9} {'ratio':>6}"
               f" {'exp vpk':>8} {'vpk traj':>9} {'vpk meas':>9} {'vpk ratio':>9}")
    velocity_ratios = []        # profiles that reach their velocity: the velocity unit
    accel_ratios = []           # profiles that never do (a triangle): the peak depends on the acceleration unit only
    for a in rows:
        run = a["run"]
        for joint, m in a["joints"].items():
            if not m:
                continue
            t_exp, v_exp = expected_velocity_move(run, m["delta_rad"], limit_units)
            ratio = None if (t_exp is None or m["profile_ms"] is None) else m["profile_ms"] / 1000.0 / t_exp
            vratio = None if not v_exp else m["peak_vtraj"] / v_exp
            if vratio is not None and run["exp"] in (1, 2):
                triangle = (run["velocity"] * VEL_UNIT) ** 2 / (run["accel"] * ACC_UNIT) >= m["delta_rad"]
                (accel_ratios if triangle else velocity_ratios).append(vratio)
            out.append(f"   {run['id']:>4} {JOINT_NAMES[joint]:<6} {run['velocity']:>5} {run['accel']:>5}"
                       f" {m['delta_rad']:>9.3f} {fmt(t_exp, '{:.3f}'):>7} {fmt(None if m['profile_ms'] is None else m['profile_ms'] / 1000, '{:.3f}'):>9}"
                       f" {fmt(ratio, '{:.2f}'):>6} {fmt(v_exp, '{:.2f}'):>8} {m['peak_vtraj']:>9.2f} {m['peak_vel']:>9.2f}"
                       f" {fmt(vratio, '{:.2f}'):>9}")
    if velocity_ratios:
        r = statistics.median(velocity_ratios)
        verdict = "confirmed" if abs(r - 1) < 0.05 else f"OFF by {100 * (r - 1):+.0f} % (use {VEL_UNIT * r:.6f} rad/s per unit)"
        out.append(f"   velocity unit (0.229 rpm = {VEL_UNIT:.6f} rad/s) from {len(velocity_ratios)} profiles that reach their velocity: {verdict}")
    if accel_ratios:
        # the peak of a triangle is sqrt(delta * a), so the ratio of the peaks is the square root of the unit ratio
        r = statistics.median(accel_ratios) ** 2
        verdict = "confirmed" if abs(r - 1) < 0.1 else f"OFF by {100 * (r - 1):+.0f} % (use {ACC_UNIT * r:.6f} rad/s^2 per unit)"
        out.append(f"   acceleration unit (214.577 rpm^2 = {ACC_UNIT:.6f} rad/s^2) from {len(accel_ratios)} short profiles: {verdict}")
    if not velocity_ratios and not accel_ratios:
        out.append("   (no profile gave a peak velocity to compare)")
    if limit_units:
        above = [(a["run"], m) for a in rows for m in a["joints"].values()
                 if m and a["run"]["velocity"] > limit_units and m["peak_vtraj"] > 1.05 * limit_units * VEL_UNIT]
        if above:
            run, m = above[0]
            out.append(f"   NOTE: run {run['id']} asked for {run['velocity']} units, above the Velocity Limit register "
                       f"({limit_units}): the profile ran at {m['peak_vtraj']:.1f} rad/s, so the limit does not cap the "
                       "profile in position mode")
    return None


def time_verdict(analyses, limit_radps, out):
    out.append("== E3: time based profile, one joint: arrival time against T")
    rows = [a for a in analyses if a["run"]["exp"] == 3 and a["run"]["mode"] == 3]
    if not rows:
        out.append("   (no runs)")
        return None
    out.append(f"   {'run':>4} {'T':>5} {'Tr':>4} {'delta':>6} {'needs rad/s':>11} {'profile ms':>10} {'arrival ms':>10}"
               f" {'arr/T':>6} {'vpk traj':>8}  verdict")
    feasible_ok = []
    saturated = []
    for a in rows:
        run = a["run"]
        for joint, m in a["joints"].items():
            if not m:
                continue
            needs = m["delta_rad"] / ((run["time"] - run["ramp"]) / 1000.0)
            feasible = needs <= 0.95 * limit_radps
            ratio = None if m["arrival_ms"] is None else m["arrival_ms"] / run["time"]
            profile_ratio = None if m["profile_ms"] is None else m["profile_ms"] / run["time"]
            best = profile_ratio if profile_ratio is not None else ratio
            if feasible:
                ok = best is not None and abs(best - 1) <= ARRIVAL_TOLERANCE
                feasible_ok.append(ok)
                verdict = "ok" if ok else "OFF"
            else:
                verdict = "too fast for the servo"
                saturated.append((run, m))
            out.append(f"   {run['id']:>4} {run['time']:>5} {run['ramp']:>4} {m['delta_rad']:>6.2f} {needs:>11.2f}"
                       f" {fmt(m['profile_ms'], '{:.0f}'):>10} {fmt(m['arrival_ms'], '{:.0f}'):>10}"
                       f" {fmt(ratio, '{:.2f}'):>6} {m['peak_vtraj']:>8.2f}  {verdict}")
    if saturated:
        fastest = max((m["peak_vel"], a["run"]["id"]) for a in analyses for m in a["joints"].values()
                      if m and a["run"]["exp"] in (2, 3))
        out.append(f"   'too fast': needs more than {0.95 * limit_radps:.1f} rad/s. The profile itself still takes T (second"
                   " column pair above), the joint follows later: its arrival column. The highest speed any joint"
                   f" reached in these runs was {fastest[0]:.1f} rad/s (run {fastest[1]}).")
    result = bool(feasible_ok) and all(feasible_ok)
    out.append(f"   criterion 1 (every feasible move arrives within +-{int(ARRIVAL_TOLERANCE * 100)} % of T): "
               f"{'PASS' if result else 'FAIL'}" + ("" if feasible_ok else " (no data)"))
    return result if feasible_ok else None


def chained_report(analyses, out):
    out.append("== E4: chained goals (tracking error at the best lag, velocity ripple, dips)")
    rows = [a for a in analyses if a["run"]["exp"] == 4]
    if not rows:
        out.append("   (no runs)")
        return {}
    out.append(f"   {'run':>4} {'path':<5} {'P':>4} {'mode':<18} {'T':>4} {'rms mrad':>8} {'lag ms':>6} {'rms@0':>7}"
               f" {'ripple':>6} {'dip %':>6} {'min seg':>7} {'med seg':>7}  continuous")
    table = {}
    for a in sorted(rows, key=lambda a: (a["run"]["path"], -a["run"]["period"], a["run"]["mode"], a["run"]["time"])):
        run = a["run"]
        m = a["joints"].get(2)
        if not m:
            continue
        if PATH_NAMES[run["path"]] == "ramp":
            ok = m["min_segment_ratio"] is not None and m["min_segment_ratio"] >= CONTINUOUS_MIN_RATIO
        else:
            ok = m["ripple"] <= RIPPLE_LIMIT
        table[(PATH_NAMES[run["path"]], run["period"], run["mode"], run["time"])] = (m, ok)
        out.append(f"   {run['id']:>4} {PATH_NAMES[run['path']]:<5} {run['period']:>4} {MODE_NAMES[run['mode']]:<18}"
                   f" {run['time']:>4} {1000 * m['rms_err_rad']:>8.1f} {m['lag_ms']:>6} {1000 * m['rms_err_zero_lag_rad']:>7.1f}"
                   f" {m['ripple']:>6.2f} {fmt(None if m['dip_fraction'] is None else 100 * m['dip_fraction'], '{:.0f}'):>6}"
                   f" {fmt(m['min_segment_ratio'], '{:.2f}'):>7} {fmt(m['median_segment_ratio'], '{:.2f}'):>7}  {'yes' if ok else 'NO'}")
    return table


def sync_report(analyses, out):
    out.append("== E5: three joints with unequal travel: do they arrive together?")
    rows = [a for a in analyses if a["run"]["exp"] == 5]
    if not rows:
        out.append("   (no runs)")
        return {}
    out.append(f"   {'run':>4} {'mode':<28} " + "".join(f"{JOINT_NAMES[j] + ' ms':>11}" for j in range(3)) + f" {'spread ms':>10}")
    spreads = {}
    for a in rows:
        run = a["run"]
        arrivals = [a["joints"].get(j) and a["joints"][j]["arrival_ms"] for j in range(3)]
        valid = [x for x in arrivals if x is not None]
        spread = max(valid) - min(valid) if len(valid) == 3 else None
        spreads[run["id"]] = (run, spread)
        out.append(f"   {run['id']:>4} {describe(run)[:28]:<28} " + "".join(f"{fmt(x, '{:.0f}'):>11}" for x in arrivals)
                   + f" {fmt(spread, '{:.0f}'):>10}")
    return spreads


def stop_report(analyses, out):
    out.append("== E6: servo side stop and reversal in the middle of a move")
    rows = [a for a in analyses if a["run"]["exp"] == 6]
    if not rows:
        out.append("   (no runs)")
        return
    for a in rows:
        run = a["run"]
        m = a["joints"].get(2)
        if not m:
            continue
        if PATH_NAMES[run["path"]] == "stop":
            out.append(f"   run {run['id']:>3} {describe(run)[:40]:<40} speed at the stop {m['speed_at_event']:.2f} rad/s,"
                       f" stopped after {fmt(m.get('stop_ms'), '{:.0f}')} ms, {fmt(m.get('stop_distance_rad'), '{:.3f}')} rad further")
        else:
            out.append(f"   run {run['id']:>3} {describe(run)[:40]:<40} velocity changes sign after {fmt(m.get('flip_ms'), '{:.0f}')} ms,"
                       f" lowest speed {fmt(m.get('min_speed_at_flip'), '{:.2f}')} rad/s, largest step in velocity"
                       f" {fmt(m.get('max_dv'), '{:.2f}')} rad/s, back at the start after {fmt(m.get('back_ms'), '{:.0f}')} ms")


def latency_report(analyses, out):
    out.append("== E7: latency, goal written (bus transaction finished) to first movement")
    by_mode = {}
    for a in analyses:
        run = a["run"]
        if run["path"] != 0 or run["exp"] not in (2, 3):
            continue
        for m in a["joints"].values():
            if m and m["latency_traj_ms"] is not None:
                by_mode.setdefault(MODE_NAMES[run["mode"]], []).append((m["latency_traj_ms"], m["latency_pos_ms"]))
    if not by_mode:
        out.append("   (no runs)")
        return
    for mode, values in sorted(by_mode.items()):
        traj = [v[0] for v in values]
        pos = [v[1] for v in values if v[1] is not None]
        out.append(f"   {mode:<10} trajectory starts after {statistics.median(traj):.1f} ms (max {max(traj):.1f}),"
                   f" position moves after {fmt(statistics.median(pos) if pos else None)} ms, {len(values)} runs")


def recommend(time_ok, chained, out):
    out.append("== Recommendation (thresholds of the plan, section 4)")
    if time_ok is None and not chained:
        out.append("   not enough data for a recommendation")
        return "none"

    def verdict(mode_id, period=40):
        entries = [(k, v) for k, v in chained.items() if k[2] == mode_id and k[1] == period]
        if not entries:
            return None
        return all(ok for _, (m, ok) in entries)

    def baseline_ratio(mode_id, period=40):
        ratios = []
        for (path, p, mode, t), (m, ok) in chained.items():
            if mode != mode_id or p != period:
                continue
            base = chained.get((path, p, 0, 0))
            if base and base[0]["rms_err_rad"] > 0:
                ratios.append(m["rms_err_rad"] / base[0]["rms_err_rad"])
        return statistics.median(ratios) if ratios else None

    time_chain = verdict(3)
    comp_chain = verdict(2)
    time_track = baseline_ratio(3)
    out.append(f"   time based: arrival within tolerance for feasible moves: {time_ok}; "
               f"continuous at P=40 ms: {time_chain}; tracking error against no profile: {fmt(time_track, '{:.2f}')}x")
    out.append(f"   velocity based, computed: continuous at P=40 ms: {comp_chain}; "
               f"tracking error against no profile: {fmt(baseline_ratio(2), '{:.2f}')}x")
    if time_ok and time_chain and (time_track is None or time_track <= 1.0):
        choice = "time"
        out.append("   -> TIME BASED profile: configure the profile times once, write only the goal per control tick")
    elif comp_chain and (baseline_ratio(2) is None or baseline_ratio(2) <= 1.0):
        choice = "velocity-computed"
        out.append("   -> VELOCITY BASED profile with computed velocity and acceleration per goal (8 or 12 byte write)")
    else:
        choice = "software"
        out.append("   -> keep the SOFTWARE INTERPOLATION at 50 Hz, synchronise the joints in software "
                   "and use the servo profile only as a speed limit")
    return choice


def report(session, out):
    runs = [session["runs"][k] for k in sorted(session["runs"])]
    out.append(f"Session: mask {session['mask']}, {len(runs)} of {session['runs_planned']} runs in the log"
               f" (estimated {session['estimate'] / 60:.1f} min), finished: {session['done']}")
    aborted = [r for r in runs if r["status"] and r["status"].startswith("ABORT") and r["exp"] != 0]
    unfinished = [r for r in runs if r["status"] is None]
    for r in aborted:
        out.append(f"   run {r['id']} ABORTED: {r['status']}")
    for r in unfinished:
        out.append(f"   run {r['id']} is incomplete (no END line)")
    if session["bad_lines"]:
        out.append(f"   {session['bad_lines']} lines could not be parsed")
    out.append("")

    analyses = [analyze_run(r) for r in runs if r["samples"][0] or r["samples"][1] or r["samples"][2]]
    report_ids(session, out)
    out.append("")
    report_selftest(runs, out)
    out.append("")
    report_directions(analyses, out)
    out.append("")
    report_units(analyses, out, velocity_limit_units(session))
    out.append("")
    report_static_error(analyses, out)
    out.append("")
    time_ok = time_verdict(analyses, velocity_limit_radps(session), out)
    out.append("")
    chained = chained_report(analyses, out)
    out.append("")
    sync_report(analyses, out)
    out.append("")
    stop_report(analyses, out)
    out.append("")
    latency_report(analyses, out)
    out.append("")
    choice = recommend(time_ok, chained, out)
    return analyses, choice



# ---------------------------------------------------------------- plots (optional, needs matplotlib)

def _pyplot():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        return None


def _run_time_axis(run, trace, joint):
    """Servo time in ms relative to the first goal of the joint."""
    goals = goals_of(run, joint)
    t0 = trace.servo_time_of_host(goals[0]["host_us"]) if goals else trace.t[0]
    return [x - t0 for x in trace.t], t0


def plot_run(plt, run, path):
    """Position (with the ideal path and the goals), velocity and load of every joint of a run."""
    joints = [j for j in range(3) if run["samples"][j]]
    if not joints:
        return False
    fig, axes = plt.subplots(3, len(joints), figsize=(5.5 * len(joints), 8), squeeze=False, sharex="col")
    for col, joint in enumerate(joints):
        trace = Trace(run, joint)
        tt, t0 = _run_time_axis(run, trace, joint)
        centre = run["centre"][joint] if run["centre"] else 0
        ax = axes[0][col]
        ax.plot(tt, trace.pos, label="position")
        ax.plot(tt, trace.ptraj, "--", label="profile (trajectory)")
        ideal_t = list(range(int(min(tt)), int(max(tt)) + 1, max(1, run["sample_ms"])))
        ax.plot(ideal_t, [centre + path_offset_rad(run, joint, x) / PULSE_RAD for x in ideal_t], ":", color="gray",
                label="ideal path")
        for g in goals_of(run, joint):
            gt = trace.servo_time_of_host(g["host_us"]) - t0
            ax.axvline(gt, color="tab:red", alpha=0.25, linewidth=0.8)
        ax.set_title(f"{JOINT_NAMES[joint]}")
        ax.set_ylabel("pulses")
        ax.legend(fontsize=7, loc="best")
        ax = axes[1][col]
        ax.plot(tt, trace.vel, label="present velocity")
        ax.plot(tt, trace.vtraj, "--", label="profile velocity")
        ax.set_ylabel("rad/s")
        ax.legend(fontsize=7, loc="best")
        ax = axes[2][col]
        ax.plot(tt, trace.load)
        ax.set_ylabel("load %")
        ax.set_xlabel("ms from the first goal")
    fig.suptitle(f"run {run['id']} (E{run['exp']}): {describe(run)}   [{run['status']}]", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=80)
    plt.close(fig)
    return True


def plot_time_based(plt, analyses, path):
    """Profile duration against T for every distance (E3)."""
    rows = [a for a in analyses if a["run"]["exp"] == 3 and a["run"]["mode"] == 3]
    points = {}
    for a in rows:
        m = a["joints"].get(2)
        if m and m["profile_ms"] is not None:
            points.setdefault(round(m["delta_rad"], 2), []).append(
                (a["run"]["time"], m["profile_ms"] / a["run"]["time"], a["run"]["ramp"] * 4 == a["run"]["time"]))
    if not points:
        return False
    fig, ax = plt.subplots(figsize=(7, 5))
    for delta, values in sorted(points.items()):
        ax.plot([v[0] for v in values], [v[1] for v in values], "o", label=f"{delta} rad")
    ax.axhspan(1 - ARRIVAL_TOLERANCE, 1 + ARRIVAL_TOLERANCE, color="green", alpha=0.12, label="+-10 %")
    ax.set_xscale("log")
    ax.set_xlabel("T (ms)")
    ax.set_ylabel("measured profile time / T")
    ax.set_title("E3: time based profile, tibia")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)
    return True


def plot_chained(plt, analyses, out_dir, written):
    """Velocity of the modes on top of each other for every path and control period (E4)."""
    groups = {}
    for a in analyses:
        run = a["run"]
        if run["exp"] == 4 and a["joints"].get(2):
            groups.setdefault((PATH_NAMES[run["path"]], run["period"]), []).append(a)
    for (path_name, period), members in sorted(groups.items()):
        fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        for a in sorted(members, key=lambda a: (a["run"]["mode"], a["run"]["time"])):
            run = a["run"]
            trace = Trace(run, 2)
            tt, _ = _run_time_axis(run, trace, 2)
            label = MODE_NAMES[run["mode"]] + (f" T={run['time']}" if run["time"] else "")
            axes[0].plot(tt, trace.vel, label=label, linewidth=0.9)
            centre = run["centre"][2]
            axes[1].plot(tt, [(p - centre) * PULSE_RAD for p in trace.pos], label=label, linewidth=0.9)
        ref = members[0]["run"]
        ideal_t = list(range(0, ref["duration"] + 1, 2))
        axes[1].plot(ideal_t, [path_offset_rad(ref, 2, x) for x in ideal_t], ":", color="black", label="ideal path")
        axes[0].set_ylabel("present velocity (rad/s)")
        axes[1].set_ylabel("position (rad from centre)")
        axes[1].set_xlabel("ms from the first goal")
        axes[0].legend(fontsize=7, ncol=2)
        axes[0].set_title(f"E4: {path_name}, goal every {period} ms")
        fig.tight_layout()
        name = os.path.join(out_dir, f"e4_{path_name}_P{period}.png")
        fig.savefig(name, dpi=90)
        plt.close(fig)
        written.append(name)


def plot_sync(plt, analyses, path):
    """Progress of the three joints towards their goals, on top of each other (E5)."""
    rows = [a for a in analyses if a["run"]["exp"] == 5]
    if not rows:
        return False
    fig, axes = plt.subplots(len(rows), 1, figsize=(8, 3 * len(rows)), squeeze=False, sharex=True)
    for ax, a in zip(axes[:, 0], rows):
        run = a["run"]
        for joint in range(3):
            m = a["joints"].get(joint)
            if not m or not run["samples"][joint]:
                continue
            trace = Trace(run, joint)
            tt, _ = _run_time_axis(run, trace, joint)
            p0 = trace.pos[0]
            ax.plot(tt, [(p - p0) / m["delta_pulses"] for p in trace.pos], label=JOINT_NAMES[joint])
        ax.axhline(1.0, color="gray", linewidth=0.6)
        ax.set_ylabel("fraction of travel")
        ax.set_title(describe(run)[:60], fontsize=9)
        ax.legend(fontsize=7)
    axes[-1][0].set_xlabel("ms from the goal")
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)
    return True


def make_plots(session, analyses, out_dir, out):
    plt = _pyplot()
    if plt is None:
        out.append("plots need matplotlib: python3 -m venv tools/.venv && tools/.venv/bin/pip install -r "
                   "tools/requirements.txt, then run the script with tools/.venv/bin/python")
        return []
    os.makedirs(out_dir, exist_ok=True)
    written = []
    for run in session["runs"].values():
        name = os.path.join(out_dir, f"run_{run['id']:03d}.png")
        if plot_run(plt, run, name):
            written.append(name)
    for name, function in (("e3_profile_time.png", plot_time_based), ("e5_sync.png", plot_sync)):
        full = os.path.join(out_dir, name)
        if function(plt, analyses, full):
            written.append(full)
    plot_chained(plt, analyses, out_dir, written)
    out.append(f"{len(written)} plots written to {out_dir}")
    return written


def write_csv(analyses, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["run", "experiment", "mode", "path", "period", "time", "ramp", "velocity", "accel", "joint",
                    "metric", "value"])
        for a in analyses:
            run = a["run"]
            for joint, m in a["joints"].items():
                for key, value in (m or {}).items():
                    if key == "joint":
                        continue
                    w.writerow([run["id"], run["exp"], MODE_NAMES[run["mode"]], PATH_NAMES[run["path"]], run["period"],
                                run["time"], run["ramp"], run["velocity"], run["accel"], JOINT_NAMES[joint], key, value])


def show_run(session, run_id, out):
    run = session["runs"].get(run_id)
    if run is None:
        out.append(f"run {run_id} not in the log")
        return
    out.append(f"run {run_id}: {describe(run)}   status {run['status']}")
    a = analyze_run(run)
    for joint, m in a["joints"].items():
        out.append(f"  {JOINT_NAMES[joint]}: " + ", ".join(f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}"
                                                            for k, v in (m or {}).items() if k != "joint"))
    for joint in range(3):
        if run["samples"][joint]:
            trace = Trace(run, joint)
            out.append(f"  {JOINT_NAMES[joint]}: {trace.n} samples, load {min(trace.load):.1f}..{max(trace.load):.1f} %")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="console log of the Bench firmware")
    parser.add_argument("--csv", help="write the metrics of all runs to this file")
    parser.add_argument("--run", type=int, help="print the details of one run")
    parser.add_argument("--plots", metavar="DIR", help="write PNG plots to this directory (needs matplotlib)")
    parser.add_argument("--session", type=int, default=-1, help="which session of the log (default: the last)")
    args = parser.parse_args(argv)

    with open(args.log, "r", errors="replace") as f:
        sessions = parse(f)
    if not sessions:
        print("no BP,BEGIN line in the log: is this the log of the Bench firmware?", file=sys.stderr)
        return 1
    session = sessions[args.session]

    out = []
    if args.run is not None:
        show_run(session, args.run, out)
    else:
        analyses, _ = report(session, out)
        if args.csv:
            write_csv(analyses, args.csv)
            out.append(f"metrics written to {args.csv}")
        if args.plots:
            make_plots(session, analyses, args.plots, out)
    print("\n".join(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
