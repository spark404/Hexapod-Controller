#!/usr/bin/env python3
"""Tests of the bench experiment analysis (tools/bench_profile_analyze.py).

Run from the Hexapod-Controller directory:  python3 tools/test_bench_profile_analyze.py

The tests drive the analysis with logs from a simulated servo (a profile follower with velocity
and acceleration limits, a time based mode that derives them from the time asked for, a few ms
of position lag and the velocity limit register), in the exact line format of the firmware, and
check that the metrics find what was built into the simulation. One test compares the Python
mirror of the ideal paths with the C code.
"""
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_profile_analyze as bpa  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PRE = bpa.PRE_TRIGGER_MS
CENTRE = 2048


def make_run(run_id=1, exp=2, mode=1, path=0, joints=4, period=0, time=0, ramp=0, velocity=0, accel=0,
             delta=(0, 0, 500), path_param=0, sample_ms=1, duration=1000):
    run = bpa.new_run([run_id, exp, mode, path, joints, period, time, ramp, velocity, accel,
                       delta[0], delta[1], delta[2], path_param, sample_ms, duration])
    run["park"] = [CENTRE + round(bpa.path_offset_rad(run, j, -1) / bpa.PULSE_RAD) for j in range(3)]
    run["centre"] = [CENTRE] * 3
    return run


def goal_schedule(run, joint):
    """(path time ms, goal offset rad or None for 'the present position') like bench_script.c"""
    path = bpa.PATH_NAMES[run["path"]]
    off = lambda t: bpa.path_offset_rad(run, joint, t)
    if path == "step":
        return [(0, off(0))]
    if path in ("ramp", "sine"):
        n = max(1, run["duration"] // run["period"])
        return [(k * run["period"], off(k * run["period"] + run["period"])) for k in range(n)]
    if path == "stop":
        return [(0, off(0)), (run["path_param"], None)]
    return [(0, off(0)), (run["path_param"], off(run["path_param"]))]


class SimulatedServo:
    """A profile follower: re-plans from its present state whenever a goal arrives."""

    def __init__(self, x, vel_limit_units=265):
        self.x = float(x)
        self.v = 0.0                       # pulses per ms
        self.goal = float(x)
        self.vmax = math.inf               # pulses per ms
        self.amax = math.inf               # pulses per ms^2
        self.vlimit = vel_limit_units * bpa.VEL_UNIT / bpa.PULSE_RAD / 1000.0

    def set_goal(self, goal, vmax, amax):
        self.goal = float(goal)
        self.vmax = min(vmax, self.vlimit)
        self.amax = amax

    def step(self):
        d = self.goal - self.x
        if math.isinf(self.vmax) or math.isinf(self.amax):
            self.x, self.v = self.goal, 0.0
            return
        if abs(d) <= 0.5 and abs(self.v) <= self.amax:
            self.x, self.v = self.goal, 0.0
            return
        v_des = math.copysign(min(self.vmax, math.sqrt(2 * self.amax * abs(d))), d)
        dv = max(-self.amax, min(self.amax, v_des - self.v))
        self.v += dv
        self.x += self.v


def velocity_profile(run):
    v = run["velocity"] * bpa.VEL_UNIT / bpa.PULSE_RAD / 1000.0
    a = run["accel"] * bpa.ACC_UNIT / bpa.PULSE_RAD / 1e6
    return v, a


def time_profile(distance, time_ms, ramp_ms):
    """What a servo derives from 'arrive in time_ms with ramps of ramp_ms'."""
    ramp = max(ramp_ms, 1)
    vmax = abs(distance) / max(time_ms - ramp, 1)
    return vmax, vmax / ramp


def simulate(run, lag_ms=3, tick0=4000, vel_limit_units=265, sag_per_load=0.0, load_pct=0.0):
    """Lines of the firmware format for one run. sag_per_load > 0 models a joint that carries the leg: the
    position stays sag_per_load * load_pct pulses short of the profile, with that load."""
    joints = [j for j in range(3) if run["joints"] >> j & 1]
    servos = {j: SimulatedServo(run["park"][j], vel_limit_units) for j in joints}
    history = {j: [] for j in joints}
    events = {j: [(PRE + t, o) for t, o in goal_schedule(run, j)] for j in joints}
    sim_goals = []
    samples = []
    path = bpa.PATH_NAMES[run["path"]]

    for ms in range(0, PRE + run["duration"]):
        for j in joints:
            s = servos[j]
            while events[j] and events[j][0][0] <= ms:
                _, offset = events[j].pop(0)
                stopping = path == "stop" and offset is None
                goal = round(s.x) if offset is None else CENTRE + round(offset / bpa.PULSE_RAD)
                if stopping:
                    if run["mode"] == 3:
                        # a stop in 20 ms with ramps of 5 ms: from the present speed to zero in the ramp
                        vmax, amax = abs(s.v) + 1e-6, abs(s.v) / 5 + 1e-6
                    else:
                        vmax, amax = (400 * bpa.VEL_UNIT / bpa.PULSE_RAD / 1000.0,
                                      500 * bpa.ACC_UNIT / bpa.PULSE_RAD / 1e6)
                elif run["mode"] == 0:
                    vmax = amax = math.inf
                elif run["mode"] == 1:
                    vmax, amax = velocity_profile(run)
                else:                           # computed velocity and acceleration, or time based
                    vmax, amax = time_profile(goal - s.x, run["time"], run["ramp"])
                s.set_goal(goal, vmax, amax)
                sim_goals.append({"host_us": ms * 1000 + 600, "joint": j, "goal": goal,
                                  "velocity": 0, "accel": 0})
            s.step()
            history[j].append(s.x)
            if ms % run["sample_ms"] == 0:
                delayed = history[j][max(0, len(history[j]) - 1 - lag_ms)]
                vel_units = round(s.v * 1000 * bpa.PULSE_RAD / bpa.VEL_UNIT)
                resting = abs(s.v) == 0 and s.x == s.goal
                sag = sag_per_load * load_pct if resting else 0.0
                samples.append({"host_us": ms * 1000 + 400, "joint": j, "tick": (tick0 + ms) % bpa.TICK_WRAP,
                                "moving": int(abs(s.v) > 0 or s.x != s.goal),
                                "load": round(load_pct * 10) if resting else 0, "vel": vel_units,
                                "pos": round(delayed - sag), "vtraj": vel_units, "ptraj": round(s.x)})

    r = run
    lines = [f"BP,RUN,{r['id']},{r['exp']},{r['mode']},{r['path']},{r['joints']},{r['period']},{r['time']},"
             f"{r['ramp']},{r['velocity']},{r['accel']},{r['delta'][0]},{r['delta'][1]},{r['delta'][2]},"
             f"{r['path_param']},{r['sample_ms']},{r['duration']}",
             "BP,START,%d,%d,%d,%d,%d,%d,%d" % (r["id"], *r["park"], *r["centre"])]
    for g in sorted(sim_goals, key=lambda g: g["host_us"]):
        lines.append(f"BP,G,{r['id']},{g['host_us']},{g['joint']},{g['goal']},{g['velocity']},{g['accel']}")
    for s in samples:
        lines.append(f"BP,S,{r['id']},{s['host_us']},{s['joint']},{s['tick']},{s['moving']},{s['load']},"
                     f"{s['vel']},{s['pos']},{s['vtraj']},{s['ptraj']}")
    lines.append(f"BP,END,{r['id']},{len(samples)},OK")
    return lines


def session_lines(runs, vel_limit=265, noise=True):
    lines = ["[INFO] something before the experiment", f"BP,BEGIN,63,{len(runs)},600"]
    for sid in (16, 17, 18):
        lines += [f"BP,ID,{sid},model_number,1060", f"BP,ID,{sid},velocity_limit,{vel_limit}",
                  f"BP,ID,{sid},drive_mode,0"]
    for run in runs:
        lines += simulate(run, vel_limit_units=vel_limit)
    lines.append("BP,DONE,OK")
    if noise:
        lines.insert(5, "garbage \x00\x01 BP,S,oops")        # a damaged line must not stop the parser
    return lines


def analyze_one(run, **kw):
    session = bpa.parse(["BP,BEGIN,63,1,1"] + simulate(run, **kw))[0]
    return bpa.analyze_run(session["runs"][run["id"]])


class ParsingTests(unittest.TestCase):
    def test_roundtrip(self):
        run = make_run(velocity=100, accel=50)
        sessions = bpa.parse(session_lines([run]))
        self.assertEqual(len(sessions), 1)
        s = sessions[0]
        self.assertEqual(s["bad_lines"], 1)
        parsed = s["runs"][1]
        self.assertEqual(parsed["delta"], [0, 0, 500])
        self.assertEqual(parsed["park"], run["park"])
        self.assertEqual(parsed["status"], "OK")
        self.assertEqual(len(parsed["goals"]), 1)
        self.assertEqual(len(parsed["samples"][2]), PRE + 1000)
        self.assertEqual(s["ids"][16]["model_number"], "1060")
        self.assertEqual(s["done"], "OK")

    def test_prefixes_and_sessions(self):
        lines = ["[INFO] BP,BEGIN,65,1,10"] + ["BP,NOTE,hello"] + ["[DEBUG] BP,BEGIN,63,2,20"]
        sessions = bpa.parse(lines)
        self.assertEqual([s["mask"] for s in sessions], [65, 63])
        self.assertEqual(sessions[0]["notes"], ["hello"])

    def test_tick_wrap(self):
        samples = [{"tick": t} for t in (32760, 32765, 3, 8)]
        self.assertEqual(bpa.unwrap_ticks(samples), [32760, 32765, 32771, 32776])


class StepTests(unittest.TestCase):
    def test_velocity_profile_matches_the_trapezoid(self):
        run = make_run(velocity=100, accel=50, delta=(0, 0, 500))
        m = analyze_one(run)["joints"][2]
        t_exp, v_exp = bpa.expected_velocity_move(run, m["delta_rad"])
        self.assertAlmostEqual(m["profile_ms"] / 1000.0, t_exp, delta=0.06 * t_exp)
        self.assertAlmostEqual(m["peak_vtraj"], v_exp, delta=0.05 * v_exp)
        # the position settles inside its tolerance a little before the profile ends
        self.assertGreater(m["arrival_ms"], 0.8 * m["profile_ms"])
        self.assertLess(m["arrival_ms"], 1.1 * m["profile_ms"])
        # the profile starts when its velocity reaches 5 % of the peak: 0.12 rad/s at 18.7 rad/s^2 is 6.4 ms
        self.assertAlmostEqual(m["latency_traj_ms"], 0.05 * 2.4 / (50 * bpa.ACC_UNIT) * 1000, delta=1.5)

    def test_triangle_profile(self):
        run = make_run(velocity=400, accel=20, delta=(0, 0, 100))
        m = analyze_one(run)["joints"][2]
        t_exp, v_exp = bpa.expected_velocity_move(run, m["delta_rad"])
        self.assertLess(v_exp, run["velocity"] * bpa.VEL_UNIT)          # never reaches the velocity
        self.assertAlmostEqual(m["peak_vtraj"], v_exp, delta=0.08 * v_exp)

    def test_time_based_arrives_in_the_time_for_any_distance(self):
        for delta in (50, 100, 200):         # all possible in 80 ms for the servo
            run = make_run(exp=3, mode=3, time=80, ramp=20, delta=(0, 0, delta), duration=600)
            m = analyze_one(run)["joints"][2]
            self.assertAlmostEqual(m["profile_ms"], 80, delta=8, msg=f"delta {delta}")

    def test_time_based_that_is_too_fast_takes_longer(self):
        run = make_run(exp=3, mode=3, time=30, ramp=7, delta=(0, 0, 1000), duration=1000)
        m = analyze_one(run)["joints"][2]
        self.assertGreater(m["profile_ms"], 45)

    def test_a_joint_that_carries_the_leg_stops_short_of_its_goal(self):
        # the femur of the robot: 21 pulses short at a load of 12.3 %
        run = make_run(exp=1, mode=1, joints=2, velocity=50, accel=20, delta=(0, 100, 0), duration=800)
        session = bpa.parse(["BP,BEGIN,63,1,1"] + simulate(run, sag_per_load=1.7, load_pct=12.3))[0]
        a = bpa.analyze_run(session["runs"][1])
        m = a["joints"][1]
        self.assertAlmostEqual(m["static_error_pulses"], -21, delta=1.5)
        self.assertIsNotNone(m["arrival_ms"])               # judged against where it settles, not the goal
        self.assertLess(m["arrival_ms"], 400)
        points = bpa.rest_points(session["runs"][1], 1)
        self.assertEqual(len(points), 2)
        for load, error in points:
            self.assertAlmostEqual(load, 12.3, delta=0.1)
            self.assertAlmostEqual(error, -21, delta=1.5)
        out = []
        bpa.report_static_error([a], out)
        text = "\n".join(out)
        self.assertIn("femur", text)
        self.assertIn("-21 pulses", text)

    def test_unit_checks_use_triangles_and_trapezoids(self):
        runs = [make_run(run_id=1, exp=1, velocity=50, accel=20, delta=(0, 0, 100)),      # triangle
                make_run(run_id=2, exp=1, velocity=100, accel=50, delta=(0, 0, 500))]     # reaches the velocity
        out = []
        analyses = [analyze_one(r) for r in runs]
        bpa.report_units(analyses, out, 265)
        text = "\n".join(out)
        self.assertIn("velocity unit", text)
        self.assertIn("acceleration unit", text)
        self.assertEqual(text.count("confirmed"), 2)

    def test_mixed_models_are_pointed_out(self):
        lines = ["BP,BEGIN,65,0,0", "BP,ID,16,model_number,1090", "BP,ID,17,model_number,1090",
                 "BP,ID,18,model_number,1060"]
        out = []
        bpa.report_ids(bpa.parse(lines)[0], out)
        text = "\n".join(out)
        self.assertIn("2XL430-W250", text)
        self.assertIn("servo 18 = XL430-W250", text)
        self.assertIn("not all the same model", text)

    def test_no_profile_jumps(self):
        run = make_run(exp=4, mode=0, delta=(0, 0, 500), duration=300)
        m = analyze_one(run)["joints"][2]
        self.assertLess(m["arrival_ms"], 10)             # the position is there after the lag of the servo
        self.assertLess(m["latency_traj_ms"], 5)


class ChainedTests(unittest.TestCase):
    def ramp(self, period, time):
        return make_run(exp=4, mode=3, path=1, period=period, time=time, ramp=max(1, time // 4),
                        delta=(0, 0, 600), path_param=600, duration=1300)

    def test_a_profile_shorter_than_the_period_stops_and_goes(self):
        m = analyze_one(self.ramp(100, 80))["joints"][2]
        self.assertLess(m["min_segment_ratio"], 0.3)
        self.assertGreater(m["dip_fraction"], 0.1)

    def test_a_profile_one_and_a_half_times_the_period_is_continuous(self):
        # (the simulated servo plans again from where it is at every goal, 1.2 P is borderline)
        m = analyze_one(self.ramp(100, 150))["joints"][2]
        self.assertGreaterEqual(m["min_segment_ratio"], bpa.CONTINUOUS_MIN_RATIO)
        self.assertLess(m["dip_fraction"], 0.1)

    def test_ripple_tells_the_two_apart(self):
        stop_go = analyze_one(self.ramp(100, 80))["joints"][2]["ripple"]
        smooth = analyze_one(self.ramp(100, 150))["joints"][2]["ripple"]
        self.assertGreater(stop_go, 4 * smooth)

    def test_lag_is_found(self):
        # the same path with the servo 40 ms late: the lag the analysis finds follows
        slow = analyze_one(self.ramp(40, 48), lag_ms=40)["joints"][2]
        quick = analyze_one(self.ramp(40, 48), lag_ms=3)["joints"][2]
        self.assertAlmostEqual(slow["lag_ms"] - quick["lag_ms"], 37, delta=4)
        self.assertLess(slow["rms_err_rad"], slow["rms_err_zero_lag_rad"])

    def test_sine(self):
        run = make_run(exp=4, mode=3, path=2, period=40, time=48, ramp=12, delta=(0, 0, 500), path_param=1000,
                       duration=1500)
        m = analyze_one(run)["joints"][2]
        self.assertLess(m["rms_err_rad"], 0.05)
        self.assertLessEqual(m["ripple"], bpa.RIPPLE_LIMIT)


class SyncAndStopTests(unittest.TestCase):
    def test_time_based_joints_arrive_together_velocity_based_do_not(self):
        sync = make_run(exp=5, mode=3, joints=7, time=80, ramp=20, delta=(40, 100, 150), sample_ms=2, duration=600)
        loose = make_run(run_id=2, exp=5, mode=1, joints=7, velocity=100, accel=100, delta=(40, 100, 150),
                         sample_ms=2, duration=600)
        arrivals = lambda run: [analyze_one(run)["joints"][j]["arrival_ms"] for j in range(3)]
        a = arrivals(sync)
        b = arrivals(loose)
        self.assertLess(max(a) - min(a), 15)
        self.assertGreater(max(b) - min(b), 30)

    def test_servo_side_stop(self):
        run = make_run(exp=6, mode=3, path=3, time=300, ramp=75, delta=(0, 0, 800), path_param=150, duration=1200)
        m = analyze_one(run)["joints"][2]
        self.assertGreater(m["speed_at_event"], 0.5)
        self.assertIsNotNone(m["stop_ms"])
        self.assertLess(m["stop_ms"], 80)

    def test_reversal(self):
        run = make_run(exp=6, mode=3, path=4, time=300, ramp=75, delta=(0, 0, 800), path_param=150, duration=1200)
        m = analyze_one(run)["joints"][2]
        self.assertIsNotNone(m["flip_ms"])
        self.assertIsNotNone(m["back_ms"])
        self.assertLess(m["flip_ms"], 150)


class ReportTests(unittest.TestCase):
    def full_session(self, time_based_is_continuous):
        runs = []
        rid = 0

        def add(**kw):
            nonlocal rid
            rid += 1
            runs.append(make_run(run_id=rid, **kw))

        add(exp=1, mode=1, velocity=100, accel=50, delta=(0, 0, 500))
        for t, tr, d in ((60, 15, 200), (60, 15, 500), (100, 25, 500)):
            add(exp=3, mode=3, time=t, ramp=tr, delta=(0, 0, d), duration=600)
        factor = 1.5 if time_based_is_continuous else 0.8
        for period in (40,):
            for path, delta, param, dur in ((1, 600, 600, 1300), (2, 500, 1000, 1500)):
                add(exp=4, mode=0, path=path, period=period, delta=(0, 0, delta), path_param=param, duration=dur)
                t = round(period * factor)
                add(exp=4, mode=3, path=path, period=period, time=t, ramp=max(1, t // 4), delta=(0, 0, delta),
                    path_param=param, duration=dur)
                add(exp=4, mode=2, path=path, period=period, time=t, ramp=max(1, t // 4), delta=(0, 0, delta),
                    path_param=param, duration=dur)
        add(exp=5, mode=3, joints=7, time=80, ramp=20, delta=(40, 100, 150), sample_ms=2, duration=600)
        add(exp=6, mode=3, path=3, time=300, ramp=75, delta=(0, 0, 800), path_param=150, duration=1200)
        add(exp=6, mode=3, path=4, time=300, ramp=75, delta=(0, 0, 800), path_param=150, duration=1200)
        return runs

    def report(self, runs):
        session = bpa.parse(session_lines(runs, noise=False))[0]
        out = []
        analyses, choice = bpa.report(session, out)
        return "\n".join(out), choice, analyses

    def test_report_recommends_time_based_when_it_chains(self):
        text, choice, _ = self.report(self.full_session(True))
        self.assertEqual(choice, "time", text)
        for heading in ("E1: identification", "E3:", "E4:", "E5:", "E6:", "E7:", "Recommendation"):
            self.assertIn(heading, text)
        self.assertIn("criterion 1", text)
        self.assertIn("PASS", text)

    def test_report_does_not_recommend_time_based_when_it_stops_and_goes(self):
        text, choice, _ = self.report(self.full_session(False))
        self.assertNotEqual(choice, "time", text)

    def test_empty_session(self):
        out = []
        session = bpa.parse(["BP,BEGIN,63,0,0"])[0]
        _, choice = bpa.report(session, out)
        self.assertEqual(choice, "none")

    def test_aborted_run_is_reported(self):
        run = make_run(velocity=100, accel=50)
        lines = session_lines([run], noise=False)
        lines = [l.replace("BP,END,1,", "BP,END,1,") for l in lines]
        lines = [("BP,END,1,%d,ABORT:out of range" % len(run["samples"][2])) if l.startswith("BP,END") else l for l in lines]
        session = bpa.parse(lines)[0]
        out = []
        bpa.report(session, out)
        self.assertIn("ABORTED", "\n".join(out))

    def test_dry_run_output(self):
        # E1 direction runs, then the E0 self test that the guard aborts
        runs = [make_run(run_id=i + 1, exp=1, mode=1, joints=1 << i, velocity=50, accel=20,
                         delta=tuple(100 if j == i else 0 for j in range(3)), duration=800) for i in range(3)]
        runs.append(make_run(run_id=4, exp=0, mode=1, velocity=100, accel=50, delta=(0, 0, 100), duration=400))
        lines = session_lines(runs, noise=False)
        lines = [l.replace("BP,END,4,", "BP,END,4,") for l in lines]
        lines = [("BP,END,4,57,ABORT:out of range" if l.startswith("BP,END,4,") else l) for l in lines]
        session = bpa.parse(lines)[0]
        out = []
        bpa.report(session, out)
        text = "\n".join(out)
        self.assertIn("E0: safety self test", text)
        self.assertIn("PASS, the guard stopped the run", text)
        self.assertNotIn("ABORTED", text)           # the planned abort is not an error
        self.assertEqual(text.count("-> ok"), 3)    # three joints moved as commanded

    def test_failed_self_test_and_wrong_direction(self):
        runs = [make_run(run_id=1, exp=1, mode=1, joints=4, velocity=50, accel=20, delta=(0, 0, 100), duration=800),
                make_run(run_id=2, exp=0, mode=1, velocity=100, accel=50, delta=(0, 0, 100), duration=400)]
        lines = session_lines(runs, noise=False)          # run 2 ends with OK: the guard did not abort
        # a joint that moves the wrong way: mirror the positions of run 1 around its start pose
        park = runs[0]["park"][2]
        mirrored = []
        for l in lines:
            f = l.split(",")
            if f[:3] == ["BP", "S", "1"]:
                f[9] = str(2 * park - int(f[9]))
                f[11] = str(2 * park - int(f[11]))
                l = ",".join(f)
            mirrored.append(l)
        session = bpa.parse(mirrored)[0]
        out = []
        bpa.report(session, out)
        text = "\n".join(out)
        self.assertIn("FAIL, the guard did not abort", text)
        self.assertIn("NOT AS COMMANDED", text)

    def test_cli_and_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, "tio-output.log")
            csv_path = os.path.join(tmp, "metrics.csv")
            with open(log, "w") as f:
                f.write("\n".join(session_lines(self.full_session(True))) + "\n")
            from io import StringIO
            from contextlib import redirect_stdout
            buf = StringIO()
            with redirect_stdout(buf):
                rc = bpa.main([log, "--csv", csv_path])
            self.assertEqual(rc, 0)
            self.assertIn("TIME BASED", buf.getvalue())
            self.assertTrue(os.path.getsize(csv_path) > 100)
            buf = StringIO()
            with redirect_stdout(buf):
                self.assertEqual(bpa.main([log, "--run", "1"]), 0)
            self.assertIn("run 1", buf.getvalue())


def _has_matplotlib():
    try:
        import matplotlib  # noqa: F401
        return True
    except ImportError:
        return False


class PlotTests(unittest.TestCase):
    def test_without_matplotlib_the_hint_is_printed(self):
        session = bpa.parse(session_lines([make_run(velocity=100, accel=50)], noise=False))[0]
        out = []
        original = bpa._pyplot
        bpa._pyplot = lambda: None
        try:
            self.assertEqual(bpa.make_plots(session, [], tempfile.gettempdir(), out), [])
        finally:
            bpa._pyplot = original
        self.assertIn("matplotlib", out[0])

    @unittest.skipUnless(_has_matplotlib(), "matplotlib is not installed (tools/.venv)")
    def test_plots_are_written(self):
        runs = ReportTests().full_session(True)
        session = bpa.parse(session_lines(runs, noise=False))[0]
        analyses = [bpa.analyze_run(r) for r in session["runs"].values()]
        with tempfile.TemporaryDirectory() as tmp:
            out = []
            written = bpa.make_plots(session, analyses, os.path.join(tmp, "plots"), out)
            names = sorted(os.path.basename(w) for w in written)
            self.assertIn("run_001.png", names)
            self.assertIn("e3_profile_time.png", names)
            self.assertIn("e5_sync.png", names)
            self.assertTrue(any(n.startswith("e4_ramp_P40") for n in names), names)
            for w in written:
                self.assertGreater(os.path.getsize(w), 2000, w)
            self.assertEqual(len(written), len(runs) + 2 + 2)       # runs, E3, E5 and ramp + sine of P=40
            self.assertIn(f"{len(written)} plots written", out[0])


@unittest.skipUnless(shutil.which("cc"), "no C compiler")
class ParityWithTheFirmwareScript(unittest.TestCase):
    """The Python mirror of the ideal paths must give what bench_script.c gives."""

    def test_paths_and_goals_match(self):
        with tempfile.TemporaryDirectory() as tmp:
            exe = os.path.join(tmp, "dump")
            subprocess.run(["cc", "-std=c99", "-I" + os.path.join(REPO, "Core/Inc"),
                            os.path.join(REPO, "tests/bench_script_dump.c"),
                            os.path.join(REPO, "Core/Src/bench_script.c"), "-lm", "-o", exe], check=True)
            out = subprocess.run([exe], check=True, capture_output=True, text=True).stdout.splitlines()

        runs = {}
        checked = 0
        for line in out:
            f = line.split(",")
            if f[0] == "RUN":
                run = bpa.new_run(f[1:])
                runs[run["id"]] = run
            elif f[0] == "OFF":
                run = runs[int(f[1])]
                py = bpa.path_offset_rad(run, int(f[2]), int(f[3]))
                self.assertAlmostEqual(py, int(f[4]) / 1e6, delta=1e-5, msg=line)
                checked += 1
            elif f[0] == "GOAL":
                run = runs[int(f[1])]
                j, k, t, c = int(f[2]), int(f[3]), int(f[4]), int(f[5]) / 1e6
                schedule = goal_schedule(run, j)
                if bpa.PATH_NAMES[run["path"]] in ("ramp", "sine"):
                    n = max(1, run["duration"] // run["period"])
                    self.assertEqual(len(schedule), n)
                self.assertEqual(schedule[k][0], t, msg=line)
                if schedule[k][1] is not None:
                    self.assertAlmostEqual(schedule[k][1], c, delta=1e-5, msg=line)
                checked += 1
        self.assertGreater(len(runs), 80)
        self.assertGreater(checked, 5000)


if __name__ == "__main__":
    unittest.main(verbosity=1)
