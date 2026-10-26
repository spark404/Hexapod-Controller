#!/usr/bin/env python3
"""Tests of tools/link_walk_report.py: python3 -m unittest tools/test_link_walk_report.py"""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import link_walk_report as r  # noqa: E402

T0 = 1000.0


def bus(t, rd=1500, ok=1500, streak=0, overruns=0, wr=300, wr_ok=300):
    return (f"{t:.3f} [INFO] [Bus] 30s: rd {rd} ok {ok} (us 4300/4500/4800) | wr {wr} ok {wr_ok} "
            f"(us 1940/1950/2030) | max fail streak {streak} | loop overruns {overruns}")


def console(*lines):
    return "\n".join(lines) + "\n"


def marks(*items):
    return "".join(f"{t:.3f} {label}\n" for t, label in items)


class Parse(unittest.TestCase):
    def test_bus_lines_and_servo_attribution(self):
        text = console(
            bus(T0 + 30, rd=1500, ok=1495, streak=1),
            f"{T0 + 30.02:.3f} [WARN] [Bus] reads without data per servo (leg*3+joint): 0:4 5:1",
            f"{T0 + 31:.3f} [INFO] [Main] something else",
            bus(T0 + 60))
        windows, fatal = r.parse_console(text)
        self.assertEqual(len(windows), 2)
        self.assertEqual((windows[0]["rd"], windows[0]["rd_ok"], windows[0]["streak"]), (1500, 1495, 1))
        self.assertEqual(windows[0]["servos"], {0: 4, 5: 1})
        self.assertEqual(windows[0]["rd_us"], [4300, 4500, 4800])
        self.assertEqual(windows[1]["servos"], {})
        self.assertEqual(fatal, [])

    def test_servo_line_belongs_to_the_window_just_before_it(self):
        text = console(bus(T0 + 30), f"{T0 + 50:.3f} [WARN] [Bus] reads without data per servo (leg*3+joint): 2:1")
        windows, _ = r.parse_console(text)
        self.assertEqual(windows[0]["servos"], {})          # 20 s later: not part of that window's report

    def test_fatal_markers(self):
        text = console(f"{T0:.3f} [ERROR] Limit alert triggered, leg 0, axis 0", f"{T0 + 1:.3f} === FATAL ERROR ===")
        self.assertEqual(len(r.parse_console(text)[1]), 2)

    def test_marks_pair_up_and_an_open_walk_ends_with_the_test(self):
        walks, start = r.parse_marks(marks((T0, "start"), (T0 + 5, "walk start"), (T0 + 8, "walk end"),
                                           (T0 + 20, "walk start"), (T0 + 25, "done")))
        self.assertEqual(walks, [(T0 + 5, T0 + 8), (T0 + 20, T0 + 25)])
        self.assertEqual(start, T0)

    def test_walking_share_of_a_window(self):
        windows, _ = r.parse_console(console(bus(T0 + 30), bus(T0 + 60)))
        walks = [(T0 + 20, T0 + 40)]                       # 10 s in the first window, 10 s in the second
        r.build(windows, walks, [])
        self.assertAlmostEqual(windows[0]["walk_s"], 10.0)
        self.assertAlmostEqual(windows[1]["walk_s"], 10.0)
        self.assertEqual(windows[0]["bouts"], 1)

    def test_link_events_are_counted_per_window(self):
        windows, _ = r.parse_console(console(bus(T0 + 30), bus(T0 + 60)))
        events = [{"t": T0 + 10, "event": "spi_error"}, {"t": T0 + 40, "event": "spi_error"},
                  {"t": T0 + 41, "event": "spi_error"},
                  {"t": T0 + 42, "event": "robot_status", "rejected": True, "state": "walking", "link_ok": True},
                  {"t": T0 + 43, "event": "robot_status", "rejected": False, "state": "walking", "link_ok": False}]
        r.build(windows, [], events)
        self.assertEqual((windows[0]["spi_errors"], windows[1]["spi_errors"]), (1, 2))
        self.assertEqual((windows[1]["rejected"], windows[1]["link_lost"]), (1, 1))


class Verdict(unittest.TestCase):
    def run_case(self, cons, mk, ev=None, **kw):
        windows, fatal = r.parse_console(cons)
        walks, _ = r.parse_marks(mk)
        r.build(windows, walks, ev or [])
        args = dict(max_failures=1, max_spi_errors=3, min_walk_s=5.0, min_windows=2)
        args.update(kw)
        return r.judge(windows, fatal, **args)

    WALKS = marks((T0 + 35, "walk start"), (T0 + 45, "walk end"), (T0 + 65, "walk start"), (T0 + 75, "walk end"))

    def test_pass(self):
        v, why = self.run_case(console(bus(T0 + 30), bus(T0 + 60), bus(T0 + 90)), self.WALKS)
        self.assertEqual((v, why), ("PASS", []))

    def test_one_failed_read_is_within_the_background_rate(self):
        v, _ = self.run_case(console(bus(T0 + 30), bus(T0 + 60, ok=1499, streak=1), bus(T0 + 90)), self.WALKS)
        self.assertEqual(v, "PASS")

    def test_failures_while_walking_fail(self):
        v, why = self.run_case(console(bus(T0 + 30), bus(T0 + 60, ok=1496, streak=1, overruns=4), bus(T0 + 90)), self.WALKS)
        self.assertEqual(v, "FAIL")
        self.assertIn("4 failed servo reads", why[0])

    def test_failures_while_standing_do_not_fail_the_walk(self):
        v, _ = self.run_case(console(bus(T0 + 30, ok=1490, streak=1), bus(T0 + 60), bus(T0 + 90)), self.WALKS)
        self.assertEqual(v, "PASS")

    def test_reads_in_a_row_fail(self):
        v, why = self.run_case(console(bus(T0 + 30), bus(T0 + 60, ok=1499, streak=3), bus(T0 + 90)), self.WALKS)
        self.assertEqual(v, "FAIL")
        self.assertIn("in a row", why[0])

    def test_write_failures_fail(self):
        v, _ = self.run_case(console(bus(T0 + 30), bus(T0 + 60, wr_ok=299), bus(T0 + 90)), self.WALKS)
        self.assertEqual(v, "FAIL")

    def test_spi_link_errors_fail(self):
        ev = [{"t": T0 + 50 + i, "event": "spi_error"} for i in range(4)]
        v, _ = self.run_case(console(bus(T0 + 30), bus(T0 + 60), bus(T0 + 90)), self.WALKS, ev)
        self.assertEqual(v, "FAIL")

    def test_a_fatal_error_fails(self):
        v, why = self.run_case(console(bus(T0 + 30), f"{T0 + 40:.3f} === FATAL ERROR ===", bus(T0 + 60), bus(T0 + 90)), self.WALKS)
        self.assertEqual(v, "FAIL")
        self.assertIn("fatal", why[0])

    def test_too_little_walking_is_inconclusive(self):
        v, why = self.run_case(console(bus(T0 + 30), bus(T0 + 60), bus(T0 + 90)),
                               marks((T0 + 35, "walk start"), (T0 + 38, "walk end")))
        self.assertEqual(v, "INCONCLUSIVE")

    def test_no_walking_at_all_is_inconclusive(self):
        v, _ = self.run_case(console(bus(T0 + 30), bus(T0 + 60)), marks((T0, "start")))
        self.assertEqual(v, "INCONCLUSIVE")


def block(t, volt="11.0 to 11.2 V (lowest: servo 3), temperature up to 36 C (servo 1)", roll="0.01", pitch="-0.01",
          accel="378 samples, none rejected", **bus_kw):
    """One report block of the firmware, as it prints it after the [Bus] line"""
    return console(
        bus(t, **bus_kw),
        f"{t + 0.02:.3f} [INFO] [Servos] input {volt}",
        f"{t + 0.03:.3f} [INFO] [Sensors] gyr -0.01  0.00  0.00 | acc  0.09  0.15  9.83 | mag -5.63 28.25 38.55 | "
        f"ekf roll {roll:>5} pitch {pitch:>5} yaw -1.79",
        f"{t + 0.04:.3f} [INFO] [Accel] {accel}")


class AttitudeAndHealth(unittest.TestCase):
    def windows(self, *blocks, walks=()):
        w, fatal = r.parse_console("".join(blocks))
        r.build(w, list(walks), [])
        return w, fatal

    def test_the_lines_of_a_block_belong_to_its_window(self):
        w, _ = self.windows(block(T0 + 30), block(T0 + 60, roll="-0.05", pitch="0.12",
                                                  volt="10.8 to 11.1 V (lowest: servo 7), temperature up to 41 C (servo 12)",
                                                  accel="4 of 380 samples rejected (length not 1 g +-2.5 m/s2), "
                                                        "last bad raw 1 2 3, 22.1 m/s2"))
        self.assertEqual((w[0]["roll"], w[0]["pitch"]), (0.01, -0.01))
        self.assertEqual(w[0]["volt"], (110, 112, 3))
        self.assertEqual(w[0]["temp"], (36, 1))
        self.assertEqual((w[0]["acc_rej"], w[0]["acc_total"]), (0, 378))
        self.assertEqual((w[1]["roll"], w[1]["pitch"]), (-0.05, 0.12))
        self.assertEqual(w[1]["volt"], (108, 111, 7))
        self.assertEqual(w[1]["temp"], (41, 12))
        self.assertEqual((w[1]["acc_rej"], w[1]["acc_total"]), (4, 380))

    def test_lines_far_from_a_bus_line_are_not_attached(self):
        text = console(bus(T0 + 30), f"{T0 + 40:.3f} [INFO] [Accel] 378 samples, none rejected")
        w, _ = r.parse_console(text)
        self.assertIsNone(w[0]["acc_rej"])

    def test_a_block_without_these_lines_still_parses(self):
        w, _ = self.windows(console(bus(T0 + 30)))
        self.assertEqual((w[0]["roll"], w[0]["volt"], w[0]["temp"], w[0]["acc_rej"]), (None, None, None, None))

    def judge(self, windows, fatal=()):
        return r.judge(windows, list(fatal), max_failures=1, max_spi_errors=3, min_walk_s=5.0, min_windows=1)

    def test_servo_supply_and_temperature_fail_the_run(self):
        walks = [(T0 + 35, T0 + 45)]
        ok, _ = self.windows(block(T0 + 30), block(T0 + 60), walks=walks)
        self.assertEqual(self.judge(ok)[0], "PASS")

        low, _ = self.windows(block(T0 + 30), block(T0 + 60, volt="9.2 to 11.0 V (lowest: servo 5), "
                                                                  "temperature up to 36 C (servo 1)"), walks=walks)
        verdict, why = self.judge(low)
        self.assertEqual(verdict, "FAIL")
        self.assertIn("9.2 V", why[0])

        hot, _ = self.windows(block(T0 + 30), block(T0 + 60, volt="11.0 to 11.1 V (lowest: servo 2), "
                                                                  "temperature up to 66 C (servo 9)"), walks=walks)
        verdict, why = self.judge(hot)
        self.assertEqual(verdict, "FAIL")
        self.assertIn("66 C", why[0])

    def test_notes_about_the_attitude_do_not_change_the_verdict(self):
        walks = [(T0 + 35, T0 + 45)]
        w, _ = self.windows(block(T0 + 30, roll="0.20", pitch="0.01"), block(T0 + 60, roll="0.30"),
                            block(T0 + 90, accel="12 of 380 samples rejected (length not 1 g +-2.5 m/s2), "
                                                 "last bad raw 1 2 3, 22.1 m/s2"), walks=walks)
        notes = r.attitude_notes(w)
        self.assertEqual(sum(1 for n in notes if n.startswith("standing, window")), 1)   # only the window at rest
        self.assertTrue(any("12 of" in n and "rejected" in n for n in notes))
        self.assertEqual(self.judge(w)[0], "PASS")

    def test_tables_and_summary(self):
        w, _ = self.windows(block(T0 + 30), block(T0 + 60, roll="0.10", pitch="-0.05"), walks=[(T0 + 40, T0 + 50)])
        table = r.fmt_attitude_table(w, T0)
        self.assertIn("roll deg", table)
        self.assertIn("5.7", table)                      # 0.10 rad
        self.assertIn("11.0-11.2", table)
        self.assertIn("0 of 378", table)
        text = r.summary(w)
        self.assertIn("roll up to 5.7 deg", text)


class Command(unittest.TestCase):
    def test_exit_codes_and_output(self):
        with tempfile.TemporaryDirectory() as d:
            def write(name, text):
                p = os.path.join(d, name)
                with open(p, "w") as f:
                    f.write(text)
                return p
            cons = write("console.log", console(bus(T0 + 30), bus(T0 + 60, ok=1496, streak=1), bus(T0 + 90)))
            mk = write("marks.txt", Verdict.WALKS)
            ev = write("events.jsonl", json.dumps({"t": T0 + 50, "event": "robot_status", "state": "walking",
                                                  "rejected": False, "link_ok": True}) + "\n")
            import io
            import contextlib
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                code = r.main(["--console", cons, "--marks", mk, "--events", ev])
            text = out.getvalue()
            self.assertEqual(code, 1)
            self.assertIn("FAIL", text)
            self.assertIn("walking", text)
            self.assertIn("robot states reported over the link: walking", text)
            empty = write("empty.log", "no bus lines here\n")
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(r.main(["--console", empty, "--marks", mk]), 2)


if __name__ == "__main__":
    unittest.main()
