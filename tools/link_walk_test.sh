#!/bin/bash
#
# Walking test with the servo bus diagnostics. Runs on the Pi, drives the robot over the SPI link
# with the fake Poom of poom-remote (no Bluetooth needed) and records what the firmware and the
# daemon say. Plan, safety and how to read the result: docs/link-walking-test.md.
#
#   tools/link_walk_test.sh          prints the plan and does nothing else
#   tools/link_walk_test.sh --go     runs it
#
# Settings (environment): WALK_S REST_S CYCLES STAND_S AFTER_S SPEED_MM_S OUT DAEMON CONFIG
#
# Needs next to this script: console_ts.py. The firmware must be running with the battery on;
# the robot walks forward, never turns (the ROTATING state has a fault, see the backlog).
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
WALK_S=${WALK_S:-3}            # one walk, seconds
REST_S=${REST_S:-7}            # standing between walks
CYCLES=${CYCLES:-9}            # walks
STAND_S=${STAND_S:-90}         # standing baseline before the walks (must stay under 120 s: power down)
AFTER_S=${AFTER_S:-35}         # standing after the walks
SPEED_MM_S=${SPEED_MM_S:-50}   # walk_speed_mm_s of the daemon config, only used for the distance below
OUT=${OUT:-$HOME/link-walk-$(date +%Y%m%d-%H%M%S)}
DAEMON=${DAEMON:-/usr/local/bin/poom-remote}
CONFIG=${CONFIG:-/etc/poom-remote/config.toml}

DIST_MM=$((WALK_S * CYCLES * SPEED_MM_S))
TOTAL_S=$((10 + 6 + STAND_S + CYCLES * (WALK_S + REST_S) + AFTER_S))

cat <<PLAN
Walking test with the bus diagnostics
  wake the robot with a short walk     (a few cm)
  stand ${STAND_S} s                         baseline with the SPI link running, 3 windows of 30 s
  ${CYCLES} x walk ${WALK_S} s, stand ${REST_S} s       $((CYCLES * WALK_S)) s of walking, about ${DIST_MM} mm forward in total
  stand ${AFTER_S} s                          closing baseline
  about ${TOTAL_S} s ($((TOTAL_S / 60)) min $((TOTAL_S % 60)) s); results in ${OUT}
Before you start: the robot has room for $((DIST_MM / 1000)).$(((DIST_MM % 1000) / 100)) m ahead or hangs free, the battery is on,
the firmware is running, nobody else uses /dev/spidev0.0 (the service poom-remote is stopped for the test
and started again at the end). Nothing in this test turns the robot.
PLAN

if [ "${1:-}" != "--go" ]; then
    echo; echo "Dry run: nothing was started. Add --go to run it."
    exit 0
fi

[ -x "$DAEMON" ] || { echo "no $DAEMON"; exit 1; }
[ -f "$HERE/console_ts.py" ] || { echo "no console_ts.py next to this script"; exit 1; }
mkdir -p "$OUT" || exit 1
MARKS="$OUT/marks.txt"; : > "$MARKS"
mark() { echo "$(date +%s.%N) $1" >> "$MARKS"; }
state() { grep '"event":"robot_status"' "$OUT/events.jsonl" 2>/dev/null | tail -n 1 | sed -n 's/.*"state":"\([a-z]*\)".*/\1/p'; }
fatal() { grep -q -E "FATAL ERROR|HARD FAULT|Limit alert" "$OUT/console.log" 2>/dev/null; }

CAP_PID=""; DAEMON_PID=""
cleanup() {
    exec 3>&- 2>/dev/null
    [ -n "$DAEMON_PID" ] && kill "$DAEMON_PID" 2>/dev/null
    [ -n "$CAP_PID" ] && kill "$CAP_PID" 2>/dev/null
    rm -f "$OUT/in.fifo"
    sudo systemctl start poom-remote
}
trap cleanup EXIT
abort() { echo "ABORT: $1"; mark "abort: $1"; echo "" >&3; echo quit >&3; exit 1; }

sudo systemctl stop poom-remote
stty -F /dev/ttyS0 115200 raw -echo cs8 -cstopb -parenb
python3 "$HERE/console_ts.py" "$OUT/console.log" & CAP_PID=$!
sleep 1

mkfifo "$OUT/in.fifo"
"$DAEMON" --source stdin --config "$CONFIG" --log "$OUT/events.jsonl" < "$OUT/in.fifo" > "$OUT/daemon.out" 2>&1 & DAEMON_PID=$!
exec 3> "$OUT/in.fifo"
mark "start"
sleep 3
[ -n "$(state)" ] || abort "no status from the robot over the SPI link (is the firmware running, is [spi] enabled?)"

# Wake: hold up until the robot walks (it stands up first if it was powered down), then release
mark "wake"; echo up >&3
for _ in $(seq 1 60); do [ "$(state)" = walking ] && break; sleep 0.5; done
echo "" >&3; mark "wake end"
[ "$(state)" = walking ] || [ "$(state)" = standing ] || abort "the robot did not reach walking (state: $(state))"
sleep 6

mark "phase standing baseline"; sleep "$STAND_S"
fatal && abort "fatal error in the console during the baseline"
[ "$(state)" = standing ] || abort "the robot is not standing after the baseline (state: $(state))"

for c in $(seq 1 "$CYCLES"); do
    mark "walk start"; echo up >&3; sleep "$WALK_S"
    mark "walk end";   echo "" >&3;  sleep "$REST_S"
    fatal && abort "fatal error in the console after walk $c"
    [ "$(state)" = standing ] || abort "the robot is not standing after walk $c (state: $(state))"
done

mark "phase standing after"; sleep "$AFTER_S"
mark "done"; echo quit >&3
wait "$DAEMON_PID" 2>/dev/null; DAEMON_PID=""

echo
echo "Done. Fetch and read it on the Mac:"
echo "  scp -r hexapod-controller.local:$(basename "$OUT") ."
echo "  python3 tools/link_walk_report.py --console $(basename "$OUT")/console.log --marks $(basename "$OUT")/marks.txt --events $(basename "$OUT")/events.jsonl"
