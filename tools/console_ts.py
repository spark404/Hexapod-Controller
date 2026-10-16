#!/usr/bin/env python3
"""Timestamped capture of the firmware console on the Pi (UART on /dev/ttyS0).

    stty -F /dev/ttyS0 115200 raw -echo cs8 -cstopb -parenb
    python3 console_ts.py console.log

Every line is written as "<unix time> <text>", flushed per line. Runs until it is killed.
Used by tools/link_walk_test.sh and read by tools/link_walk_report.py. A plain `cat` loses the
time, and `tio` does not run detached without a terminal.
"""
import sys
import time

if len(sys.argv) != 2:
    sys.exit(__doc__)
out = open(sys.argv[1], "w", buffering=1)
with open("/dev/ttyS0", "rb", buffering=0) as port:
    buf = b""
    while True:
        chunk = port.read(256)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.replace(b"\r", b"").replace(b"\0", b"").decode(errors="replace")
            out.write("%.3f %s\n" % (time.time(), text))
