#!/bin/sh
# Runs the host tests that need no hardware and no ARM toolchain:
#   - the servo bus diagnostics (Core/Src/bus_diag.c)
#   - the servo profile bench experiment script (Core/Src/bench_script.c)
#   - the analysis script of the bench experiment (tools/bench_profile_analyze.py), including a check
#     that its Python mirror of the ideal paths agrees with the C code
# Run from the Hexapod-Controller directory:  sh tests/run_host_tests.sh
# The dynamixel library has its own tests (cmake in ../dynamixel).
set -e
cd "$(dirname "$0")/.."
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
dnm=${DYNAMIXEL_INCLUDE:-../dynamixel/include}
flags="-std=c99 -Wall -Wextra -pedantic -Werror"

cc $flags -ICore/Inc -I"$dnm" tests/bus_diag_test.c Core/Src/bus_diag.c -o "$tmp/bus_diag_test"
"$tmp/bus_diag_test"

cc $flags -ICore/Inc tests/bench_script_test.c Core/Src/bench_script.c -lm -o "$tmp/bench_script_test"
"$tmp/bench_script_test"

# the plot tests need matplotlib, which is in the venv (python3 -m venv tools/.venv; see tools/requirements.txt)
if [ -x tools/.venv/bin/python ]; then py=tools/.venv/bin/python; else py=python3; fi
"$py" tools/test_bench_profile_analyze.py
