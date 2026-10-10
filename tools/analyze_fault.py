#!/usr/bin/env python3
"""
analyze_fault.py — Symbolize ARM Cortex-M fault dumps using addr2line.

Usage:
    python3 analyze_fault.py [fault_dump.txt] [--elf path/to/firmware.elf]

If no fault file is given, reads from stdin.
Default ELF: cmake-build-debug/Hexapod.elf
"""

import re
import sys
import subprocess
import argparse
from pathlib import Path

DEFAULT_ELF = "cmake-build-debug/Hexapod.elf"
ADDR2LINE   = "arm-none-eabi-addr2line"

REGISTER_NAMES = {
    "R0", "R1", "R2", "R3", "R4", "R5", "R6", "R7",
    "R8", "R9", "R10", "R11", "R12", "LR", "PC", "PSR",
}

# Registers that are likely code addresses worth symbolising
CODE_REGISTERS = {"LR", "PC"}


def addr2line(elf: Path, address: int) -> str:
    """Return 'function\\nfile:line' for an address, or empty string on failure."""
    # Strip Thumb bit so addr2line resolves correctly
    addr = address & ~1
    try:
        result = subprocess.run(
            [ADDR2LINE, "-e", str(elf), "-f", "-C", f"0x{addr:08x}"],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return ""


def format_symbol(raw: str) -> str:
    """Format the two-line addr2line output into a single readable string."""
    lines = raw.splitlines()
    if len(lines) >= 2:
        func, loc = lines[0], lines[1]
        if func == "??":
            return f"<unknown>  ({loc})"
        return f"{func}  ({loc})"
    return raw


def parse_fault(text: str) -> dict:
    """Extract sections, registers, and backtraces from a fault dump."""
    sections = []
    current   = None

    section_header = re.compile(r"^=== (.+?) ===\s*$")
    register_line  = re.compile(r"^\s*(R\d+|LR|PC|PSR)\s*=\s*(0x[0-9A-Fa-f]+)")
    backtrace_line  = re.compile(r"^\s*#(\d+)\s+(0x[0-9A-Fa-f]+)")
    message_line   = re.compile(r"^(?!=)(.+)$")  # non-section-header non-empty lines

    for line in text.splitlines():
        m = section_header.match(line)
        if m:
            current = {"title": m.group(1), "messages": [], "registers": {}, "backtrace": []}
            sections.append(current)
            continue

        if current is None:
            continue

        m = register_line.match(line)
        if m:
            current["registers"][m.group(1)] = int(m.group(2), 16)
            continue

        m = backtrace_line.match(line)
        if m:
            current["backtrace"].append((int(m.group(1)), int(m.group(2), 16)))
            continue

        stripped = line.strip()
        if stripped and stripped != "System halted.":
            current["messages"].append(stripped)

    return sections


def symbolise_section(section: dict, elf: Path) -> None:
    title     = section["title"]
    messages  = section["messages"]
    registers = section["registers"]
    backtrace = section["backtrace"]

    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")

    if messages:
        for msg in messages:
            print(f"  {msg}")
        print()

    if registers:
        print("  Registers:")
        for name, val in registers.items():
            line = f"    {name:<4} = 0x{val:08x}"
            if name in CODE_REGISTERS:
                sym = addr2line(elf, val)
                if sym:
                    line += f"  =>  {format_symbol(sym)}"
            print(line)
        print()

    if backtrace:
        print("  Backtrace:")
        for idx, addr in backtrace:
            sym  = addr2line(elf, addr)
            desc = f"  #{idx:02d}  0x{addr:08x}"
            if sym:
                desc += f"  =>  {format_symbol(sym)}"
            else:
                desc += "  =>  <unknown>"
            print(desc)
        print()


def main() -> None:
    parser = argparse.ArgumentParser(description="Symbolize ARM Cortex-M fault dumps.")
    parser.add_argument("fault_file", nargs="?", help="Fault dump text file (default: stdin)")
    parser.add_argument("--elf", default=DEFAULT_ELF, help=f"ELF file (default: {DEFAULT_ELF})")
    args = parser.parse_args()

    elf = Path(args.elf)
    if not elf.exists():
        print(f"ERROR: ELF not found: {elf}", file=sys.stderr)
        sys.exit(1)

    if args.fault_file:
        text = Path(args.fault_file).read_text()
    else:
        if sys.stdin.isatty():
            print("Paste fault dump, then press Ctrl+D:")
        text = sys.stdin.read()

    sections = parse_fault(text)
    if not sections:
        print("No fault sections found in input.", file=sys.stderr)
        sys.exit(1)

    print(f"ELF: {elf.resolve()}")
    for section in sections:
        symbolise_section(section, elf)


if __name__ == "__main__":
    main()
