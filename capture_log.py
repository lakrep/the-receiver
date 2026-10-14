#!/usr/bin/env python3
"""Capture the-receiver serial log to output.log in the current directory.

Usage:  python capture_log.py COM3 [--baud 115200] [--out output.log]
Requires: pip install pyserial

The firmware already prefixes every line with [hh:mm:ss.ms], so lines are
written as received. The file is opened in append mode; Ctrl+C stops capture.
If the port disappears (USB unplugged, board reset) the script keeps retrying.
"""
import argparse
import sys
import time

import serial

RETRY_DELAY = 2.0


def capture(port, baud, out_path):
    with open(out_path, "a", encoding="utf-8", newline="\n") as out:
        print(f"Logging {port} @ {baud} -> {out_path} (Ctrl+C to stop)")
        while True:
            try:
                with serial.Serial(port, baud, timeout=1) as ser:
                    print(f"Connected to {port}")
                    while True:
                        raw = ser.readline()
                        if not raw:
                            continue
                        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                        out.write(line + "\n")
                        out.flush()
                        print(line)
            except serial.SerialException as e:
                print(f"Port error: {e}; retrying in {RETRY_DELAY:.0f}s", file=sys.stderr)
                time.sleep(RETRY_DELAY)


def main():
    ap = argparse.ArgumentParser(description="Capture the-receiver serial log")
    ap.add_argument("port", help="serial port name, e.g. COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="output.log")
    args = ap.parse_args()
    try:
        capture(args.port, args.baud, args.out)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
