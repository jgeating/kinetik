#!/usr/bin/env python3
"""
Simple serial telemetry test receiver for TelemetrySerial.
Reads newline-delimited JSON packets from the robot and prints them.

Usage:
    python test.py               # auto-detect port
    python test.py --port COM3   # Windows
    python test.py --port /dev/cu.usbmodem14101  # macOS
    python test.py --baud 921600
"""

import argparse
from telemetry_client import TelemetryClient


def _default_callback(key: str, value: object) -> None:
    """Pretty-prints each key/value to stdout."""
    if isinstance(value, list):
        formatted = "[" + ", ".join(f"{v:.6g}" if isinstance(v, float) else str(v) for v in value) + "]"
    elif isinstance(value, float):
        formatted = f"{value:.6g}"
    else:
        formatted = str(value)
    print(f"  {key}: {formatted}")

def _debug_callback(line: str) -> None:
    print(f"[DBG] {line}")


def main():
    ap = argparse.ArgumentParser(description="Serial telemetry test receiver")
    ap.add_argument("--port", default=None, help="Serial port (auto-detected if omitted)")
    ap.add_argument("--baud", default=115200, type=int, help="Baud rate (default: 115200)")
    args = ap.parse_args()

    client = TelemetryClient(port=args.port, baudrate=args.baud)
    client.on_update(_default_callback)
    # client.on_debug(_debug_callback)
    client.run()


if __name__ == "__main__":
    main()
