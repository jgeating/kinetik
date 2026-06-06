#!/usr/bin/env python3
"""
Simple serial telemetry test receiver for TelemetrySerial.
Reads newline-delimited JSON packets from the robot and prints them.

Usage:
    python telemetry_client_serial_test.py               # auto-detect port
    python telemetry_client_serial_test.py --port COM3   # Windows
    python telemetry_client_serial_test.py --port /dev/cu.usbmodem14101  # macOS
    python telemetry_client_serial_test.py --baud 921600
"""

import sys
import json
import argparse
import glob
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed.  Run: pip install pyserial")
    sys.exit(1)


def find_teensy_port():
    """Return the first likely Teensy/Arduino serial port, or None."""
    # Prefer ports whose description mentions Teensy or Arduino
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "teensy" in desc or "arduino" in desc:
            return p.device

    # Fall back to usbmodem / usbserial patterns on macOS/Linux
    for pattern in ("/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]

    # On Windows, try the highest-numbered COM port
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if ports:
        return sorted(ports)[-1]

    return None


def print_packet(packet: dict, packet_num: int, elapsed: float):
    """Pretty-print a single telemetry packet."""
    print(f"\n--- Packet #{packet_num}  t={elapsed:.3f}s ---")
    for key, value in sorted(packet.items()):
        if isinstance(value, list):
            formatted = "[" + ", ".join(f"{v:.6g}" if isinstance(v, float) else str(v) for v in value) + "]"
        elif isinstance(value, float):
            formatted = f"{value:.6g}"
        else:
            formatted = str(value)
        print(f"  {key}: {formatted}")


def main():
    ap = argparse.ArgumentParser(description="Serial telemetry test receiver")
    ap.add_argument("--port",  default=None,   help="Serial port (auto-detected if omitted)")
    ap.add_argument("--baud",  default=115200,  type=int, help="Baud rate (default: 115200)")
    ap.add_argument("--count", default=0,       type=int, help="Stop after N packets (0 = run forever)")
    args = ap.parse_args()

    port = args.port or find_teensy_port()
    if not port:
        print("Error: no serial port found.  Specify one with --port.")
        sys.exit(1)

    print(f"Connecting to {port} at {args.baud} baud ...")
    try:
        ser = serial.Serial(port, args.baud, timeout=2)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        sys.exit(1)

    ser.reset_input_buffer()
    print("Connected. Waiting for packets. Press Ctrl-C to stop.\n")

    packet_count = 0
    start_time = None

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue  # timeout — keep waiting

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            # Record start time on first data
            if start_time is None:
                start_time = datetime.now()

            elapsed = (datetime.now() - start_time).total_seconds() if start_time else 0.0

            try:
                packet = json.loads(line)
                packet_count += 1
                print_packet(packet, packet_count, elapsed)
            except json.JSONDecodeError:
                # Plain-text debug output from Serial.println() — just print it
                print(f"[DBG] {line}")

            if args.count and packet_count >= args.count:
                break

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print(f"\nDone. Received {packet_count} JSON packets.")


if __name__ == "__main__":
    main()
