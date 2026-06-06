"""
telemetry_client.py — TelemetryClient and find_teensy_port helper.

Reads newline-delimited JSON telemetry from a serial port and fires a
callback for every key/value pair in each packet.
"""

import sys
import json
import glob
from typing import Callable

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed.  Run: pip install pyserial")
    sys.exit(1)


def find_teensy_port() -> str | None:
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


class TelemetryClient:
    """
    Reads newline-delimited JSON telemetry from a serial port and fires a
    callback for every key/value pair in each packet.

    Example
    -------
    def on_value(key, value):
        print(f"{key} = {value}")

    client = TelemetryClient(port="COM3")
    client.on_update(on_value)
    client.run()          # blocks; press Ctrl-C to stop
    """

    def __init__(self, port: str | None = None, baudrate: int = 115200):
        self.port = port or find_teensy_port()
        self.baudrate = baudrate
        self._callback: Callable[[str, object], None] | None = None
        self._debug_callback: Callable[[str], None] | None = None
        self.packet_count = 0

    def on_update(self, callback: Callable[[str, object], None]) -> None:
        """Register a callback invoked as callback(key, value) for each field."""
        self._callback = callback

    def on_debug(self, callback: Callable[[str], None]) -> None:
        """Register a callback for non-JSON lines (e.g. Serial.println output).
        If not set, debug lines are silently ignored."""
        self._debug_callback = callback

    def run(self) -> None:
        """Open the serial port and loop until Ctrl-C."""
        if not self.port:
            print("Error: no serial port found.  Specify one with --port.")
            sys.exit(1)

        print(f"Connecting to {self.port} at {self.baudrate} baud ...")
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=2)
        except serial.SerialException as e:
            print(f"Error opening port: {e}")
            sys.exit(1)

        ser.reset_input_buffer()
        print("Connected. Waiting for packets. Press Ctrl-C to stop.\n")

        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue  # timeout — keep waiting

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                try:
                    packet = json.loads(line)
                    self.packet_count += 1
                    if self._callback is not None and packet is not None:
                        for key, value in packet.items():
                            self._callback(key, value)
                except json.JSONDecodeError:
                    if self._debug_callback is not None:
                        self._debug_callback(line)

        except KeyboardInterrupt:
            pass
        finally:
            ser.close()
            print(f"\nDone. Received {self.packet_count} JSON packets.")
