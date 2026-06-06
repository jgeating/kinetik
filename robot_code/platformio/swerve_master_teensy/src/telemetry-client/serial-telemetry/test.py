#!/usr/bin/env python3
"""
Serial telemetry receiver — live CoP (centre-of-pressure) spatial display.
Shows swerve/cop_x and swerve/cop_y as a moving dot on a 2-D plot.

Usage:
    python test.py               # auto-detect port
    python test.py --port COM3   # Windows
    python test.py --port /dev/cu.usbmodem14101  # macOS
    python test.py --baud 921600
"""

import argparse
import sys
import threading
import time
from collections import deque

import pyqtgraph as pg
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel

from telemetry_client import TelemetryClient

TRAIL_SECONDS = 5  # how many seconds of history to show in the trail


class CopWindow(QMainWindow):
    def __init__(self, client: TelemetryClient):
        super().__init__()
        self.setWindowTitle("Centre of Pressure")
        self.resize(600, 640)

        # Each entry is (timestamp, x, y); unbounded — trimmed by age in _refresh
        self._trail_pts: deque[tuple[float, float, float]] = deque()
        self._pending: dict[str, float] = {}
        self._lock = threading.Lock()

        # ── Layout ──────────────────────────────────────────────────
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self._status = QLabel("Waiting for data…")
        layout.addWidget(self._status)

        self._plot = pg.PlotWidget()
        self._plot.setLabel("bottom", "X")
        self._plot.setLabel("left", "Y")
        self._plot.setTitle("swerve/cop")
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._plot.setAspectLocked(True)
        self._plot.setXRange(-1, 1, padding=0)
        self._plot.setYRange(-1, 1, padding=0)
        self._plot.disableAutoRange()

        # Origin crosshair
        self._plot.addItem(pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen("w", width=1, style=pg.QtCore.Qt.DashLine)))
        self._plot.addItem(pg.InfiniteLine(pos=0, angle=0,  pen=pg.mkPen("w", width=1, style=pg.QtCore.Qt.DashLine)))

        layout.addWidget(self._plot)

        # Trail (semi-transparent line of past positions for combined CoP)
        self._trail = self._plot.plot([], [], pen=pg.mkPen("c", width=1), alpha=0.4)
        # Combined CoP dot (red)
        self._dot   = self._plot.plot([], [], pen=None, symbol="o",
                                      symbolSize=14, symbolBrush="r")
        # Left CoP dot (green)
        self._dot_l = self._plot.plot([], [], pen=None, symbol="o",
                                      symbolSize=10, symbolBrush="g")
        # Right CoP dot (yellow)
        self._dot_r = self._plot.plot([], [], pen=None, symbol="o",
                                      symbolSize=10, symbolBrush="y")

        # ── Wire up telemetry ────────────────────────────────────────
        client.on_update(self._on_update)
        self._thread = threading.Thread(target=client.run, daemon=True)
        self._thread.start()

        # ── Refresh timer ────────────────────────────────────────────
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(50)  # 20 Hz

    # Called from the serial thread — keep it fast and lock-safe
    def _on_update(self, key: str, value: object) -> None:
        if key in ("swerve/cop_x", "swerve/cop_y",
                   "swerve/cop_x_l", "swerve/cop_y_l",
                   "swerve/cop_x_r", "swerve/cop_y_r"):
            with self._lock:
                self._pending[key] = float(value)

    # Called from the Qt main thread via QTimer
    def _refresh(self) -> None:
        with self._lock:
            pending = self._pending.copy()
            self._pending.clear()

        now = time.monotonic()

        x   = pending.get("swerve/cop_x")
        y   = pending.get("swerve/cop_y")
        x_l = pending.get("swerve/cop_x_l")
        y_l = pending.get("swerve/cop_y_l")
        x_r = pending.get("swerve/cop_x_r")
        y_r = pending.get("swerve/cop_y_r")

        if x is not None and y is not None:
            self._trail_pts.append((now, x, y))

        # Drop points older than TRAIL_SECONDS
        cutoff = now - TRAIL_SECONDS
        while self._trail_pts and self._trail_pts[0][0] < cutoff:
            self._trail_pts.popleft()

        if not self._trail_pts:
            return

        xs = [p[1] for p in self._trail_pts]
        ys = [p[2] for p in self._trail_pts]
        self._trail.setData(xs, ys)
        self._dot.setData([xs[-1]], [ys[-1]])

        self._dot_l.setData([x_l] if x_l is not None else [],
                             [y_l] if y_l is not None else [])
        self._dot_r.setData([x_r] if x_r is not None else [],
                             [y_r] if y_r is not None else [])

        status = f"CoP  x={xs[-1]:.3f}  y={ys[-1]:.3f}"
        if x_l is not None and y_l is not None:
            status += f"   L({x_l:.3f}, {y_l:.3f})"
        if x_r is not None and y_r is not None:
            status += f"   R({x_r:.3f}, {y_r:.3f})"
        self._status.setText(status)

def _on_debug(line: str) -> None:
    print(f"\n[DBG] {line}")

def main():
    ap = argparse.ArgumentParser(description="CoP spatial display over serial telemetry")
    ap.add_argument("--port", default=None, help="Serial port (auto-detected if omitted)")
    ap.add_argument("--baud", default=115200, type=int, help="Baud rate (default: 115200)")
    args = ap.parse_args()

    client = TelemetryClient(port=args.port, baudrate=args.baud)

    client.on_debug(_on_debug)

    app = QApplication(sys.argv)
    window = CopWindow(client)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
