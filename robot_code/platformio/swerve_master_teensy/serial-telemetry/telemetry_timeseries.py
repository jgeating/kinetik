#!/usr/bin/env python3
"""Telemetry viewer: CoP spatial + time-series plots."""

import argparse, sys, threading, time, json
from collections import deque
import pyqtgraph as pg
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, QTabWidget
from telemetry_client import TelemetryClient

COLORS = ["red", "cyan", "green", "yellow", "magenta", "white", "blue", "orange"]


class CopTab(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        self._status = QLabel("Waiting...")
        layout.addWidget(self._status)
        self._plot = pg.PlotWidget()
        self._plot.setLabel("bottom", "X")
        self._plot.setLabel("left", "Y")
        self._plot.setTitle("CoP")
        self._plot.showGrid(alpha=0.3)
        self._plot.setAspectLocked(True)
        self._plot.setXRange(-1, 1, padding=0)
        self._plot.setYRange(-1, 1, padding=0)
        self._plot.disableAutoRange()
        self._plot.addItem(pg.InfiniteLine(pos=0, angle=90, pen=pg.mkPen("w", width=1, style=pg.QtCore.Qt.DashLine)))
        self._plot.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen("w", width=1, style=pg.QtCore.Qt.DashLine)))
        layout.addWidget(self._plot)
        self._trail_pts = deque()
        self._trail = self._plot.plot([], [], pen=pg.mkPen("c", width=1), alpha=0.4)
        self._dot = self._plot.plot([], [], pen=None, symbol="o", symbolSize=14, symbolBrush="r")
        self._dot_l = self._plot.plot([], [], pen=None, symbol="o", symbolSize=10, symbolBrush="g")
        self._dot_r = self._plot.plot([], [], pen=None, symbol="o", symbolSize=10, symbolBrush="y")
        self._data = {}
        
    def on_data(self, key, val):
        if key in ("swerve/cop_x", "swerve/cop_y", "swerve/cop_x_l", "swerve/cop_y_l", "swerve/cop_x_r", "swerve/cop_y_r"):
            self._data[key] = float(val)
    
    def refresh(self):
        now = time.monotonic()
        x = self._data.get("swerve/cop_x")
        y = self._data.get("swerve/cop_y")
        if x is not None and y is not None:
            self._trail_pts.append((now, x, y))
        cutoff = now - 5
        while self._trail_pts and self._trail_pts[0][0] < cutoff:
            self._trail_pts.popleft()
        if self._trail_pts:
            xs = [p[1] for p in self._trail_pts]
            ys = [p[2] for p in self._trail_pts]
            self._trail.setData(xs, ys)
            self._dot.setData([xs[-1]], [ys[-1]])
            self._status.setText(f"CoP x={xs[-1]:.3f} y={ys[-1]:.3f}")
        x_l = self._data.get("swerve/cop_x_l")
        y_l = self._data.get("swerve/cop_y_l")
        x_r = self._data.get("swerve/cop_x_r")
        y_r = self._data.get("swerve/cop_y_r")
        self._dot_l.setData([x_l] if x_l else [], [y_l] if y_l else [])
        self._dot_r.setData([x_r] if x_r else [], [y_r] if y_r else [])


class TimeSeriesTab(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        self._status = QLabel("Waiting...")
        layout.addWidget(self._status)
        self._plot = pg.PlotWidget()
        self._plot.setLabel("bottom", "Time (s)")
        self._plot.setLabel("left", "Value")
        self._plot.setTitle("Time Series")
        self._plot.showGrid(alpha=0.3)
        self._plot.disableAutoRange()  # Disable expensive autoranging
        self._plot.addLegend()
        layout.addWidget(self._plot)
        self._channels = {}
        self._plots = {}
        self._dirty = set()
        self._start_time = time.monotonic()
        self._cop_keys = {"swerve/cop_x", "swerve/cop_y", "swerve/cop_x_l", "swerve/cop_y_l", "swerve/cop_x_r", "swerve/cop_y_r"}
    
    def on_data(self, key, val):
        if key in self._cop_keys or not isinstance(val, (int, float)):
            return
        if key not in self._channels:
            color = COLORS[len(self._channels) % len(COLORS)]
            self._channels[key] = deque()
            self._plots[key] = self._plot.plot([], [], pen=pg.mkPen(color, width=2), name=key)
        self._channels[key].append((time.monotonic(), float(val)))
        self._dirty.add(key)  # Mark as needing update
    
    def refresh(self):
        now = time.monotonic()
        cutoff = now - 5  # Reduced from 10s to 5s to keep fewer points
        for key in self._channels:
            while self._channels[key] and self._channels[key][0][0] < cutoff:
                self._channels[key].popleft()
        # Only update plots that received new data
        for key in self._dirty:
            if self._channels[key]:
                times = [p[0] - self._start_time for p in self._channels[key]]
                values = [p[1] for p in self._channels[key]]
                self._plots[key].setData(times, values)
        self._dirty.clear()
        status = " | ".join(f"{k}={self._channels[k][-1][1]:.2f}" for k in list(self._channels.keys())[:3] if self._channels[k])
        self._status.setText(status or "Waiting...")


def main():
    # Create QApplication FIRST (required before any QWidget)
    app = QApplication(sys.argv)
    
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", default=115200, type=int)
    args = ap.parse_args()

    client = TelemetryClient(port=args.port, baudrate=args.baud)
    client.on_debug(lambda x: print(f"[DBG] {x}"))
    
    cop_tab = CopTab()
    ts_tab = TimeSeriesTab()
    
    # Single dispatcher callback that feeds both tabs
    def dispatch(key, val):
        cop_tab.on_data(key, val)
        ts_tab.on_data(key, val)
    
    client.on_update(dispatch)
    
    # Start telemetry in background
    threading.Thread(target=client.run, daemon=True).start()
    
    # Create GUI
    win = QMainWindow()
    win.setWindowTitle("Telemetry: CoP + Time Series")
    win.resize(900, 700)
    tabs = QTabWidget()
    tabs.addTab(cop_tab, "CoP")
    tabs.addTab(ts_tab, "Time Series")
    win.setCentralWidget(tabs)
    
    # Refresh timer
    timer = QTimer()
    timer.timeout.connect(lambda: [cop_tab.refresh(), ts_tab.refresh()])
    timer.start(100)  # 10 Hz instead of 20 Hz to reduce rendering overhead
    
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
