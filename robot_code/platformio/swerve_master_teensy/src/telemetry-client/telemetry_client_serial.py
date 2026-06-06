#!/usr/bin/env python3
"""
Serial-based telemetry client for swerve robot.
Receives JSON telemetry packets over USB serial and displays live plots.
"""

import sys
import json
import threading
import glob
import time
import numpy as np
from collections import defaultdict, deque
from datetime import datetime

import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QCheckBox, QLabel, QGridLayout, QGroupBox, QScrollArea
)

try:
    import serial
except ImportError:
    print("Error: pyserial not installed. Run: pip3 install pyserial")
    sys.exit(1)


class DataModel(QObject):
    """Manages telemetry data and emits signals when new data arrives."""
    data_updated = pyqtSignal()

    def __init__(self, history_length=500):
        super().__init__()
        self.history_length = history_length
        self.data = defaultdict(lambda: deque(maxlen=history_length))
        self.timestamps = deque(maxlen=history_length)
        self.start_time = None
        self.packet_count = 0

    def add_packet(self, packet):
        """Add a telemetry packet and update timestamps."""
        if self.start_time is None:
            self.start_time = datetime.now()

        elapsed = (datetime.now() - self.start_time).total_seconds()
        self.timestamps.append(elapsed)
        self.packet_count += 1

        for key, value in packet.items():
            if isinstance(value, (int, float)):
                self.data[key].append(float(value))
            elif isinstance(value, list) and all(isinstance(v, (int, float)) for v in value):
                # For arrays, store each element separately with indexed key
                for i, v in enumerate(value):
                    self.data[f"{key}[{i}]"].append(float(v))

        self.data_updated.emit()

    def get_time_series(self, key):
        """Return (timestamps, values) for a given key."""
        if key not in self.data:
            return np.array([]), np.array([])
        return np.array(self.timestamps), np.array(self.data[key])

    def get_keys(self):
        """Return sorted list of all available keys."""
        return sorted(self.data.keys())

    def clear(self):
        """Clear all data."""
        self.data.clear()
        self.timestamps.clear()
        self.packet_count = 0
        self.start_time = None


class SerialReceiver(threading.Thread):
    """Background thread that listens for serial telemetry packets."""
    
    def __init__(self, data_model, port=None, baudrate=115200):
        super().__init__(daemon=True)
        self.data_model = data_model
        self.port = port or self._find_teensy_port()
        self.baudrate = baudrate
        self.running = True
        self.packet_count = 0
        self.error_count = 0

    @staticmethod
    def _find_teensy_port():
        """Find the first available Teensy serial port."""
        for port in sorted(glob.glob('/dev/cu.usbmodem*')):
            try:
                s = serial.Serial(port, 115200, timeout=0.5)
                s.close()
                return port
            except Exception:
                pass
        return None

    def run(self):
        """Listen for serial telemetry packets."""
        if not self.port:
            print("[-] No Teensy found on serial ports")
            return

        print(f"[+] Connecting to {self.port}...")
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
        except Exception as e:
            print(f"[-] Failed to open serial port {self.port}: {e}")
            return

        print(f"[+] Serial listener started on {self.port}")
        ser.reset_input_buffer()

        while self.running:
            try:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                
                if not line:
                    continue
                
                # Print all lines for debugging
                print(f"  {line}")
                
                # Look for telemetry packets marked with [TELEM]
                if line.startswith("[TELEM]"):
                    json_str = line[7:].strip()  # Remove [TELEM] prefix
                    try:
                        packet = json.loads(json_str)
                        self.data_model.add_packet(packet)
                        self.packet_count += 1
                    except json.JSONDecodeError:
                        self.error_count += 1
                        
            except Exception as e:
                if self.running:
                    print(f"[-] Serial error: {e}")
                    self.error_count += 1
                    time.sleep(0.5)

        ser.close()

    def stop(self):
        """Stop the receiver thread."""
        self.running = False


class TelemetryClient(QMainWindow):
    """Main application window for telemetry visualization."""

    def __init__(self, port=None):
        super().__init__()
        self.setWindowTitle("Swerve Robot Telemetry (Serial)")
        self.setGeometry(100, 100, 1600, 900)

        # Data model
        self.data_model = DataModel(history_length=500)

        # Start serial receiver
        self.serial_receiver = SerialReceiver(self.data_model, port=port)
        self.serial_receiver.start()

        # Track which variables are being plotted
        self.active_plots = {}  # key -> PlotWidget
        self.variable_checkboxes = {}  # key -> QCheckBox

        # UI Setup
        self.setup_ui()

        # Update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)  # 20 Hz update rate

        # Connect data model signal
        self.data_model.data_updated.connect(self.on_data_updated)

    def setup_ui(self):
        """Create the main UI."""
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QHBoxLayout(central)

        # Left side: plot area
        self.plot_layout = QVBoxLayout()
        plot_container = QWidget()
        plot_container.setLayout(self.plot_layout)
        main_layout.addWidget(plot_container, stretch=3)

        # Right side: controls
        control_layout = QVBoxLayout()

        # Status
        self.status_label = QLabel("Waiting for packets...")
        control_layout.addWidget(self.status_label)

        # Variable selector
        variables_group = QGroupBox("Telemetry Variables")
        variables_layout = QVBoxLayout()
        
        # Scrollable checkbox list
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        checkbox_widget = QWidget()
        self.checkbox_layout = QVBoxLayout(checkbox_widget)
        scroll.setWidget(checkbox_widget)
        variables_layout.addWidget(scroll)

        variables_group.setLayout(variables_layout)
        control_layout.addWidget(variables_group)

        # Special plot for center of pressure (XY scatter)
        cop_group = QGroupBox("Center of Pressure (XY)")
        cop_layout = QVBoxLayout()
        self.cop_plot = pg.PlotWidget()
        self.cop_plot.setLabel('left', 'Y (front/back)', units='')
        self.cop_plot.setLabel('bottom', 'X (left/right)', units='')
        self.cop_plot.setTitle("Center of Pressure")
        self.cop_scatter = self.cop_plot.plot([], [], pen=None, symbol='o', symbolSize=10, symbolBrush='r')
        cop_layout.addWidget(self.cop_plot)
        cop_group.setLayout(cop_layout)
        control_layout.addWidget(cop_group)

        # Add controls to main layout
        control_widget = QWidget()
        control_widget.setLayout(control_layout)
        control_widget.setMaximumWidth(350)
        main_layout.addWidget(control_widget, stretch=1)

    def on_data_updated(self):
        """Called when new data arrives."""
        # Update available variables
        current_keys = set(self.data_model.get_keys())
        existing_keys = set(self.variable_checkboxes.keys())

        # Add new checkboxes for newly discovered variables
        for key in sorted(current_keys - existing_keys):
            checkbox = QCheckBox(key)
            checkbox.setChecked(False)
            checkbox.stateChanged.connect(lambda state, k=key: self.toggle_plot(k, state))
            self.variable_checkboxes[key] = checkbox
            self.checkbox_layout.addWidget(checkbox)

        # Update status
        packet_count = self.data_model.packet_count
        key_count = len(current_keys)
        self.status_label.setText(
            f"Packets: {packet_count} | Variables: {key_count} | Errors: {self.serial_receiver.error_count}"
        )

    def toggle_plot(self, key, state):
        """Add or remove a plot based on checkbox state."""
        if state == Qt.Checked:
            # Create new plot
            plot_widget = pg.PlotWidget()
            plot_widget.setLabel('left', key)
            plot_widget.setLabel('bottom', 'Time', units='s')
            plot_widget.setTitle(key)
            plot_widget.addLegend()
            
            curve = plot_widget.plot([], [], pen='b', name=key)
            
            self.plot_layout.addWidget(plot_widget)
            self.active_plots[key] = {
                'widget': plot_widget,
                'curve': curve,
            }
        else:
            # Remove plot
            if key in self.active_plots:
                plot_data = self.active_plots[key]
                plot_data['widget'].deleteLater()
                del self.active_plots[key]

    def update_plots(self):
        """Update all active plots with latest data."""
        for key, plot_data in self.active_plots.items():
            times, values = self.data_model.get_time_series(key)
            if len(times) > 0:
                plot_data['curve'].setData(times, values)

        # Update center of pressure plot
        if 'swerve/cop_x' in self.data_model.data and 'swerve/cop_y' in self.data_model.data:
            cop_x = np.array(self.data_model.data['swerve/cop_x'])
            cop_y = np.array(self.data_model.data['swerve/cop_y'])
            if len(cop_x) > 0:
                self.cop_scatter.setData(cop_x, cop_y)
                # Auto-scale
                self.cop_plot.setRange(
                    xRange=(-400, 400),
                    yRange=(-400, 400),
                    padding=0.1
                )

    def closeEvent(self, event):
        """Clean up on exit."""
        self.serial_receiver.stop()
        self.timer.stop()
        event.accept()


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Serial telemetry client for swerve robot")
    ap.add_argument("--port", default=None, help="Serial port (auto-detected if not specified)")
    args = ap.parse_args()
    
    app = QApplication(sys.argv)
    client = TelemetryClient(port=args.port)
    client.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
