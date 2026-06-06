#!/usr/bin/env python3
"""
Real-time telemetry client for swerve robot.
Receives JSON telemetry packets over UDP and displays live plots.
"""

import sys
import socket
import json
import threading
import numpy as np
from collections import defaultdict, deque
from datetime import datetime

import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QCheckBox, QLabel, QGridLayout, QGroupBox, QScrollArea
)


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


class UDPReceiver(threading.Thread):
    """Background thread that listens for UDP telemetry packets."""
    
    def __init__(self, data_model, port=8888):
        super().__init__(daemon=True)
        self.data_model = data_model
        self.port = port
        self.running = True
        self.packet_count = 0
        self.error_count = 0

    def run(self):
        """Listen for UDP packets."""
        import time
        
        sock = None
        # Try to bind with retries
        for attempt in range(5):
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                except AttributeError:
                    pass  # SO_REUSEPORT not available on all platforms
                sock.bind(("0.0.0.0", self.port))
                print(f"UDP listener started on port {self.port}")
                break
            except OSError as e:
                if attempt < 4:
                    print(f"Failed to bind to port {self.port}, retrying ({attempt+1}/5)...")
                    time.sleep(1)
                    if sock:
                        sock.close()
                else:
                    print(f"Failed to bind to port {self.port} after 5 attempts: {e}")
                    return

        while self.running:
            try:
                data, addr = sock.recvfrom(4096)
                try:
                    packet = json.loads(data.decode("utf-8"))
                    self.data_model.add_packet(packet)
                    self.packet_count += 1
                except json.JSONDecodeError as e:
                    print(f"Failed to parse JSON from {addr}: {e}")
                    self.error_count += 1
            except Exception as e:
                if self.running:
                    print(f"UDP error: {e}")

        if sock:
            sock.close()

    def stop(self):
        """Stop the receiver thread."""
        self.running = False


class TelemetryClient(QMainWindow):
    """Main application window for telemetry visualization."""

    def __init__(self, port=8888):
        super().__init__()
        self.setWindowTitle("Swerve Robot Telemetry")
        self.setGeometry(100, 100, 1600, 900)

        # Data model
        self.data_model = DataModel(history_length=500)

        # Start UDP receiver
        self.udp_receiver = UDPReceiver(self.data_model, port=port)
        self.udp_receiver.start()

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
            f"Packets: {packet_count} | Variables: {key_count} | Errors: {self.udp_receiver.error_count}"
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
        self.udp_receiver.stop()
        self.timer.stop()
        event.accept()


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Swerve robot telemetry GUI client")
    ap.add_argument("--port", type=int, default=8888,
                    help="UDP port for telemetry (default 8888)")
    args = ap.parse_args()
    
    app = QApplication(sys.argv)
    client = TelemetryClient(port=args.port)
    client.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
