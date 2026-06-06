# Swerve Robot Telemetry Client

A real-time telemetry visualization client for the swerve robot, receiving JSON data over UDP and displaying live plots.

## Features

- **Real-time plotting** of any telemetry variable (time-series line graphs)
- **Center of Pressure (COP) visualization** — XY scatter plot showing rider weight distribution
- **Variable selection** — checkbox UI to toggle which variables to plot
- **Live status** — packet count, variable count, and error tracking
- **Fast updates** — 20 Hz refresh rate for smooth visualization

## Setup

### Install Dependencies

```bash
pip install -r requirements.txt
```

### Network Configuration

The client expects UDP telemetry packets on **port 8888**. Make sure:
- Teensy is configured to send to your machine's IP address (check `Telemetry.cpp` / `Telemetry::start()`)
- Your firewall allows UDP traffic on port 8888
- Both devices are on the same network

### Run the Client

```bash
python telemetry_client.py
```

The window will open and start listening for packets. Once the Teensy is powered on and running, telemetry data will appear.

## Available Telemetry Variables

### Pad Data (Sent every ~30ms in main loop)

- **`swerve/pad_raw_forces`** — Array of 8 raw load cell readings (ADC → Newtons)
- **`swerve/pad_forces`** — Array of 8 zeroed load cell forces (N)
- **`swerve/cop_x`** — Center of Pressure, left/right axis (normalized -1 to 1)
- **`swerve/cop_y`** — Center of Pressure, front/back axis (normalized -1 to 1)
- **`swerve/cop_z`** — Center of Pressure, rotational component (normalized -1 to 1)
- **`swerve/total_weight`** — Total weight on pads (N)

## Usage

1. **Select variables to plot:** Check the boxes in the right panel
2. **View live data:** Line graphs update in real-time on the left
3. **Monitor center of pressure:** The XY scatter plot shows dynamic weight distribution

## Future Extensions

- [ ] Data logging to CSV/HDF5
- [ ] Playback of recorded sessions
- [ ] Command interface (send control signals back to robot)
- [ ] Custom plot layouts and data streaming
