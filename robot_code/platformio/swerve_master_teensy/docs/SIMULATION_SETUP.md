# Swerve Robot Simulation Setup Guide

## ✅ **STATUS: WORKING PERFECTLY!**
**Both Teensy and Windows native simulation builds are now fully functional.**

This guide walks you through setting up the swerve robot simulation environment on your Windows laptop, allowing you to run and test robot code without physical hardware.

## Overview

The simulation system uses a Hardware Abstraction Layer (HAL) that allows the same robot code to run on:
- **Real Hardware**: Teensy 4.1 with actual sensors and motors
- **Desktop Simulation**: Windows laptop with simulated hardware via WebSocket

## Prerequisites

### Required Software

1. **PlatformIO** (for building and managing the project)
2. **GCC Compiler** (for native C++ compilation)
3. **Git** (for version control)
4. **VS Code** (recommended IDE)

### System Requirements

- Windows 10/11 (64-bit)
- 4GB+ RAM
- 1GB+ free disk space
- Internet connection (for initial setup)

## Installation Steps

### Step 1: Install PlatformIO

#### Option A: Via VS Code Extension (Recommended)
1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Open VS Code
3. Go to Extensions (Ctrl+Shift+X)
4. Search for "PlatformIO IDE"
5. Install the PlatformIO IDE extension
6. Restart VS Code

#### Option B: Via Python pip
```bash
# Install Python 3.7+ first, then:
pip install platformio
```

### Step 2: Install GCC Compiler (MSYS2)

1. **Download MSYS2**
   - Go to https://www.msys2.org/
   - Download the installer (msys2-x86_64-*.exe)
   - Run the installer with default settings

2. **Update MSYS2**
   ```bash
   # In MSYS2 terminal:
   pacman -Syu
   ```
   (May require terminal restart)

3. **Install GCC and Development Tools**
   ```bash
   # Install GCC compiler
   pacman -S mingw-w64-x86_64-gcc

   # Install GDB debugger
   pacman -S mingw-w64-x86_64-gdb

   # Install Make and build tools
   pacman -S mingw-w64-x86_64-make

   # Install CMake (optional)
   pacman -S mingw-w64-x86_64-cmake
   ```

4. **Add MSYS2 to Windows PATH**

   **Option A: Via PowerShell (Administrator)**
   ```powershell
   [Environment]::SetEnvironmentVariable("PATH", $env:PATH + ";C:\msys64\mingw64\bin;C:\msys64\ucrt64\bin;C:\msys64\usr\bin", [EnvironmentVariableTarget]::Machine)
   ```

   **Option B: Via System Properties**
   - Right-click "This PC" → Properties
   - Advanced System Settings → Environment Variables
   - Edit "Path" in System Variables
   - Add these paths:
     - `C:\msys64\mingw64\bin`
     - `C:\msys64\ucrt64\bin`
     - `C:\msys64\usr\bin`

5. **Verify Installation**
   ```bash
   gcc --version
   ```
   Should show GCC version information.

### Step 3: Clone and Setup Project

1. **Clone the Repository**
   ```bash
   git clone <repository-url>
   cd kinetik/robot_code/platformio/swerve_master_teensy
   ```

2. **Open in VS Code**
   ```bash
   code .
   ```

3. **Install Project Dependencies**
   PlatformIO will automatically install dependencies when you first build.

## Building and Running

### For Real Hardware (Teensy 4.1)

1. **Connect Teensy 4.1** to your computer via USB
2. **Build and Upload**
   ```bash
   # Via PlatformIO CLI
   platformio run -e teensy41 --target upload

   # Or via VS Code
   # Press Ctrl+Shift+P → "PlatformIO: Upload"
   ```

### For Desktop Simulation

1. **Build Native Version**
   ```bash
   # Via PlatformIO CLI
   platformio run -e native

   # Or via VS Code
   # Select "native" environment and build
   ```

2. **Run Simulation**
   ```bash
   # Navigate to build output
   cd .pio/build/native

   # Run the simulation
   ./program.exe

   # Or with custom server URL
   ./program.exe --server-url ws://localhost:8080
   ```

## Simulation Architecture

### Hardware Abstraction Layer (HAL)

The HAL provides consistent interfaces for:
- **CAN Bus Communication** (`ICANBus`)
- **IMU Sensor** (`IIMU`)
- **RC Receiver** (`IRCReceiver`)
- **Serial Communication** (`ISerial`)

### Implementation Switching

Compile-time flags determine which implementation to use:
- `HAL_IMPLEMENTATION=HAL_REAL` → Real hardware
- `HAL_IMPLEMENTATION=HAL_SIM` → Simulation

### WebSocket Communication

The simulation communicates with external tools via WebSocket:
- **Port**: 8080 (default)
- **Protocol**: JSON messages
- **Bidirectional**: Send commands, receive sensor data

## Usage Examples

### Basic Simulation Run

```bash
# Build and run simulation
platformio run -e native
.pio/build/native/program.exe

# Output:
# ========================================
#   Swerve Robot Simulation Mode
#   HAL Implementation: Simulation
# ========================================
# Connecting to simulation server at ws://localhost:8080...
# Connected to simulation server successfully!
# Initializing HAL...
# Running robot setup...
# Starting main loop...
# Press Ctrl+C to stop
```

### With Custom Server

```bash
# Connect to different simulation server
.pio/build/native/program.exe --server-url ws://192.168.1.100:9090
```

### Monitoring Output

The simulation provides real-time telemetry:
```
Yaw: 45.23, Vx: 1.50, Vy: 0.00, Omega: 0.25, E-stop: CLEAR
Loop time: 4500 µs
```

## Troubleshooting

### Common Issues

1. **"gcc not found"**
   - Verify MSYS2 installation
   - Check PATH environment variable
   - Restart terminal/VS Code

2. **"PlatformIO not found"**
   - Install PlatformIO extension in VS Code
   - Or install via pip: `pip install platformio`

3. **Build errors with Arduino libraries**
   - Normal for simulation build
   - Use HAL interfaces instead of direct Arduino calls

4. **WebSocket connection failed**
   - Start simulation server first
   - Check firewall settings
   - Verify server URL and port

### Getting Help

1. **Check build output** for specific error messages
2. **Enable verbose mode**: `platformio run -e native -v`
3. **Check PlatformIO documentation**: https://docs.platformio.org/
4. **Review HAL interface documentation** in `include/hal/`

## Development Workflow

### Typical Development Cycle

1. **Write/modify robot code** using HAL interfaces
2. **Test on simulation** for quick iteration
3. **Deploy to real hardware** when ready
4. **Use web interface** for remote monitoring/control

### Best Practices

- **Always use HAL interfaces** instead of direct hardware calls
- **Test in simulation first** before deploying to hardware
- **Use version control** to track changes
- **Document custom HAL implementations**

## Next Steps

After successful setup:
1. **Create WebSocket simulation server** (see SIMULATION_SERVER.md)
2. **Build web-based control interface** (see WEB_INTERFACE.md)
3. **Customize HAL implementations** for your specific needs
4. **Add additional sensors/actuators** to the simulation

## File Structure

```
robot_code/platformio/swerve_master_teensy/
├── docs/                    # Documentation
├── include/hal/             # HAL interface headers
│   ├── real/               # Real hardware implementations
│   ├── sim/                # Simulation implementations
│   └── *.h                 # Abstract interfaces
├── src/                    # Source code
│   ├── main_native.cpp     # Native entry point
│   ├── robot_main.cpp      # Main robot logic
│   └── ...                 # Existing robot code
├── lib/                    # Libraries
└── platformio.ini          # Build configuration
```
