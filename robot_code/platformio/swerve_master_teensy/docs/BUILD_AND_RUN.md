# Build and Run Guide

## 🚀 Quick Start

### Prerequisites
- [x] PlatformIO installed (via VSCode extension or standalone)
- [x] Windows 10/11 with PowerShell
- [x] MSYS2 installed (for native simulation builds)

### Building for Simulation (Windows Native)

#### Method 1: Using PlatformIO Command Line
```powershell
# Navigate to project directory
cd robot_code\platformio\swerve_master_teensy

# Build simulation
&"$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e native

# Run simulation
.\.pio\build\native\program.exe
```

#### Method 2: Using VSCode
1. Open project in VSCode
2. Open PlatformIO sidebar
3. Select `native` environment
4. Click "Build" or "Upload and Monitor"

### Building for Teensy Hardware

```powershell
# Build for Teensy 4.1
&"$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e teensy41

# Upload to Teensy (requires Teensy connected)
&"$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e teensy41 -t upload
```

## 🎯 What You'll See

### Simulation Output
```
========================================
  Swerve Robot Simulation Mode
  HAL Implementation: Simulation
  Running swerve_master.ino
========================================
Running robot setup...
Initializing HAL in simulation mode...
Serial initialized at 460800 baud
Simulation setup complete!
Starting main loop...
Press Ctrl+C to stop
Loop count: 1
Loop count: 29
Simulation loop running...
Loop count: 1000
```

### Key Features Working
- ✅ **Arduino-style setup()/loop()**: Same code structure as real hardware
- ✅ **HAL Abstraction**: GPIO, ADC, Timer, Wire interfaces working
- ✅ **RC Receiver Simulation**: Mock RC input values
- ✅ **Cross-platform Build**: Same code compiles for both Teensy and Windows
- ✅ **Real-time Loop**: 4.5ms target loop time maintained

## 🔧 Troubleshooting

### PlatformIO Command Not Found
```powershell
# Check if PlatformIO is installed
&"$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" --version

# If not found, install PlatformIO Core
pip install platformio
```

### Build Errors

#### "Arduino.h not found" in simulation
- **Cause**: Library trying to include Arduino.h in native build
- **Solution**: Library is excluded via `lib_ignore` in platformio.ini

#### "abs() conflicts" errors
- **Cause**: Macro conflicts between Arduino compatibility and std library
- **Solution**: Already resolved by removing conflicting abs() definitions

#### Missing MSYS2 tools
```powershell
# Add MSYS2 to PATH (if not already done)
$env:PATH += ";C:\msys64\mingw64\bin"
```

### Runtime Issues

#### Simulation exits immediately
- **Cause**: Exception in setup() or loop()
- **Solution**: Check console output for error messages

#### No RC receiver input
- **Cause**: Mock RC receiver returns default values
- **Solution**: This is expected - real RC input requires hardware

## 📁 Build Artifacts

### Simulation Build
```
.pio\build\native\
├── program.exe          # Simulation executable
├── src\                 # Compiled object files
└── lib\                 # Library objects
```

### Teensy Build
```
.pio\build\teensy41\
├── firmware.hex         # Teensy firmware
├── firmware.elf         # Debug symbols
└── src\                 # Compiled object files
```

## 🎮 Advanced Usage

### Running with Arguments
```powershell
# Future: WebSocket server URL
.\.pio\build\native\program.exe --server-url ws://localhost:8080
```

### Debug Build
```powershell
# Build with debug symbols
&"$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e native --verbose
```

### Clean Build
```powershell
# Clean and rebuild
&"$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e native -t clean
&"$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e native
```

## 📊 Performance

### Simulation Performance
- **Loop Rate**: ~222 Hz (4.5ms target)
- **Memory Usage**: ~10MB RAM
- **CPU Usage**: <5% on modern systems

### Teensy Performance
- **Flash Usage**: 66KB / 8MB (0.8%)
- **RAM Usage**: 75KB / 1MB (7.3%)
- **Loop Rate**: 222 Hz (4.5ms actual)

## 🔄 Development Workflow

### 1. Code Changes
1. Edit source files in `src/` or `include/`
2. Build for simulation first: `pio run -e native`
3. Test simulation: `.\.pio\build\native\program.exe`
4. Build for Teensy: `pio run -e teensy41`
5. Upload to hardware: `pio run -e teensy41 -t upload`

### 2. Adding New HAL Interfaces
1. Create interface in `include/hal/I*.h`
2. Add to `HALFactory.h`
3. Implement mock in `HALFactory.cpp`
4. Add real implementation (future)
5. Update documentation

### 3. Testing
```powershell
# Quick simulation test
&"$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e native && .\.pio\build\native\program.exe
```

## 📚 Related Documentation
- `HAL_STATUS.md` - Current implementation status
- `CONVERSION_CHECKLIST.md` - Detailed conversion progress
- `SIMULATION_SETUP.md` - Simulation environment setup
