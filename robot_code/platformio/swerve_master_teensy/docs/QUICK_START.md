# Quick Start Guide

## TL;DR - Get Simulation Running

### Prerequisites Check
```bash
# Check if tools are installed
gcc --version          # Should show GCC version
platformio --version   # Should show PlatformIO version
```

### Build and Run (5 minutes)
```bash
# 1. Navigate to project
cd robot_code/platformio/swerve_master_teensy

# 2. Build simulation
platformio run -e native

# 3. Run simulation
.pio/build/native/program.exe
```

## Command Reference

### Building

| Target | Command | Description |
|--------|---------|-------------|
| Teensy Hardware | `platformio run -e teensy41` | Build for real hardware |
| Desktop Simulation | `platformio run -e native` | Build for simulation |
| Upload to Teensy | `platformio run -e teensy41 -t upload` | Build and upload |
| Clean Build | `platformio run -t clean` | Clean build files |

### Running Simulation

| Command | Description |
|---------|-------------|
| `./program.exe` | Run with default settings |
| `./program.exe --server-url ws://localhost:8080` | Custom server URL |
| `./program.exe --help` | Show help |

### Development

| Command | Description |
|---------|-------------|
| `platformio run -v` | Verbose build output |
| `platformio device list` | List connected devices |
| `platformio lib list` | Show installed libraries |

## HAL Usage Examples

### Reading IMU Data
```cpp
#include "hal/HALFactory.h"

void setup() {
    HAL::initialize();
    
    if (HAL::imu->begin()) {
        Serial.println("IMU ready");
    }
}

void loop() {
    Vector3D angles = HAL::imu->getEulerAngles();
    Serial.print("Yaw: ");
    Serial.println(angles.z);
}
```

### Reading RC Input
```cpp
void loop() {
    HAL::rcReceiver->update();
    
    double leftStick = HAL::rcReceiver->getLeftVertical();
    bool signalLost = HAL::rcReceiver->isSignalLost();
    
    if (!signalLost) {
        // Use RC input
        Serial.print("Left stick: ");
        Serial.println(leftStick);
    }
}
```

### Sending CAN Messages
```cpp
void sendMotorCommand(uint32_t motorId, float velocity) {
    CANMessage msg;
    msg.id = motorId;
    msg.len = 4;
    memcpy(msg.data, &velocity, sizeof(velocity));
    
    HAL::canBus->write(msg);
}
```

## Troubleshooting Quick Fixes

### Build Issues

**Error: `gcc not found`**
```bash
# Add MSYS2 to PATH
set PATH=%PATH%;C:\msys64\mingw64\bin
```

**Error: `Arduino.h not found` (in simulation)**
- Normal for simulation build
- Use HAL interfaces instead

**Error: `FlexCAN_T4.h not found` (in simulation)**
- Normal for simulation build  
- Real hardware code is excluded in simulation

### Runtime Issues

**Error: `WebSocket connection failed`**
- Start simulation server first
- Check if port 8080 is available
- Try different port: `--server-url ws://localhost:9090`

**Error: `HAL initialization failed`**
- Check if all HAL components are implemented
- Enable debug output for more details

## File Locations

| File Type | Location |
|-----------|----------|
| Built executable | `.pio/build/native/program.exe` |
| Build logs | `.pio/build/native/` |
| HAL interfaces | `include/hal/*.h` |
| Robot code | `src/robot_main.cpp` |
| Configuration | `platformio.ini` |

## Environment Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `HAL_IMPLEMENTATION` | Set implementation type | `HAL_SIM` or `HAL_REAL` |
| `SIMULATION_SERVER` | Default server URL | `ws://localhost:8080` |

## VS Code Integration

### Useful Extensions
- PlatformIO IDE
- C/C++ Extension Pack
- GitLens

### Key Shortcuts
- `Ctrl+Shift+P` → PlatformIO commands
- `Ctrl+Alt+B` → Build project
- `Ctrl+Alt+U` → Upload to device
- `Ctrl+Alt+S` → Serial monitor

### Tasks
- Build: `Terminal → Run Task → PlatformIO: Build`
- Upload: `Terminal → Run Task → PlatformIO: Upload`
- Monitor: `Terminal → Run Task → PlatformIO: Monitor`

## Common Workflows

### Testing New Feature
1. Modify code using HAL interfaces
2. Build simulation: `platformio run -e native`
3. Test in simulation: `./program.exe`
4. Build for hardware: `platformio run -e teensy41`
5. Upload to Teensy: `platformio run -e teensy41 -t upload`

### Debugging Simulation
1. Build with debug: `platformio run -e native --build-type debug`
2. Run with GDB: `gdb ./program.exe`
3. Set breakpoints and debug

### Remote Development
1. Build simulation on laptop
2. Run simulation server on robot computer
3. Connect via WebSocket: `--server-url ws://robot-ip:8080`

## Performance Tips

- **Simulation loop time**: ~4.5ms (matches real hardware)
- **WebSocket update rate**: 10Hz (adjustable)
- **CAN message throughput**: 1000+ messages/sec
- **Memory usage**: <100MB typical

## Getting Help

1. **Check documentation**: `docs/` folder
2. **Enable verbose output**: Add `-v` flag
3. **Check HAL implementation**: Review `include/hal/` files
4. **PlatformIO docs**: https://docs.platformio.org/
