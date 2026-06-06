# Ethernet Telemetry Integration Summary

## What Was Done

This integration enables real-time telemetry monitoring of the swerve robot via Ethernet and UDP, with live visualization through a Python client GUI.

## Files Modified / Created

### New Files
1. **`include/NetworkConfig.h`** - Network configuration (IP addresses, MAC, port)
2. **`ETHERNET_TELEMETRY_SETUP.md`** - Complete setup and troubleshooting guide

### Modified Files
1. **`platformio.ini`** - Added NativeEthernet library dependency
2. **`include/Telemetry.h`** - Added NetworkConfig.h include
3. **`src/Telemetry.cpp`** - Updated to use NetworkConfig; improved debug output
4. **`src/penny_v4.cpp`** - Added comprehensive swerve module telemetry data

## Key Features

### Hardware Support
- Teensy 4.1 with built-in Ethernet (RJ45 port)
- NativeEthernet library for UDP communication
- Graceful fallback if Ethernet hardware unavailable

### Telemetry Data Sent (30ms interval)
**Pad/Force Data:**
- `swerve/pad_raw_forces` - Raw load cell readings (8 sensors)
- `swerve/pad_forces` - Calibrated load cell forces (8 sensors)
- `swerve/cop_x`, `cop_y`, `cop_z` - Center of pressure coordinates
- `swerve/total_weight` - Total weight measurement

**Swerve Module Data:**
- `swerve/steer_angles_deg` - Steering angles for all 4 modules (degrees)
- `swerve/drive_velocities` - Drive velocities for all 4 modules (m/s)

**System Status:**
- `swerve/mode` - Operating mode (0=teleop, 1=pads, 2=test)
- `swerve/rc_connected` - RC receiver connected status
- `swerve/motors_enabled` - Motors active status

### Protocol
- **Transport**: UDP/IP
- **Encoding**: JSON (human-readable)
- **Port**: 8888 (configurable)
- **Packet Size**: ~200-300 bytes
- **Bandwidth**: ~6-10 KB/s
- **Latency**: ~30ms per packet

## Configuration

IP addresses and network settings are defined in `include/NetworkConfig.h`:

```cpp
#define TEENSY_IP_ADDR      192, 168, 0, 100     // Adjust to your network
#define RECEIVER_IP_ADDR    192, 168, 0, 50      // Your computer's IP
#define TELEMETRY_UDP_PORT  8888                 // Port number
```

**See `ETHERNET_TELEMETRY_SETUP.md` for detailed configuration instructions.**

## Python Client

Located in `src/telemetry-client/telemetry_client.py`

Features:
- Real-time UDP packet reception
- Dynamic plot generation for any telemetry variable
- Live center-of-pressure (COP) scatter plot
- Auto-discovering telemetry variables
- Packet statistics and error tracking

Install dependencies:
```bash
pip install -r src/telemetry-client/requirements.txt
```

Run client:
```bash
python src/telemetry-client/telemetry_client.py
```

## Next Steps

1. **Configure Network**: Edit `include/NetworkConfig.h` with your network IPs
2. **Build Firmware**: `platformio run -e penny_v4`
3. **Upload**: `platformio run -e penny_v4 -t upload`
4. **Test**: Run Python client and verify data reception
5. **Monitor Serial**: Use `platformio device monitor -e penny_v4` to debug

## Troubleshooting

See `ETHERNET_TELEMETRY_SETUP.md` for comprehensive troubleshooting guide including:
- Hardware detection issues
- Network configuration validation
- Firewall and network troubleshooting
- Debug mode enablement

## Performance Notes

- JSON encoding adds ~10-20% overhead vs binary format
- No guaranteed delivery (UDP is unreliable)
- Can add TCP fallback if needed for mission-critical data
- Telemetry doesn't block motor control (non-blocking send)

## Future Enhancements

Possible improvements:
- Add motor current telemetry (ODrive/VESC specific)
- Add IMU data (accelerometer, gyro, orientation)
- Implement packet acknowledgment
- Switch to higher-speed binary protocol if needed
- Add motion planning state telemetry
- Log telemetry to file on Teensy SD card
