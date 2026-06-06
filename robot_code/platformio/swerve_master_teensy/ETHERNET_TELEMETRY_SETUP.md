# Ethernet Telemetry Setup Guide

This guide explains how to set up and test Ethernet-based telemetry for the swerve robot.

## Hardware Setup

1. **RJ45 Connector**: Ensure the RJ45 Ethernet port on the Teensy is wired correctly
2. **Network Cable**: Use an Ethernet cable to connect the Teensy to the same network as your computer
3. **Power**: Ensure the Teensy and any network hardware are powered

## IP Configuration

The Ethernet telemetry uses static IP assignment by default. You must configure the IP addresses to match your network.

### Step 1: Determine Your Network Configuration

Find out the IP address of your computer:

**macOS/Linux:**
```bash
ifconfig | grep "inet " | grep -v 127.0.0.1
```

**Windows:**
```cmd
ipconfig
```

Look for your Ethernet or Wi-Fi adapter's IP address (usually in the form `192.168.x.x` or `10.0.x.x`).

### Step 2: Update NetworkConfig.h

Edit `include/NetworkConfig.h` and update the IP addresses:

```cpp
// Example: If your computer is at 192.168.1.50, use:
#define TEENSY_IP_ADDR      192, 168, 1, 100     // Teensy static IP (pick an unused address)
#define RECEIVER_IP_ADDR    192, 168, 1, 50      // Your computer's IP
```

**Important:**
- Choose a **unique** IP address for the Teensy (not used by any other device)
- Make sure both devices are on the **same subnet** (first 3 octets should match)
- The UDP port (8888) should not conflict with other applications

### Step 3: Update MAC Address (Optional)

If you have multiple Teensy boards, ensure each has a unique MAC address in `NetworkConfig.h`:

```cpp
#define TEENSY_MAC_ADDR {0xA8, 0x61, 0x0A, 0xAE, 0x00, 0xB9}
```

## Building and Uploading

### Using PlatformIO CLI:

```bash
# Build the firmware
platformio run -e penny_v4

# Upload to Teensy
platformio run -e penny_v4 -t upload

# Monitor serial output
platformio device monitor -e penny_v4
```

### Using VS Code with PlatformIO Extension:

1. Open the PlatformIO Home (VS Code sidebar)
2. Select `penny_v4` environment
3. Click "Build" to compile
4. Click "Upload" to flash the Teensy
5. Click "Monitor" to see serial output

## Testing Telemetry Reception

### Step 1: Verify Ethernet Hardware

After uploading, open the Serial Monitor (460800 baud) and look for:

```
[Telemetry] SUCCESS: Ethernet initialized. Teensy IP: 192.168.1.100
[Telemetry] Sending telemetry to 192.168.1.50:8888
```

If you see errors about hardware not found or cable not connected:
- Check the Ethernet hardware connection
- Verify the RJ45 cable is properly connected
- Try different USB ports or cables

### Step 2: Install Python Dependencies

```bash
cd src/telemetry-client
pip install -r requirements.txt
```

### Step 3: Run the Python Telemetry Client

```bash
python telemetry_client.py
```

You should see:
```
UDP listener started on port 8888
```

### Step 4: Trigger Telemetry Data

The robot sends telemetry data every 30ms. To test:

1. Power on the Teensy and robot hardware
2. Connect the Ethernet cable
3. Run the Python client
4. The Python GUI should populate with telemetry data and display live plots

Expected telemetry variables:
- `swerve/cop_x` - Center of pressure X coordinate
- `swerve/cop_y` - Center of pressure Y coordinate
- `swerve/cop_z` - Center of pressure Z coordinate
- `swerve/pad_forces` - Load cell readings (8 sensors)
- `swerve/pad_raw_forces` - Raw load cell readings
- `swerve/total_weight` - Total weight on pads

## Troubleshooting

### "Ethernet hardware not found"
- Check the Teensy 4.x model (must be Teensy 4.1 with built-in Ethernet)
- Verify the RJ45 connector is properly soldered
- Try a different USB cable or port

### "Ethernet cable not connected"
- Physically inspect the RJ45 connection
- Test with a different Ethernet cable
- Verify the network switch/hub is powered on

### "Failed to obtain IP address"
- Check the IP configuration in `NetworkConfig.h`
- Make sure the Teensy IP is unique on the network
- Verify the subnet matches your network (first 3 octets)
- Try disabling DHCP on other devices to avoid conflicts

### Python client shows no data
- Verify the Teensy's IP and port settings match `NetworkConfig.h`
- Check that your firewall allows UDP traffic on port 8888
- Verify the computer's IP matches `RECEIVER_IP_ADDR` in `NetworkConfig.h`
- Run with verbose output: `python telemetry_client.py --debug`

### Intermittent or dropped packets
- Reduce other network traffic
- Ensure good quality Ethernet cable
- Check for electromagnetic interference near cables
- Consider reducing telemetry send frequency if needed

## Advanced: DHCP Configuration

For automatic IP assignment via DHCP, edit `NetworkConfig.h`:

```cpp
#define TEENSY_IP_ADDR      0, 0, 0, 0          // Enable DHCP
```

Then check the serial output to see the assigned IP address. Note: DHCP addresses may change between reboots.

## Network Debug Mode

To enable debug output in firmware, set in `NetworkConfig.h`:

```cpp
#define NETWORK_DEBUG       1
```

This adds extra diagnostic messages to help troubleshoot network issues.

## Performance Notes

- UDP telemetry is sent every ~30ms (outer loop timing)
- JSON packets are typically 200-300 bytes
- Bandwidth usage: ~6-10 KB/second
- No real-time guarantees (UDP is connectionless)

If you need reliable delivery, consider switching to TCP or adding packet acknowledgment.
