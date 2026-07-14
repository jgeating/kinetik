# Moteus N1 / ODrive CAN Integration Notes

## Critical: CAN vs CAN-FD Protocol Compatibility

### The Issue
- **ODrive & VESC**: Use **Classic CAN** (11-bit IDs, 8-byte frames max)
- **Moteus N1**: Uses **CAN-FD** (11-bit IDs, 64-byte frames, higher bitrate)

### Will There Be Conflicts?

**SHORT ANSWER**: Likely **YES** if you try to share a single CAN bus. This needs careful hardware configuration.

### Why Conflicts Occur

1. **Bitrate Mismatch**: 
   - ODrive/VESC typically run at 500 kbps (CAN 2.0)
   - Moteus N1 can use CAN-FD at 1 Mbps+ (arbitration) + 5 Mbps (data phase)
   - Mismatched bitrates = corrupted frames / timeouts

2. **Transceiver Compatibility**:
   - Standard CAN transceivers (TJA1050, SN65HVD) = CAN 2.0 only
   - CAN-FD transceivers needed for Moteus (TJA1057, TCAN1462)
   - Some CAN-FD transceivers are backward compatible with CAN 2.0, but not all

3. **Frame Format**:
   - CAN 2.0 frames won't parse correctly as CAN-FD and vice versa
   - The Teensy's CAN3 hardware can handle both, but firmware must match

### Recommended Solutions

#### Option A: **Separate CAN Buses** (RECOMMENDED - Safest)
```
Front CAN Bus (CAN3):
  - ODrive steering motors (CAN 2.0 @ 500 kbps)

Back CAN Bus (CAN1):
  - ODrive steering motors (CAN 2.0 @ 500 kbps)

Separate CAN-FD Bus (CAN2):
  - Moteus N1 drive motors (CAN-FD @ 1 Mbps)
```

**Pros**: No conflicts, each motor type runs optimal protocol
**Cons**: Requires 3 CAN buses + 2 transceivers (you already have 2 for ODrive)

#### Option B: **Shared Unified CAN Bus** (EXPERIMENTAL - Not Recommended)
If you want to try mixed CAN/CAN-FD on one bus:

1. **Use CAN-FD-capable transceivers** on all buses (TJA1057 or TCAN1462)
2. **Run ENTIRE bus at CAN 2.0 mode** (not CAN-FD)
   - Moteus library can be configured to use classic CAN commands instead of CAN-FD
   - Limits Moteus N1 to 8-byte frames only
   - May lose some Moteus N1 advanced features (large telemetry payloads)
3. **Sync all nodes** to same bitrate (500 kbps is common compromise)

**Pros**: Uses fewer CAN buses
**Cons**: Moteus N1 not running at full capabilities, higher risk of timeouts/errors

### Current Project Status

Your hardware (Teensy 4.1 with FlexCAN_T4):
- ✅ Has 3 CAN-FD capable hardware interfaces (CAN1, CAN2, CAN3)
- ❌ Likely using standard CAN transceivers for CAN1/CAN3 (ODrive/VESC)
- ❓ CAN2 availability / transceiver type unknown

### Action Items

**Before deploying Moteus N1**:

1. **Check transceiver specs** on your hardware
   - Look at the CAN shield/board documentation
   - Are they CAN 2.0 only or CAN-FD capable?

2. **Decide on bus topology**:
   - If you want to share buses: need CAN-FD transceivers + configure all nodes to CAN 2.0 mode
   - If you have a separate CAN bus available: Use it (safest option)

3. **Update Moteus config** if mixing on one bus:
   - Modify MoteusN1.h to disable CAN-FD features
   - Set bitrate matching in platformio.ini / setup code

### References
- ODrive CAN Protocol: 500 kbps, Classic CAN
- Vesc CAN Protocol: 500 kbps, Classic CAN
- Moteus N1 CAN-FD: 1 Mbps (arb) / 5 Mbps (data), CAN-FD capable
- Teensy 4.1 Hardware: CAN-FD capable on all 3 CAN ports

## Implementation in This Project

Your project now supports easy bus switching via compile-time configuration in `src/penny_v4.cpp`:

```cpp
// Line ~21 in penny_v4.cpp
#define ODRIVES_ON_CAN3 true
```

### Configuration A (ODRIVES_ON_CAN3 = true)
- **CAN3**: All ODrive steering motors (Classic CAN @ 500 kbps)
- **CAN1**: All Moteus N1 drive motors (CAN-FD @ 1 Mbps)

### Configuration B (ODRIVES_ON_CAN3 = false)
- **CAN1**: All ODrive steering motors (Classic CAN @ 500 kbps)
- **CAN3**: All Moteus N1 drive motors (CAN-FD @ 1 Mbps)

### To Switch Configurations for Debugging

1. Open `src/penny_v4.cpp`
2. Change the `ODRIVES_ON_CAN3` define on line ~21
3. Rebuild: `pio run -e penny_v4`
4. Upload and test

This allows you to quickly verify that both motor types work correctly on either CAN bus without code duplication.
