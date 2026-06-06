#!/usr/bin/env bash
# Upload firmware, capture serial startup, and ping the Teensy IP to verify
# Usage: ./scripts/upload_and_verify.sh [SERIAL_PORT] [TEENSY_IP]
# Defaults: /dev/cu.usbmodem161557301 192.168.86.100

set -euo pipefail
SERIAL_PORT=${1-/dev/cu.usbmodem161557301}
TEENSY_IP=${2-192.168.86.100}
UPLOAD_ENV=${PIO_ENV:-penny_v4}

echo "Uploading firmware (env=$UPLOAD_ENV)..."
if ! platformio run -e $UPLOAD_ENV -t upload; then
  echo "Upload failed. If the Teensy didn't enter bootloader, press the reset button and run again."
  exit 2
fi

echo "Upload reported SUCCESS. Reading serial for startup messages (5s)..."
python3 - <<PY
import serial, time, sys
try:
    ser = serial.Serial('$SERIAL_PORT', 460800, timeout=0.5)
except Exception as e:
    print('Failed to open serial port:', e)
    sys.exit(3)
start = time.time()
found = False
while time.time() - start < 5:
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
            if '[Telemetry] SUCCESS' in line or 'Ethernet initialized' in line:
                found = True
    except Exception:
        pass
    time.sleep(0.05)
ser.close()
if not found:
    print('\nNo telemetry initialization message seen on serial.')
else:
    print('\nTelemetry initialization message observed on serial.')
PY

echo "Pinging $TEENSY_IP (3 packets)..."
if ping -c 3 -W 1 $TEENSY_IP >/dev/null 2>&1; then
  echo "Ping successful: $TEENSY_IP is reachable on the network."
else
  echo "Ping failed: $TEENSY_IP not reachable. Teensy may not have gotten the configured IP or router blocked traffic."
fi

echo "Done." 
