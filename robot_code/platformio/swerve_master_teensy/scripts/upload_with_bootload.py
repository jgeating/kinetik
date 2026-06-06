#!/usr/bin/env python3
"""
Autonomous Teensy upload script with software bootload trigger.
Sends BOOTLOAD command via serial, waits for bootloader, then runs platformio upload.
No manual reset needed.
"""

import sys
import glob
import time
import subprocess
import serial

def find_teensy_port():
    """Find the first available Teensy serial port."""
    for port in sorted(glob.glob('/dev/cu.usbmodem*')):
        try:
            s = serial.Serial(port, 115200, timeout=0.5)
            s.close()
            return port
        except Exception:
            pass
    return None

def send_bootload_command(port):
    """Send BOOTLOAD command to trigger bootloader mode."""
    print(f"[*] Sending BOOTLOAD to {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(0.5)
        ser.write(b"BOOTLOAD\n")
        ser.close()
        print("[+] BOOTLOAD sent. Waiting for bootloader...")
        time.sleep(1.5)
        return True
    except Exception as e:
        print(f"[-] Failed to send BOOTLOAD: {e}")
        return False

def run_platformio_upload(env="penny_v4"):
    """Run platformio upload command."""
    print(f"[*] Running platformio upload for {env}...")
    result = subprocess.run(
        ["platformio", "run", "-e", env, "-t", "upload"],
        cwd="."
    )
    return result.returncode == 0

def main():
    if len(sys.argv) > 1:
        env = sys.argv[1]
    else:
        env = "penny_v4"

    # Find Teensy
    port = find_teensy_port()
    if not port:
        print("[-] No Teensy found. Is it connected via USB?")
        return 1

    print(f"[+] Found Teensy at {port}")

    # Send bootload command
    if not send_bootload_command(port):
        print("[-] Could not communicate with Teensy")
        return 1

    # Run upload
    if run_platformio_upload(env):
        print("[+] Upload successful!")
        return 0
    else:
        print("[-] Upload failed")
        return 1

if __name__ == "__main__":
    sys.exit(main())
