#!/usr/bin/env python3
"""
Offline telemetry diagnostic for the Teensy swerve robot.

Run this AFTER joining the `penny_robot_5g` (or `penny_robot`) WiFi.
No internet required. It will:
  1. Show your Mac's IP/subnet on the penny_robot network.
  2. Read the Teensy serial heartbeat to learn its DHCP-assigned IP.
  3. Listen on UDP :8888 for telemetry packets and report what arrives.

Usage:
    python3 offline_diag.py
    python3 offline_diag.py --seconds 30
"""

import argparse
import glob
import socket
import subprocess
import sys
import time

UDP_PORT = 8888


def show_local_network():
    print("=== Mac network interfaces (IPv4) ===")
    try:
        out = subprocess.run(["ifconfig"], capture_output=True, text=True).stdout
        for line in out.splitlines():
            if "inet " in line and "127.0.0.1" not in line:
                print("  " + line.strip())
    except Exception as e:
        print(f"  (could not read ifconfig: {e})")
    try:
        gw = subprocess.run(
            ["route", "-n", "get", "default"], capture_output=True, text=True
        ).stdout
        for line in gw.splitlines():
            if "gateway" in line or "interface" in line:
                print("  " + line.strip())
    except Exception:
        pass
    print()


def read_teensy_ip(seconds=8):
    """Read the serial heartbeat to find the Teensy's assigned IP."""
    try:
        import serial  # pyserial
    except ImportError:
        print("[serial] pyserial not installed; skipping serial read.")
        print("         (pip3 install pyserial)  -- not required if UDP works.")
        return None

    print(f"=== Reading Teensy serial for {seconds}s (looking for [HB] line) ===")
    end = time.time() + seconds
    port = None
    found_ip = None
    while time.time() < end:
        if port is None:
            for cand in sorted(glob.glob("/dev/cu.usbmodem*")):
                try:
                    port = serial.Serial(cand, 115200, timeout=0.2)
                    break
                except Exception:
                    port = None
            if port is None:
                time.sleep(0.2)
                continue
        try:
            line = port.readline().decode(errors="replace").strip()
            if not line:
                continue
            if "[HB]" in line or "[Telemetry]" in line:
                print("  " + line)
            if "[HB]" in line and "ip=" in line:
                ip = line.split("ip=")[1].split()[0]
                if ip and ip != "0.0.0.0":
                    found_ip = ip
        except Exception:
            try:
                port.close()
            except Exception:
                pass
            port = None
            time.sleep(0.3)
    if found_ip:
        print(f"\n  >>> Teensy IP (from heartbeat): {found_ip}")
        if found_ip.startswith("192.168.86."):
            print("  >>> WARNING: Teensy still on 192.168.86.x (home net).")
            print("      Confirm the cable goes to the penny_robot router.")
    else:
        print("\n  >>> No assigned IP seen yet (DHCP may still be retrying).")
    print()
    return found_ip


def listen_udp(seconds=15):
    print(f"=== Listening on UDP 0.0.0.0:{UDP_PORT} for {seconds}s ===")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(1.0)
    end = time.time() + seconds
    count = 0
    first_from = None
    while time.time() < end:
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        count += 1
        if first_from is None:
            first_from = addr
            preview = data[:120].decode(errors="replace")
            print(f"  First packet from {addr}: {preview}")
    sock.close()
    print(f"\n  >>> Total packets received: {count}")
    if count:
        print(f"  >>> SUCCESS: telemetry is flowing from {first_from[0]}.")
    else:
        print("  >>> No packets. Checklist:")
        print("      - Mac joined penny_robot_5g? (not home WiFi)")
        print("      - Teensy heartbeat shows a non-zero IP on this subnet?")
        print("      - macOS firewall not blocking python? "
              "(System Settings > Network > Firewall)")
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=15,
                    help="UDP listen duration (default 15)")
    ap.add_argument("--serial-seconds", type=int, default=8,
                    help="serial read duration (default 8)")
    args = ap.parse_args()

    show_local_network()
    read_teensy_ip(args.serial_seconds)
    listen_udp(args.seconds)
    print("Done. If no packets arrived, re-run after confirming the WiFi/IP above.")


if __name__ == "__main__":
    sys.exit(main())
