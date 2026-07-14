#!/usr/bin/env python3
"""Flash penny_v4 via BOOTLOAD serial trigger, then capture serial output."""
import argparse
import glob
import subprocess
import sys
import time

import serial


def find_port(timeout=15):
    end = time.time() + timeout
    while time.time() < end:
        ports = glob.glob('/dev/cu.usbmodem*')
        if ports:
            return ports[0]
        time.sleep(0.3)
    return None


def send_bootload(port):
    try:
        s = serial.Serial(port, 115200, timeout=1)
        s.write(b'BOOTLOAD\n')
        s.flush()
        time.sleep(0.5)
        s.close()
        return True
    except (OSError, serial.SerialException) as e:
        print(f'[script] bootload send failed: {e}')
        return False


def upload():
    r = subprocess.run(
        ['pio', 'run', '-e', 'penny_v4', '-t', 'upload'],
        capture_output=True, text=True)
    ok = r.returncode == 0
    tail = '\n'.join((r.stdout + r.stderr).splitlines()[-6:])
    print(f'[script] upload {"OK" if ok else "FAILED"}\n{tail}')
    return ok


def monitor(seconds):
    end = time.time() + seconds
    s = None
    while time.time() < end:
        if s is None:
            port = find_port(timeout=max(1, end - time.time()))
            if not port:
                break
            try:
                s = serial.Serial(port, 115200, timeout=0.5)
                print(f'[script] monitoring {port}')
            except (OSError, serial.SerialException):
                time.sleep(0.3)
                continue
        try:
            line = s.readline().decode(errors='replace').strip()
            if line:
                print(line)
        except (OSError, serial.SerialException):
            print('[script] port dropped, reconnecting...')
            try:
                s.close()
            except Exception:
                pass
            s = None
            time.sleep(0.3)
    if s:
        s.close()


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--no-flash', action='store_true')
    p.add_argument('--seconds', type=float, default=8)
    args = p.parse_args()

    if not args.no_flash:
        port = find_port()
        if not port:
            sys.exit('[script] no Teensy serial port found')
        send_bootload(port)
        time.sleep(1.5)
        if not upload():
            sys.exit(1)
        time.sleep(0.5)
    monitor(args.seconds)
