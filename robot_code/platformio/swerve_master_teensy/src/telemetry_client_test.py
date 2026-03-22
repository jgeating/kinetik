import socket
import json

UDP_IP = "0.0.0.0"   # Bind to all interfaces
UDP_PORT = 8888      # Must match m_udpPort on the Teensy

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening for telemetry on port {UDP_PORT} ...")

    while True:
        data, addr = sock.recvfrom(1024)
        try:
            packet = json.loads(data.decode("utf-8"))
            print(f"[{addr[0]}] {packet}")
        except json.JSONDecodeError as e:
            print(f"Failed to parse packet from {addr}: {e} | raw: {data}")

if __name__ == "__main__":
    main()
