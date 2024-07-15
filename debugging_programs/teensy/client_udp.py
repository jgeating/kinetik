import socket
import struct

format = "<ff"

def udp_client():
    UDP_IP = "192.168.0.5"  # Server IP address
    UDP_PORT = 8888       # Server port number
    MESSAGE = b"Hello, Server!"

    print(f"UDP target IP: {UDP_IP}")
    print(f"UDP target port: {UDP_PORT}")
    print(f"Message: {MESSAGE}")

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))


    buffer = b''

    while True:
        data, server = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        (position, velocity) = struct.unpack(format, data)
        print("data:", position, velocity)

if __name__ == "__main__":
    udp_client()