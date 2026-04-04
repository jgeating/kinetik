import socket
import json
import NTHelper

# ---------------------------------------------------------------------------
# NetworkTables server
# ---------------------------------------------------------------------------
# Start a server so that any NT client (e.g. Shuffleboard, Glass, or the
# robot) can connect and read the published telemetry values.
NT_SERVER_PORT = 5810   # default NT4 port; change if needed

inst = NTHelper.get_instance()
inst.startServer(listen_address="", port4=NT_SERVER_PORT)
print(f"NetworkTables server started on port {NT_SERVER_PORT}")

# Publish a few static / example entries so the server has something to show
# right away even before the first UDP packet arrives.
NTHelper.set_string ("/telemetry/status",           "waiting")
NTHelper.set_double ("/telemetry/packets_received",  0.0)
NTHelper.set_boolean("/telemetry/connected",         False)
NTHelper.set_double_array("/telemetry/swerve/wheel_speeds", [0.0, 0.0, 0.0, 0.0])
NTHelper.set_double_array("/telemetry/swerve/wheel_angles", [0.0, 0.0, 0.0, 0.0])

# ---------------------------------------------------------------------------
# UDP listener
# ---------------------------------------------------------------------------
UDP_IP   = "0.0.0.0"   # bind to all interfaces
UDP_PORT = 8888        # must match m_udpPort on the Teensy


def publish_packet(packet: dict) -> None:
    """Forward every key in a telemetry packet to NetworkTables (logging is handled by NTHelper)."""
    for key, value in packet.items():
        nt_key = f"/telemetry/{key}"

        # bool must be checked before (int, float) because bool is a subclass of int
        if isinstance(value, bool):
            NTHelper.set_boolean(nt_key, value)
        elif isinstance(value, (int, float)):
            NTHelper.set_double(nt_key, float(value))
        elif isinstance(value, str):
            NTHelper.set_string(nt_key, value)
        elif isinstance(value, list) and all(isinstance(v, bool) for v in value):
            NTHelper.set_boolean_array(nt_key, value)
        elif isinstance(value, list) and all(isinstance(v, (int, float)) for v in value):
            NTHelper.set_double_array(nt_key, [float(v) for v in value])
        elif isinstance(value, list) and all(isinstance(v, str) for v in value):
            NTHelper.set_string_array(nt_key, value)
        # Unknown / nested types are skipped; add handling here as needed.


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening for telemetry on UDP port {UDP_PORT} ...")

    packets_received = 0

    while True:
        data, addr = sock.recvfrom(4096)
        try:
            packet = json.loads(data.decode("utf-8"))
            packets_received += 1
            print(f"[{addr[0]}] {packet}")

            # Update bookkeeping (NTHelper also logs each value automatically)
            NTHelper.set_string ("/telemetry/status",          "receiving")
            NTHelper.set_boolean("/telemetry/connected",        True)
            NTHelper.set_double ("/telemetry/packets_received", float(packets_received))

            # Forward the packet payload (NTHelper logs each value automatically)
            publish_packet(packet)

        except json.JSONDecodeError as e:
            print(f"Failed to parse packet from {addr}: {e} | raw: {data}")


if __name__ == "__main__":
    main()
