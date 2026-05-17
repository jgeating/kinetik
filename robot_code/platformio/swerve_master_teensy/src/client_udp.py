import socket
import struct
import time
import numpy as np
from collections import deque
from pathlib import Path

UDP_IP = "192.168.0.5"  # Your laptop IP
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Data storage
data_buffer = []
timestamps_buffer = []
BUFFER_SIZE = 50000  # Save to disk every 50k samples
SAVE_DIR = Path("telemetry_data")
SAVE_DIR.mkdir(exist_ok=True)

# Session info
start_time = time.time()
file_counter = 0

def save_buffer():
    """Save accumulated data to NumPy files."""
    global file_counter, data_buffer, timestamps_buffer
    
    if len(data_buffer) == 0:
        return
    
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    data_path = SAVE_DIR / f"data_{timestamp_str}_{file_counter:03d}.npy"
    ts_path = SAVE_DIR / f"timestamps_{timestamp_str}_{file_counter:03d}.npy"
    
    np.save(data_path, np.array(data_buffer, dtype=np.float64))
    np.save(ts_path, np.array(timestamps_buffer, dtype=np.float64))
    
    print(f"Saved {len(data_buffer)} samples to {data_path}")
    data_buffer = []
    timestamps_buffer = []
    file_counter += 1

print(f"Listening for telemetry on {UDP_IP}:{UDP_PORT}")
print(f"Data will be saved to {SAVE_DIR}/")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        num_doubles = len(data) // 8
        doubles = struct.unpack('<' + 'd' * num_doubles, data)
        
        current_time = time.time() - start_time
        
        for value in doubles:
            print(f"[{current_time:.3f}] {value:.6f}")
            data_buffer.append(value)
            timestamps_buffer.append(current_time)
        
        # Save when buffer is full
        if len(data_buffer) >= BUFFER_SIZE:
            save_buffer()

except KeyboardInterrupt:
    print("\nShutting down...")
    save_buffer()  # Save remaining data
    sock.close()
    print("Done. Load data with: np.load('telemetry_data/data_*.npy')")