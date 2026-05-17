"""
Utility script to load, analyze, and visualize NumPy telemetry files.
Usage:
    python analyze_telemetry.py
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import glob

TELEMETRY_DIR = Path("telemetry_data")

def load_session(timestamp_str=None):
    """Load all .npy files from a session."""
    if timestamp_str is None:
        # Find latest session
        files = sorted(glob.glob(str(TELEMETRY_DIR / "data_*.npy")))
        if not files:
            print("No telemetry files found!")
            return None, None
        timestamp_str = files[-1].split('_')[1]  # Extract from filename
    
    data_files = sorted(glob.glob(str(TELEMETRY_DIR / f"data_{timestamp_str}_*.npy")))
    ts_files = sorted(glob.glob(str(TELEMETRY_DIR / f"timestamps_{timestamp_str}_*.npy")))
    
    all_data = np.concatenate([np.load(f) for f in data_files])
    all_ts = np.concatenate([np.load(f) for f in ts_files])
    
    print(f"Loaded {len(all_data)} samples, {len(data_files)} files, duration: {all_ts[-1]:.2f}s")
    return all_data, all_ts

def plot_data(data, timestamps):
    """Plot telemetry data."""
    plt.figure(figsize=(12, 6))
    plt.plot(timestamps, data, linewidth=0.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.title("Telemetry Data")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def stats(data):
    """Print statistics."""
    print(f"  Min: {np.min(data):.6f}")
    print(f"  Max: {np.max(data):.6f}")
    print(f"  Mean: {np.mean(data):.6f}")
    print(f"  Std: {np.std(data):.6f}")

if __name__ == "__main__":
    print("Loading telemetry...")
    data, ts = load_session()
    
    if data is not None:
        print("Statistics:")
        stats(data)
        
        print("\nPlotting...")
        plot_data(data, ts)
        
        # Optional: Save as merged .npy for easy access
        np.save(TELEMETRY_DIR / "merged_data.npy", data)
        np.save(TELEMETRY_DIR / "merged_timestamps.npy", ts)
        print("Saved merged files to telemetry_data/")
