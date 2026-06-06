# Serial Telemetry Client

Live GUI that reads JSON telemetry from the robot over USB serial and plots the centre-of-pressure (CoP) position in real time.

## Files

| File | Purpose |
|---|---|
| `telemetry_client.py` | `TelemetryClient` class — handles serial connection and fires a callback per key/value |
| `test.py` | PyQt5/pyqtgraph GUI that plots CoP dots and trail |
| `requirements.txt` | Python dependencies |

## Setup

**1. Create the virtual environment** (one time only):

```powershell
# Windows
python -m venv .venv

# macOS / Linux
python3 -m venv .venv
```

**2. Install dependencies:**

```powershell
# Windows
.venv\Scripts\pip install -r requirements.txt

# macOS / Linux
.venv/bin/pip install -r requirements.txt
```

## Running

Activate the venv first, then run `test.py`:

```powershell
# Windows
.venv\Scripts\activate
python test.py

# macOS / Linux
source .venv/bin/activate
python test.py
```

Or run directly without activating:

```powershell
# Windows
.venv\Scripts\python test.py

# macOS / Linux
.venv/bin/python test.py
```

### Options

| Flag | Default | Description |
|---|---|---|
| `--port` | auto-detect | Serial port (e.g. `COM3`, `/dev/cu.usbmodem14101`) |
| `--baud` | `115200` | Baud rate |

```powershell
# Windows
python test.py --port COM5
python test.py --port COM5 --baud 921600

# macOS / Linux
python test.py --port /dev/cu.usbmodem14101
python test.py --port /dev/ttyACM0 --baud 921600
```

## Using `TelemetryClient` in your own script

```python
from telemetry_client import TelemetryClient

def on_value(key, value):
    print(f"{key} = {value}")

client = TelemetryClient(port="COM3")
client.on_update(on_value)
client.run()  # blocks until Ctrl-C
```
