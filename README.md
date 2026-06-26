# RISE LAB - Autonomous Hexacopter Drone

## How to clone this repo

```bash
git clone --recurse-submodules https://github.com/CarlosT25-png/rise-hexacopter.git
```

If you already cloned without submodules:

```bash
cd rise-hexacopter && git submodule update --init --recursive
```

## Run the code

### MacOS/Windows (Simulation - SITL)

#### Prerequisites

- Python 3.7+
- [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)

#### Setup

1. Create and activate a virtual environment, then install dependencies:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

1. Open QGroundControl.
2. Run the SITL simulator:

```bash
./ardupilot/Tools/autotest/sim_vehicle.py -v Copter -f hexa --console --out=127.0.0.1:14551
```

1. Run the flight script:

```bash
python main.py --no-start-mavproxy
```

1. Verify the drone's path in QGroundControl.

### Jetson Nano

#### Setup

1. Install MAVProxy (only the first time):

```bash
sudo apt update
sudo apt install -y python3-pip

sudo apt install -y python3-dev python3-wxgtk4.0 build-essential

sudo python3 -m pip install --upgrade pip
sudo python3 -m pip install numpy pyparsing wxPython gnureadline billiard

sudo python3 -m pip install MAVProxy
mavproxy.py --version
```

1. Copy this project to the Jetson (do not include the `.venv` folder).
2. Create a virtual environment and install dependencies:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Pixhawk connection defaults are in `main.py` (`SERIAL_PORT="auto"`, `BAUD`). LoRa: `RX_LORA_PORT`, `BAUD_LORA`. Override Pixhawk port with `--serial /dev/ttyACM1`.

#### Run the code

```bash
source .venv/bin/activate
python3 main.py
```

Interactive menu options: sequential motor test, simultaneous motor test, hover flight, and LoRa RX.

To start the LoRa listener directly (no menu):

```bash
source .venv/bin/activate
python3 main.py --lora
```

LoRa JSON commands:

| `msg` | Action |
|-------|--------|
| `"1"` | Sequential motor test |
| `"2"` | Simultaneous motor test |
| `"3"` | Both motor tests |
| `"4"` | Hover flight |

#### LoRa RX systemd service (auto-restart on failure)

Use this to run the LoRa listener in the background on boot and restart it if the process crashes (serial disconnect, Pixhawk link lost, etc.).

1. Create the service file (adjust paths and user if needed):

```bash
sudo nano /etc/systemd/system/venator-lora.service
```

2. Paste the following, updating `User`, `WorkingDirectory`, and `ExecStart` to match your Jetson setup:

```ini
[Unit]
Description=Venator LoRa RX listener
After=network.target systemd-udev-settle.service
Wants=systemd-udev-settle.service

[Service]
Type=simple
User=jetson
WorkingDirectory=/home/jetson/hexacopter
Environment=PYTHONUNBUFFERED=1
ExecStart=/home/jetson/hexacopter/.venv/bin/python3 -u main.py --lora
Restart=on-failure
RestartSec=5
StandardInput=null
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

3. Enable and start the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable venator-lora.service
sudo systemctl start venator-lora.service
```

`systemctl start` only prints **Started venator-lora.service** — that means systemd launched the process, not that LoRa is already listening. View the application logs with:

```bash
sudo systemctl restart venator-lora.service
sudo journalctl -u venator-lora.service -f
```

You should see lines like:

```text
Venator LoRa RX starting...
Auto-detecting Pixhawk (3 port(s) to try)...
  Trying /dev/ttyACM2...
Pixhawk heartbeat OK on /dev/ttyACM2 ...
Listening on /dev/ttyUSB0 @ 115200 baud
Ready — waiting for LoRa packets...
```

4. Useful commands:

```bash
sudo systemctl status venator-lora.service   # check status
sudo journalctl -u venator-lora.service -f   # follow logs
sudo systemctl restart venator-lora.service  # manual restart
sudo systemctl stop venator-lora.service     # stop
```

The service exits with code 0 on a normal stop and code 1 on unexpected errors, so `Restart=on-failure` will restart after crashes but not after a deliberate `systemctl stop`.

**Important:** `ExecStart` must include `--lora`. Do **not** pass `--serial /dev/ttyACM0` unless you want to force that port — leave it out so Pixhawk is auto-detected on `/dev/ttyACM*`.

If you see `could not open port /dev/ttyACM0`, the Jetson is running an old `main.py`. Pull the latest code and confirm `SERIAL_PORT = "auto"` in `main.py`.
