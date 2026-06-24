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

`main.py` auto-starts MAVProxy by default and forwards to `127.0.0.1:14551` and `192.168.1.100:14550`.

#### Run the code

```bash
source .venv/bin/activate
python3 main.py
```

If MAVProxy is already running in another terminal, use `--no-start-mavproxy`:

```bash
python3 main.py --no-start-mavproxy
```

Direct serial (no MAVProxy; QGC won't receive telemetry):

```bash
python3 main.py --serial /dev/ttyTHS1 --baudrate 57600
```
