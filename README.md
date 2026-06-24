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
python main.py
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

If `grpcio` fails to install from pip on ARM, install it from apt first:

```bash
sudo apt install -y python3-grpcio
pip install -r requirements.txt
```

#### Run the code

1. Run MAVProxy (expects the Jetson is connected to the Pixhawk through the GPIO pin):

```bash
mavproxy.py --master=/dev/ttyTHS1 --baudrate=57600 --out=udp:127.0.0.1:14551 --out=udp:192.168.1.100:14550
```

1. Activate the Python virtual environment:

```bash
source .venv/bin/activate
```

1. Run the flight script:

```bash
python3 main.py
```
