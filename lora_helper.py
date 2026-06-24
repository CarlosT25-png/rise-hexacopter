"""
LoRa serial listener — prints text received from the RX module.

Connection: LoRa RX module -> Jetson USB, /dev/ttyUSB0 @ 115200 baud
Messages are read as UTF-8 text, one line per packet (newline-terminated).
"""

import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required for LoRa. Run: pip install pyserial")
    sys.exit(1)

# ----------------------------------------------------------------------
# TUNABLE PARAMETERS
# ----------------------------------------------------------------------
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
# ----------------------------------------------------------------------


def listen(port=DEFAULT_PORT, baud=DEFAULT_BAUD):
    """
    Open the LoRa serial port and print each received text line.
    Blocks until KeyboardInterrupt.
    """
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as exc:
        print(f"Cannot open LoRa port {port} @ {baud}: {exc}")
        sys.exit(1)

    print(f"Listening on {port} @ {baud} baud  (Ctrl+C to stop)\n")

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                continue

            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] {text}")
    finally:
        ser.close()
        print("\nLoRa listener stopped.")
