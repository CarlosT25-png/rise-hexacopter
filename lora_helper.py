"""
LoRa serial listener — receives 1-byte mission IDs from the RX module.

Connection: LoRa RX module -> Jetson USB, /dev/ttyUSB0 @ 115200 baud
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

# 1-byte mission IDs (extend as missions are added in Missions.md)
MISSION_MAP = {
    1: "Hover",
}
# ----------------------------------------------------------------------


def print_mission_map():
    """Print a readable table of byte -> mission name."""
    print()
    print("=" * 42)
    print("  LoRa Mission Byte Map")
    print("=" * 42)
    print(f"  {'Byte':<8} {'Hex':<8} Mission")
    print("  " + "-" * 36)
    for byte_val in sorted(MISSION_MAP):
        name = MISSION_MAP[byte_val]
        print(f"  {byte_val:<8} 0x{byte_val:02X}    {name}")
    print("=" * 42)
    print()


def decode_byte(value):
    """Return the mission name for a received byte, or an unknown label."""
    if value in MISSION_MAP:
        return MISSION_MAP[value]
    return f"Unknown (not in map)"


def listen(port=DEFAULT_PORT, baud=DEFAULT_BAUD):
    """
    Open the LoRa serial port and print each 1-byte message as it arrives.
    Blocks until KeyboardInterrupt.
    """
    print_mission_map()

    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as exc:
        print(f"Cannot open LoRa port {port} @ {baud}: {exc}")
        sys.exit(1)

    print(f"Listening on {port} @ {baud} baud  (Ctrl+C to stop)")
    print("Waiting for 1-byte packets...\n")

    try:
        while True:
            raw = ser.read(1)
            if not raw:
                continue

            value = raw[0]
            mission = decode_byte(value)
            ts = time.strftime("%H:%M:%S")
            print(
                f"[{ts}] LoRa RX: byte={value}  "
                f"hex=0x{value:02X}  ->  {mission}"
            )
    finally:
        ser.close()
        print("\nLoRa listener stopped.")
