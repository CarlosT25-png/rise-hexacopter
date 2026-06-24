"""
VENATOR MOTOR TEST CODE
SAFETY: PROPS OFF / BARE MOTORS ONLY. Secure the frame before running.

Connection: Jetson J41 UART -> Pixhawk TELEM2, 57600 baud, /dev/ttyACM1
"""

import time
import sys
from pymavlink import mavutil

from lora_helper import listen as listen_lora

# ----------------------------------------------------------------------
# TUNABLE PARAMETERS
# ----------------------------------------------------------------------
SERIAL_PORT   = "/dev/ttyACM1"
RX_LORA_PORT  = "/dev/ttyUSB0"
BAUD          = 57600
BAUD_LORA     = 115200

THROTTLE_PCT  = 5      # % throttle. Bump to ~8-10 if a motor won't spin up.
DURATION_S    = 2      # seconds each motor spins
NUM_MOTORS    = 6      # hexacopter
SETTLE_S      = 1.0    # pause between motors in sequential mode

# DO_MOTOR_TEST throttle type: 0 = percent, 1 = PWM, 2 = pilot passthrough
THROTTLE_TYPE = 0      # percent
# ----------------------------------------------------------------------


def connect():
    print(f"Connecting to {SERIAL_PORT} @ {BAUD}...")
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD)
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat OK — system {master.target_system}, "
          f"component {master.target_component}")
    return master


def motor_test(master, motor_num, throttle, duration):
    """Send a single DO_MOTOR_TEST command for one motor."""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,  # 209
        0,                  # confirmation
        motor_num,          # param1: motor number (1-based, ArduPilot test order)
        THROTTLE_TYPE,      # param2: throttle type (0 = percent)
        throttle,           # param3: throttle value
        duration,           # param4: timeout (seconds)
        0,                  # param5: motor count (0 = single motor)
        0,                  # param6: test order
        0                   # param7: unused
    )


def run_sequential(master):
    print("\n=== SEQUENTIAL TEST: motors 1..6, one at a time ===")
    for m in range(1, NUM_MOTORS + 1):
        print(f"  Motor {m}: {THROTTLE_PCT}% for {DURATION_S}s")
        motor_test(master, m, THROTTLE_PCT, DURATION_S)
        time.sleep(DURATION_S + SETTLE_S)
    print("Sequential test complete.")


def run_simultaneous(master):
    print("\n=== SIMULTANEOUS TEST: all 6 motors together ===")
    print(f"  All motors: {THROTTLE_PCT}% for {DURATION_S}s")
    for m in range(1, NUM_MOTORS + 1):
        motor_test(master, m, THROTTLE_PCT, DURATION_S)
        time.sleep(0.02)  # 20ms breather for the UART buffer
    time.sleep(DURATION_S + 0.5)
    print("Simultaneous test complete.")


def run_both(master):
    run_sequential(master)
    time.sleep(2)
    run_simultaneous(master)


def handle_lora_msg(master, msg):
    """Run a motor test based on the LoRa msg field; ignore anything else."""
    if msg == "1":
        run_sequential(master)
    elif msg == "2":
        run_simultaneous(master)
    elif msg == "3":
        run_both(master)


def main():
    print("=" * 55)
    print("VENATOR HEXACOPTER — MOTOR TEST / LoRa LISTENER")
    print("=" * 55)
    print("\nSelect mode:")
    print("  1) Sequential motor test (1..6 one at a time)")
    print("  2) Simultaneous motor test (all 6 together)")
    print("  3) Both motor tests (sequential, then simultaneous)")
    print("  4) Listen to LoRa (JSON commands: 1=sequential, 2=simultaneous, 3=both)")
    choice = input("Choice [1/2/3/4]: ").strip()

    if choice in ("1", "2", "3", "4"):
        ans = input("Props removed and frame secured? (yes/no): ").strip().lower()
        if ans != "yes":
            print("Aborting. Remove props first.")
            sys.exit(1)

    if choice == "4":
        master = connect()
        listen_lora(
            port=RX_LORA_PORT,
            baud=BAUD_LORA,
            on_msg=lambda msg: handle_lora_msg(master, msg),
        )
        return

    master = connect()

    if choice == "1":
        run_sequential(master)
    elif choice == "2":
        run_simultaneous(master)
    elif choice == "3":
        run_both(master)
    else:
        print("Invalid choice. Exiting.")
        sys.exit(1)

    print("\nDone. Motors should be stopped.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user. Motors will time out and stop.")