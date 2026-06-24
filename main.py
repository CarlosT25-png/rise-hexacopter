"""
VENATOR HEXACOPTER — motor test, hover flight, and LoRa control.

Connection: Jetson J41 UART -> Pixhawk TELEM2, 57600 baud, /dev/ttyACM1
"""

import time
import sys
import threading
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

TAKEOFF_ALT_M = 5.0    # meters above home for hover flight
HOVER_TIME_S  = 30     # seconds to hold altitude before landing
ALT_TOLERANCE_M = 0.5  # meters — considered "at altitude" within this band
TAKEOFF_TIMEOUT_S = 60 # max seconds to wait to reach takeoff altitude
PREARM_TIMEOUT_S = 120 # max seconds to wait for ArduPilot pre-arm checks
ARM_TIMEOUT_S    = 30  # max seconds to retry arming
# ----------------------------------------------------------------------


def connect():
    print(f"Connecting to {SERIAL_PORT} @ {BAUD}...")
    master = mavutil.mavlink_connection(
        SERIAL_PORT,
        baud=BAUD,
        source_system=255,
        source_component=0,
    )
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat OK — system {master.target_system}, "
          f"component {master.target_component}")
    start_gcs_heartbeat(master)
    return master


def start_gcs_heartbeat(master):
    """Act as a GCS so ArduPilot sees a ground station (like QGroundControl)."""
    def _loop():
        while True:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
            time.sleep(1)

    threading.Thread(target=_loop, daemon=True).start()


def drain_statustext(master):
    while True:
        msg = master.recv_match(type="STATUSTEXT", blocking=False)
        if msg is None:
            break
        print(f"  FC: {msg.text}")


def wait_prearm_ok(master, timeout_s=PREARM_TIMEOUT_S):
    print("Waiting for ArduPilot pre-arm checks...")
    prearm_bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
    start = time.time()
    last_update = 0.0

    while time.time() - start < timeout_s:
        drain_statustext(master)
        msg = master.recv_match(type="SYS_STATUS", blocking=True, timeout=1)
        if msg is None:
            continue

        healthy = (
            (msg.onboard_control_sensors_present & prearm_bit)
            and (msg.onboard_control_sensors_enabled & prearm_bit)
            and (msg.onboard_control_sensors_health & prearm_bit)
        )
        if healthy:
            print("Pre-arm checks passed.")
            return True

        now = time.time()
        if now - last_update >= 5:
            print("  Still waiting for pre-arm (GPS/EKF/compass, etc.)...")
            last_update = now

    drain_statustext(master)
    return False


def arm_vehicle(master, timeout_s=ARM_TIMEOUT_S):
    ack_results = {
        mavutil.mavlink.MAV_RESULT_DENIED: "DENIED (pre-arm check failed)",
        mavutil.mavlink.MAV_RESULT_FAILED: "FAILED",
        mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: "TEMPORARILY_REJECTED",
    }

    print("Arming...")
    start = time.time()
    last_arm_send = 0.0

    while time.time() - start < timeout_s:
        drain_statustext(master)
        if master.motors_armed():
            print("Armed.")
            return True

        if time.time() - last_arm_send >= 2:
            master.arducopter_arm()
            last_arm_send = time.time()

        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Arm command accepted.")
            elif ack.result != mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                reason = ack_results.get(ack.result, f"code {ack.result}")
                print(f"  Arm rejected: {reason}")

    drain_statustext(master)
    return False


def set_mode_and_wait(master, mode, timeout_s=10):
    master.set_mode_apm(mode)
    start = time.time()
    while time.time() - start < timeout_s:
        if master.flightmode == mode:
            return True
        time.sleep(0.2)
    return False


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


def run_hover_flight(master):
    print(f"\n=== HOVER FLIGHT: {TAKEOFF_ALT_M} m for {HOVER_TIME_S} s ===")

    if not wait_prearm_ok(master):
        print("Pre-arm checks did not pass in time.")
        print("Common causes: no GPS fix, EKF not ready, safety switch, throttle not low.")
        sys.exit(1)

    print("Setting GUIDED mode...")
    if not set_mode_and_wait(master, "GUIDED"):
        print(f"Failed to enter GUIDED mode (current: {master.flightmode}).")
        sys.exit(1)

    if not arm_vehicle(master):
        print("Failed to arm.")
        print("Check the FC messages above (PreArm: ...) and fix the reported issue.")
        sys.exit(1)

    print(f"Takeoff to {TAKEOFF_ALT_M} m...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        TAKEOFF_ALT_M,
    )

    if not wait_for_altitude(master, TAKEOFF_ALT_M):
        print("Takeoff timed out. Switching to LAND.")
        master.set_mode_apm("LAND")
        sys.exit(1)

    print(f"Hovering for {HOVER_TIME_S} s...")
    hover_end = time.time() + HOVER_TIME_S
    while time.time() < hover_end:
        alt_m = get_relative_alt_m(master)
        remaining = int(hover_end - time.time())
        if alt_m is not None:
            print(f"  Hover: {alt_m:.1f} m  ({remaining}s left)")
        time.sleep(1)

    print("Landing...")
    master.set_mode_apm("LAND")
    time.sleep(5)
    print("Hover flight complete.")


def get_relative_alt_m(master):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
    if msg is None:
        return None
    return msg.relative_alt / 1000.0


def wait_for_altitude(master, target_alt_m, tolerance_m=ALT_TOLERANCE_M,
                      timeout_s=TAKEOFF_TIMEOUT_S):
    start = time.time()
    while time.time() - start < timeout_s:
        alt_m = get_relative_alt_m(master)
        if alt_m is not None:
            print(f"  Altitude: {alt_m:.1f} m")
            if alt_m >= target_alt_m - tolerance_m:
                return True
    return False


def handle_lora_msg(master, msg):
    """Run a command based on the LoRa msg field; ignore anything else."""
    if msg == "1":
        run_sequential(master)
    elif msg == "2":
        run_simultaneous(master)
    elif msg == "3":
        run_both(master)
    elif msg == "4":
        run_hover_flight(master)


def main():
    print("=" * 55)
    print("VENATOR HEXACOPTER — MOTOR TEST / FLIGHT / LoRa")
    print("=" * 55)
    print("\nSelect mode:")
    print("  1) Sequential motor test (1..6 one at a time)")
    print("  2) Simultaneous motor test (all 6 together)")
    print("  3) Both motor tests (sequential, then simultaneous)")
    print(f"  4) Hover flight ({TAKEOFF_ALT_M} m for {HOVER_TIME_S} s)")
    print(f"  5) Listen to LoRa (JSON: 1=sequential, 2=simultaneous, 3=both, 4=hover)")
    choice = input("Choice [1/2/3/4/5]: ").strip()

    if choice in ("1", "2", "3"):
        ans = input("Props removed and frame secured? (yes/no): ").strip().lower()
        if ans != "yes":
            print("Aborting. Remove props first.")
            sys.exit(1)
    elif choice == "5":
        ans = input(
            "Ready to listen? msgs 1-3 need props off; msg 4 hover needs props on. (yes/no): "
        ).strip().lower()
        if ans != "yes":
            print("Aborting.")
            sys.exit(1)
    elif choice == "4":
        ans = input("Area clear, props on, and ready to fly? (yes/no): ").strip().lower()
        if ans != "yes":
            print("Aborting.")
            sys.exit(1)
    else:
        print("Invalid choice. Exiting.")
        sys.exit(1)

    master = connect()

    if choice == "1":
        run_sequential(master)
    elif choice == "2":
        run_simultaneous(master)
    elif choice == "3":
        run_both(master)
    elif choice == "4":
        run_hover_flight(master)
    elif choice == "5":
        listen_lora(
            port=RX_LORA_PORT,
            baud=BAUD_LORA,
            on_msg=lambda msg: handle_lora_msg(master, msg),
        )
        return

    print("\nDone.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user. Motors will time out and stop.")