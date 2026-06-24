"""
VENATOR HEXACOPTER — motor test, hover flight, and LoRa control.

Connection: Jetson J41 UART -> Pixhawk TELEM2, 57600 baud, /dev/ttyACM1
"""

import argparse
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


def setup_service_logging():
    """Print to journal immediately when running under systemd (no TTY)."""
    try:
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    except (AttributeError, OSError):
        pass


def log(msg):
    print(msg, flush=True)


def connect():
    log(f"Connecting to Pixhawk {SERIAL_PORT} @ {BAUD}...")
    master = mavutil.mavlink_connection(
        SERIAL_PORT,
        baud=BAUD,
        source_system=255,
        source_component=0,
    )
    master.mavlink_lock = threading.Lock()
    log("Waiting for Pixhawk heartbeat...")
    with master.mavlink_lock:
        master.wait_heartbeat()
    if master.target_component in (0, mavutil.mavlink.MAV_COMP_ID_ALL):
        master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
    log(f"Pixhawk heartbeat OK — system {master.target_system}, "
        f"component {master.target_component}")
    start_gcs_heartbeat(master)
    return master


def start_gcs_heartbeat(master):
    """Act as a GCS so ArduPilot sees a ground station (like QGroundControl)."""
    def _loop():
        while True:
            with master.mavlink_lock:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
            time.sleep(1)

    threading.Thread(target=_loop, daemon=True).start()


def recv_match_locked(master, **kwargs):
    with master.mavlink_lock:
        return master.recv_match(**kwargs)


def drain_statustext(master):
    while True:
        msg = recv_match_locked(master, type="STATUSTEXT", blocking=False)
        if msg is None:
            break
        log(f"  FC: {msg.text}")


def wait_prearm_ok(master, timeout_s=PREARM_TIMEOUT_S):
    print("Waiting for ArduPilot pre-arm checks...")
    prearm_bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
    start = time.time()
    last_update = 0.0

    while time.time() - start < timeout_s:
        drain_statustext(master)
        msg = recv_match_locked(master, type="SYS_STATUS", blocking=True, timeout=1)
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
            with master.mavlink_lock:
                master.arducopter_arm()
            last_arm_send = time.time()

        ack = recv_match_locked(master, type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Arm command accepted.")
            elif ack.result != mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                reason = ack_results.get(ack.result, f"code {ack.result}")
                print(f"  Arm rejected: {reason}")

    drain_statustext(master)
    return False


def set_mode_and_wait(master, mode, timeout_s=10):
    with master.mavlink_lock:
        master.set_mode_apm(mode)
    start = time.time()
    while time.time() - start < timeout_s:
        recv_match_locked(master, type="HEARTBEAT", blocking=True, timeout=0.2)
        if master.flightmode == mode:
            return True
        time.sleep(0.2)
    return False


def wait_command_ack(master, command, timeout_s=3):
    start = time.time()
    while time.time() - start < timeout_s:
        ack = recv_match_locked(master, type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack and ack.command == command:
            return ack.result
    return None


def motor_test(master, motor_num, throttle, duration):
    """Send a single DO_MOTOR_TEST command for one motor."""
    with master.mavlink_lock:
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
    result = wait_command_ack(master, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST)
    if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        log(f"  Motor {motor_num}: command accepted")
        return True
    if result is None:
        log(f"  Motor {motor_num}: no ACK from Pixhawk (check serial link)")
    else:
        log(f"  Motor {motor_num}: rejected (MAV_RESULT={result})")
    drain_statustext(master)
    return False


def run_sequential(master):
    log("=== SEQUENTIAL TEST: motors 1..6, one at a time ===")
    for m in range(1, NUM_MOTORS + 1):
        log(f"  Motor {m}: {THROTTLE_PCT}% for {DURATION_S}s")
        motor_test(master, m, THROTTLE_PCT, DURATION_S)
        time.sleep(DURATION_S + SETTLE_S)
    log("Sequential test complete.")


def run_simultaneous(master):
    log("=== SIMULTANEOUS TEST: all 6 motors together ===")
    log(f"  All motors: {THROTTLE_PCT}% for {DURATION_S}s")
    for m in range(1, NUM_MOTORS + 1):
        motor_test(master, m, THROTTLE_PCT, DURATION_S)
        time.sleep(0.02)  # 20ms breather for the UART buffer
    time.sleep(DURATION_S + 0.5)
    log("Simultaneous test complete.")


def run_both(master):
    run_sequential(master)
    time.sleep(2)
    run_simultaneous(master)


def run_hover_flight(master, exit_on_error=True):
    log(f"=== HOVER FLIGHT: {TAKEOFF_ALT_M} m for {HOVER_TIME_S} s ===")

    if not wait_prearm_ok(master):
        log("Pre-arm checks did not pass in time.")
        log("Common causes: no GPS fix, EKF not ready, safety switch, throttle not low.")
        if exit_on_error:
            sys.exit(1)
        return False

    log("Setting GUIDED mode...")
    if not set_mode_and_wait(master, "GUIDED"):
        log(f"Failed to enter GUIDED mode (current: {master.flightmode}).")
        if exit_on_error:
            sys.exit(1)
        return False

    if not arm_vehicle(master):
        log("Failed to arm.")
        log("Check the FC messages above (PreArm: ...) and fix the reported issue.")
        if exit_on_error:
            sys.exit(1)
        return False

    log(f"Takeoff to {TAKEOFF_ALT_M} m...")
    with master.mavlink_lock:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            TAKEOFF_ALT_M,
        )

    if not wait_for_altitude(master, TAKEOFF_ALT_M):
        log("Takeoff timed out. Switching to LAND.")
        with master.mavlink_lock:
            master.set_mode_apm("LAND")
        if exit_on_error:
            sys.exit(1)
        return False

    log(f"Hovering for {HOVER_TIME_S} s...")
    hover_end = time.time() + HOVER_TIME_S
    while time.time() < hover_end:
        alt_m = get_relative_alt_m(master)
        remaining = int(hover_end - time.time())
        if alt_m is not None:
            log(f"  Hover: {alt_m:.1f} m  ({remaining}s left)")
        time.sleep(1)

    log("Landing...")
    with master.mavlink_lock:
        master.set_mode_apm("LAND")
    time.sleep(5)
    log("Hover flight complete.")
    return True


def get_relative_alt_m(master):
    msg = recv_match_locked(master, type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
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


def normalize_lora_msg(msg):
    text = str(msg).strip()
    if text.endswith(".0") and text[:-2].isdigit():
        text = text[:-2]
    return text


def handle_lora_msg(master, msg):
    """Run a command based on the LoRa msg field; ignore anything else."""
    msg = normalize_lora_msg(msg)
    commands = {
        "1": ("sequential motor test", run_sequential),
        "2": ("simultaneous motor test", run_simultaneous),
        "3": ("both motor tests", run_both),
        "4": ("hover flight", lambda m: run_hover_flight(m, exit_on_error=False)),
    }
    if msg not in commands:
        log(f"Unknown LoRa msg {msg!r} — ignored")
        return

    label, handler = commands[msg]
    log(f"Running LoRa command {msg}: {label}")
    try:
        handler(master)
        log(f"LoRa command {msg} finished")
    except Exception as exc:
        log(f"LoRa command {msg} failed: {exc}")
        drain_statustext(master)


def parse_args():
    parser = argparse.ArgumentParser(description="Venator hexacopter control")
    parser.add_argument(
        "--lora",
        action="store_true",
        help="start LoRa RX listener (menu option 5), no prompts",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="force interactive menu (default when run from a terminal)",
    )
    return parser.parse_args()


def should_run_lora(args):
    if args.lora:
        return True
    if args.interactive:
        return False
    if not sys.stdin.isatty():
        log("Non-interactive session detected — starting LoRa RX")
        return True
    return False


def run_lora_mode():
    log("Venator LoRa RX starting...")
    log("=" * 55)
    log("VENATOR HEXACOPTER — LoRa RX")
    log("=" * 55)
    log(f"LoRa port: {RX_LORA_PORT} @ {BAUD_LORA}")
    master = connect()
    log("Pixhawk connected. Opening LoRa serial port...")
    listen_lora(
        port=RX_LORA_PORT,
        baud=BAUD_LORA,
        on_msg=lambda msg: handle_lora_msg(master, msg),
    )
    log("LoRa listener exited.")


def main():
    setup_service_logging()
    args = parse_args()
    if should_run_lora(args):
        run_lora_mode()
        return

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

    if choice == "5":
        run_lora_mode()
        return

    master = connect()

    if choice == "1":
        run_sequential(master)
    elif choice == "2":
        run_simultaneous(master)
    elif choice == "3":
        run_both(master)
    elif choice == "4":
        run_hover_flight(master)

    print("\nDone.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(0)
    except SystemExit:
        raise
    except Exception as exc:
        print(f"\nFatal error: {exc}")
        sys.exit(1)