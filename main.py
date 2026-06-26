"""
VENATOR HEXACOPTER — motor test, hover flight, and LoRa control.

Connection: Jetson J41 UART -> Pixhawk TELEM2, 57600 baud (port auto-detected)
"""

import argparse
import sys
import time

from hex_mavlink._common import dialect
from hex_mavlink.connection import connect_with_retry as _connect_with_retry
from hex_mavlink.connection import recv_match_locked
from hex_mavlink import arming, flight, modes
from hex_mavlink.messages import send_command_long
from hex_mavlink.status import drain_statustext
from lora_helper import listen as listen_lora

# ----------------------------------------------------------------------
# TUNABLE PARAMETERS
# ----------------------------------------------------------------------
SERIAL_PORT = "auto"   # "auto", or e.g. "/dev/ttyACM1" to force a port
RX_LORA_PORT = "/dev/ttyUSB0"
BAUD = 57600
BAUD_LORA = 115200
DETECT_TIMEOUT_S = 3
CONNECT_RETRY_S = 60
CONNECT_RETRY_INTERVAL_S = 5

THROTTLE_PCT = 5       # % throttle. Bump to ~8-10 if a motor won't spin up.
DURATION_S = 2         # seconds each motor spins
NUM_MOTORS = 6         # hexacopter
SETTLE_S = 1.0         # pause between motors in sequential mode

# DO_MOTOR_TEST throttle type: 0 = percent, 1 = PWM, 2 = pilot passthrough
THROTTLE_TYPE = 0

TAKEOFF_ALT_M = 5.0
HOVER_TIME_S = 30
ALT_TOLERANCE_M = 0.5
TAKEOFF_TIMEOUT_S = 60
PREARM_TIMEOUT_S = 15
ARM_TIMEOUT_S = 15
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


def connect_with_retry(port=None, baud=None, timeout_s=CONNECT_RETRY_S):
    """Retry Pixhawk connection — USB ports may appear a few seconds after boot."""
    return _connect_with_retry(
        port=port,
        baud=baud or BAUD,
        timeout_s=timeout_s,
        retry_interval_s=CONNECT_RETRY_INTERVAL_S,
        exclude=(RX_LORA_PORT,),
        heartbeat_timeout_s=DETECT_TIMEOUT_S,
        log=log,
        fatal=True,
    )


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
        send_command_long(
            master,
            dialect.MAV_CMD_DO_MOTOR_TEST,
            param1=motor_num,
            param2=THROTTLE_TYPE,
            param3=throttle,
            param4=duration,
        )
    result = wait_command_ack(master, dialect.MAV_CMD_DO_MOTOR_TEST)
    if result == dialect.MAV_RESULT_ACCEPTED:
        log("  Motor {}: command accepted".format(motor_num))
        return True
    if result is None:
        log("  Motor {}: no ACK from Pixhawk (check serial link)".format(motor_num))
    else:
        log("  Motor {}: rejected (MAV_RESULT={})".format(motor_num, result))
    drain_statustext(master, log=log)
    return False


def run_sequential(master):
    log("=== SEQUENTIAL TEST: motors 1..6, one at a time ===")
    for motor in range(1, NUM_MOTORS + 1):
        log("  Motor {}: {}% for {}s".format(motor, THROTTLE_PCT, DURATION_S))
        motor_test(master, motor, THROTTLE_PCT, DURATION_S)
        time.sleep(DURATION_S + SETTLE_S)
    log("Sequential test complete.")


def run_simultaneous(master):
    log("=== SIMULTANEOUS TEST: all 6 motors together ===")
    log("  All motors: {}% for {}s".format(THROTTLE_PCT, DURATION_S))
    for motor in range(1, NUM_MOTORS + 1):
        motor_test(master, motor, THROTTLE_PCT, DURATION_S)
        time.sleep(0.02)
    time.sleep(DURATION_S + 0.5)
    log("Simultaneous test complete.")


def run_both(master):
    run_sequential(master)
    time.sleep(2)
    run_simultaneous(master)


def arm_vehicle(master, timeout_s=ARM_TIMEOUT_S):
    ack_results = {
        dialect.MAV_RESULT_DENIED: "DENIED (pre-arm check failed)",
        dialect.MAV_RESULT_FAILED: "FAILED",
        dialect.MAV_RESULT_TEMPORARILY_REJECTED: "TEMPORARILY_REJECTED",
    }

    log("Arming...")
    start = time.time()
    last_arm_send = 0.0

    while time.time() - start < timeout_s:
        drain_statustext(master, log=log)
        if master.motors_armed():
            log("Armed.")
            return True

        if time.time() - last_arm_send >= 2:
            with master.mavlink_lock:
                master.arducopter_arm()
            last_arm_send = time.time()

        ack = recv_match_locked(master, type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack and ack.command == dialect.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == dialect.MAV_RESULT_ACCEPTED:
                log("Arm command accepted.")
            elif ack.result != dialect.MAV_RESULT_IN_PROGRESS:
                reason = ack_results.get(ack.result, "code {}".format(ack.result))
                log("  Arm rejected: {}".format(reason))

    drain_statustext(master, log=log)
    return False


def set_mode_and_wait(master, mode, timeout_s=10):
    if not modes.set_mode(master, mode, timeout=timeout_s):
        return False

    start = time.time()
    while time.time() - start < timeout_s:
        recv_match_locked(master, type="HEARTBEAT", blocking=True, timeout=0.2)
        if master.flightmode == mode:
            return True
        time.sleep(0.2)
    return False


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
            log("  Altitude: {:.1f} m".format(alt_m))
            if alt_m >= target_alt_m - tolerance_m:
                return True
    return False


def run_hover_flight(master, exit_on_error=True, skip_prearm=False):
    log("=== HOVER FLIGHT: {} m for {} s ===".format(TAKEOFF_ALT_M, HOVER_TIME_S))

    if skip_prearm:
        log("WARNING: skipping pre-arm wait (--skip-prearm). FC may still deny arming.")
    elif not arming.wait_prearm_ok(master, timeout_s=PREARM_TIMEOUT_S, log=log):
        if exit_on_error:
            sys.exit(1)
        return False

    log("Setting GUIDED mode...")
    if not set_mode_and_wait(master, "GUIDED"):
        log("Failed to enter GUIDED mode (current: {}).".format(master.flightmode))
        if exit_on_error:
            sys.exit(1)
        return False

    if not arm_vehicle(master):
        log("Failed to arm.")
        log("Check the FC messages above (PreArm: ...) and fix the reported issue.")
        if exit_on_error:
            sys.exit(1)
        return False

    log("Takeoff to {} m...".format(TAKEOFF_ALT_M))
    with master.mavlink_lock:
        flight.takeoff(master, altitude_m=TAKEOFF_ALT_M)

    if not wait_for_altitude(master, TAKEOFF_ALT_M):
        log("Takeoff timed out. Switching to LAND.")
        set_mode_and_wait(master, "LAND")
        if exit_on_error:
            sys.exit(1)
        return False

    log("Hovering for {} s...".format(HOVER_TIME_S))
    hover_end = time.time() + HOVER_TIME_S
    while time.time() < hover_end:
        alt_m = get_relative_alt_m(master)
        remaining = int(hover_end - time.time())
        if alt_m is not None:
            log("  Hover: {:.1f} m  ({}s left)".format(alt_m, remaining))
        time.sleep(1)

    log("Landing...")
    set_mode_and_wait(master, "LAND")
    time.sleep(5)
    log("Hover flight complete.")
    return True


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
        log("Unknown LoRa msg {!r} — ignored".format(msg))
        return

    label, handler = commands[msg]
    log("Running LoRa command {}: {}".format(msg, label))
    try:
        handler(master)
        log("LoRa command {} finished".format(msg))
    except Exception as exc:
        log("LoRa command {} failed: {}".format(msg, exc))
        drain_statustext(master, log=log)


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
    parser.add_argument(
        "--serial",
        default=None,
        metavar="PORT",
        help="Pixhawk serial port (default: auto-detect /dev/ttyACM*)",
    )
    parser.add_argument(
        "--skip-prearm",
        action="store_true",
        help="run hover flight immediately, without waiting for pre-arm checks",
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


def confirm_props_removed():
    ans = input("Props removed and frame secured? (yes/no): ").strip().lower()
    if ans != "yes":
        print("Aborting. Remove props first.")
        sys.exit(1)


def run_lora_mode(serial_port=None):
    log("Venator LoRa RX starting...")
    log("=" * 55)
    log("VENATOR HEXACOPTER — LoRa RX")
    log("=" * 55)
    log("LoRa port: {} @ {}".format(RX_LORA_PORT, BAUD_LORA))
    master = connect_with_retry(port=serial_port)
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
    if args.skip_prearm:
        master = connect_with_retry(port=args.serial)
        run_hover_flight(master, skip_prearm=True)
        return

    if should_run_lora(args):
        run_lora_mode(serial_port=args.serial)
        return

    print("=" * 55)
    print("VENATOR HEXACOPTER — MOTOR TEST / FLIGHT / LoRa")
    print("=" * 55)
    print("\nSelect mode:")
    print("  1) Sequential motor test (1..6 one at a time)")
    print("  2) Simultaneous motor test (all 6 together)")
    print("  3) Both motor tests (sequential, then simultaneous)")
    print("  4) Hover flight ({} m for {} s)".format(TAKEOFF_ALT_M, HOVER_TIME_S))
    print("  5) Listen to LoRa (JSON: 1=sequential, 2=simultaneous, 3=both, 4=hover)")
    choice = input("Choice [1/2/3/4/5]: ").strip()

    if choice in ("1", "2", "3"):
        confirm_props_removed()
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
        run_lora_mode(serial_port=args.serial)
        return

    master = connect_with_retry(port=args.serial)

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
        print("\nFatal error: {}".format(exc))
        sys.exit(1)
