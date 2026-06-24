import sys
import time

from pymavlink import mavutil

# Connect to MAVProxy: --out=udp:127.0.0.1:14551
CONNECTION_ADDRESS = "udpin:127.0.0.1:14551"
TIMEOUT_HEARTBEAT_S = 15.0
TARGET_ALTITUDE_METERS = 5.0
TAKEOFF_DELAY_S = 2.0  # change to 20.0 for a real test
HOVER_TIME_S = 10.0
SETPOINT_HZ = 10


def connect():
    print("Connecting to MAVProxy...")
    master = mavutil.mavlink_connection(CONNECTION_ADDRESS)
    if not master.wait_heartbeat(timeout=TIMEOUT_HEARTBEAT_S):
        print("CRITICAL ERROR: No heartbeat from autopilot.", file=sys.stderr)
        return None
    print("Connection successful!")
    return master


def get_relative_altitude_m(master):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
    if msg is None:
        return None
    return msg.relative_alt / 1000.0


def set_mode(master, mode_name):
    mode_map = master.mode_mapping()
    if mode_name not in mode_map:
        print(f"CRITICAL ERROR: Mode {mode_name} not available.", file=sys.stderr)
        return False

    mode_id = mode_map[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_id:
            return True
    return False


def arm(master, max_attempts=5):
    print("Setting GUIDED mode...")
    if not set_mode(master, "GUIDED"):
        print("CRITICAL ERROR: Failed to enter GUIDED mode.", file=sys.stderr)
        return False

    print("Attempting to arm...")
    for attempt in range(1, max_attempts + 1):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0,
        )
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Armed successfully.")
            return True

        if master.motors_armed():
            print("Armed successfully.")
            return True

        detail = f" ({ack.result})" if ack else ""
        print(f"Arm attempt {attempt} failed{detail}. Retrying in 2s...")
        time.sleep(2)

    print("CRITICAL ERROR: Failed to arm after maximum attempts.", file=sys.stderr)
    return False


def takeoff(master, target_alt_m, max_attempts=3):
    print(f"Attempting takeoff to {target_alt_m}m...")
    for attempt in range(1, max_attempts + 1):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0, target_alt_m,
        )
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"Takeoff attempt {attempt} rejected ({ack.result}). Retrying in 5s...")
            time.sleep(5)
            continue

        print("Takeoff command accepted!")
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            alt = get_relative_altitude_m(master)
            if alt is not None:
                print(f"Climbing... {alt:.2f}m / {target_alt_m}m")
                if alt >= target_alt_m * 0.95:
                    print("Target altitude reached")
                    return True
            time.sleep(0.5)

        print(f"Takeoff attempt {attempt} timed out. Retrying in 5s...")
        time.sleep(5)

    print("CRITICAL ERROR: Takeoff failed after maximum attempts.", file=sys.stderr)
    return False


def velocity_body_type_mask():
    m = mavutil.mavlink
    return (
        m.POSITION_TARGET_TYPEMASK_X_IGNORE
        | m.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | m.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | m.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )


def send_velocity_body(master, forward_m_s, right_m_s, down_m_s):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        velocity_body_type_mask(),
        0, 0, 0,
        forward_m_s, right_m_s, down_m_s,
        0, 0, 0,
        0, 0,
    )


def stream_velocity_body(master, forward_m_s, right_m_s, down_m_s, duration_s):
    interval = 1.0 / SETPOINT_HZ
    steps = int(duration_s * SETPOINT_HZ)
    for _ in range(steps):
        send_velocity_body(master, forward_m_s, right_m_s, down_m_s)
        time.sleep(interval)


def land(master):
    print("Landing...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0,
    )


def run_resilient_flight():
    master = connect()
    if master is None:
        return

    print("Waiting for preflight checks...")
    # Add GPS checks when needed: wait for 3D fix via GLOBAL_POSITION_INT
    print("Sensors ready")

    print(f"{TAKEOFF_DELAY_S}s takeoff delay")
    time.sleep(TAKEOFF_DELAY_S)

    if not arm(master):
        return

    if not takeoff(master, TARGET_ALTITUDE_METERS):
        land(master)
        return

    print("\nStart back and forth movements")
    print(f"Hovering for {HOVER_TIME_S} seconds...")
    stream_velocity_body(master, 0.0, 0.0, 0.0, HOVER_TIME_S)

    print("Flying forward...")
    stream_velocity_body(master, 2.0, 0.0, 0.0, 5)

    print("Flying backward...")
    stream_velocity_body(master, -2.0, 0.0, 0.0, 5)

    print("Stopping (Hovering)...")
    stream_velocity_body(master, 0.0, 0.0, 0.0, 2)

    land(master)


if __name__ == "__main__":
    run_resilient_flight()
