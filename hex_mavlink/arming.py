"""Arming and disarming helpers.

Source: pymavlink-utils/arm-disarm.py
"""

import time
from typing import Callable, List, Optional, Set

from pymavlink import mavutil

from hex_mavlink._common import (
    dialect,
    recv_message_dict,
    send_and_wait_ack,
    wait_for_message,
)
from hex_mavlink.connection import recv_match_locked
from hex_mavlink.messages import send_command_long
from hex_mavlink.status import statustext_string

__all__ = [
    "VEHICLE_ARM",
    "VEHICLE_DISARM",
    "wait_prearm",
    "wait_prearm_ok",
    "request_prearm_checks",
    "format_prearm_failures",
    "log_prearm_failures",
    "arm",
    "disarm",
    "is_armed",
    "wait_armed",
    "wait_disarmed",
]

LogFn = Callable[[str], None]

VEHICLE_ARM = 1
VEHICLE_DISARM = 0

PREARM_REQUEST_INTERVAL_S = 5.0
PREARM_COLLECT_AFTER_TIMEOUT_S = 2.0


def _default_log(msg: str) -> None:
    print(msg, flush=True)


def _sensor_status_name(bit_mask: int) -> str:
    sensor_enums = mavutil.mavlink.enums.get("MAV_SYS_STATUS_SENSOR", {})
    entry = sensor_enums.get(bit_mask)
    if entry is None:
        return "UNKNOWN_0x{:X}".format(bit_mask)
    name = entry.name
    if name.startswith("MAV_SYS_STATUS_"):
        name = name[len("MAV_SYS_STATUS_"):]
    return name


def _unhealthy_sensor_labels(msg) -> List[str]:
    present = msg.onboard_control_sensors_present
    enabled = msg.onboard_control_sensors_enabled
    health = msg.onboard_control_sensors_health
    sensor_enums = mavutil.mavlink.enums.get("MAV_SYS_STATUS_SENSOR", {})
    labels = []
    for bit_mask in sorted(sensor_enums.keys()):
        if bit_mask == 0:
            continue
        if (present & enabled & bit_mask) and not (health & bit_mask):
            labels.append(_sensor_status_name(bit_mask))
    return labels


def _describe_prearm_bit(msg):
    bit = dialect.MAV_SYS_STATUS_PREARM_CHECK
    present = bool(msg.onboard_control_sensors_present & bit)
    enabled = bool(msg.onboard_control_sensors_enabled & bit)
    healthy = bool(msg.onboard_control_sensors_health & bit)
    issues = []
    if not present:
        issues.append("pre-arm status not reported by autopilot")
    elif not enabled:
        issues.append("pre-arm checks disabled (ARMING_CHECK parameter)")
    elif not healthy:
        issues.append("aggregate pre-arm check failing")
    return issues, healthy


def _sys_status_prearm_healthy(msg) -> bool:
    prearm_bit = dialect.MAV_SYS_STATUS_PREARM_CHECK
    return bool(
        (msg.onboard_control_sensors_present & prearm_bit)
        and (msg.onboard_control_sensors_enabled & prearm_bit)
        and (msg.onboard_control_sensors_health & prearm_bit)
    )


def request_prearm_checks(vehicle) -> None:
    """Ask ArduPilot to run pre-arm checks and emit PreArm: STATUSTEXT messages."""
    lock = getattr(vehicle, "mavlink_lock", None)
    if lock is not None:
        with lock:
            send_command_long(vehicle, dialect.MAV_CMD_RUN_PREARM_CHECKS)
    else:
        send_command_long(vehicle, dialect.MAV_CMD_RUN_PREARM_CHECKS)


def _poll_prearm_statustext(vehicle, prearm_seen: Set[str], log: LogFn) -> None:
    """Drain STATUSTEXT; collect unique PreArm:/Arm: lines for failure reporting."""
    while True:
        msg = recv_match_locked(vehicle, type="STATUSTEXT", blocking=False)
        if msg is None:
            break
        text = statustext_string(msg)
        if text.startswith("PreArm:") or text.startswith("Arm:"):
            prearm_seen.add(text)
        else:
            log("  FC: {}".format(text))


def format_prearm_failures(prearm_messages, sys_status_msg=None) -> List[str]:
    """Return human-readable pre-arm failure lines."""
    lines = ["Pre-arm checks did not pass. Individual failures:"]

    if sys_status_msg is not None:
        for issue in _describe_prearm_bit(sys_status_msg)[0]:
            lines.append("  - {}".format(issue))
        for label in _unhealthy_sensor_labels(sys_status_msg):
            if label == "PREARM_CHECK":
                continue
            lines.append("  - unhealthy: {}".format(label.replace("SENSOR_", "")))

    if prearm_messages:
        lines.append("  ArduPilot reports:")
        for text in sorted(prearm_messages):
            detail = text.split(":", 1)[-1].strip()
            lines.append("  - {}".format(detail))
    else:
        lines.append("  - no PreArm: detail messages received yet")

    return lines


def log_prearm_failures(prearm_messages, sys_status_msg=None, log: Optional[LogFn] = None) -> None:
    """Log individual pre-arm failure reasons."""
    log_fn = log or _default_log
    for line in format_prearm_failures(prearm_messages, sys_status_msg):
        log_fn(line)


def wait_prearm_ok(
    vehicle,
    timeout_s: float = 120.0,
    log: Optional[LogFn] = None,
) -> bool:
    """Wait for pre-arm checks; report individual failures on timeout."""
    log_fn = log or _default_log
    log_fn("Waiting for ArduPilot pre-arm checks...")
    start = time.time()
    last_update = 0.0
    last_prearm_request = 0.0
    prearm_seen = set()
    last_sys_status = None

    request_prearm_checks(vehicle)
    last_prearm_request = start

    while time.time() - start < timeout_s:
        _poll_prearm_statustext(vehicle, prearm_seen, log_fn)
        msg = recv_match_locked(vehicle, type="SYS_STATUS", blocking=True, timeout=1)
        if msg is None:
            continue

        last_sys_status = msg
        if _sys_status_prearm_healthy(msg):
            log_fn("Pre-arm checks passed.")
            return True

        now = time.time()
        if now - last_prearm_request >= PREARM_REQUEST_INTERVAL_S:
            request_prearm_checks(vehicle)
            last_prearm_request = now

        if now - last_update >= PREARM_REQUEST_INTERVAL_S:
            if prearm_seen:
                log_fn("  Still waiting for pre-arm — latest reports:")
                for text in sorted(prearm_seen)[-5:]:
                    log_fn("    {}".format(text.split(":", 1)[-1].strip()))
            else:
                log_fn("  Still waiting for pre-arm (GPS/EKF/compass, etc.)...")
            last_update = now

    request_prearm_checks(vehicle)
    collect_until = time.time() + PREARM_COLLECT_AFTER_TIMEOUT_S
    while time.time() < collect_until:
        _poll_prearm_statustext(vehicle, prearm_seen, log_fn)
        msg = recv_match_locked(vehicle, type="SYS_STATUS", blocking=True, timeout=0.5)
        if msg is not None:
            last_sys_status = msg

    log_prearm_failures(prearm_seen, last_sys_status, log=log_fn)
    return False


def wait_prearm(vehicle, timeout: Optional[float] = 120.0, log: Optional[LogFn] = None) -> bool:
    """Wait until pre-arm checks pass (alias for wait_prearm_ok)."""
    return wait_prearm_ok(vehicle, timeout_s=timeout or 120.0, log=log)


def _arm_disarm_message(vehicle, arm: int):
    return dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation=0,
        param1=arm,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )


def is_armed(vehicle) -> bool:
    """Return True if the latest HEARTBEAT indicates the vehicle is armed."""
    message = recv_message_dict(
        vehicle,
        dialect.MAVLink_heartbeat_message.msgname,
        timeout=1.0,
        blocking=True,
    )
    if message is None:
        return False
    armed_bit = message["base_mode"] & dialect.MAV_MODE_FLAG_SAFETY_ARMED
    return armed_bit == dialect.MAV_MODE_FLAG_SAFETY_ARMED


def arm(vehicle, timeout: Optional[float] = 30.0) -> bool:
    """Send arm command and wait for COMMAND_ACK acceptance."""
    vehicle.mav.send(_arm_disarm_message(vehicle, VEHICLE_ARM))
    ack = send_and_wait_ack(vehicle, dialect.MAV_CMD_COMPONENT_ARM_DISARM, timeout=timeout)
    return (
        ack["result"] == dialect.MAV_RESULT_ACCEPTED
        and ack["command"] == dialect.MAV_CMD_COMPONENT_ARM_DISARM
    )


def disarm(vehicle, timeout: Optional[float] = 30.0) -> bool:
    """Send disarm command and wait for COMMAND_ACK acceptance."""
    vehicle.mav.send(_arm_disarm_message(vehicle, VEHICLE_DISARM))
    ack = send_and_wait_ack(vehicle, dialect.MAV_CMD_COMPONENT_ARM_DISARM, timeout=timeout)
    return (
        ack["result"] == dialect.MAV_RESULT_ACCEPTED
        and ack["command"] == dialect.MAV_CMD_COMPONENT_ARM_DISARM
    )


def wait_armed(vehicle, timeout: Optional[float] = 30.0) -> bool:
    """Wait until HEARTBEAT reports the vehicle is armed."""
    wait_for_message(
        vehicle,
        dialect.MAVLink_heartbeat_message.msgname,
        predicate=lambda msg: (
            msg["base_mode"] & dialect.MAV_MODE_FLAG_SAFETY_ARMED
        )
        == dialect.MAV_MODE_FLAG_SAFETY_ARMED,
        timeout=timeout,
    )
    return True


def wait_disarmed(vehicle, timeout: Optional[float] = 30.0) -> bool:
    """Wait until HEARTBEAT reports the vehicle is disarmed."""
    wait_for_message(
        vehicle,
        dialect.MAVLink_heartbeat_message.msgname,
        predicate=lambda msg: (
            msg["base_mode"] & dialect.MAV_MODE_FLAG_SAFETY_ARMED
        )
        != dialect.MAV_MODE_FLAG_SAFETY_ARMED,
        timeout=timeout,
    )
    return True
