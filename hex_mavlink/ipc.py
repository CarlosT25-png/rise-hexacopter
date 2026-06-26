"""Inter-process MAVLink communication helpers.

Source: pymavlink-utils/send-receive.py
"""

import threading
import time

import pymavlink.mavutil as mavutil

from hex_mavlink._common import dialect

__all__ = [
    "create_ipc_connection",
    "start_heartbeat_thread",
    "send_heartbeat_once",
]


def create_ipc_connection(
    device: str,
    source_system: int = 1,
    source_component: int = 1,
    force_connected: bool = False,
):
    """Create a MAVLink connection for inter-process communication."""
    kwargs = {
        "device": device,
        "source_system": source_system,
        "source_component": source_component,
    }
    if force_connected:
        kwargs["force_connected"] = True
    return mavutil.mavlink_connection(**kwargs)


def send_heartbeat_once(connection) -> None:
    """Send a single onboard-controller heartbeat."""
    heartbeat = dialect.MAVLink_heartbeat_message(
        type=dialect.MAV_TYPE_ONBOARD_CONTROLLER,
        autopilot=dialect.MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=dialect.MAV_STATE_ACTIVE,
        mavlink_version=3,
    )
    connection.mav.send(heartbeat)


def start_heartbeat_thread(
    connection,
    interval_s: float = 1.0,
    daemon: bool = True,
) -> threading.Thread:
    """Start a background thread that sends heartbeats at a fixed interval."""

    def _loop():
        while True:
            send_heartbeat_once(connection)
            time.sleep(interval_s)

    thread = threading.Thread(target=_loop, daemon=daemon)
    thread.start()
    return thread
