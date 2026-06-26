"""Vehicle connection helpers for Pixhawk serial links and MAVProxy-style URLs.

Pixhawk auto-detect matches main.py: scan serial ports, wait for heartbeat,
install mavlink_lock, and start a background GCS heartbeat.

Source: pymavlink-utils/vehicle-connection.py (network URLs)
        main.py (serial auto-detect)
"""

import glob
import os
import sys
import threading
import time
from typing import Callable, Optional, Sequence

from pymavlink import mavutil

from hex_mavlink._common import VehicleTimeout

__all__ = [
    "PixhawkConnectionError",
    "DEFAULT_BAUD",
    "DEFAULT_SERIAL_PORT",
    "DEFAULT_LORA_EXCLUDE",
    "list_serial_port_candidates",
    "open_pixhawk",
    "detect_pixhawk_port",
    "connect",
    "connect_with_retry",
    "start_gcs_heartbeat",
    "recv_match_locked",
    "wait_heartbeat",
    "is_mavlink_device_url",
]

DEFAULT_BAUD = 57600
DEFAULT_SERIAL_PORT = "auto"
DEFAULT_DETECT_TIMEOUT_S = 3.0
DEFAULT_CONNECT_RETRY_S = 60.0
DEFAULT_CONNECT_RETRY_INTERVAL_S = 5.0
DEFAULT_LORA_EXCLUDE = ("/dev/ttyUSB0",)

LogFn = Callable[[str], None]
MAVLINK_URL_PREFIXES = (
    "udp:",
    "udpin:",
    "udpout:",
    "udpbcast:",
    "tcp:",
    "tcpin:",
)


class PixhawkConnectionError(Exception):
    """Raised when Pixhawk connection cannot be established."""


def _default_log(msg: str) -> None:
    print(msg, flush=True)


def is_mavlink_device_url(device: str) -> bool:
    """Return True for MAVProxy-style connection strings (UDP/TCP)."""
    return device.startswith(MAVLINK_URL_PREFIXES)


def list_serial_port_candidates(exclude: Sequence[str] = ()) -> list:
    """Return unique serial device paths, stable by-id links first."""
    exclude_real = {
        os.path.realpath(path)
        for path in exclude
        if os.path.exists(path)
    }
    patterns = (
        "/dev/serial/by-id/*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    )
    seen = set()
    candidates = []
    for pattern in patterns:
        for path in sorted(glob.glob(pattern)):
            if not os.path.exists(path) or os.path.isdir(path):
                continue
            real = os.path.realpath(path)
            if real in exclude_real or real in seen:
                continue
            seen.add(real)
            candidates.append(real)
    return candidates


def open_pixhawk(
    port: str,
    baud: int = DEFAULT_BAUD,
    heartbeat_timeout_s: float = DEFAULT_DETECT_TIMEOUT_S,
    log: LogFn = _default_log,
):
    """Open a serial Pixhawk connection and wait for heartbeat."""
    log("Connecting to {} @ {}...".format(port, baud))
    master = mavutil.mavlink_connection(
        port,
        baud=baud,
        source_system=255,
        source_component=0,
    )
    master.mavlink_lock = threading.Lock()
    log("Waiting for Pixhawk heartbeat...")
    with master.mavlink_lock:
        master.wait_heartbeat(timeout=heartbeat_timeout_s)
    if master.target_component in (0, mavutil.mavlink.MAV_COMP_ID_ALL):
        master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
    log(
        "Pixhawk heartbeat OK on {} — system {}, component {}".format(
            port, master.target_system, master.target_component
        )
    )
    return master


def detect_pixhawk_port(
    baud: int = DEFAULT_BAUD,
    exclude: Sequence[str] = DEFAULT_LORA_EXCLUDE,
    heartbeat_timeout_s: float = DEFAULT_DETECT_TIMEOUT_S,
    log: LogFn = _default_log,
):
    """Try each candidate serial port until a Pixhawk heartbeat is found."""
    candidates = list_serial_port_candidates(exclude=exclude)
    if not candidates:
        log("No serial devices found to scan")
        return None

    log("Auto-detecting Pixhawk ({} port(s) to try)...".format(len(candidates)))
    for port in candidates:
        log("  Trying {}...".format(port))
        master = None
        try:
            master = open_pixhawk(
                port,
                baud=baud,
                heartbeat_timeout_s=heartbeat_timeout_s,
                log=log,
            )
            log("Using Pixhawk on {}".format(port))
            return master
        except Exception as exc:
            log("  Not Pixhawk on {}: {}".format(port, exc))
            if master is not None:
                try:
                    master.close()
                except Exception:
                    pass
    return None


def _connect_mavlink_url(
    device: str,
    baud: Optional[int] = None,
    heartbeat_timeout_s: float = DEFAULT_DETECT_TIMEOUT_S,
    install_lock: bool = True,
    **kwargs,
):
    """Open a UDP/TCP MAVLink connection and wait for heartbeat."""
    conn_kwargs = dict(kwargs)
    if baud is not None:
        conn_kwargs["baud"] = baud
    master = mavutil.mavlink_connection(device=device, **conn_kwargs)
    if install_lock and not hasattr(master, "mavlink_lock"):
        master.mavlink_lock = threading.Lock()
    with master.mavlink_lock:
        if not master.wait_heartbeat(timeout=heartbeat_timeout_s):
            raise VehicleTimeout("Timed out waiting for heartbeat on {}".format(device))
    return master


def connect(
    port: Optional[str] = None,
    baud: Optional[int] = None,
    exclude: Sequence[str] = DEFAULT_LORA_EXCLUDE,
    start_gcs: bool = True,
    heartbeat_timeout_s: float = DEFAULT_DETECT_TIMEOUT_S,
    log: LogFn = _default_log,
    **kwargs,
):
    """Connect to Pixhawk on serial (auto-detect by default) or a MAVLink URL."""
    baud = baud or DEFAULT_BAUD
    use_port = port if port is not None else DEFAULT_SERIAL_PORT

    if is_mavlink_device_url(use_port):
        master = _connect_mavlink_url(
            use_port,
            baud=baud,
            heartbeat_timeout_s=heartbeat_timeout_s,
            **kwargs,
        )
        if start_gcs:
            start_gcs_heartbeat(master)
        return master

    if use_port != "auto" and not os.path.exists(use_port):
        log("{} does not exist — auto-detecting Pixhawk...".format(use_port))
        use_port = "auto"

    master = None
    if use_port == "auto":
        master = detect_pixhawk_port(
            baud=baud,
            exclude=exclude,
            heartbeat_timeout_s=heartbeat_timeout_s,
            log=log,
        )
    else:
        try:
            master = open_pixhawk(
                use_port,
                baud=baud,
                heartbeat_timeout_s=heartbeat_timeout_s,
                log=log,
            )
        except Exception as exc:
            log("Failed on {}: {}".format(use_port, exc))
            log("Falling back to auto-detect...")
            master = detect_pixhawk_port(
                baud=baud,
                exclude=exclude,
                heartbeat_timeout_s=heartbeat_timeout_s,
                log=log,
            )

    if master is None:
        return None

    if start_gcs:
        start_gcs_heartbeat(master)
    return master


def connect_with_retry(
    port: Optional[str] = None,
    baud: Optional[int] = None,
    timeout_s: float = DEFAULT_CONNECT_RETRY_S,
    retry_interval_s: float = DEFAULT_CONNECT_RETRY_INTERVAL_S,
    exclude: Sequence[str] = DEFAULT_LORA_EXCLUDE,
    start_gcs: bool = True,
    fatal: bool = False,
    log: LogFn = _default_log,
    **kwargs,
):
    """Retry Pixhawk connection until timeout (USB ports may appear after boot)."""
    start = time.time()
    while time.time() - start < timeout_s:
        master = connect(
            port=port,
            baud=baud,
            exclude=exclude,
            start_gcs=start_gcs,
            log=log,
            **kwargs,
        )
        if master is not None:
            return master
        remaining = int(timeout_s - (time.time() - start))
        log(
            "Pixhawk not found, retrying in {}s ({}s left)...".format(
                retry_interval_s, remaining
            )
        )
        time.sleep(retry_interval_s)

    message = "Failed to connect to Pixhawk. Check USB cable and TELEM2 wiring."
    log(message)
    if fatal:
        sys.exit(1)
    raise PixhawkConnectionError(message)


def start_gcs_heartbeat(master) -> None:
    """Act as a GCS so ArduPilot sees a ground station (like QGroundControl)."""
    if not hasattr(master, "mavlink_lock"):
        master.mavlink_lock = threading.Lock()

    def _loop():
        while True:
            with master.mavlink_lock:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
            time.sleep(1)

    threading.Thread(target=_loop, daemon=True).start()


def recv_match_locked(master, **kwargs):
    """recv_match while holding the vehicle mavlink_lock."""
    with master.mavlink_lock:
        return master.recv_match(**kwargs)


def wait_heartbeat(vehicle, timeout: float = 5.0) -> None:
    """Wait until a heartbeat is received from the vehicle."""
    lock = getattr(vehicle, "mavlink_lock", None)
    if lock is not None:
        with lock:
            ok = vehicle.wait_heartbeat(timeout=timeout)
    else:
        ok = vehicle.wait_heartbeat(timeout=timeout)
    if not ok:
        raise VehicleTimeout("Timed out waiting for heartbeat")
