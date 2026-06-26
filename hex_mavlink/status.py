"""Status text, onboard logging, and terrain helpers.

Sources:
  pymavlink-utils/send-status-text.py
  pymavlink-utils/logging-onboard.py
  pymavlink-utils/terrain-check.py
"""

from typing import Dict, Optional, Union

from hex_mavlink._common import dialect, wait_for_message

__all__ = [
    "send_statustext",
    "log_onboard_data",
    "check_terrain",
]


def send_statustext(
    vehicle,
    text: str,
    severity: int = dialect.MAV_SEVERITY_INFO,
) -> None:
    """Send a STATUSTEXT message to the GCS or autopilot."""
    message = dialect.MAVLink_statustext_message(
        severity=severity,
        text=text.encode("utf-8"),
    )
    vehicle.mav.send(message)


def log_onboard_data(
    vehicle,
    text: str,
    severity: int = dialect.MAV_SEVERITY_DEBUG,
) -> None:
    """Log data to the autopilot dataflash via STATUSTEXT (saved as MSG)."""
    send_statustext(vehicle, text, severity=severity)


def check_terrain(
    vehicle,
    latitude: float,
    longitude: float,
    timeout: Optional[float] = 30.0,
) -> Dict[str, Union[int, float]]:
    """Check terrain availability at a location and return TERRAIN_REPORT."""
    lat_int = int(latitude * 1e7)
    lon_int = int(longitude * 1e7)

    message = dialect.MAVLink_terrain_check_message(lat=lat_int, lon=lon_int)
    vehicle.mav.send(message)

    return wait_for_message(
        vehicle,
        dialect.MAVLink_terrain_report_message.msgname,
        predicate=lambda msg: msg["lat"] == lat_int and msg["lon"] == lon_int,
        timeout=timeout,
    )
