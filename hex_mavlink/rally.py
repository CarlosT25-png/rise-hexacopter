"""Rally point helpers.

Sources:
  pymavlink-utils/get-rally.py
  pymavlink-utils/set-rally.py
"""

from typing import List, Optional, Sequence, Tuple

from hex_mavlink._common import (
    VehicleTimeout,
    deadline,
    dialect,
    remaining_timeout,
    wait_for_message,
    wait_param_value,
)
from hex_mavlink.parameters import get_param, set_param

__all__ = ["get_rally_count", "get_rally_points", "upload_rally_points"]

RallyPoint = Tuple[float, float, float]
PARAM_INDEX = -1
RALLY_TOTAL = "RALLY_TOTAL"


def get_rally_count(vehicle, timeout: Optional[float] = 30.0) -> int:
    """Return RALLY_TOTAL parameter value."""
    message = dialect.MAVLink_param_request_read_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        param_id=RALLY_TOTAL.encode("utf-8"),
        param_index=PARAM_INDEX,
    )
    vehicle.mav.send(message)
    response = wait_param_value(vehicle, RALLY_TOTAL, timeout=timeout)
    return int(response["param_value"])


def get_rally_points(vehicle, timeout: Optional[float] = 60.0) -> List[RallyPoint]:
    """Download all rally points as (lat, lng, alt) tuples."""
    rally_count = get_rally_count(vehicle, timeout=timeout)
    rally_list = []
    for idx in range(rally_count):
        message = dialect.MAVLink_rally_fetch_point_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            idx=idx,
        )
        vehicle.mav.send(message)
        response = wait_for_message(
            vehicle,
            dialect.MAVLink_rally_point_message.msgname,
            timeout=timeout,
        )
        rally_list.append(
            (response["lat"] * 1e-7, response["lng"] * 1e-7, response["alt"])
        )
    return rally_list


def upload_rally_points(
    vehicle,
    rally_list: Sequence[RallyPoint],
    timeout: Optional[float] = 120.0,
) -> None:
    """Upload rally points to the vehicle."""
    end = deadline(timeout)
    while True:
        remaining = remaining_timeout(end)
        if end is not None and remaining == 0.0:
            raise VehicleTimeout("Timed out setting RALLY_TOTAL")
        set_param(vehicle, RALLY_TOTAL, len(rally_list), timeout=remaining)
        value = get_param(vehicle, RALLY_TOTAL, timeout=remaining)
        if int(value) == len(rally_list):
            break

    idx = 0
    while idx < len(rally_list):
        remaining = remaining_timeout(end)
        if end is not None and remaining == 0.0:
            raise VehicleTimeout("Timed out uploading rally points")

        message = dialect.MAVLink_rally_point_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            idx=idx,
            count=len(rally_list),
            lat=int(rally_list[idx][0] * 1e7),
            lng=int(rally_list[idx][1] * 1e7),
            alt=int(rally_list[idx][2]),
            break_alt=0,
            land_dir=0,
            flags=0,
        )
        vehicle.mav.send(message)

        fetch = dialect.MAVLink_rally_fetch_point_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            idx=idx,
        )
        vehicle.mav.send(fetch)

        response = wait_for_message(
            vehicle,
            dialect.MAVLink_rally_point_message.msgname,
            timeout=remaining,
        )
        if (
            response["idx"] == idx
            and response["count"] == len(rally_list)
            and response["lat"] == int(rally_list[idx][0] * 1e7)
            and response["lng"] == int(rally_list[idx][1] * 1e7)
            and response["alt"] == int(rally_list[idx][2])
        ):
            idx += 1
