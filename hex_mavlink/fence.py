"""Geofence helpers.

Sources:
  pymavlink-utils/get-fence.py
  pymavlink-utils/set-fence.py
  pymavlink-utils/fence-enable.py
"""

from typing import List, Optional, Sequence, Tuple, Union

from hex_mavlink._common import (
    VehicleTimeout,
    deadline,
    dialect,
    remaining_timeout,
    wait_for_message,
    wait_param_value,
)
from hex_mavlink.parameters import get_param, set_param

__all__ = [
    "FENCE_ENABLE_MODES",
    "get_fence_count",
    "get_fence",
    "upload_fence",
    "set_fence_enabled",
]

FencePoint = Tuple[float, float]
FENCE_ENABLE_MODES = {
    "DISABLE": 0,
    "ENABLE": 1,
    "DISABLE_FLOOR_ONLY": 2,
}

PARAM_INDEX = -1
FENCE_TOTAL = "FENCE_TOTAL"
FENCE_ACTION = "FENCE_ACTION"


def get_fence_count(vehicle, timeout: Optional[float] = 30.0) -> int:
    """Return FENCE_TOTAL parameter value."""
    message = dialect.MAVLink_param_request_read_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        param_id=FENCE_TOTAL.encode("utf-8"),
        param_index=PARAM_INDEX,
    )
    vehicle.mav.send(message)
    response = wait_param_value(vehicle, FENCE_TOTAL, timeout=timeout)
    return int(response["param_value"])


def get_fence(vehicle, timeout: Optional[float] = 60.0) -> List[FencePoint]:
    """Download all fence points as (lat, lng) tuples."""
    fence_count = get_fence_count(vehicle, timeout=timeout)
    fence_list = []
    for idx in range(fence_count):
        message = dialect.MAVLink_fence_fetch_point_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            idx=idx,
        )
        vehicle.mav.send(message)
        response = wait_for_message(
            vehicle,
            dialect.MAVLink_fence_point_message.msgname,
            timeout=timeout,
        )
        fence_list.append((response["lat"], response["lng"]))
    return fence_list


def _set_param_until(
    vehicle,
    param_id: str,
    expected_value: Union[int, float],
    timeout: Optional[float] = 30.0,
) -> None:
    end = deadline(timeout)
    while True:
        remaining = remaining_timeout(end)
        if end is not None and remaining == 0.0:
            raise VehicleTimeout(
                "Timed out setting {!r} to {!r}".format(param_id, expected_value)
            )
        set_param(vehicle, param_id, expected_value, timeout=remaining)
        value = get_param(vehicle, param_id, timeout=remaining)
        if int(value) == int(expected_value):
            return


def upload_fence(
    vehicle,
    fence_list: Sequence[FencePoint],
    timeout: Optional[float] = 120.0,
) -> None:
    """Upload a geofence polygon to the vehicle."""
    fence_action_original = int(
        get_param(vehicle, FENCE_ACTION, timeout=timeout)
    )

    _set_param_until(vehicle, FENCE_ACTION, dialect.FENCE_ACTION_NONE, timeout=timeout)
    _set_param_until(vehicle, FENCE_TOTAL, 0, timeout=timeout)
    _set_param_until(vehicle, FENCE_TOTAL, len(fence_list), timeout=timeout)

    idx = 0
    end = deadline(timeout)
    while idx < len(fence_list):
        remaining = remaining_timeout(end)
        if end is not None and remaining == 0.0:
            raise VehicleTimeout("Timed out uploading fence points")

        message = dialect.MAVLink_fence_point_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            idx=idx,
            count=len(fence_list),
            lat=fence_list[idx][0],
            lng=fence_list[idx][1],
        )
        vehicle.mav.send(message)

        fetch = dialect.MAVLink_fence_fetch_point_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            idx=idx,
        )
        vehicle.mav.send(fetch)

        response = wait_for_message(
            vehicle,
            dialect.MAVLink_fence_point_message.msgname,
            timeout=remaining,
        )
        latitude = response["lat"]
        longitude = response["lng"]
        if latitude != 0.0 and longitude != 0:
            idx += 1

    _set_param_until(vehicle, FENCE_ACTION, fence_action_original, timeout=timeout)


def set_fence_enabled(
    vehicle,
    mode: Union[str, int] = "ENABLE",
) -> None:
    """Enable or disable the geofence via MAV_CMD_DO_FENCE_ENABLE."""
    if isinstance(mode, str):
        mode_key = mode.upper()
        if mode_key not in FENCE_ENABLE_MODES:
            raise ValueError("Not supported fence operation: {!r}".format(mode))
        param1 = FENCE_ENABLE_MODES[mode_key]
    else:
        param1 = mode

    message = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_DO_FENCE_ENABLE,
        confirmation=0,
        param1=param1,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    vehicle.mav.send(message)
