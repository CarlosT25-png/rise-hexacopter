"""Parameter get/set helpers.

Source: pymavlink-utils/get-set-parameter.py
"""

from typing import Any, Optional, Union

from hex_mavlink._common import dialect, wait_param_value

__all__ = ["request_param_list", "get_param", "set_param"]

PARAM_INDEX = -1


def _param_id_bytes(param_id: str) -> bytes:
    return param_id.encode("utf-8")


def request_param_list(vehicle) -> None:
    """Request the full parameter list from the vehicle."""
    message = dialect.MAVLink_param_request_list_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
    )
    vehicle.mav.send(message)


def get_param(
    vehicle,
    param_id: str,
    timeout: Optional[float] = 30.0,
) -> float:
    """Request and return a single parameter value."""
    message = dialect.MAVLink_param_request_read_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        param_id=_param_id_bytes(param_id),
        param_index=PARAM_INDEX,
    )
    vehicle.mav.send(message)
    response = wait_param_value(vehicle, param_id, timeout=timeout)
    return response["param_value"]


def set_param(
    vehicle,
    param_id: str,
    param_value: Union[int, float],
    param_type: int = dialect.MAV_PARAM_TYPE_REAL32,
    timeout: Optional[float] = 30.0,
) -> float:
    """Set a parameter and return the confirmed value."""
    message = dialect.MAVLink_param_set_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        param_id=_param_id_bytes(param_id),
        param_value=param_value,
        param_type=param_type,
    )
    vehicle.mav.send(message)
    response = wait_param_value(vehicle, param_id, timeout=timeout)
    return response["param_value"]
