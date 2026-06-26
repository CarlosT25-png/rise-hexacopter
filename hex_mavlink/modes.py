"""Flight mode helpers.

Source: pymavlink-utils/change-mode.py
"""

from typing import Dict, Optional

from hex_mavlink._common import (
    VehicleTimeout,
    dialect,
    recv_message_dict,
    send_and_wait_ack,
    wait_for_message,
)

__all__ = ["supported_modes", "get_mode", "set_mode"]


def supported_modes(vehicle) -> Dict[str, int]:
    """Return the vehicle's supported flight mode name to id mapping."""
    return vehicle.mode_mapping()


def _mode_name_from_id(flight_modes: Dict[str, int], mode_id: int) -> str:
    flight_mode_names = list(flight_modes.keys())
    flight_mode_ids = list(flight_modes.values())
    flight_mode_index = flight_mode_ids.index(mode_id)
    return flight_mode_names[flight_mode_index]


def get_mode(vehicle) -> str:
    """Return the current flight mode name from HEARTBEAT."""
    flight_modes = supported_modes(vehicle)
    message = recv_message_dict(
        vehicle,
        dialect.MAVLink_heartbeat_message.msgname,
        blocking=True,
    )
    if message is None:
        raise VehicleTimeout("No HEARTBEAT received")
    return _mode_name_from_id(flight_modes, message["custom_mode"])


def set_mode(vehicle, flight_mode: str = "GUIDED", timeout: Optional[float] = 10.0) -> bool:
    """Change flight mode and wait for COMMAND_ACK acceptance."""
    flight_modes = supported_modes(vehicle)
    if flight_mode not in flight_modes:
        raise ValueError("{!r} is not supported".format(flight_mode))

    set_mode_message = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_DO_SET_MODE,
        confirmation=0,
        param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        param2=flight_modes[flight_mode],
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    vehicle.mav.send(set_mode_message)

    ack = wait_for_message(
        vehicle,
        dialect.MAVLink_command_ack_message.msgname,
        predicate=lambda msg: msg["command"] == dialect.MAV_CMD_DO_SET_MODE,
        timeout=timeout,
    )
    return ack["result"] == dialect.MAV_RESULT_ACCEPTED
