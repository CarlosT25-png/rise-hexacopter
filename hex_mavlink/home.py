"""Home position helpers.

Sources:
  pymavlink-utils/home-get-set.py
  pymavlink-utils/distance-home.py
"""

from typing import Dict, Optional, Tuple

import geopy.distance

from hex_mavlink._common import dialect, recv_message_dict, wait_for_message

__all__ = [
    "get_home",
    "set_home",
    "request_home",
    "distance_from_home",
    "get_vehicle_position",
]

HomeLocation = Tuple[float, float]
Position = Tuple[float, float]


def request_home(vehicle) -> None:
    """Request HOME_POSITION via MAV_CMD_REQUEST_MESSAGE."""
    message = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_REQUEST_MESSAGE,
        confirmation=0,
        param1=dialect.MAVLINK_MSG_ID_HOME_POSITION,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    vehicle.mav.send(message)


def get_home(vehicle, timeout: Optional[float] = 30.0) -> Dict[str, float]:
    """Return home position as latitude, longitude, and altitude in meters."""
    message = wait_for_message(
        vehicle,
        dialect.MAVLink_home_position_message.msgname,
        timeout=timeout,
    )
    return {
        "latitude": message["latitude"] * 1e-7,
        "longitude": message["longitude"] * 1e-7,
        "altitude": message["altitude"] * 1e-3,
    }


def set_home(
    vehicle,
    latitude: float,
    longitude: float,
    altitude: float,
) -> None:
    """Set home position via MAV_CMD_DO_SET_HOME."""
    command = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_DO_SET_HOME,
        confirmation=0,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        param5=latitude,
        param6=longitude,
        param7=altitude,
    )
    vehicle.mav.send(command)


def get_vehicle_position(vehicle, timeout: Optional[float] = 5.0) -> Position:
    """Return current vehicle lat/lon from GLOBAL_POSITION_INT."""
    message = recv_message_dict(
        vehicle,
        dialect.MAVLink_global_position_int_message.msgname,
        timeout=timeout,
        blocking=True,
    )
    if message is None:
        raise TimeoutError("No GLOBAL_POSITION_INT received")
    return (message["lat"] * 1e-7, message["lon"] * 1e-7)


def distance_from_home(vehicle, timeout: Optional[float] = 30.0) -> float:
    """Return geodesic distance in meters from home to current position."""
    request_home(vehicle)
    home = get_home(vehicle, timeout=timeout)
    home_location = (home["latitude"], home["longitude"])
    vehicle_location = get_vehicle_position(vehicle, timeout=timeout)
    return geopy.distance.GeodesicDistance(home_location, vehicle_location).meters
