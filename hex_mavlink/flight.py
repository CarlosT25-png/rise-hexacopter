"""Flight command helpers.

Sources:
  pymavlink-utils/takeoff-land.py
  pymavlink-utils/goto-location.py
  pymavlink-utils/set-speed.py
  pymavlink-utils/set-yaw.py
  pymavlink-utils/pause-resume.py
"""

from typing import Dict, List, Optional, Tuple, Union

import geopy.distance

from hex_mavlink._common import (
    VehicleTimeout,
    deadline,
    dialect,
    recv_message_dict,
    remaining_timeout,
)

__all__ = [
    "DIRECTION_CCW",
    "DIRECTION_DEFAULT",
    "DIRECTION_CW",
    "ANGLE_ABSOLUTE",
    "ANGLE_RELATIVE",
    "PAUSE",
    "RESUME",
    "takeoff",
    "land",
    "get_relative_altitude",
    "wait_altitude",
    "goto_location",
    "wait_at_location",
    "set_ground_speed",
    "set_yaw",
    "pause",
    "resume",
]

DIRECTION_CCW = -1
DIRECTION_DEFAULT = 0
DIRECTION_CW = 1
ANGLE_ABSOLUTE = 0
ANGLE_RELATIVE = 1
PAUSE = 0
RESUME = 1

LocationDict = Dict[str, float]


def takeoff(vehicle, altitude_m: float = 50.0) -> None:
    """Send NAV_TAKEOFF command."""
    command = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_NAV_TAKEOFF,
        confirmation=0,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=altitude_m,
    )
    vehicle.mav.send(command)


def land(vehicle) -> None:
    """Send NAV_LAND command."""
    command = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_NAV_LAND,
        confirmation=0,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    vehicle.mav.send(command)


def get_relative_altitude(vehicle, timeout: Optional[float] = 5.0) -> float:
    """Return relative altitude in meters from GLOBAL_POSITION_INT."""
    message = recv_message_dict(
        vehicle,
        dialect.MAVLink_global_position_int_message.msgname,
        timeout=timeout,
        blocking=True,
    )
    if message is None:
        raise VehicleTimeout("No GLOBAL_POSITION_INT received")
    return message["relative_alt"] * 1e-3


def wait_altitude(
    vehicle,
    target_m: float,
    tolerance_m: float = 1.0,
    timeout: Optional[float] = 120.0,
    above: bool = True,
) -> bool:
    """Wait until relative altitude reaches target within tolerance."""
    end = deadline(timeout)
    while True:
        remaining = remaining_timeout(end)
        if end is not None and remaining == 0.0:
            raise VehicleTimeout("Timed out waiting for altitude {:.1f} m".format(target_m))

        relative_altitude = get_relative_altitude(vehicle, timeout=remaining)
        if above:
            if target_m - relative_altitude < tolerance_m:
                return True
        else:
            if relative_altitude < tolerance_m:
                return True


def goto_location(
    vehicle,
    latitude: float,
    longitude: float,
    altitude: float,
) -> None:
    """Send a GUIDED waypoint using MISSION_ITEM_INT."""
    message = dialect.MAVLink_mission_item_int_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        seq=0,
        frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=dialect.MAV_CMD_NAV_WAYPOINT,
        current=2,
        autocontinue=0,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        x=int(latitude * 1e7),
        y=int(longitude * 1e7),
        z=altitude,
    )
    vehicle.mav.send(message)


def wait_at_location(
    vehicle,
    latitude: float,
    longitude: float,
    tolerance_m: float = 1.0,
    timeout: Optional[float] = 300.0,
) -> bool:
    """Wait until the vehicle is within tolerance of the target lat/lon."""
    target = (latitude, longitude)
    end = deadline(timeout)
    current_location = {"latitude": 0.0, "longitude": 0.0}

    while True:
        remaining = remaining_timeout(end)
        if end is not None and remaining == 0.0:
            raise VehicleTimeout("Timed out waiting at location")

        message = recv_message_dict(
            vehicle,
            msg_type=[
                dialect.MAVLink_position_target_global_int_message.msgname,
                dialect.MAVLink_global_position_int_message.msgname,
            ],
            timeout=remaining,
            blocking=True,
        )
        if message is None:
            continue

        if message["mavpackettype"] == dialect.MAVLink_global_position_int_message.msgname:
            current_location["latitude"] = message["lat"] * 1e-7
            current_location["longitude"] = message["lon"] * 1e-7

        if message["mavpackettype"] == dialect.MAVLink_position_target_global_int_message.msgname:
            target_lat = message["lat_int"] * 1e-7
            target_lon = message["lon_int"] * 1e-7
            distance = geopy.distance.GeodesicDistance(
                (current_location["latitude"], current_location["longitude"]),
                (target_lat, target_lon),
            ).meters
            if distance < tolerance_m:
                return True


def set_ground_speed(vehicle, speed_mps: float) -> None:
    """Set ground speed in meters per second."""
    message = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_DO_CHANGE_SPEED,
        confirmation=0,
        param1=0,
        param2=speed_mps,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    vehicle.mav.send(message)


def set_yaw(
    vehicle,
    angle_deg: float = 90.0,
    angular_speed: float = 0.0,
    direction: int = DIRECTION_CCW,
    angle_type: int = ANGLE_RELATIVE,
) -> None:
    """Set vehicle yaw using MAV_CMD_CONDITION_YAW."""
    message = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_CONDITION_YAW,
        confirmation=0,
        param1=angle_deg,
        param2=angular_speed,
        param3=direction,
        param4=angle_type,
        param5=0,
        param6=0,
        param7=0,
    )
    vehicle.mav.send(message)


def _pause_continue(vehicle, action: int) -> None:
    command = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_DO_PAUSE_CONTINUE,
        confirmation=0,
        param1=action,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    vehicle.mav.send(command)


def pause(vehicle) -> None:
    """Pause an autonomous flight."""
    _pause_continue(vehicle, PAUSE)


def resume(vehicle) -> None:
    """Resume a paused autonomous flight."""
    _pause_continue(vehicle, RESUME)
