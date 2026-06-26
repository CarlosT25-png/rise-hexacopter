"""Mission upload/download helpers.

Sources:
  pymavlink-utils/get-mission.py
  pymavlink-utils/set-mission.py
  pymavlink-utils/set-mission-partial.py
  pymavlink-utils/clear-mission.py
  pymavlink-utils/set-current.py
"""

from typing import Dict, List, Optional, Sequence, Tuple, Union

from hex_mavlink._common import (
    VehicleTimeout,
    deadline,
    dialect,
    recv_message_dict,
    remaining_timeout,
    wait_for_message,
)

__all__ = [
    "get_mission_count",
    "get_mission",
    "upload_mission",
    "upload_mission_partial",
    "clear_mission",
    "get_current_mission_item",
    "set_current_mission_item",
]

MissionItemDict = Dict[str, Union[int, float, str]]
Waypoint = Tuple[float, float, float]


def get_mission_count(vehicle, timeout: Optional[float] = 30.0) -> int:
    """Return the number of mission items on the vehicle."""
    message = dialect.MAVLink_mission_request_list_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        mission_type=dialect.MAV_MISSION_TYPE_MISSION,
    )
    vehicle.mav.send(message)
    response = wait_for_message(
        vehicle,
        dialect.MAVLink_mission_count_message.msgname,
        timeout=timeout,
    )
    return response["count"]


def get_mission(vehicle, timeout: Optional[float] = 120.0) -> List[MissionItemDict]:
    """Download the full mission from the vehicle."""
    count = get_mission_count(vehicle, timeout=timeout)
    mission_item_list = []
    for seq in range(count):
        message = dialect.MAVLink_mission_request_int_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            seq=seq,
            mission_type=dialect.MAV_MISSION_TYPE_MISSION,
        )
        vehicle.mav.send(message)
        item = wait_for_message(
            vehicle,
            dialect.MAVLink_mission_item_int_message.msgname,
            timeout=timeout,
        )
        mission_item_list.append(item)
    return mission_item_list


def _mission_item_for_seq(
    vehicle,
    seq: int,
    target_locations: Sequence[Waypoint],
):
    if seq == 0:
        return dialect.MAVLink_mission_item_int_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            seq=seq,
            frame=dialect.MAV_FRAME_GLOBAL,
            command=dialect.MAV_CMD_NAV_WAYPOINT,
            current=0,
            autocontinue=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            x=0,
            y=0,
            z=0,
            mission_type=dialect.MAV_MISSION_TYPE_MISSION,
        )
    if seq == 1:
        return dialect.MAVLink_mission_item_int_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            seq=seq,
            frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=dialect.MAV_CMD_NAV_TAKEOFF,
            current=0,
            autocontinue=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            x=0,
            y=0,
            z=target_locations[0][2],
            mission_type=dialect.MAV_MISSION_TYPE_MISSION,
        )
    waypoint = target_locations[seq - 2]
    return dialect.MAVLink_mission_item_int_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        seq=seq,
        frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=dialect.MAV_CMD_NAV_WAYPOINT,
        current=0,
        autocontinue=0,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        x=int(waypoint[0] * 1e7),
        y=int(waypoint[1] * 1e7),
        z=waypoint[2],
        mission_type=dialect.MAV_MISSION_TYPE_MISSION,
    )


def upload_mission(
    vehicle,
    target_locations: Sequence[Waypoint],
    timeout: Optional[float] = 120.0,
) -> bool:
    """Upload a mission with home, takeoff, and waypoint items."""
    count = len(target_locations) + 2
    message = dialect.MAVLink_mission_count_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        count=count,
        mission_type=dialect.MAV_MISSION_TYPE_MISSION,
    )
    vehicle.mav.send(message)

    end = deadline(timeout)
    while True:
        remaining = remaining_timeout(end)
        if end is not None and remaining == 0.0:
            raise VehicleTimeout("Timed out uploading mission")

        incoming = recv_message_dict(vehicle, timeout=remaining, blocking=True)
        if incoming is None:
            continue

        if incoming["mavpackettype"] == dialect.MAVLink_mission_request_message.msgname:
            if incoming["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION:
                seq = incoming["seq"]
                item = _mission_item_for_seq(vehicle, seq, target_locations)
                vehicle.mav.send(item)

        elif incoming["mavpackettype"] == dialect.MAVLink_mission_ack_message.msgname:
            if (
                incoming["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION
                and incoming["type"] == dialect.MAV_MISSION_ACCEPTED
            ):
                return True


def upload_mission_partial(
    vehicle,
    items: Dict[int, Waypoint],
    start_index: int,
    end_index: int,
    timeout: Optional[float] = 120.0,
) -> bool:
    """Upload a partial mission item range."""
    message = dialect.MAVLink_mission_write_partial_list_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        start_index=start_index,
        end_index=end_index,
        mission_type=dialect.MAV_MISSION_TYPE_MISSION,
    )
    vehicle.mav.send(message)

    end = deadline(timeout)
    while True:
        remaining = remaining_timeout(end)
        if end is not None and remaining == 0.0:
            raise VehicleTimeout("Timed out uploading partial mission")

        incoming = recv_message_dict(vehicle, timeout=remaining, blocking=True)
        if incoming is None:
            continue

        if incoming["mavpackettype"] == dialect.MAVLink_mission_request_message.msgname:
            if incoming["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION:
                seq = incoming["seq"]
                waypoint = items[seq]
                item = dialect.MAVLink_mission_item_int_message(
                    target_system=vehicle.target_system,
                    target_component=vehicle.target_component,
                    seq=seq,
                    frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    command=dialect.MAV_CMD_NAV_WAYPOINT,
                    current=0,
                    autocontinue=0,
                    param1=0,
                    param2=0,
                    param3=0,
                    param4=0,
                    x=int(waypoint[0] * 1e7),
                    y=int(waypoint[1] * 1e7),
                    z=waypoint[2],
                    mission_type=dialect.MAV_MISSION_TYPE_MISSION,
                )
                vehicle.mav.send(item)

        elif incoming["mavpackettype"] == dialect.MAVLink_mission_ack_message.msgname:
            if (
                incoming["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION
                and incoming["type"] == dialect.MAV_MISSION_ACCEPTED
            ):
                return True


def clear_mission(vehicle, timeout: Optional[float] = 60.0) -> int:
    """Clear all mission items and return the new mission count."""
    get_mission_count(vehicle, timeout=timeout)

    message = dialect.MAVLink_mission_clear_all_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        mission_type=dialect.MAV_MISSION_TYPE_MISSION,
    )
    vehicle.mav.send(message)
    return get_mission_count(vehicle, timeout=timeout)


def get_current_mission_item(vehicle, timeout: Optional[float] = 30.0) -> int:
    """Return the current mission item sequence number."""
    message = wait_for_message(
        vehicle,
        dialect.MAVLink_mission_current_message.msgname,
        timeout=timeout,
    )
    return message["seq"]


def set_current_mission_item(
    vehicle,
    seq_desired: int,
    timeout: Optional[float] = 30.0,
) -> bool:
    """Set the current mission item and verify via MISSION_CURRENT."""
    message = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_DO_SET_MISSION_CURRENT,
        confirmation=0,
        param1=seq_desired,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    vehicle.mav.send(message)

    response = wait_for_message(
        vehicle,
        dialect.MAVLink_mission_current_message.msgname,
        timeout=timeout,
    )
    return response["seq"] == seq_desired
