"""MAVLink message send/receive helpers.

Sources:
  pymavlink-utils/receive-message.py
  pymavlink-utils/send-message.py
  pymavlink-utils/request-stream.py
  pymavlink-utils/request-message.py
  pymavlink-utils/request-defaults.py
"""

from typing import Any, Dict, List, Optional, Union

from hex_mavlink._common import (
    dialect,
    recv_message,
    recv_message_dict,
    wait_for_message,
)

__all__ = [
    "recv_message",
    "recv_message_dict",
    "send_command_long",
    "send_banner",
    "request_data_stream",
    "request_message_interval",
    "request_message",
    "request_autopilot_version",
]


def send_command_long(
    vehicle,
    command: int,
    param1: float = 0,
    param2: float = 0,
    param3: float = 0,
    param4: float = 0,
    param5: float = 0,
    param6: float = 0,
    param7: float = 0,
    confirmation: int = 0,
) -> None:
    """Send a COMMAND_LONG message to the vehicle."""
    message = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=command,
        confirmation=confirmation,
        param1=param1,
        param2=param2,
        param3=param3,
        param4=param4,
        param5=param5,
        param6=param6,
        param7=param7,
    )
    vehicle.mav.send(message)


def send_banner(vehicle) -> None:
    """Send MAV_CMD_DO_SEND_BANNER and return STATUSTEXT/COMMAND_ACK responses."""
    send_command_long(vehicle, dialect.MAV_CMD_DO_SEND_BANNER)


def request_data_stream(
    vehicle,
    stream_id: int = 0,
    message_rate: float = 4.0,
    start_stop: int = 1,
) -> None:
    """Request default data streams via REQUEST_DATA_STREAM."""
    message = dialect.MAVLink_request_data_stream_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        req_stream_id=stream_id,
        req_message_rate=message_rate,
        start_stop=start_stop,
    )
    vehicle.mav.send(message)


def request_message_interval(
    vehicle,
    message_id: int,
    interval_us: float,
) -> None:
    """Request a message interval via MAV_CMD_SET_MESSAGE_INTERVAL."""
    send_command_long(
        vehicle,
        dialect.MAV_CMD_SET_MESSAGE_INTERVAL,
        param1=message_id,
        param2=interval_us,
    )


def _msgname_for_id(message_id: int) -> str:
    for msgid, msgclass in dialect.mavlink_map.items():
        if msgid == message_id:
            if hasattr(msgclass, "msgname"):
                return msgclass.msgname
            return msgclass.name
    raise ValueError("Unknown MAVLink message id: {}".format(message_id))


def request_message(
    vehicle,
    message_id: int,
    timeout: Optional[float] = 30.0,
) -> Dict[str, Any]:
    """Request a single message via MAV_CMD_REQUEST_MESSAGE and wait for it."""
    send_command_long(
        vehicle,
        dialect.MAV_CMD_REQUEST_MESSAGE,
        param1=message_id,
    )
    msg_name = _msgname_for_id(message_id)
    return wait_for_message(vehicle, msg_name, timeout=timeout)


def request_autopilot_version(
    vehicle,
    timeout: Optional[float] = 30.0,
) -> Dict[str, Any]:
    """Request AUTOPILOT_VERSION from the vehicle."""
    return request_message(
        vehicle,
        dialect.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        timeout=timeout,
    )
