"""Shared helpers for hex_mavlink.

Thread safety: this library does not acquire vehicle.mavlink_lock. Callers using
multiple threads (as in main.py) should wrap recv/send externally.
"""

import time
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import pymavlink.dialects.v20.all as dialect

__all__ = [
    "dialect",
    "VehicleTimeout",
    "recv_message",
    "recv_message_dict",
    "wait_for_message",
    "send_and_wait_ack",
    "wait_param_value",
    "deadline",
    "remaining_timeout",
]

Vehicle = Any
MessageDict = Dict[str, Any]
Predicate = Callable[[MessageDict], bool]


class VehicleTimeout(Exception):
    """Raised when a blocking MAVLink operation exceeds its timeout."""


def deadline(timeout: Optional[float]) -> Optional[float]:
    if timeout is None:
        return None
    return time.monotonic() + timeout


def remaining_timeout(end: Optional[float]) -> Optional[float]:
    if end is None:
        return None
    return max(0.0, end - time.monotonic())


def recv_message(
    vehicle: Vehicle,
    msg_type: Optional[Union[str, List[str]]] = None,
    timeout: Optional[float] = None,
    blocking: bool = True,
):
    """Receive a MAVLink message, optionally filtered by type."""
    return vehicle.recv_match(
        type=msg_type,
        blocking=blocking,
        timeout=timeout,
    )


def recv_message_dict(
    vehicle: Vehicle,
    msg_type: Optional[Union[str, List[str]]] = None,
    timeout: Optional[float] = None,
    blocking: bool = True,
) -> Optional[MessageDict]:
    """Receive a message and convert it to a dictionary."""
    message = recv_message(vehicle, msg_type=msg_type, timeout=timeout, blocking=blocking)
    if message is None:
        return None
    return message.to_dict()


def wait_for_message(
    vehicle: Vehicle,
    msg_type: Union[str, List[str]],
    predicate: Optional[Predicate] = None,
    timeout: Optional[float] = None,
) -> MessageDict:
    """Block until a matching message is received or timeout expires."""
    end = deadline(timeout)
    while True:
        remaining = remaining_timeout(end)
        if end is not None and remaining == 0.0:
            raise VehicleTimeout("Timed out waiting for {!r}".format(msg_type))

        message = recv_message_dict(
            vehicle,
            msg_type=msg_type,
            timeout=remaining,
            blocking=True,
        )
        if message is None:
            continue
        if predicate is None or predicate(message):
            return message


def send_and_wait_ack(
    vehicle: Vehicle,
    command: int,
    timeout: Optional[float] = 10.0,
) -> MessageDict:
    """Wait for COMMAND_ACK for the given command id."""
    return wait_for_message(
        vehicle,
        dialect.MAVLink_command_ack_message.msgname,
        predicate=lambda msg: msg["command"] == command,
        timeout=timeout,
    )


def wait_param_value(
    vehicle: Vehicle,
    param_id: str,
    timeout: Optional[float] = 30.0,
) -> MessageDict:
    """Wait for a PARAM_VALUE message for the given parameter name."""
    return wait_for_message(
        vehicle,
        dialect.MAVLink_param_value_message.msgname,
        predicate=lambda msg: msg["param_id"] == param_id,
        timeout=timeout,
    )
