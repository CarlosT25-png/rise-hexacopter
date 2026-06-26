"""Arming and disarming helpers.

Source: pymavlink-utils/arm-disarm.py
"""

from typing import Optional

from hex_mavlink._common import (
    dialect,
    recv_message_dict,
    send_and_wait_ack,
    wait_for_message,
)

__all__ = [
    "VEHICLE_ARM",
    "VEHICLE_DISARM",
    "wait_prearm",
    "arm",
    "disarm",
    "is_armed",
    "wait_armed",
    "wait_disarmed",
]

VEHICLE_ARM = 1
VEHICLE_DISARM = 0


def _arm_disarm_message(vehicle, arm: int):
    return dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation=0,
        param1=arm,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )


def wait_prearm(vehicle, timeout: Optional[float] = 120.0) -> bool:
    """Wait until the vehicle reports pre-arm checks passed."""
    wait_for_message(
        vehicle,
        dialect.MAVLink_sys_status_message.msgname,
        predicate=lambda msg: (
            msg["onboard_control_sensors_health"] & dialect.MAV_SYS_STATUS_PREARM_CHECK
        )
        == dialect.MAV_SYS_STATUS_PREARM_CHECK,
        timeout=timeout,
    )
    return True


def is_armed(vehicle) -> bool:
    """Return True if the latest HEARTBEAT indicates the vehicle is armed."""
    message = recv_message_dict(
        vehicle,
        dialect.MAVLink_heartbeat_message.msgname,
        timeout=1.0,
        blocking=True,
    )
    if message is None:
        return False
    armed_bit = message["base_mode"] & dialect.MAV_MODE_FLAG_SAFETY_ARMED
    return armed_bit == dialect.MAV_MODE_FLAG_SAFETY_ARMED


def arm(vehicle, timeout: Optional[float] = 30.0) -> bool:
    """Send arm command and wait for COMMAND_ACK acceptance."""
    vehicle.mav.send(_arm_disarm_message(vehicle, VEHICLE_ARM))
    ack = send_and_wait_ack(vehicle, dialect.MAV_CMD_COMPONENT_ARM_DISARM, timeout=timeout)
    return (
        ack["result"] == dialect.MAV_RESULT_ACCEPTED
        and ack["command"] == dialect.MAV_CMD_COMPONENT_ARM_DISARM
    )


def disarm(vehicle, timeout: Optional[float] = 30.0) -> bool:
    """Send disarm command and wait for COMMAND_ACK acceptance."""
    vehicle.mav.send(_arm_disarm_message(vehicle, VEHICLE_DISARM))
    ack = send_and_wait_ack(vehicle, dialect.MAV_CMD_COMPONENT_ARM_DISARM, timeout=timeout)
    return (
        ack["result"] == dialect.MAV_RESULT_ACCEPTED
        and ack["command"] == dialect.MAV_CMD_COMPONENT_ARM_DISARM
    )


def wait_armed(vehicle, timeout: Optional[float] = 30.0) -> bool:
    """Wait until HEARTBEAT reports the vehicle is armed."""
    wait_for_message(
        vehicle,
        dialect.MAVLink_heartbeat_message.msgname,
        predicate=lambda msg: (
            msg["base_mode"] & dialect.MAV_MODE_FLAG_SAFETY_ARMED
        )
        == dialect.MAV_MODE_FLAG_SAFETY_ARMED,
        timeout=timeout,
    )
    return True


def wait_disarmed(vehicle, timeout: Optional[float] = 30.0) -> bool:
    """Wait until HEARTBEAT reports the vehicle is disarmed."""
    wait_for_message(
        vehicle,
        dialect.MAVLink_heartbeat_message.msgname,
        predicate=lambda msg: (
            msg["base_mode"] & dialect.MAV_MODE_FLAG_SAFETY_ARMED
        )
        != dialect.MAV_MODE_FLAG_SAFETY_ARMED,
        timeout=timeout,
    )
    return True
