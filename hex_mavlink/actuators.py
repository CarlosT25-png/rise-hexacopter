"""Servo, relay, and RC channel helpers.

Sources:
  pymavlink-utils/set-servo.py
  pymavlink-utils/set-relay.py
  pymavlink-utils/rc-override.py
  pymavlink-utils/rc-servo.py
"""

from typing import Dict, List, Optional

from hex_mavlink._common import dialect, wait_for_message

__all__ = [
    "set_servo",
    "repeat_servo",
    "set_relay",
    "repeat_relay",
    "build_rc_override",
    "send_rc_override",
    "read_rc_channels",
    "read_servo_outputs",
    "clamp_pwm",
]

RC_IGNORE = 65535


def set_servo(vehicle, channel: int, pwm_value: int) -> None:
    """Set a servo channel PWM value."""
    command = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_DO_SET_SERVO,
        confirmation=0,
        param1=channel,
        param2=pwm_value,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    vehicle.mav.send(command)


def repeat_servo(
    vehicle,
    channel: int,
    pwm_value: int,
    count: int,
    period: float,
) -> None:
    """Pulse a servo channel repeatedly."""
    command = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_DO_REPEAT_SERVO,
        confirmation=0,
        param1=channel,
        param2=pwm_value,
        param3=count,
        param4=period,
        param5=0,
        param6=0,
        param7=0,
    )
    vehicle.mav.send(command)


def set_relay(vehicle, instance: int, state: int) -> None:
    """Set a relay on (1) or off (0)."""
    command = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_DO_SET_RELAY,
        confirmation=0,
        param1=instance,
        param2=state,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    vehicle.mav.send(command)


def repeat_relay(
    vehicle,
    instance: int,
    count: int,
    period: float,
) -> None:
    """Toggle a relay repeatedly."""
    command = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=dialect.MAV_CMD_DO_REPEAT_RELAY,
        confirmation=0,
        param1=instance,
        param2=count,
        param3=period,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    vehicle.mav.send(command)


def clamp_pwm(value: int, offset: int = 0) -> int:
    """Clamp a PWM value to the 1000-2000 microsecond range."""
    value = value + offset
    if value < 1000:
        value = 1000
    if value > 2000:
        value = 2000
    return value


def build_rc_override(vehicle, channels: Dict[int, int]):
    """Build an RC_CHANNELS_OVERRIDE message from 1-based channel values."""
    temp_channels = [RC_IGNORE] * 18
    for channel_number, pwm in channels.items():
        if 1 <= channel_number <= 18:
            temp_channels[channel_number - 1] = pwm
    return dialect.MAVLink_rc_channels_override_message(
        vehicle.target_system,
        vehicle.target_component,
        *temp_channels,
    )


def send_rc_override(vehicle, channels: Dict[int, int]) -> None:
    """Send RC channel overrides (1-based channel numbers, 65535 to ignore)."""
    message = build_rc_override(vehicle, channels)
    vehicle.mav.send(message)


def read_rc_channels(vehicle, timeout: Optional[float] = 30.0) -> Dict[str, int]:
    """Wait for and return an RC_CHANNELS message as a dictionary."""
    return wait_for_message(
        vehicle,
        dialect.MAVLink_rc_channels_message.msgname,
        timeout=timeout,
    )


def read_servo_outputs(vehicle, timeout: Optional[float] = 30.0) -> Dict[str, int]:
    """Wait for and return a SERVO_OUTPUT_RAW message as a dictionary."""
    return wait_for_message(
        vehicle,
        dialect.MAVLink_servo_output_raw_message.msgname,
        timeout=timeout,
    )
