import math
from typing import Tuple


def wheel_turn_rates(
    linear_velocity: float,
    angular_velocity: float,
    wheel_radius: float,
    wheel_separation: float,
    gear_ratio: float,
    max_motor_turns_per_second: float,
) -> Tuple[float, float]:
    if wheel_radius <= 0.0:
        raise ValueError('wheel_radius must be positive')
    if wheel_separation <= 0.0:
        raise ValueError('wheel_separation must be positive')
    if gear_ratio <= 0.0:
        raise ValueError('gear_ratio must be positive')

    left_velocity = linear_velocity - angular_velocity * wheel_separation / 2.0
    right_velocity = linear_velocity + angular_velocity * wheel_separation / 2.0
    conversion = gear_ratio / (2.0 * math.pi * wheel_radius)

    left_turns = left_velocity * conversion
    right_turns = right_velocity * conversion
    peak = max(abs(left_turns), abs(right_turns))
    if peak > max_motor_turns_per_second:
        scale = max_motor_turns_per_second / peak
        left_turns *= scale
        right_turns *= scale

    return left_turns, right_turns
