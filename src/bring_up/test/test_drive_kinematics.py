import math

import pytest

from bring_up.drive_kinematics import wheel_turn_rates


def calculate(linear, angular, limit=1000.0):
    return wheel_turn_rates(
        linear,
        angular,
        wheel_radius=0.1,
        wheel_separation=0.64,
        gear_ratio=30.0,
        max_motor_turns_per_second=limit,
    )


def test_straight_motion_has_equal_wheel_rates():
    left, right = calculate(0.5, 0.0)
    expected = 0.5 * 30.0 / (2.0 * math.pi * 0.1)
    assert left == pytest.approx(expected)
    assert right == pytest.approx(expected)


def test_rotation_has_opposite_wheel_rates():
    left, right = calculate(0.0, 1.0)
    assert left == pytest.approx(-right)
    assert left < 0.0 < right


def test_rates_are_scaled_without_changing_curvature():
    unlimited = calculate(1.0, 0.5)
    limited = calculate(1.0, 0.5, limit=20.0)
    assert max(abs(value) for value in limited) == pytest.approx(20.0)
    assert limited[0] / limited[1] == pytest.approx(
        unlimited[0] / unlimited[1]
    )


def test_invalid_geometry_is_rejected():
    with pytest.raises(ValueError):
        wheel_turn_rates(0.0, 0.0, 0.0, 0.64, 30.0, 60.0)
