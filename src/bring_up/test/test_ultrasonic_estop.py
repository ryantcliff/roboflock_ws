import math

from bring_up.ultrasonic_estop import UltrasonicLogic


def make(required=2):
    return UltrasonicLogic(['left', 'center', 'right'], min_safe_distance=0.15,
                           max_reading_age=1.0, required_readings=required)


def test_clear_with_no_readings():
    assert not make().unsafe(now=0.0)


def test_single_close_reading_is_ignored():
    logic = make()
    logic.on_range('center', 0.29, now=0.00)
    logic.on_range('center', 0.05, now=0.05)  # one-off glitch
    assert not logic.unsafe(now=0.06)
    logic.on_range('center', 0.29, now=0.10)
    assert not logic.unsafe(now=0.11)


def test_two_close_readings_in_a_row_stop():
    logic = make()
    logic.on_range('center', 0.10, now=0.00)
    logic.on_range('center', 0.11, now=0.05)
    assert logic.unsafe(now=0.06)


def test_close_readings_must_be_consecutive():
    logic = make()
    logic.on_range('right', 0.10, now=0.00)
    logic.on_range('right', 0.40, now=0.05)
    logic.on_range('right', 0.10, now=0.10)
    assert not logic.unsafe(now=0.11)


def test_counts_are_per_sensor():
    logic = make()
    logic.on_range('left', 0.10, now=0.00)
    logic.on_range('right', 0.10, now=0.05)
    assert not logic.unsafe(now=0.06)


def test_clears_after_one_far_reading():
    logic = make()
    logic.on_range('left', 0.10, now=0.00)
    logic.on_range('left', 0.10, now=0.05)
    assert logic.unsafe(now=0.06)
    logic.on_range('left', 0.60, now=0.10)
    assert not logic.unsafe(now=0.11)


def test_no_echo_is_clear():
    logic = make()
    logic.on_range('center', math.inf, now=0.00)
    logic.on_range('center', math.inf, now=0.05)
    assert not logic.unsafe(now=0.06)


def test_stale_readings_are_ignored():
    logic = make()
    logic.on_range('center', 0.10, now=0.00)
    logic.on_range('center', 0.10, now=0.05)
    assert not logic.unsafe(now=1.10)


def test_required_readings_of_one_stops_immediately():
    logic = make(required=1)
    logic.on_range('center', 0.10, now=0.00)
    assert logic.unsafe(now=0.01)
