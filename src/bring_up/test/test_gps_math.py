import pytest

from bring_up.gps_math import offset_to_fix


def test_zero_offset_preserves_origin():
    assert offset_to_fix(40.0, -119.0, 0.0, 0.0) == (40.0, -119.0)


def test_north_offset_increases_latitude():
    latitude, longitude = offset_to_fix(40.0, -119.0, 0.0, 10.0)
    assert latitude > 40.0
    assert longitude == pytest.approx(-119.0)


def test_east_offset_increases_longitude():
    latitude, longitude = offset_to_fix(40.0, -119.0, 10.0, 0.0)
    assert latitude == pytest.approx(40.0)
    assert longitude > -119.0
