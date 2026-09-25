from bring_up.gps_monitor import FixHealth
from sensor_msgs.msg import NavSatStatus


def test_stale_before_first_fix():
    assert FixHealth(2.0).state(now=0.0) == 'stale'


def test_ok_with_fresh_fix():
    health = FixHealth(2.0)
    health.on_fix(NavSatStatus.STATUS_FIX, now=0.0)
    assert health.state(now=1.0) == 'ok'


def test_no_fix_status():
    health = FixHealth(2.0)
    health.on_fix(NavSatStatus.STATUS_NO_FIX, now=0.0)
    assert health.state(now=0.5) == 'no_fix'


def test_stale_after_max_age():
    health = FixHealth(2.0)
    health.on_fix(NavSatStatus.STATUS_GBAS_FIX, now=0.0)
    assert health.state(now=2.5) == 'stale'
