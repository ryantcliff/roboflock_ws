from bring_up.e_stop import EStopLogic


def make(**kwargs):
    params = dict(stop_button=1, reset_button=9, joy_timeout=0.5)
    params.update(kwargs)
    return EStopLogic(**params)


def buttons(*pressed, count=12):
    return [1 if i in pressed else 0 for i in range(count)]


def test_starts_stopped():
    assert make().stopped


def test_reset_arms_when_joy_fresh():
    logic = make()
    logic.on_joy(buttons(9), now=0.0)
    assert not logic.update(now=0.1)


def test_reset_ignored_while_stop_held():
    logic = make()
    logic.on_joy(buttons(1, 9), now=0.0)
    assert logic.update(now=0.1)


def test_stop_button_latches():
    logic = make()
    logic.on_joy(buttons(9), now=0.0)
    logic.update(now=0.0)
    logic.on_joy(buttons(1), now=0.1)
    assert logic.update(now=0.1)
    logic.on_joy(buttons(), now=0.2)
    assert logic.update(now=0.2)


def test_joy_timeout_stops_and_latches():
    logic = make()
    logic.on_joy(buttons(9), now=0.0)
    logic.update(now=0.0)
    assert logic.update(now=0.6)
    logic.on_joy(buttons(), now=0.7)
    assert logic.update(now=0.7)


def test_no_joy_ever_stays_stopped():
    assert make().update(now=10.0)


def test_short_buttons_array_does_not_crash():
    logic = make()
    logic.on_joy([0, 0], now=0.0)
    assert logic.update(now=0.1)
