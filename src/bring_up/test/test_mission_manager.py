from bring_up.mission_manager import MissionLogic


def make(mode='follow'):
    return MissionLogic(mode=mode, stale_after={'follow': 3.0, 'home': 3.0}, retry_delay=2.0)


def start(logic, mode, now=0.0):
    logic.on_target(mode, now)
    assert logic.step(now) == ('send', mode)
    logic.on_accepted()


def test_idle_never_sends():
    logic = make('idle')
    logic.on_target('follow', 0.0)
    logic.on_target('home', 0.0)
    assert logic.step(0.1) is None


def test_waits_for_target_of_current_mode():
    logic = make('home')
    logic.on_target('follow', 0.0)
    assert logic.step(0.1) is None
    logic.on_target('home', 0.2)
    assert logic.step(0.2) == ('send', 'home')


def test_cancels_when_target_stale():
    logic = make()
    start(logic, 'follow')
    assert logic.step(2.9) is None
    assert logic.step(3.1) == ('cancel', 'follow target stale')


def test_mode_switch_cancels_then_sends_new_mode():
    logic = make()
    start(logic, 'follow')
    logic.on_target('home', 0.5)
    logic.set_mode('home')
    assert logic.step(0.6) == ('cancel', 'mode changed to home')
    assert logic.step(0.7) is None  # waiting for the cancel to finish
    logic.on_finished(0.8, cancelled_by_us=True)
    logic.on_target('home', 0.8)
    assert logic.step(0.8) == ('send', 'home')


def test_stop_cancels():
    logic = make()
    start(logic, 'follow')
    logic.set_mode('idle')
    assert logic.step(0.1) == ('cancel', 'mode changed to idle')


def test_retry_delay_after_abort_but_not_after_mode_change():
    logic = make()
    start(logic, 'follow')
    logic.on_finished(1.0, cancelled_by_us=False)
    logic.on_target('follow', 1.5)
    assert logic.step(1.5) is None
    logic.set_mode('home')  # a new mode clears the retry delay
    logic.on_target('home', 1.6)
    assert logic.step(1.6) == ('send', 'home')
