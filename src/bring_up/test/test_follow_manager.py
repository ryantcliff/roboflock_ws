from bring_up.follow_manager import FollowLogic


def make(enabled=True):
    return FollowLogic(enabled=enabled, stale_after=3.0, retry_delay=2.0)


def test_waits_for_target():
    assert make().step(now=0.0) is None


def test_sends_once_when_fresh():
    logic = make()
    logic.on_target(now=0.0)
    assert logic.step(now=0.1) == 'send'
    assert logic.step(now=0.2) is None


def test_disabled_never_sends():
    logic = make(enabled=False)
    logic.on_target(now=0.0)
    assert logic.step(now=0.1) is None


def test_cancels_when_target_stale():
    logic = make()
    logic.on_target(now=0.0)
    logic.step(now=0.1)
    logic.on_accepted()
    assert logic.step(now=2.9) is None
    assert logic.step(now=3.1) == 'cancel'


def test_cancels_when_disabled():
    logic = make()
    logic.on_target(now=0.0)
    logic.step(now=0.1)
    logic.on_accepted()
    logic.enabled = False
    assert logic.step(now=0.2) == 'cancel'


def test_resends_immediately_after_own_cancel():
    logic = make()
    logic.on_target(now=0.0)
    logic.step(now=0.1)
    logic.on_accepted()
    logic.step(now=3.5)
    logic.on_finished(now=3.6, cancelled_by_us=True)
    logic.on_target(now=4.0)
    assert logic.step(now=4.0) == 'send'


def test_retry_delay_after_abort():
    logic = make()
    logic.on_target(now=0.0)
    logic.step(now=0.1)
    logic.on_accepted()
    logic.on_finished(now=1.0, cancelled_by_us=False)
    logic.on_target(now=1.5)
    assert logic.step(now=1.5) is None
    assert logic.step(now=3.0) == 'send'
