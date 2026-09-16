from bring_up.safety_logic import SafetyController


def make_controller(require_ultrasonic=False):
    return SafetyController(
        command_timeout=0.5,
        range_timeout=0.5,
        stop_distance=0.3,
        require_ultrasonic=require_ultrasonic,
    )


def test_teleop_has_priority_over_navigation():
    controller = make_controller()
    controller.set_command('nav', 0.4, 0.1, 1.0)
    controller.set_command('teleop', 0.2, -0.3, 1.0)

    assert controller.evaluate(1.1) == (0.2, -0.3, 'running')


def test_stale_command_stops_motion():
    controller = make_controller()
    controller.set_command('nav', 0.4, 0.0, 1.0)

    assert controller.evaluate(1.6) == (0.0, 0.0, 'command_timeout')


def test_close_obstacle_stops_forward_motion():
    controller = make_controller()
    controller.set_command('nav', 0.4, 0.0, 1.0)
    controller.set_range('center', 0.2, 1.0)

    assert controller.evaluate(1.1) == (0.0, 0.0, 'ultrasonic_stop')


def test_reverse_motion_can_escape_obstacle():
    controller = make_controller(require_ultrasonic=True)
    controller.set_command('teleop', -0.2, 0.0, 1.0)

    assert controller.evaluate(1.1) == (-0.2, 0.0, 'running')


def test_missing_required_ultrasonic_blocks_forward_motion():
    controller = make_controller(require_ultrasonic=True)
    controller.set_command('nav', 0.4, 0.0, 1.0)

    assert controller.evaluate(1.1) == (
        0.0,
        0.0,
        'ultrasonic_unavailable',
    )


def test_emergency_stop_is_latched_until_reset():
    controller = make_controller()
    controller.set_command('teleop', 0.2, 0.0, 1.0)
    controller.set_emergency_stop(True)
    controller.set_emergency_stop(False)

    assert controller.evaluate(1.1) == (0.0, 0.0, 'emergency_stop')
    assert controller.reset()
    assert controller.evaluate(1.1) == (0.2, 0.0, 'running')
