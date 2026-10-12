from path_planner_server.rotation_assist import RotationAssist


def make_assist():
    return RotationAssist(0.8, 0.001, 0.03, 0.05, 0.4, 0.2, 0.5, 0.5, 0.3)


def step(assist, now, command=0.18, measured=0.0, vx=0.0):
    assist.update_odom(measured, now)
    return assist.apply(vx, command, now)


def start_pulse(assist):
    assert step(assist, 0.0) == 0.18
    assert step(assist, 0.2) == 0.18
    assert step(assist, 0.41) == 0.8


def test_stall_pulse_has_bounded_duration_and_cooldown():
    a = make_assist()
    start_pulse(a)
    assert step(a, 0.5) == 0.8
    assert step(a, 0.62) == 0.18
    for now in (0.7, 0.9, 1.1):
        assert step(a, now) == 0.18
    assert step(a, 1.13) == 0.8


def test_movement_does_not_interrupt_bounded_pulse():
    a = make_assist()
    start_pulse(a)
    assert step(a, 0.45, measured=0.1) == 0.8
    assert step(a, 0.5, command=0.06, measured=0.2) == 0.8
    assert step(a, 0.62, command=0.06, measured=0.2) == 0.06


def test_longer_pulse_still_expires_while_turning():
    a = make_assist()
    a.pulse_duration = 0.6
    start_pulse(a)
    for now in (0.5, 0.7, 0.9, 1.0):
        assert step(a, now, measured=0.2) == 0.8
    assert step(a, 1.02, measured=0.2) == 0.18


def test_stop_and_reversal_interrupt_pulse():
    a = make_assist()
    start_pulse(a)
    assert step(a, 0.45, command=0.0) == 0.0
    assert step(a, 0.5, command=-0.18) == -0.18
    assert step(a, 0.7, command=-0.18) == -0.18
    assert step(a, 0.91, command=-0.18) == -0.8
    assert step(a, 0.95, command=0.18) == 0.18


def test_missing_or_stale_odometry_disables_assistance():
    a = make_assist()
    assert a.apply(0.0, 0.18, 0.0) == 0.18
    start_pulse(a)
    assert a.apply(0.0, 0.18, 0.92) == 0.18
    assert a.pulse_until is None


def test_command_gap_and_forward_motion_reset_stall():
    a = make_assist()
    start_pulse(a)
    assert step(a, 0.8) == 0.18
    assert step(a, 0.9, vx=0.1) == 0.18
    assert step(a, 1.0) == 0.18
