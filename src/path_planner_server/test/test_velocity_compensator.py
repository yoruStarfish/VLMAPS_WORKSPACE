import math

import pytest

from path_planner_server.velocity_compensator import VelocityCompensator


def step(c, t, vx=0.0, wz=0.06, measured=0.0):
    c.update_odom(measured, t)
    return c.apply(vx, wz, t)


def test_already_rotating_does_not_trigger_assistance():
    c = VelocityCompensator()
    for i in range(60):
        assert step(c, i * 0.05, measured=0.2) == (0.0, 0.06)


def test_tiny_corrections_do_not_trigger_assistance():
    c = VelocityCompensator()
    for i in range(60):
        assert step(c, i * 0.05, wz=0.004) == (0.0, 0.004)


def test_stall_assistance_is_bounded_and_does_not_remove_forward_velocity():
    c = VelocityCompensator()
    samples = [step(c, i * 0.05, vx=0.02) for i in range(25)]
    assert all(v == 0.02 for v, _ in samples)
    assert max(w for _, w in samples) > 0.5
    assert samples[24][1] == pytest.approx(0.06)
    assert all(abs(b[1] - a[1]) <= 0.200001
               for a, b in zip(samples, samples[1:]))


def test_reversal_cancels_pulse_and_does_not_jump_to_opposite_boost():
    c = VelocityCompensator()
    for i in range(12):
        step(c, i * 0.05)
    old = c.output_wz
    _, w = step(c, 0.6, wz=-0.06)
    assert w >= 0.0
    assert abs(w - old) <= 0.200001
    assert c.assist.pulse_until is None
    assert c.assist.stall_since == 0.6


def test_zero_command_stops_immediately():
    c = VelocityCompensator()
    for i in range(12):
        step(c, i * 0.05)
    assert c.output_wz > 0.5
    assert step(c, 0.6, wz=0.0) == (0.0, 0.0)
    assert c.assist.pulse_until is None


def test_missing_or_stale_odom_never_prolongs_boost():
    c = VelocityCompensator()
    assert c.apply(0.02, 0.06, 0.0) == (0.02, 0.06)
    for i in range(12):
        step(c, i * 0.05)
    assert c.apply(0.02, 0.06, 0.9) == (0.02, 0.06)
    assert c.assist.pulse_until is None


def test_command_gap_resets_assistance():
    c = VelocityCompensator()
    for i in range(12):
        step(c, i * 0.05)
    assert step(c, 1.2) == (0.0, 0.06)
    assert c.assist.pulse_until is None


def test_invalid_input_stops_and_resets():
    c = VelocityCompensator()
    assert step(c, 0.0, wz=math.nan) == (0.0, 0.0)
    assert c.last_time is None


def test_invalid_acceleration_is_rejected():
    with pytest.raises(ValueError):
        VelocityCompensator(max_angular_accel=0.0)
