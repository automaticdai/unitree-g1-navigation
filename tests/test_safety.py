import math
import pytest
from g1_navigation.safety import CommandGate, Limits, required_health


def healthy(gate, now):
    for name in gate.required:
        gate.update_health(name, True, now)


def test_arming_requires_every_health_input():
    gate = CommandGate()
    assert not gate.arm(0.)
    for name in gate.required[:-1]:
        gate.update_health(name, True, 0.)
    assert not gate.arm(0.)
    healthy(gate, 0.)
    assert gate.arm(0.)


def test_prearm_command_is_never_replayed_and_velocity_is_bounded():
    gate = CommandGate()
    healthy(gate, 1.)
    gate.receive((1., 1., 1.), 1.)
    assert gate.arm(1.)
    assert gate.output(1.) == (0., 0., 0.)
    gate.receive((1., 1., -1.), 1.1)
    assert gate.output(1.1) == (0.2, 0., -0.3)
    gate.receive((-1., 0., 0.), 1.2)
    assert gate.output(1.2) == (0., 0., 0.)


def test_command_loss_latches_and_requires_explicit_rearm():
    gate = CommandGate()
    healthy(gate, 0.)
    gate.arm(0.)
    gate.receive((0.1, 0., 0.), 0.)
    assert gate.output(0.1)[0] == 0.1
    assert gate.output(0.26) == (0., 0., 0.)
    healthy(gate, 0.3)
    gate.receive((0.1, 0., 0.), 0.3)
    assert gate.output(0.3) == (0., 0., 0.)
    assert not gate.armed
    assert gate.arm(0.3)
    assert gate.output(0.3) == (0., 0., 0.)


@pytest.mark.parametrize('failed', CommandGate().required)
def test_unhealthy_input_stops_immediately(failed):
    gate = CommandGate()
    healthy(gate, 0.)
    gate.arm(0.)
    gate.receive((0.1, 0., 0.), 0.)
    gate.update_health(failed, False, 0.1)
    assert not gate.armed
    assert gate.output(0.1) == (0., 0., 0.)


def test_fresh_commands_do_not_mask_stale_health():
    gate = CommandGate()
    healthy(gate, 0.)
    gate.arm(0.)
    gate.receive((0.1, 0., 0.), 0.6)
    assert gate.output(0.6) == (0., 0., 0.)
    assert 'stale' in gate.reason


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_commands_latch_fault(value):
    gate = CommandGate()
    healthy(gate, 0.)
    gate.arm(0.)
    gate.receive((value, 0., 0.), 0.)
    assert gate.output(0.) == (0., 0., 0.)
    assert not gate.armed


def test_clock_reversal_stops_and_initial_wait_expires():
    gate = CommandGate()
    healthy(gate, 10.)
    gate.arm(10.)
    assert gate.output(9.) == (0., 0., 0.)
    assert not gate.armed
    healthy(gate, 11.)
    gate.arm(11.)
    healthy(gate, 22.)
    assert gate.output(22.) == (0., 0., 0.)
    assert not gate.armed


@pytest.mark.parametrize('kwargs', [{'max_vx': -1.}, {'max_wz': math.nan}, {'health_timeout': 0.}])
def test_bad_limits_rejected(kwargs):
    with pytest.raises(ValueError):
        Limits(**kwargs)


def test_reverse_is_clamped_to_reverse_limit():
    gate = CommandGate(Limits(max_vx_reverse=0.1))
    healthy(gate, 0.)
    gate.arm(0.)
    gate.receive((-1., 0., 0.), 0.)
    assert gate.output(0.) == (-0.1, 0., 0.)


def test_reverse_disabled_by_default():
    gate = CommandGate()
    healthy(gate, 0.)
    gate.arm(0.)
    gate.receive((-0.05, 0., 0.), 0.)
    assert gate.output(0.) == (0., 0., 0.)


@pytest.mark.parametrize('profile,hardware,expected', [
    ('navigation', False, ('perception', 'localization', 'robot', 'operator')),
    ('navigation', True, ('perception', 'localization', 'robot', 'remote')),
    ('teleop', False, ('perception', 'robot', 'operator')),
    ('teleop', True, ('perception', 'robot', 'operator', 'remote'))])
def test_required_health_per_profile(profile, hardware, expected):
    assert required_health(profile, hardware) == expected


def test_unknown_profile_rejected():
    with pytest.raises(ValueError):
        required_health('mapping', False)


def test_zero_commands_keep_gate_armed():
    gate = CommandGate(required=required_health('teleop', False))
    healthy(gate, 0.)
    gate.arm(0.)
    for t in (0.1, 0.2, 0.3):
        healthy(gate, t)
        gate.receive((0., 0., 0.), t)
        assert gate.output(t) == (0., 0., 0.)
    assert gate.armed
