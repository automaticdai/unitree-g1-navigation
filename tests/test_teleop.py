import math

import pytest

from g1_navigation.teleop import Gamepad, GamepadConfig


def joy(lin=0., ang=0., deadman=0, arm=0, disarm=0, cfg=GamepadConfig()):
    axes = [0.] * 8
    buttons = [0] * 11
    axes[cfg.axis_linear] = lin
    axes[cfg.axis_angular] = ang
    if cfg.deadman_axis >= 0:
        axes[cfg.deadman_axis] = -1. if deadman else 1.  # Xbox trigger: +1 released, -1 pressed
    else:
        buttons[cfg.deadman_button] = deadman
    buttons[cfg.arm_button] = arm
    buttons[cfg.disarm_button] = disarm
    return axes, buttons


def test_no_sample_is_unhealthy_and_zero():
    g = Gamepad()
    assert not g.healthy(0.)
    assert g.command(0.) == (0., 0.)


def test_deadman_gates_motion_and_release_stops_immediately():
    g = Gamepad()
    g.update(*joy(lin=1.), 0.)
    assert g.command(0.) == (0., 0.)
    g.update(*joy(lin=1., deadman=1), 0.05)
    g.command(0.05)
    for t in (0.1, 0.5, 1.0):
        g.update(*joy(lin=1., deadman=1), t)
        out = g.command(t)
    assert out == pytest.approx((0.2, 0.))
    g.update(*joy(lin=1.), 1.05)
    assert g.command(1.05) == (0., 0.)


def test_ramp_limits_increase_only():
    g = Gamepad()
    g.update(*joy(lin=1., deadman=1), 0.)
    g.command(0.)
    g.update(*joy(lin=1., deadman=1), 0.1)
    assert g.command(0.1)[0] == pytest.approx(0.04)  # accel 0.4 * 0.1


def test_reversal_passes_through_zero():
    g = Gamepad()
    for t in (0., 0.5, 1.0):
        g.update(*joy(lin=1., deadman=1), t)
        g.command(t)
    g.update(*joy(lin=-1., deadman=1), 1.05)
    assert g.command(1.05)[0] == pytest.approx(-0.02)  # zeroed then 0.4*0.05
    assert all(g.command(t)[0] >= -0.1 for t in (1.1, 2., 3.))


def test_deadzone_and_reverse_scaling():
    g = Gamepad()
    for t in (0., 1., 2.):
        g.update(*joy(lin=0.05, ang=0.05, deadman=1), t)
    assert g.command(2.) == (0., 0.)
    g2 = Gamepad()
    for t in (0., 1., 2.):
        g2.update(*joy(lin=-1., ang=-1., deadman=1), t)
        out = g2.command(t)
    assert out == pytest.approx((-0.1, -0.3))


def test_stale_joy_is_unhealthy_and_zero():
    g = Gamepad()
    for t in (0., 0.5, 1.):
        g.update(*joy(lin=1., deadman=1), t)
        g.command(t)
    assert g.healthy(1.25)
    assert not g.healthy(1.26)
    assert g.command(1.3) == (0., 0.)


def test_arm_requires_deadman_and_neutral_sticks_on_rising_edge():
    g = Gamepad()
    assert g.update(*joy(arm=1), 0.) == []  # no deadman
    assert g.update(*joy(), 0.05) == []
    assert g.update(*joy(lin=0.5, deadman=1, arm=1), 0.1) == []  # sticks deflected
    assert g.update(*joy(deadman=1), 0.15) == []
    assert g.update(*joy(deadman=1, arm=1), 0.2) == ['arm']
    assert g.update(*joy(deadman=1, arm=1), 0.25) == []  # held, no repeat


def test_disarm_always_on_rising_edge():
    g = Gamepad()
    assert g.update(*joy(lin=1., disarm=1), 0.) == ['disarm']
    assert g.update(*joy(lin=1., disarm=1), 0.05) == []


def test_short_or_nonfinite_sample_is_unhealthy_without_exception():
    g = Gamepad()
    g.update(*joy(deadman=1), 0.)
    assert g.update([0.], [0], 0.1) == []
    assert not g.healthy(0.3)
    g2 = Gamepad()
    g2.update(*joy(lin=math.nan, deadman=1), 0.)
    assert not g2.healthy(0.) and g2.command(0.) == (0., 0.)


def test_invalid_sample_is_unhealthy_immediately_until_next_valid():
    g = Gamepad()
    g.update(*joy(deadman=1), 0.)
    assert g.healthy(0.05)
    assert g.update([0.], [0], 0.1) == []
    assert not g.healthy(0.1)
    assert g.command(0.1) == (0., 0.)
    g.update(*joy(deadman=1), 0.15)
    assert g.healthy(0.15)


def test_invalid_sample_does_not_update_edge_state():
    g = Gamepad()
    g.update(*joy(disarm=1), 0.)
    assert g.update([0.], [0], 0.05) == []
    # Button still held on next valid sample: not a new rising edge.
    assert g.update(*joy(disarm=1), 0.1) == []


@pytest.mark.parametrize('kwargs', [
    {'deadzone': 1.}, {'max_vx': -1.}, {'joy_timeout': 0.}, {'accel_vx': math.inf},
    {'axis_linear': -1}, {'deadman_button': 1.5}, {'accel_wz': 0.}, {'max_wz': math.nan},
])
def test_bad_config_rejected(kwargs):
    with pytest.raises(ValueError):
        GamepadConfig(**kwargs)


def test_out_of_range_axes_are_clamped_to_limits():
    g = Gamepad()
    for t in (0., 1., 2.):
        g.update(*joy(lin=5., ang=5., deadman=1), t)
        out = g.command(t)
    assert out == pytest.approx((0.2, 0.3))
    g2 = Gamepad()
    for t in (0., 1., 2.):
        g2.update(*joy(lin=-5., ang=-5., deadman=1), t)
        out = g2.command(t)
    assert out == pytest.approx((-0.1, -0.3))


def test_default_deadman_is_left_trigger_past_threshold():
    cfg = GamepadConfig()
    assert (cfg.deadman_axis, cfg.deadman_threshold) == (2, -0.5)
    for trigger, moving in ((1., False), (0., False), (-0.4, False), (-0.6, True), (-1., True)):
        g = Gamepad()
        axes, buttons = joy(lin=1.)
        axes[2] = trigger
        buttons[4] = 1  # LB is ignored while the trigger is the deadman.
        for t in (0., 0.5, 1.):
            g.update(axes, buttons, t)
            out = g.command(t)
        assert (out[0] > 0) == moving, trigger


def test_button_deadman_when_axis_disabled():
    cfg = GamepadConfig(deadman_axis=-1)
    g = Gamepad(cfg)
    for t in (0., 0.5, 1.):
        g.update(*joy(lin=1., deadman=1, cfg=cfg), t)
        out = g.command(t)
    assert out == pytest.approx((0.2, 0.))
    axes, buttons = joy(lin=1., cfg=cfg)
    axes[2] = -1.  # Trigger is ignored while LB is the deadman.
    g.update(axes, buttons, 1.05)
    assert g.command(1.05) == (0., 0.)


def test_nonfinite_or_missing_deadman_axis_is_unhealthy():
    g = Gamepad()
    axes, buttons = joy(lin=1., deadman=1)
    axes[2] = math.nan
    assert g.update(axes, buttons, 0.) == [] and not g.healthy(0.)
    assert g.update(axes[:2], buttons, 0.) == [] and not g.healthy(0.)


def test_arm_with_trigger_held():
    g = Gamepad()
    assert g.update(*joy(arm=1), 0.) == []
    assert g.update(*joy(deadman=1), 0.05) == []
    assert g.update(*joy(deadman=1, arm=1), 0.1) == ['arm']


@pytest.mark.parametrize('kwargs', [{'deadman_axis': -2}, {'deadman_threshold': 1.}, {'deadman_threshold': -1.},
                                    {'deadman_threshold': math.nan}])
def test_bad_deadman_axis_config_rejected(kwargs):
    with pytest.raises(ValueError):
        GamepadConfig(**kwargs)
