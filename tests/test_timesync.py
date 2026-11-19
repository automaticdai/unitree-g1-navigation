import math

import pytest

from g1_navigation.timesync import ClockOffset


def test_offset_is_minimum_delay_and_preserves_relative_timing():
    c = ClockOffset()
    remote0 = 1000.
    # Remote clock 1617 s behind; latency jitters 2..20 ms.
    for i, latency in enumerate([0.02, 0.005, 0.002, 0.015]):
        c.update(remote0 + 0.1 * i, remote0 + 1617. + 0.1 * i + latency)
    assert c.offset == pytest.approx(1617.002)
    a, b = c.local(remote0 + 0.300), c.local(remote0 + 0.305)
    assert b - a == pytest.approx(0.005)


def test_old_samples_leave_the_window():
    c = ClockOffset(window=1.)
    c.update(0., 10.001)          # unusually fast message
    for t in range(1, 30):
        c.update(t * 0.1, 10. + t * 0.1 + 0.01)
    assert c.offset == pytest.approx(10.01)


def test_backward_clock_step_resets():
    c = ClockOffset(jump=1.)
    c.update(100., 200.)
    c.update(50., 200.1)          # remote clock jumped back 50 s
    assert c.resets == 1 and c.offset == pytest.approx(150.1)


def test_forward_clock_step_is_followed_immediately():
    c = ClockOffset()
    c.update(100., 200.)
    c.update(160., 200.1)         # remote jumped forward: smaller delay wins at once
    assert c.offset == pytest.approx(40.1)


def test_invalid_input():
    with pytest.raises(ValueError):
        ClockOffset(window=0.)
    with pytest.raises(ValueError):
        ClockOffset().update(math.nan, 0.)
    with pytest.raises(ValueError):
        ClockOffset().local(0.)
