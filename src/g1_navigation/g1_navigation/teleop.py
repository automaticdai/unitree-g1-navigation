"""ROS-free gamepad teleop logic: deadman, deadzone, ramp, and arm/disarm edges."""
from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class GamepadConfig:
    axis_linear: int = 1        # left stick vertical (Xbox layout via joy)
    axis_angular: int = 0       # left stick horizontal
    deadman_button: int = 4     # LB
    arm_button: int = 7         # Start
    disarm_button: int = 6      # Back
    deadzone: float = 0.1
    max_vx: float = 0.2
    max_vx_reverse: float = 0.1
    max_wz: float = 0.3
    accel_vx: float = 0.4       # m/s^2, increases only
    accel_wz: float = 0.8       # rad/s^2, increases only
    joy_timeout: float = 0.25

    def __post_init__(self):
        for name in ('axis_linear', 'axis_angular', 'deadman_button', 'arm_button', 'disarm_button'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f'{name} must be an integer >= 0')
        for name in ('deadzone', 'max_vx', 'max_vx_reverse', 'max_wz',
                     'accel_vx', 'accel_wz', 'joy_timeout'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if self.joy_timeout <= 0 or self.accel_vx <= 0 or self.accel_wz <= 0:
            raise ValueError('joy_timeout and accelerations must be positive')
        if self.deadzone >= 1:
            raise ValueError('deadzone must be < 1')


class Gamepad:
    def __init__(self, config: GamepadConfig = GamepadConfig()):
        self.config = config
        self._valid = False
        self._last_valid = None
        self._lin = 0.0
        self._ang = 0.0
        self._deadman = False
        self._prev_arm = False
        self._prev_disarm = False
        self._current = (0.0, 0.0)
        self._last_command = None

    def _in_deadzone(self, value):
        return abs(value) < self.config.deadzone

    def update(self, axes: Sequence[float], buttons: Sequence[int], now: float) -> list:
        """Record a /joy sample; return events, each 'arm' or 'disarm'."""
        c = self.config
        valid = (len(axes) > max(c.axis_linear, c.axis_angular)
                 and len(buttons) > max(c.deadman_button, c.arm_button, c.disarm_button))
        if valid:
            lin, ang = axes[c.axis_linear], axes[c.axis_angular]
            valid = math.isfinite(lin) and math.isfinite(ang)
        if not valid:
            # Unhealthy immediately; edge state and last-valid time untouched.
            self._valid = False
            return []
        self._valid = True
        self._last_valid = now
        self._lin, self._ang = float(lin), float(ang)
        self._deadman = bool(buttons[c.deadman_button])
        arm = bool(buttons[c.arm_button])
        disarm = bool(buttons[c.disarm_button])
        events = []
        if arm and not self._prev_arm and self._deadman \
                and self._in_deadzone(self._lin) and self._in_deadzone(self._ang):
            events.append('arm')
        if disarm and not self._prev_disarm:
            events.append('disarm')
        self._prev_arm = arm
        self._prev_disarm = disarm
        return events

    def healthy(self, now: float) -> bool:
        return self._valid and 0 <= now - self._last_valid <= self.config.joy_timeout

    def _rescale(self, value):
        dz = self.config.deadzone
        value = max(-1.0, min(1.0, value))
        if abs(value) < dz:
            return 0.0
        return math.copysign((abs(value) - dz) / (1 - dz), value)

    def _target(self, now):
        if not self.healthy(now) or not self._deadman:
            return (0.0, 0.0)
        c = self.config
        lin = self._rescale(self._lin)
        vx = lin * (c.max_vx if lin > 0 else c.max_vx_reverse)
        wz = self._rescale(self._ang) * c.max_wz
        return (vx, wz)

    @staticmethod
    def _ramp(current, target, limit):
        if current * target < 0:
            current = 0.0
        if abs(target) <= abs(current):
            return target
        step = math.copysign(min(limit, abs(target - current)), target)
        return current + step

    def command(self, now: float) -> tuple:
        """(vx, wz) after deadzone, scaling, deadman, ramp. Call once per 20 Hz tick."""
        dt = 0.0 if self._last_command is None else max(0.0, now - self._last_command)
        self._last_command = now
        if not self.healthy(now) or not self._deadman:
            self._current = (0.0, 0.0)
            return self._current
        target = self._target(now)
        c = self.config
        self._current = (self._ramp(self._current[0], target[0], c.accel_vx * dt),
                         self._ramp(self._current[1], target[1], c.accel_wz * dt))
        return self._current
