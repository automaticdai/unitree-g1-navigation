"""Fail-closed command gate. All deadlines use a monotonic clock."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Limits:
    max_vx: float = 0.2
    max_vy: float = 0.0
    max_wz: float = 0.3
    command_timeout: float = 0.25
    health_timeout: float = 0.5
    first_command_timeout: float = 10.0

    def __post_init__(self):
        values = vars(self)
        if not all(math.isfinite(v) and v >= 0 for v in values.values()):
            raise ValueError('Limits must be finite and nonnegative')
        if self.command_timeout <= 0 or self.health_timeout <= 0 or self.first_command_timeout <= 0:
            raise ValueError('Timeouts must be positive')


class CommandGate:
    def __init__(self, limits=Limits(), required=('perception', 'localization', 'robot', 'operator')):
        self.limits = limits
        self.required = required
        self.health = {}
        self.armed = False
        self.reason = 'disarmed'
        self.command = (0.0, 0.0, 0.0)
        self.command_at = None
        self.armed_at = None

    def update_health(self, name, healthy, now):
        self.health[name] = (bool(healthy), now)
        if name in self.required and not healthy and self.armed:
            self.stop(f'{name} unhealthy')

    def health_failure(self, now):
        for name in self.required:
            value = self.health.get(name)
            if value is None or not value[0]:
                return f'{name} unhealthy or missing'
            if not 0 <= now - value[1] <= self.limits.health_timeout:
                return f'{name} stale'
        return ''

    def arm(self, now):
        failure = self.health_failure(now)
        if failure:
            self.stop(failure)
            return False
        self.armed = True
        self.reason = 'armed; awaiting new command'
        self.command = (0.0, 0.0, 0.0)
        # Never replay a command received before arming.
        self.command_at = None
        self.armed_at = now
        return True

    def stop(self, reason='operator stop'):
        self.armed = False
        self.reason = reason
        self.command = (0.0, 0.0, 0.0)
        self.command_at = None

    def receive(self, command, now):
        if len(command) != 3 or not all(math.isfinite(v) for v in command):
            self.stop('invalid velocity')
            return
        if not self.armed:
            return
        # No backwards/lateral walking in the first operating profile.
        vx, vy, wz = command
        l = self.limits
        self.command = (max(0.0, min(vx, l.max_vx)),
                        max(-l.max_vy, min(vy, l.max_vy)),
                        max(-l.max_wz, min(wz, l.max_wz)))
        self.command_at = now

    def output(self, now):
        if self.armed:
            failure = self.health_failure(now)
            if failure:
                self.stop(failure)
            elif self.command_at is None and not 0 <= now - self.armed_at <= self.limits.first_command_timeout:
                self.stop('no command after arming')
            elif self.command_at is not None and not 0 <= now - self.command_at <= self.limits.command_timeout:
                self.stop('command stale')
        return self.command if self.armed else (0.0, 0.0, 0.0)
