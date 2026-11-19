"""Map a remote sensor clock onto local time without disturbing relative timing.

Unitree's onboard computers stamp /utlidar messages with their own clock, which
is not synchronised with this machine (observed ~27 minutes apart). Freshness
checks (collision monitor source_timeout, perception health) compare stamps to
local time, so stale-looking data would be silently ignored as "no obstacles".

offset = min over a sliding window of (local receive time - remote stamp), i.e.
the clock offset plus the smallest transport latency seen. One estimator is
shared by all streams from the same clock, so LiDAR/IMU relative timing (which
FAST-LIO depends on) is preserved exactly.
"""
from collections import deque
import math


class ClockOffset:
    def __init__(self, window: float = 5.0, jump: float = 1.0):
        if not (math.isfinite(window) and window > 0 and math.isfinite(jump) and jump > 0):
            raise ValueError('window and jump must be positive')
        self.window, self.jump = window, jump
        self.samples = deque()  # (received, received - stamp)
        self.offset = None
        self.resets = 0

    def update(self, stamp: float, received: float) -> float:
        """Record one message; return the offset to add to remote stamps."""
        if not (math.isfinite(stamp) and math.isfinite(received)):
            raise ValueError('non-finite time')
        delay = received - stamp
        if self.offset is not None and delay > self.offset + self.jump:
            # Remote clock stepped backwards (or a long stall): start again.
            self.samples.clear()
            self.resets += 1
        self.samples.append((received, delay))
        while self.samples and received - self.samples[0][0] > self.window:
            self.samples.popleft()
        self.offset = min(d for _, d in self.samples)
        return self.offset

    def local(self, stamp: float) -> float:
        if self.offset is None:
            raise ValueError('no samples yet')
        return stamp + self.offset
