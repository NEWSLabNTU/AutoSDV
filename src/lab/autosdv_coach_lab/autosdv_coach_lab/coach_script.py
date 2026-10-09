"""Scripted coach board for the simulator, and the detector it pretends to be.

ROS-free. virtual_coach_node wraps it.

The board moves along the ego's heading at the moment the script starts,
`initial_distance` ahead of the ego's start pose and `lateral_offset` to its
left, with a piecewise-constant speed. The detector model then turns the true
board-relative-to-ego position into what a LiDAR detector would publish:
Gaussian noise, a fixed latency (the stamp is the sample time, the message
leaves `latency` later), scripted dropout windows, and random drops.
"""

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class CoachScript:
    initial_distance: float = 6.0
    lateral_offset: float = 0.0
    # Segment i lasts durations[i] seconds at a target speed of speeds[i] m/s.
    # After the last segment the target is zero.
    durations: List[float] = field(default_factory=lambda: [10.0, 25.0])
    speeds: List[float] = field(default_factory=lambda: [0.0, 0.8])
    # How fast the board changes speed, m/s^2; 0 = instantly. A walking person
    # starts and stops at roughly 1 m/s^2. An instant stop is the worst case
    # for the tracker (constant velocity until the detections disagree).
    accel: float = 0.0

    def __post_init__(self):
        if len(self.durations) != len(self.speeds):
            raise ValueError('durations and speeds must have the same length')
        if any(d < 0.0 for d in self.durations):
            raise ValueError('segment durations must be non-negative')
        if self.accel < 0.0:
            raise ValueError('accel must be non-negative')
        self._table = None
        if self.accel > 0.0:
            self._build_table()

    _DT = 0.01

    def _build_table(self):
        # Integrate the rate-limited speed once; distance() interpolates.
        n = int((self.total_duration + 30.0) / self._DT) + 1
        v, d = 0.0, self.initial_distance
        ds, vs = [d], [v]
        for i in range(1, n):
            target = self.target_speed((i - 0.5) * self._DT)
            step = self.accel * self._DT
            v = min(target, v + step) if target > v else max(target, v - step)
            d += v * self._DT
            ds.append(d)
            vs.append(v)
        self._table = (ds, vs)

    @property
    def total_duration(self) -> float:
        return sum(self.durations)

    def target_speed(self, t: float) -> float:
        if t < 0.0:
            return 0.0
        for dur, v in zip(self.durations, self.speeds):
            if t < dur:
                return v
            t -= dur
        return 0.0

    def _lookup(self, t, k):
        tab = self._table[k]
        x = max(t, 0.0) / self._DT
        i = int(x)
        if i >= len(tab) - 1:
            return tab[-1]
        a = x - i
        return tab[i] + a * (tab[i + 1] - tab[i])

    def distance(self, t: float) -> float:
        """Board distance along the start heading at script time t."""
        if self._table is not None:
            return self._lookup(t, 0)
        d = self.initial_distance
        t = max(t, 0.0)
        for dur, v in zip(self.durations, self.speeds):
            step = min(t, dur)
            d += v * step
            t -= step
            if t <= 0.0:
                break
        return d

    def speed(self, t: float) -> float:
        if self._table is not None:
            return self._lookup(t, 1)
        return self.target_speed(t)

    def board_world(self, t, anchor) -> Tuple[float, float]:
        """Board (x, y) in the odometry frame. anchor = (x, y, yaw) of the ego
        when the script started."""
        ax, ay, ayaw = anchor
        d = self.distance(t)
        c, s = math.cos(ayaw), math.sin(ayaw)
        return (ax + d * c - self.lateral_offset * s,
                ay + d * s + self.lateral_offset * c)


def world_to_base_link(bx, by, ego_x, ego_y, ego_yaw) -> Tuple[float, float]:
    dx, dy = bx - ego_x, by - ego_y
    c, s = math.cos(ego_yaw), math.sin(ego_yaw)
    return (c * dx + s * dy, -s * dx + c * dy)


@dataclass
class DetectorModel:
    noise_std: float = 0.03            # m, per axis
    latency: float = 0.1               # s, sample -> publish
    dropout_prob: float = 0.0          # per sample
    # Flattened [start, end, start, end, ...] in script time: no detections
    # are produced inside any window.
    dropout_windows: List[float] = field(default_factory=list)
    max_range: float = 15.0            # m, nothing beyond is detected
    fov: float = math.radians(120.0)   # full horizontal field of view
    seed: Optional[int] = None

    def __post_init__(self):
        if len(self.dropout_windows) % 2:
            raise ValueError('dropout_windows must be [start, end] pairs')
        self.rng = random.Random(self.seed)

    def in_window(self, t: float) -> bool:
        w = self.dropout_windows
        return any(w[i] <= t < w[i + 1] for i in range(0, len(w), 2))

    def sample(self, t_script, x, y):
        """Detection of a board at base_link (x, y), or None if not detected."""
        if self.in_window(t_script):
            return None
        if self.dropout_prob > 0.0 and self.rng.random() < self.dropout_prob:
            return None
        if math.hypot(x, y) > self.max_range:
            return None
        if abs(math.atan2(y, x)) > self.fov / 2.0:
            return None
        return (x + self.rng.gauss(0.0, self.noise_std),
                y + self.rng.gauss(0.0, self.noise_std))
